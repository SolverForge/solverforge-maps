use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::sync::OnceLock;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use tokio::sync::Mutex;
use tokio::time::sleep;

use super::*;
use crate::routing::cache::{reset_cache_metrics, CacheStats};
use crate::routing::BoundingBox;

static FETCH_TEST_LOCK: OnceLock<Mutex<()>> = OnceLock::new();

fn fetch_test_lock() -> &'static Mutex<()> {
    FETCH_TEST_LOCK.get_or_init(|| Mutex::new(()))
}

fn test_network() -> RoadNetwork {
    RoadNetwork::from_test_data(&[(0.0, 0.0), (0.0, 0.01)], &[(0, 1, 60.0, 1_000.0)])
}

async fn reset_test_state() {
    RoadNetwork::clear_cache().await;
    in_flight_loads().lock().await.clear();
    reset_cache_metrics();
}

fn unique_cache_dir(prefix: &str) -> std::path::PathBuf {
    let suffix = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system time before unix epoch")
        .as_nanos();
    std::env::temp_dir().join(format!(
        "solverforge-maps-{prefix}-{}-{suffix}",
        std::process::id()
    ))
}

async fn assert_cache_stats(expected: CacheStats) {
    let stats = RoadNetwork::cache_stats().await;
    assert_eq!(stats.networks_cached, expected.networks_cached);
    assert_eq!(stats.load_requests, expected.load_requests);
    assert_eq!(stats.memory_hits, expected.memory_hits);
    assert_eq!(stats.disk_hits, expected.disk_hits);
    assert_eq!(stats.network_fetches, expected.network_fetches);
    assert_eq!(stats.in_flight_waits, expected.in_flight_waits);
}

#[tokio::test]
async fn load_or_insert_allows_different_keys_to_progress_concurrently() {
    let _guard = fetch_test_lock().lock().await;
    reset_test_state().await;

    let start = Instant::now();
    let first = async {
        RoadNetwork::load_or_insert("region-a".to_string(), async {
            sleep(Duration::from_millis(100)).await;
            Ok(test_network())
        })
        .await
        .map(|network| network.node_count())
    };
    let second = async {
        RoadNetwork::load_or_insert("region-b".to_string(), async {
            sleep(Duration::from_millis(100)).await;
            Ok(test_network())
        })
        .await
        .map(|network| network.node_count())
    };
    let (left, right) = tokio::join!(first, second);
    left.expect("first load should succeed");
    right.expect("second load should succeed");

    assert!(
        start.elapsed() < Duration::from_millis(180),
        "different keys should not serialize slow loads"
    );
}

#[tokio::test]
async fn load_or_insert_deduplicates_same_key_work() {
    let _guard = fetch_test_lock().lock().await;
    reset_test_state().await;

    let loads = Arc::new(AtomicUsize::new(0));

    let first = {
        let loads = loads.clone();
        async move {
            RoadNetwork::load_or_insert("region-a".to_string(), async move {
                loads.fetch_add(1, Ordering::Relaxed);
                sleep(Duration::from_millis(50)).await;
                Ok(test_network())
            })
            .await
            .map(|network| network.node_count())
        }
    };
    let second = {
        let loads = loads.clone();
        async move {
            RoadNetwork::load_or_insert("region-a".to_string(), async move {
                loads.fetch_add(1, Ordering::Relaxed);
                sleep(Duration::from_millis(50)).await;
                Ok(test_network())
            })
            .await
            .map(|network| network.node_count())
        }
    };

    let (left, right) = tokio::join!(first, second);
    left.expect("first load should succeed");
    right.expect("second load should succeed");

    assert_eq!(loads.load(Ordering::Relaxed), 1);
}

#[tokio::test]
async fn load_or_fetch_records_network_then_memory_hit() {
    let _guard = fetch_test_lock().lock().await;
    reset_test_state().await;

    let bbox = BoundingBox::new(39.95, -75.17, 39.96, -75.16);
    let cache_dir = unique_cache_dir("network-memory");
    let (endpoint, requests, handle) =
        spawn_overpass_server(vec![("200 OK", overpass_fixture_json())]);
    let config = NetworkConfig::new()
        .overpass_endpoints(vec![endpoint])
        .cache_dir(&cache_dir)
        .overpass_max_retries(0);

    let first = RoadNetwork::load_or_fetch(&bbox, &config, None).await;
    assert!(first.is_ok(), "first load should succeed");
    let second = RoadNetwork::load_or_fetch(&bbox, &config, None).await;
    assert!(second.is_ok(), "second load should hit memory cache");

    handle.join().expect("server thread should finish");
    assert_eq!(requests.load(Ordering::Relaxed), 1);

    assert_cache_stats(CacheStats {
        networks_cached: 1,
        total_nodes: 0,
        total_edges: 0,
        memory_bytes: 0,
        load_requests: 2,
        memory_hits: 1,
        disk_hits: 0,
        network_fetches: 1,
        in_flight_waits: 0,
    })
    .await;

    let _ = tokio::fs::remove_dir_all(&cache_dir).await;
}

#[tokio::test]
async fn load_or_fetch_records_disk_hit_without_network_fetch() {
    let _guard = fetch_test_lock().lock().await;
    reset_test_state().await;

    let bbox = BoundingBox::new(39.95, -75.17, 39.96, -75.16);
    let cache_dir = unique_cache_dir("disk-hit");
    tokio::fs::create_dir_all(&cache_dir)
        .await
        .expect("cache dir should be created");
    let cache_path = cache_dir.join(format!("{}.json", bbox.cache_key()));
    let cached = CachedNetwork {
        version: CACHE_VERSION,
        nodes: vec![
            CachedNode {
                lat: 39.95,
                lng: -75.16,
            },
            CachedNode {
                lat: 39.96,
                lng: -75.17,
            },
        ],
        edges: vec![CachedEdge {
            from: 0,
            to: 1,
            travel_time_s: 60.0,
            distance_m: 1_000.0,
        }],
    };
    let data = serde_json::to_string(&cached).expect("cached network should serialize");
    tokio::fs::write(&cache_path, data)
        .await
        .expect("cache file should be written");

    let config = NetworkConfig::new().cache_dir(&cache_dir);
    let network = RoadNetwork::load_or_fetch(&bbox, &config, None).await;
    assert!(network.is_ok(), "disk cache load should succeed");

    assert_cache_stats(CacheStats {
        networks_cached: 1,
        total_nodes: 0,
        total_edges: 0,
        memory_bytes: 0,
        load_requests: 1,
        memory_hits: 0,
        disk_hits: 1,
        network_fetches: 0,
        in_flight_waits: 0,
    })
    .await;

    let _ = tokio::fs::remove_dir_all(&cache_dir).await;
}

#[tokio::test]
async fn load_or_fetch_records_waiter_for_same_key_contention() {
    let _guard = fetch_test_lock().lock().await;
    reset_test_state().await;

    let bbox = BoundingBox::new(39.95, -75.17, 39.96, -75.16);
    let cache_dir = unique_cache_dir("waiter");
    let (endpoint, requests, handle) =
        spawn_overpass_server(vec![("200 OK", overpass_fixture_json())]);
    let config = NetworkConfig::new()
        .overpass_endpoints(vec![endpoint])
        .cache_dir(&cache_dir)
        .overpass_max_retries(0);

    let first = RoadNetwork::load_or_fetch(&bbox, &config, None);
    let second = RoadNetwork::load_or_fetch(&bbox, &config, None);
    let (left, right) = tokio::join!(first, second);
    assert!(left.is_ok(), "first concurrent load should succeed");
    assert!(right.is_ok(), "second concurrent load should succeed");

    handle.join().expect("server thread should finish");
    assert_eq!(requests.load(Ordering::Relaxed), 1);

    assert_cache_stats(CacheStats {
        networks_cached: 1,
        total_nodes: 0,
        total_edges: 0,
        memory_bytes: 0,
        load_requests: 2,
        memory_hits: 1,
        disk_hits: 0,
        network_fetches: 1,
        in_flight_waits: 1,
    })
    .await;

    let _ = tokio::fs::remove_dir_all(&cache_dir).await;
}

#[tokio::test]
async fn acquire_in_flight_slot_does_not_count_existing_unlocked_slot_as_wait() {
    let _guard = fetch_test_lock().lock().await;
    reset_test_state().await;

    let key = "burst-window";
    let slot = Arc::new(Mutex::new(()));
    in_flight_loads()
        .lock()
        .await
        .insert(key.to_string(), slot.clone());

    let (_slot, acquired_guard, waited) = acquire_in_flight_slot(key).await;
    assert!(
        !waited,
        "existing slot without lock contention should not count as a wait"
    );
    drop(acquired_guard);

    cleanup_in_flight_slot(key, &slot).await;
}

#[tokio::test]
async fn acquire_in_flight_slot_reports_wait_when_lock_is_held() {
    let _guard = fetch_test_lock().lock().await;
    reset_test_state().await;

    let key = "held-slot";
    let slot = Arc::new(Mutex::new(()));
    let held_guard = slot.clone().lock_owned().await;
    in_flight_loads()
        .lock()
        .await
        .insert(key.to_string(), slot.clone());

    let waiter = tokio::spawn(async move {
        let (_slot, guard, waited) = acquire_in_flight_slot(key).await;
        (guard, waited)
    });

    tokio::task::yield_now().await;
    drop(held_guard);

    let (acquired_guard, waited) = waiter
        .await
        .expect("waiter task should complete after lock release");
    assert!(waited, "blocked acquisition should count as a wait");
    drop(acquired_guard);

    cleanup_in_flight_slot(key, &slot).await;
}

fn overpass_fixture_json() -> &'static str {
    r#"{
        "elements": [
            {"type": "node", "id": 1, "lat": 39.95, "lon": -75.16},
            {"type": "node", "id": 2, "lat": 39.96, "lon": -75.17},
            {"type": "way", "id": 10, "nodes": [1, 2], "tags": {"highway": "residential"}}
        ]
    }"#
}

fn spawn_overpass_server(
    responses: Vec<(&'static str, &'static str)>,
) -> (String, Arc<AtomicUsize>, thread::JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener should bind");
    let address = format!(
        "http://{}/api/interpreter",
        listener.local_addr().expect("listener addr")
    );
    let requests = Arc::new(AtomicUsize::new(0));
    let served = requests.clone();

    let handle = thread::spawn(move || {
        for (status, body) in responses {
            let (mut stream, _) = listener.accept().expect("connection should arrive");
            let mut buffer = [0_u8; 4096];
            let _ = stream.read(&mut buffer);
            let response = format!(
                "HTTP/1.1 {}\r\nContent-Length: {}\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n{}",
                status,
                body.len(),
                body
            );
            stream
                .write_all(response.as_bytes())
                .expect("response should write");
            served.fetch_add(1, Ordering::Relaxed);
        }
    });

    (address, requests, handle)
}

#[tokio::test]
async fn fetch_retries_same_endpoint_until_success() {
    let _guard = fetch_test_lock().lock().await;
    let (endpoint, requests, handle) = spawn_overpass_server(vec![
        ("429 Too Many Requests", r#"{"elements":[]}"#),
        ("200 OK", overpass_fixture_json()),
    ]);

    let bbox = BoundingBox::try_new(39.94, -75.18, 39.97, -75.15).expect("bbox should build");
    let config = NetworkConfig::default()
        .overpass_url(endpoint)
        .overpass_max_retries(1)
        .overpass_retry_backoff(Duration::from_millis(1));

    let network = RoadNetwork::fetch(&bbox, &config, None)
        .await
        .expect("fetch should succeed after retry");

    assert_eq!(network.node_count(), 2);
    assert_eq!(requests.load(Ordering::Relaxed), 2);
    handle.join().expect("server should join");
}

#[tokio::test]
async fn fetch_falls_back_to_second_endpoint() {
    let _guard = fetch_test_lock().lock().await;
    let (primary, primary_requests, primary_handle) =
        spawn_overpass_server(vec![("503 Service Unavailable", r#"{"elements":[]}"#)]);
    let (secondary, secondary_requests, secondary_handle) =
        spawn_overpass_server(vec![("200 OK", overpass_fixture_json())]);

    let bbox = BoundingBox::try_new(39.94, -75.18, 39.97, -75.15).expect("bbox should build");
    let config = NetworkConfig::default()
        .overpass_endpoints(vec![primary, secondary])
        .overpass_max_retries(0)
        .overpass_retry_backoff(Duration::from_millis(1));

    let network = RoadNetwork::fetch(&bbox, &config, None)
        .await
        .expect("fetch should fall back to second endpoint");

    assert_eq!(network.node_count(), 2);
    assert_eq!(primary_requests.load(Ordering::Relaxed), 1);
    assert_eq!(secondary_requests.load(Ordering::Relaxed), 1);
    primary_handle.join().expect("primary server should join");
    secondary_handle
        .join()
        .expect("secondary server should join");
}
