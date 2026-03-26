//! Overpass API fetching and caching for road networks.

use std::collections::HashMap;
use std::future::Future;
use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use tokio::sync::{mpsc::Sender, Mutex, OwnedMutexGuard};
use tokio::time::sleep;
use tracing::{debug, info};

use super::bbox::BoundingBox;
use super::cache::{
    cache, in_flight_loads, record_disk_hit, record_in_flight_wait, record_load_request,
    record_memory_hit, record_network_fetch, CachedEdge, CachedNetwork, CachedNode, NetworkRef,
    CACHE_VERSION,
};
use super::config::{ConnectivityPolicy, NetworkConfig};
use super::coord::Coord;
use super::error::RoutingError;
use super::network::{EdgeData, RoadNetwork};
use super::osm::OverpassResponse;
use super::progress::RoutingProgress;

impl RoadNetwork {
    pub async fn load_or_fetch(
        bbox: &BoundingBox,
        config: &NetworkConfig,
        progress: Option<&Sender<RoutingProgress>>,
    ) -> Result<NetworkRef, RoutingError> {
        let cache_key = bbox.cache_key();
        record_load_request();

        if let Some(tx) = progress {
            let _ = tx.send(RoutingProgress::CheckingCache { percent: 0 }).await;
        }

        {
            let cache_guard = cache().read().await;
            if cache_guard.contains_key(&cache_key) {
                record_memory_hit();
                info!("Using in-memory cached road network for {}", cache_key);
                if let Some(tx) = progress {
                    let _ = tx
                        .send(RoutingProgress::CheckingCache { percent: 10 })
                        .await;
                }
                return Ok(NetworkRef::new(cache_guard, cache_key));
            }
        }
        if let Some(tx) = progress {
            let _ = tx.send(RoutingProgress::CheckingCache { percent: 5 }).await;
        }

        Self::load_or_insert(cache_key, async {
            tokio::fs::create_dir_all(&config.cache_dir).await?;
            let cache_path = config.cache_dir.join(format!("{}.json", bbox.cache_key()));

            if tokio::fs::try_exists(&cache_path).await.unwrap_or(false) {
                info!("Loading road network from file cache: {:?}", cache_path);
                if let Some(tx) = progress {
                    let _ = tx.send(RoutingProgress::CheckingCache { percent: 8 }).await;
                }
                match Self::load_from_file(&cache_path, config).await {
                    Ok(network) => {
                        record_disk_hit();
                        if let Some(tx) = progress {
                            let _ = tx
                                .send(RoutingProgress::BuildingGraph { percent: 50 })
                                .await;
                        }
                        return Ok(network);
                    }
                    Err(e) => info!("File cache invalid ({}), downloading fresh", e),
                }
            } else {
                info!("Downloading road network from Overpass API");
            }

            record_network_fetch();
            let network = Self::fetch_from_api(bbox, config, progress).await?;
            network.save_to_file(&cache_path).await?;
            info!("Saved road network to file cache: {:?}", cache_path);
            Ok(network)
        })
        .await
    }

    pub async fn fetch(
        bbox: &BoundingBox,
        config: &NetworkConfig,
        progress: Option<&Sender<RoutingProgress>>,
    ) -> Result<Self, RoutingError> {
        Self::fetch_from_api(bbox, config, progress).await
    }

    async fn fetch_from_api(
        bbox: &BoundingBox,
        config: &NetworkConfig,
        progress: Option<&Sender<RoutingProgress>>,
    ) -> Result<Self, RoutingError> {
        let highway_regex = config.highway_regex();
        let query = format!(
            r#"[out:json][timeout:120];
(
  way["highway"~"{}"]
    ({},{},{},{});
);
(._;>;);
out body;"#,
            highway_regex, bbox.min_lat, bbox.min_lng, bbox.max_lat, bbox.max_lng
        );

        debug!("Overpass query:\n{}", query);
        info!(
            "Preparing Overpass query for bbox: {:.4},{:.4} to {:.4},{:.4}",
            bbox.min_lat, bbox.min_lng, bbox.max_lat, bbox.max_lng
        );

        if let Some(tx) = progress {
            let _ = tx
                .send(RoutingProgress::DownloadingNetwork {
                    percent: 10,
                    bytes: 0,
                })
                .await;
        }

        let client = reqwest::Client::builder()
            .connect_timeout(config.connect_timeout)
            .read_timeout(config.read_timeout)
            .timeout(config.read_timeout)
            .user_agent("SolverForge/0.5.0")
            .build()
            .map_err(|e| RoutingError::Network(e.to_string()))?;

        if let Some(tx) = progress {
            let _ = tx
                .send(RoutingProgress::DownloadingNetwork {
                    percent: 15,
                    bytes: 0,
                })
                .await;
        }

        let bytes = fetch_overpass_bytes(&client, &query, config, progress).await?;

        let bytes_len = bytes.len();
        if let Some(tx) = progress {
            let _ = tx
                .send(RoutingProgress::DownloadingNetwork {
                    percent: 30,
                    bytes: bytes_len,
                })
                .await;
        }

        if let Some(tx) = progress {
            let _ = tx
                .send(RoutingProgress::ParsingOsm {
                    percent: 32,
                    nodes: 0,
                    edges: 0,
                })
                .await;
        }

        let osm_data: OverpassResponse =
            serde_json::from_slice(&bytes).map_err(|e| RoutingError::Parse(e.to_string()))?;

        info!("Downloaded {} OSM elements", osm_data.elements.len());

        if let Some(tx) = progress {
            let _ = tx
                .send(RoutingProgress::ParsingOsm {
                    percent: 35,
                    nodes: osm_data.elements.len(),
                    edges: 0,
                })
                .await;
        }

        if let Some(tx) = progress {
            let _ = tx
                .send(RoutingProgress::BuildingGraph { percent: 40 })
                .await;
        }

        let network = Self::build_from_osm(&osm_data, config)?;

        if let Some(tx) = progress {
            let _ = tx
                .send(RoutingProgress::BuildingGraph { percent: 50 })
                .await;
            let _ = tx.send(RoutingProgress::Complete).await;
        }

        Ok(network)
    }

    pub(super) fn build_from_osm(
        osm: &OverpassResponse,
        config: &NetworkConfig,
    ) -> Result<Self, RoutingError> {
        let mut network = Self::new();

        let mut nodes: HashMap<i64, (f64, f64)> = HashMap::new();
        for elem in &osm.elements {
            if elem.elem_type == "node" {
                if let (Some(lat), Some(lon)) = (elem.lat, elem.lon) {
                    nodes.insert(elem.id, (lat, lon));
                }
            }
        }

        info!("Parsed {} nodes", nodes.len());

        let mut way_count = 0;
        for elem in &osm.elements {
            if elem.elem_type == "way" {
                if let Some(ref node_ids) = elem.nodes {
                    let highway = elem.tags.as_ref().and_then(|t| t.highway.as_deref());
                    let oneway = elem.tags.as_ref().and_then(|t| t.oneway.as_deref());
                    let maxspeed = elem.tags.as_ref().and_then(|t| t.maxspeed.as_deref());
                    let speed = config
                        .speed_profile
                        .speed_mps(maxspeed, highway.unwrap_or("residential"));
                    let is_oneway_forward = matches!(oneway, Some("yes") | Some("1"));
                    let is_oneway_reverse = matches!(oneway, Some("-1"));

                    for window in node_ids.windows(2) {
                        let n1_id = window[0];
                        let n2_id = window[1];

                        let Some(&(lat1, lng1)) = nodes.get(&n1_id) else {
                            continue;
                        };
                        let Some(&(lat2, lng2)) = nodes.get(&n2_id) else {
                            continue;
                        };

                        let idx1 = network.get_or_create_node(lat1, lng1);
                        let idx2 = network.get_or_create_node(lat2, lng2);

                        let coord1 = Coord::new(lat1, lng1);
                        let coord2 = Coord::new(lat2, lng2);
                        let distance = super::geo::haversine_distance(coord1, coord2);
                        let travel_time = distance / speed;

                        let edge_data = EdgeData {
                            travel_time_s: travel_time,
                            distance_m: distance,
                        };

                        if is_oneway_reverse {
                            // oneway=-1 means traffic flows opposite to way direction
                            network.add_edge(idx2, idx1, edge_data);
                        } else {
                            // Forward direction (always added unless reverse-only)
                            network.add_edge(idx1, idx2, edge_data.clone());
                            if !is_oneway_forward {
                                // Bidirectional road
                                network.add_edge(idx2, idx1, edge_data);
                            }
                        }
                    }

                    way_count += 1;
                }
            }
        }

        info!(
            "Built graph with {} nodes and {} edges from {} ways",
            network.node_count(),
            network.edge_count(),
            way_count
        );

        let scc_count = network.strongly_connected_components();
        match config.connectivity_policy {
            ConnectivityPolicy::KeepAll => {
                if scc_count > 1 {
                    info!(
                        "Road network has {} SCCs, preserving all components by configuration",
                        scc_count
                    );
                }
            }
            ConnectivityPolicy::LargestStronglyConnectedComponent => {
                if scc_count > 1 {
                    info!(
                        "Road network has {} SCCs, filtering to largest component",
                        scc_count
                    );
                    network.filter_to_largest_scc();
                    info!(
                        "After SCC filter: {} nodes, {} edges",
                        network.node_count(),
                        network.edge_count()
                    );
                }
            }
        }

        network.build_spatial_index();

        Ok(network)
    }

    async fn load_from_file(path: &Path, config: &NetworkConfig) -> Result<Self, RoutingError> {
        let data = tokio::fs::read_to_string(path).await?;

        let cached: CachedNetwork = match serde_json::from_str(&data) {
            Ok(c) => c,
            Err(e) => {
                info!("Cache file corrupted, will re-download: {}", e);
                let _ = tokio::fs::remove_file(path).await;
                return Err(RoutingError::Parse(e.to_string()));
            }
        };

        if cached.version != CACHE_VERSION {
            info!(
                "Cache version mismatch (got {}, need {}), will re-download",
                cached.version, CACHE_VERSION
            );
            let _ = tokio::fs::remove_file(path).await;
            return Err(RoutingError::Parse("cache version mismatch".into()));
        }

        let mut network = Self::new();

        for node in &cached.nodes {
            network.add_node_at(node.lat, node.lng);
        }

        for edge in &cached.edges {
            network.add_edge_by_index(edge.from, edge.to, edge.travel_time_s, edge.distance_m);
        }

        let scc_count = network.strongly_connected_components();
        match config.connectivity_policy {
            ConnectivityPolicy::KeepAll => {
                if scc_count > 1 {
                    info!(
                        "Cached network has {} SCCs, preserving all components by configuration",
                        scc_count
                    );
                }
            }
            ConnectivityPolicy::LargestStronglyConnectedComponent => {
                if scc_count > 1 {
                    info!(
                        "Cached network has {} SCCs, filtering to largest component",
                        scc_count
                    );
                    network.filter_to_largest_scc();
                    info!(
                        "After SCC filter: {} nodes, {} edges",
                        network.node_count(),
                        network.edge_count()
                    );
                }
            }
        }

        network.build_spatial_index();

        Ok(network)
    }

    async fn save_to_file(&self, path: &Path) -> Result<(), RoutingError> {
        let nodes: Vec<CachedNode> = self
            .nodes_iter()
            .map(|(lat, lng)| CachedNode { lat, lng })
            .collect();

        let edges: Vec<CachedEdge> = self
            .edges_iter()
            .map(|(from, to, travel_time_s, distance_m)| CachedEdge {
                from,
                to,
                travel_time_s,
                distance_m,
            })
            .collect();

        let cached = CachedNetwork {
            version: CACHE_VERSION,
            nodes,
            edges,
        };
        let data =
            serde_json::to_string(&cached).map_err(|e| RoutingError::Parse(e.to_string()))?;
        tokio::fs::write(path, data).await?;

        Ok(())
    }

    async fn load_or_insert<F>(cache_key: String, load: F) -> Result<NetworkRef, RoutingError>
    where
        F: Future<Output = Result<RoadNetwork, RoutingError>>,
    {
        if let Some(cached) = Self::get_cached_network(cache_key.clone()).await {
            return Ok(cached);
        }

        let (slot, _slot_guard, waited) = acquire_in_flight_slot(&cache_key).await;
        if waited {
            record_in_flight_wait();
        }

        if let Some(cached) = Self::get_cached_network(cache_key.clone()).await {
            cleanup_in_flight_slot(&cache_key, &slot).await;
            return Ok(cached);
        }

        let network = load.await?;

        {
            let mut cache_guard = cache().write().await;
            cache_guard.entry(cache_key.clone()).or_insert(network);
        }

        cleanup_in_flight_slot(&cache_key, &slot).await;

        Self::get_cached_network(cache_key).await.ok_or_else(|| {
            RoutingError::Network("cached network disappeared after insertion".to_string())
        })
    }

    async fn get_cached_network(cache_key: String) -> Option<NetworkRef> {
        let cache_guard = cache().read().await;
        if cache_guard.contains_key(&cache_key) {
            Some(NetworkRef::new(cache_guard, cache_key))
        } else {
            None
        }
    }
}

async fn fetch_overpass_bytes(
    client: &reqwest::Client,
    query: &str,
    config: &NetworkConfig,
    progress: Option<&Sender<RoutingProgress>>,
) -> Result<Vec<u8>, RoutingError> {
    let endpoints = overpass_endpoints(config);
    let mut failures = Vec::new();

    for (endpoint_index, endpoint) in endpoints.iter().enumerate() {
        for attempt in 0..=config.overpass_max_retries {
            info!(
                "Sending request to Overpass API endpoint {} attempt {}: {}",
                endpoint_index + 1,
                attempt + 1,
                endpoint
            );

            let response = client
                .post(endpoint)
                .body(query.to_owned())
                .header("Content-Type", "text/plain")
                .send()
                .await;

            match response {
                Ok(response) if response.status().is_success() => {
                    info!(
                        "Received successful Overpass response from {} with status {}",
                        endpoint,
                        response.status()
                    );

                    if let Some(tx) = progress {
                        let _ = tx
                            .send(RoutingProgress::DownloadingNetwork {
                                percent: 25,
                                bytes: 0,
                            })
                            .await;
                    }

                    return response
                        .bytes()
                        .await
                        .map(|bytes| bytes.to_vec())
                        .map_err(|error| {
                            RoutingError::Network(format!(
                                "Overpass response body read failed from {} on attempt {}: {}",
                                endpoint,
                                attempt + 1,
                                error
                            ))
                        });
                }
                Ok(response) => {
                    let status = response.status();
                    failures.push(format!(
                        "{} attempt {} returned HTTP {}",
                        endpoint,
                        attempt + 1,
                        status
                    ));

                    if is_retryable_status(status) && attempt < config.overpass_max_retries {
                        sleep(retry_backoff(config.overpass_retry_backoff, attempt)).await;
                        continue;
                    }

                    break;
                }
                Err(error) => {
                    failures.push(format!(
                        "{} attempt {} failed: {}",
                        endpoint,
                        attempt + 1,
                        error
                    ));

                    if is_retryable_error(&error) && attempt < config.overpass_max_retries {
                        sleep(retry_backoff(config.overpass_retry_backoff, attempt)).await;
                        continue;
                    }

                    break;
                }
            }
        }
    }

    Err(RoutingError::Network(format!(
        "Overpass fetch failed after trying {} endpoint(s): {}",
        endpoints.len(),
        failures.join("; ")
    )))
}

fn overpass_endpoints(config: &NetworkConfig) -> Vec<String> {
    if config.overpass_endpoints.is_empty() {
        vec![config.overpass_url.clone()]
    } else {
        config.overpass_endpoints.clone()
    }
}

fn retry_backoff(base: Duration, attempt: usize) -> Duration {
    base.saturating_mul((attempt + 1) as u32)
}

fn is_retryable_status(status: reqwest::StatusCode) -> bool {
    status.is_server_error()
        || status == reqwest::StatusCode::TOO_MANY_REQUESTS
        || status == reqwest::StatusCode::REQUEST_TIMEOUT
}

fn is_retryable_error(error: &reqwest::Error) -> bool {
    error.is_timeout() || error.is_connect() || error.is_request()
}

async fn acquire_in_flight_slot(cache_key: &str) -> (Arc<Mutex<()>>, OwnedMutexGuard<()>, bool) {
    let slot = {
        let mut in_flight = in_flight_loads().lock().await;
        match in_flight.get(cache_key) {
            Some(slot) => slot.clone(),
            None => {
                let slot = Arc::new(Mutex::new(()));
                in_flight.insert(cache_key.to_string(), slot.clone());
                slot
            }
        }
    };

    match slot.clone().try_lock_owned() {
        Ok(guard) => (slot, guard, false),
        Err(_) => {
            let guard = slot.clone().lock_owned().await;
            (slot, guard, true)
        }
    }
}

async fn cleanup_in_flight_slot(cache_key: &str, slot: &Arc<Mutex<()>>) {
    let mut in_flight = in_flight_loads().lock().await;
    let should_remove = in_flight
        .get(cache_key)
        .map(|current| Arc::ptr_eq(current, slot) && Arc::strong_count(slot) == 2)
        .unwrap_or(false);

    if should_remove {
        in_flight.remove(cache_key);
    }
}

#[cfg(test)]
mod tests {
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
            memory_hits: 0,
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
}
