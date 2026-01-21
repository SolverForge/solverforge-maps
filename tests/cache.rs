//! Async tests for cache operations.

use solverforge_maps::RoadNetwork;
use std::collections::HashMap;
use tokio::sync::RwLock;

#[tokio::test]
async fn cache_access() {
    let cache: RwLock<HashMap<String, RoadNetwork>> = RwLock::new(HashMap::new());
    let mut write_guard = cache.write().await;
    write_guard.insert("test_key".to_string(), RoadNetwork::new());
    drop(write_guard);

    let read_guard = cache.read().await;
    assert!(read_guard.contains_key("test_key"));
}
