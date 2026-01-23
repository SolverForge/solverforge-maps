//! Zero-erasure cache for road networks.

use std::collections::HashMap;
use std::mem::size_of;
use std::ops::Deref;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::OnceLock;

use serde::{Deserialize, Serialize};
use tokio::sync::{RwLock, RwLockReadGuard};

use super::bbox::BoundingBox;
use super::network::RoadNetwork;

pub const CACHE_VERSION: u32 = 4;

static NETWORK_CACHE: OnceLock<RwLock<HashMap<String, RoadNetwork>>> = OnceLock::new();
static CACHE_HITS: AtomicU64 = AtomicU64::new(0);
static CACHE_MISSES: AtomicU64 = AtomicU64::new(0);

pub(crate) fn cache() -> &'static RwLock<HashMap<String, RoadNetwork>> {
    NETWORK_CACHE.get_or_init(|| RwLock::new(HashMap::new()))
}

pub(crate) fn record_hit() {
    CACHE_HITS.fetch_add(1, Ordering::Relaxed);
}

pub(crate) fn record_miss() {
    CACHE_MISSES.fetch_add(1, Ordering::Relaxed);
}

#[derive(Debug, Clone)]
pub struct CacheStats {
    pub networks_cached: usize,
    pub total_nodes: usize,
    pub total_edges: usize,
    pub memory_bytes: usize,
    pub hits: u64,
    pub misses: u64,
}

impl CacheStats {
    pub fn hit_ratio(&self) -> f64 {
        let total = self.hits + self.misses;
        if total == 0 {
            0.0
        } else {
            self.hits as f64 / total as f64
        }
    }
}

/// RAII guard providing zero-cost access to a cached RoadNetwork.
pub struct NetworkRef {
    guard: RwLockReadGuard<'static, HashMap<String, RoadNetwork>>,
    key: String,
}

impl Deref for NetworkRef {
    type Target = RoadNetwork;

    fn deref(&self) -> &RoadNetwork {
        self.guard
            .get(&self.key)
            .expect("cached network disappeared")
    }
}

impl NetworkRef {
    pub(crate) fn new(
        guard: RwLockReadGuard<'static, HashMap<String, RoadNetwork>>,
        key: String,
    ) -> Self {
        debug_assert!(
            guard.contains_key(&key),
            "NetworkRef created for missing key"
        );
        Self { guard, key }
    }

    pub fn cache_key(&self) -> &str {
        &self.key
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct CachedNetwork {
    pub version: u32,
    pub nodes: Vec<CachedNode>,
    pub edges: Vec<CachedEdge>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct CachedNode {
    pub lat: f64,
    pub lng: f64,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct CachedEdge {
    pub from: usize,
    pub to: usize,
    pub travel_time_s: f64,
    pub distance_m: f64,
}

impl RoadNetwork {
    pub async fn cache_stats() -> CacheStats {
        let guard = cache().read().await;

        let mut total_nodes = 0usize;
        let mut total_edges = 0usize;
        let mut memory_bytes = 0usize;

        for network in guard.values() {
            let nodes = network.node_count();
            let edges = network.edge_count();
            total_nodes += nodes;
            total_edges += edges;

            // Estimate memory: node data + edge data + hashmap overhead
            memory_bytes += nodes * (size_of::<f64>() * 2 + size_of::<usize>() * 2);
            memory_bytes += edges * (size_of::<f64>() * 2 + size_of::<usize>() * 2);
        }

        CacheStats {
            networks_cached: guard.len(),
            total_nodes,
            total_edges,
            memory_bytes,
            hits: CACHE_HITS.load(Ordering::Relaxed),
            misses: CACHE_MISSES.load(Ordering::Relaxed),
        }
    }

    pub async fn clear_cache() {
        let mut guard = cache().write().await;
        guard.clear();
    }

    pub async fn evict(bbox: &BoundingBox) -> bool {
        let cache_key = bbox.cache_key();
        let mut guard = cache().write().await;
        guard.remove(&cache_key).is_some()
    }

    pub async fn cached_regions() -> Vec<BoundingBox> {
        let guard = cache().read().await;
        guard
            .keys()
            .filter_map(|key| parse_cache_key(key))
            .collect()
    }
}

fn parse_cache_key(key: &str) -> Option<BoundingBox> {
    let parts: Vec<&str> = key.split('_').collect();
    if parts.len() != 4 {
        return None;
    }

    let min_lat: f64 = parts[0].parse().ok()?;
    let min_lng: f64 = parts[1].parse().ok()?;
    let max_lat: f64 = parts[2].parse().ok()?;
    let max_lng: f64 = parts[3].parse().ok()?;

    BoundingBox::try_new(min_lat, min_lng, max_lat, max_lng).ok()
}
