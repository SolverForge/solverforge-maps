//! Zero-erasure cache for road networks.

use std::collections::HashMap;
use std::ops::Deref;
use std::sync::OnceLock;

use serde::{Deserialize, Serialize};
use tokio::sync::{RwLock, RwLockReadGuard};

use super::network::RoadNetwork;

pub const CACHE_VERSION: u32 = 2;

static NETWORK_CACHE: OnceLock<RwLock<HashMap<String, RoadNetwork>>> = OnceLock::new();

pub(crate) fn cache() -> &'static RwLock<HashMap<String, RoadNetwork>> {
    NETWORK_CACHE.get_or_init(|| RwLock::new(HashMap::new()))
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
