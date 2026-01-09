//! Cache data structures for road network persistence.

use serde::{Deserialize, Serialize};

/// Cache format version. Bump this when changing the cache structure.
pub const CACHE_VERSION: u32 = 1;

/// Cached road network for file persistence.
#[derive(Debug, Serialize, Deserialize)]
pub struct CachedNetwork {
    /// Cache format version for automatic invalidation.
    pub version: u32,
    pub nodes: Vec<CachedNode>,
    pub edges: Vec<CachedEdge>,
}

/// A cached node.
#[derive(Debug, Serialize, Deserialize)]
pub struct CachedNode {
    pub lat: f64,
    pub lng: f64,
}

/// A cached edge.
#[derive(Debug, Serialize, Deserialize)]
pub struct CachedEdge {
    pub from: usize,
    pub to: usize,
    pub travel_time_s: f64,
    pub distance_m: f64,
}
