//! Overpass API fetching and caching for road networks.

use std::collections::HashMap;
use std::path::Path;
use std::sync::{Arc, OnceLock};
use tokio::sync::RwLock;
use tracing::{debug, error, info};

use super::bbox::BoundingBox;
use super::cache::{CachedEdge, CachedNetwork, CachedNode, CACHE_VERSION};
use super::error::RoutingError;
use super::geo::{get_speed_for_highway, haversine_distance};
use super::network::{EdgeData, RoadNetwork};
use super::osm::OverpassResponse;

/// In-memory cache of road networks, keyed by bbox cache key.
static NETWORK_CACHE: OnceLock<RwLock<HashMap<String, Arc<RoadNetwork>>>> = OnceLock::new();

fn network_cache() -> &'static RwLock<HashMap<String, Arc<RoadNetwork>>> {
    NETWORK_CACHE.get_or_init(|| RwLock::new(HashMap::new()))
}

/// Overpass API URL.
const OVERPASS_URL: &str = "https://overpass-api.de/api/interpreter";

/// Cache directory for downloaded OSM data.
const CACHE_DIR: &str = ".osm_cache";

impl RoadNetwork {
    /// Loads or fetches road network for a bounding box.
    ///
    /// Uses three-tier caching:
    /// 1. In-memory cache (instant, per-process)
    /// 2. File cache (fast, persists across restarts)
    /// 3. Overpass API download (slow, ~5-30s)
    pub async fn load_or_fetch(bbox: &BoundingBox) -> Result<Arc<Self>, RoutingError> {
        let cache_key = bbox.cache_key();

        // 1. Check in-memory cache (fast path, read lock)
        {
            let cache = network_cache().read().await;
            if let Some(network) = cache.get(&cache_key) {
                info!("Using in-memory cached road network for {}", cache_key);
                return Ok(Arc::clone(network));
            }
        }

        // 2. Acquire write lock and double-check
        let mut cache = network_cache().write().await;
        if let Some(network) = cache.get(&cache_key) {
            info!("Using in-memory cached road network for {}", cache_key);
            return Ok(Arc::clone(network));
        }

        // 3. Try loading from file cache
        tokio::fs::create_dir_all(CACHE_DIR).await?;
        let cache_path = Path::new(CACHE_DIR).join(format!("{}.json", cache_key));

        let network = if tokio::fs::try_exists(&cache_path).await.unwrap_or(false) {
            info!("Loading road network from file cache: {:?}", cache_path);
            match Self::load_from_cache(&cache_path).await {
                Ok(n) => n,
                Err(e) => {
                    info!("File cache invalid ({}), downloading fresh", e);
                    let n = Self::from_bbox(bbox).await?;
                    n.save_to_cache(&cache_path).await?;
                    info!("Saved road network to file cache: {:?}", cache_path);
                    n
                }
            }
        } else {
            // 4. Download from Overpass API
            info!("Downloading road network from Overpass API");
            let n = Self::from_bbox(bbox).await?;
            n.save_to_cache(&cache_path).await?;
            info!("Saved road network to file cache: {:?}", cache_path);
            n
        };

        let network = Arc::new(network);
        cache.insert(cache_key, Arc::clone(&network));

        Ok(network)
    }

    /// Downloads and builds road network from Overpass API.
    pub async fn from_bbox(bbox: &BoundingBox) -> Result<Self, RoutingError> {
        let query = format!(
            r#"[out:json][timeout:120];
(
  way["highway"~"^(motorway|trunk|primary|secondary|tertiary|residential|unclassified|service|living_street)$"]
    ({},{},{},{});
);
(._;>;);
out body;"#,
            bbox.min_lat, bbox.min_lng, bbox.max_lat, bbox.max_lng
        );

        debug!("Overpass query:\n{}", query);

        info!(
            "Preparing Overpass query for bbox: {:.4},{:.4} to {:.4},{:.4}",
            bbox.min_lat, bbox.min_lng, bbox.max_lat, bbox.max_lng
        );

        let client = reqwest::Client::builder()
            .connect_timeout(std::time::Duration::from_secs(30))
            .read_timeout(std::time::Duration::from_secs(180))
            .timeout(std::time::Duration::from_secs(180))
            .user_agent("SolverForge/0.4.0")
            .build()
            .map_err(|e| RoutingError::Network(e.to_string()))?;

        info!("Sending request to Overpass API...");

        let response = client
            .post(OVERPASS_URL)
            .body(query)
            .header("Content-Type", "text/plain")
            .send()
            .await
            .map_err(|e| {
                error!("Overpass request failed: {}", e);
                RoutingError::Network(e.to_string())
            })?;

        info!("Received response: status={}", response.status());

        if !response.status().is_success() {
            return Err(RoutingError::Network(format!(
                "Overpass API returned status {}",
                response.status()
            )));
        }

        let osm_data: OverpassResponse = response
            .json()
            .await
            .map_err(|e| RoutingError::Parse(e.to_string()))?;

        info!("Downloaded {} OSM elements", osm_data.elements.len());

        Self::build_from_osm(&osm_data)
    }

    /// Builds the road network from parsed OSM data.
    pub(super) fn build_from_osm(osm: &OverpassResponse) -> Result<Self, RoutingError> {
        let mut network = Self::new();

        // First pass: collect all nodes
        let mut nodes: HashMap<i64, (f64, f64)> = HashMap::new();
        for elem in &osm.elements {
            if elem.elem_type == "node" {
                if let (Some(lat), Some(lon)) = (elem.lat, elem.lon) {
                    nodes.insert(elem.id, (lat, lon));
                }
            }
        }

        info!("Parsed {} nodes", nodes.len());

        // Second pass: process ways and build graph
        let mut way_count = 0;
        for elem in &osm.elements {
            if elem.elem_type == "way" {
                if let Some(ref node_ids) = elem.nodes {
                    let highway = elem.tags.as_ref().and_then(|t| t.highway.as_deref());
                    let oneway = elem.tags.as_ref().and_then(|t| t.oneway.as_deref());
                    let speed = get_speed_for_highway(highway.unwrap_or("residential"));
                    let is_oneway = matches!(oneway, Some("yes") | Some("1"));

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

                        let distance = haversine_distance(lat1, lng1, lat2, lng2);
                        let travel_time = distance / speed;

                        let edge_data = EdgeData {
                            travel_time_s: travel_time,
                            distance_m: distance,
                            geometry: vec![(lat1, lng1), (lat2, lng2)],
                        };

                        network.add_edge(idx1, idx2, edge_data.clone());

                        if !is_oneway {
                            network.add_edge(idx2, idx1, edge_data);
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

        Ok(network)
    }

    /// Loads road network from cache file.
    async fn load_from_cache(path: &Path) -> Result<Self, RoutingError> {
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
            network.add_edge_by_index(
                edge.from,
                edge.to,
                edge.travel_time_s,
                edge.distance_m,
            );
        }

        Ok(network)
    }

    /// Saves road network to cache file.
    async fn save_to_cache(&self, path: &Path) -> Result<(), RoutingError> {
        let nodes: Vec<CachedNode> = self.nodes_iter()
            .map(|(lat, lng)| CachedNode { lat, lng })
            .collect();

        let edges: Vec<CachedEdge> = self.edges_iter()
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
        let data = serde_json::to_string(&cached).map_err(|e| RoutingError::Parse(e.to_string()))?;
        tokio::fs::write(path, data).await?;

        Ok(())
    }
}
