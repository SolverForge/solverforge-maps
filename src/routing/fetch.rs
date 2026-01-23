//! Overpass API fetching and caching for road networks.

use std::collections::HashMap;
use std::path::Path;

use tokio::sync::mpsc::Sender;
use tracing::{debug, info};

use super::bbox::BoundingBox;
use super::cache::{
    cache, record_hit, record_miss, CachedEdge, CachedNetwork, CachedNode, NetworkRef,
    CACHE_VERSION,
};
use super::config::NetworkConfig;
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

        if let Some(tx) = progress {
            let _ = tx.send(RoutingProgress::CheckingCache { percent: 0 }).await;
        }

        {
            let cache_guard = cache().read().await;
            if cache_guard.contains_key(&cache_key) {
                record_hit();
                info!("Using in-memory cached road network for {}", cache_key);
                if let Some(tx) = progress {
                    let _ = tx
                        .send(RoutingProgress::CheckingCache { percent: 10 })
                        .await;
                }
                return Ok(NetworkRef::new(cache_guard, cache_key));
            }
        }
        record_miss();

        if let Some(tx) = progress {
            let _ = tx.send(RoutingProgress::CheckingCache { percent: 5 }).await;
        }

        {
            let mut cache_guard = cache().write().await;
            if !cache_guard.contains_key(&cache_key) {
                tokio::fs::create_dir_all(&config.cache_dir).await?;
                let cache_path = config.cache_dir.join(format!("{}.json", cache_key));

                let network = if tokio::fs::try_exists(&cache_path).await.unwrap_or(false) {
                    info!("Loading road network from file cache: {:?}", cache_path);
                    if let Some(tx) = progress {
                        let _ = tx.send(RoutingProgress::CheckingCache { percent: 8 }).await;
                    }
                    match Self::load_from_file(&cache_path).await {
                        Ok(n) => {
                            if let Some(tx) = progress {
                                let _ = tx
                                    .send(RoutingProgress::BuildingGraph { percent: 50 })
                                    .await;
                            }
                            n
                        }
                        Err(e) => {
                            info!("File cache invalid ({}), downloading fresh", e);
                            let n = Self::fetch_from_api(bbox, config, progress).await?;
                            n.save_to_file(&cache_path).await?;
                            info!("Saved road network to file cache: {:?}", cache_path);
                            n
                        }
                    }
                } else {
                    info!("Downloading road network from Overpass API");
                    let n = Self::fetch_from_api(bbox, config, progress).await?;
                    n.save_to_file(&cache_path).await?;
                    info!("Saved road network to file cache: {:?}", cache_path);
                    n
                };

                cache_guard.insert(cache_key.clone(), network);
            }
        }

        let cache_guard = cache().read().await;
        Ok(NetworkRef::new(cache_guard, cache_key))
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

        info!("Sending request to Overpass API...");

        if let Some(tx) = progress {
            let _ = tx
                .send(RoutingProgress::DownloadingNetwork {
                    percent: 15,
                    bytes: 0,
                })
                .await;
        }

        let response = client
            .post(&config.overpass_url)
            .body(query)
            .header("Content-Type", "text/plain")
            .send()
            .await
            .map_err(|e| RoutingError::Network(e.to_string()))?;

        info!("Received response: status={}", response.status());

        if !response.status().is_success() {
            return Err(RoutingError::Network(format!(
                "Overpass API returned status {}",
                response.status()
            )));
        }

        if let Some(tx) = progress {
            let _ = tx
                .send(RoutingProgress::DownloadingNetwork {
                    percent: 25,
                    bytes: 0,
                })
                .await;
        }

        let bytes = response
            .bytes()
            .await
            .map_err(|e| RoutingError::Network(e.to_string()))?;

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
                    let speed = config
                        .speed_profile
                        .speed_mps(highway.unwrap_or("residential"));
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

        // Filter to largest strongly connected component to ensure all nodes are reachable
        let scc_count = network.strongly_connected_components();
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

        network.build_spatial_index();

        Ok(network)
    }

    async fn load_from_file(path: &Path) -> Result<Self, RoutingError> {
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

        // Filter to largest SCC (cached networks from older versions may not be filtered)
        let scc_count = network.strongly_connected_components();
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
}

impl RoadNetwork {
    #[doc(hidden)]
    pub async fn load_or_fetch_simple(bbox: &BoundingBox) -> Result<NetworkRef, RoutingError> {
        Self::load_or_fetch(bbox, &NetworkConfig::default(), None).await
    }
}
