//! Road network graph core types and routing.

use ordered_float::OrderedFloat;
use petgraph::algo::astar;
use petgraph::graph::{DiGraph, NodeIndex};
use std::collections::HashMap;

use super::geo::{coord_key, haversine_distance};

/// Node data in the road graph.
#[derive(Debug, Clone)]
pub struct NodeData {
    pub lat: f64,
    pub lng: f64,
}

/// Edge data in the road graph.
#[derive(Debug, Clone)]
pub struct EdgeData {
    /// Travel time in seconds.
    pub travel_time_s: f64,
    /// Distance in meters.
    pub distance_m: f64,
    /// Intermediate geometry points.
    #[allow(dead_code)]
    pub geometry: Vec<(f64, f64)>,
}

/// Result of a route computation.
#[derive(Debug, Clone)]
pub struct RouteResult {
    /// Travel time in seconds.
    pub duration_seconds: i64,
    /// Distance in meters.
    pub distance_meters: f64,
    /// Full route geometry (lat, lng pairs).
    pub geometry: Vec<(f64, f64)>,
}

/// Road network graph built from OSM data.
pub struct RoadNetwork {
    /// Directed graph with travel times as edge weights.
    pub(super) graph: DiGraph<NodeData, EdgeData>,
    /// Map from (lat_e7, lng_e7) to node index.
    pub(super) coord_to_node: HashMap<(i64, i64), NodeIndex>,
}

impl RoadNetwork {
    /// Creates an empty road network.
    pub fn new() -> Self {
        Self {
            graph: DiGraph::new(),
            coord_to_node: HashMap::new(),
        }
    }

    /// Gets or creates a node for the given coordinates.
    pub(super) fn get_or_create_node(&mut self, lat: f64, lng: f64) -> NodeIndex {
        let key = coord_key(lat, lng);
        if let Some(&idx) = self.coord_to_node.get(&key) {
            idx
        } else {
            let idx = self.graph.add_node(NodeData { lat, lng });
            self.coord_to_node.insert(key, idx);
            idx
        }
    }

    /// Adds a node at specific coordinates (for cache loading).
    pub(super) fn add_node_at(&mut self, lat: f64, lng: f64) -> NodeIndex {
        let idx = self.graph.add_node(NodeData { lat, lng });
        let key = coord_key(lat, lng);
        self.coord_to_node.insert(key, idx);
        idx
    }

    /// Adds an edge between two nodes.
    pub(super) fn add_edge(&mut self, from: NodeIndex, to: NodeIndex, data: EdgeData) {
        self.graph.add_edge(from, to, data);
    }

    /// Adds an edge by node indices (for cache loading).
    pub(super) fn add_edge_by_index(
        &mut self,
        from: usize,
        to: usize,
        travel_time_s: f64,
        distance_m: f64,
    ) {
        let from_idx = NodeIndex::new(from);
        let to_idx = NodeIndex::new(to);
        self.graph.add_edge(
            from_idx,
            to_idx,
            EdgeData {
                travel_time_s,
                distance_m,
                geometry: vec![],
            },
        );
    }

    /// Returns an iterator over all nodes as (lat, lng) pairs.
    pub(super) fn nodes_iter(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.graph
            .node_indices()
            .filter_map(|idx| self.graph.node_weight(idx).map(|n| (n.lat, n.lng)))
    }

    /// Returns an iterator over all edges as (from_idx, to_idx, travel_time_s, distance_m).
    pub(super) fn edges_iter(&self) -> impl Iterator<Item = (usize, usize, f64, f64)> + '_ {
        self.graph.edge_indices().filter_map(|idx| {
            let (from, to) = self.graph.edge_endpoints(idx)?;
            let weight = self.graph.edge_weight(idx)?;
            Some((from.index(), to.index(), weight.travel_time_s, weight.distance_m))
        })
    }

    /// Returns a reference to the underlying graph (for matrix computation).
    pub(super) fn graph(&self) -> &DiGraph<NodeData, EdgeData> {
        &self.graph
    }

    /// Finds the nearest road node to the given coordinates.
    pub fn snap_to_road(&self, lat: f64, lng: f64) -> Option<NodeIndex> {
        self.coord_to_node
            .iter()
            .min_by_key(|((lat_e7, lng_e7), _)| {
                let node_lat = *lat_e7 as f64 / 1e7;
                let node_lng = *lng_e7 as f64 / 1e7;
                OrderedFloat(haversine_distance(lat, lng, node_lat, node_lng))
            })
            .map(|(_, &idx)| idx)
    }

    /// Computes shortest path between two coordinates.
    pub fn route(&self, from: (f64, f64), to: (f64, f64)) -> Option<RouteResult> {
        let start = self.snap_to_road(from.0, from.1)?;
        let end = self.snap_to_road(to.0, to.1)?;

        if start == end {
            return Some(RouteResult {
                duration_seconds: 0,
                distance_meters: 0.0,
                geometry: vec![from, to],
            });
        }

        let (cost, path) = astar(
            &self.graph,
            start,
            |n| n == end,
            |e| OrderedFloat(e.weight().travel_time_s),
            |_| OrderedFloat(0.0),
        )?;

        let total_time = cost.0;

        let geometry: Vec<(f64, f64)> = path
            .iter()
            .filter_map(|&idx| self.graph.node_weight(idx).map(|n| (n.lat, n.lng)))
            .collect();

        let mut distance = 0.0;
        for window in path.windows(2) {
            if let Some(edge) = self.graph.find_edge(window[0], window[1]) {
                if let Some(weight) = self.graph.edge_weight(edge) {
                    distance += weight.distance_m;
                }
            }
        }

        Some(RouteResult {
            duration_seconds: total_time.round() as i64,
            distance_meters: distance,
            geometry,
        })
    }

    /// Returns the number of nodes in the graph.
    pub fn node_count(&self) -> usize {
        self.graph.node_count()
    }

    /// Returns the number of edges in the graph.
    pub fn edge_count(&self) -> usize {
        self.graph.edge_count()
    }
}

impl Default for RoadNetwork {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_network() {
        let network = RoadNetwork::new();
        assert_eq!(network.node_count(), 0);
        assert_eq!(network.edge_count(), 0);
    }

    #[test]
    fn test_snap_to_road_empty() {
        let network = RoadNetwork::new();
        assert!(network.snap_to_road(39.95, -75.16).is_none());
    }
}
