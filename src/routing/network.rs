//! Road network graph core types and routing.

use ordered_float::OrderedFloat;
use petgraph::algo::astar;
use petgraph::graph::{DiGraph, NodeIndex};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use super::coord::Coord;
use super::geo::{coord_key, haversine_distance};

#[derive(Debug, Clone)]
pub struct NodeData {
    pub lat: f64,
    pub lng: f64,
}

impl NodeData {
    #[inline]
    pub fn coord(&self) -> Coord {
        Coord::new(self.lat, self.lng)
    }
}

#[derive(Debug, Clone)]
pub struct EdgeData {
    pub travel_time_s: f64,
    pub distance_m: f64,
    #[allow(dead_code)]
    pub geometry: Vec<Coord>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteResult {
    pub duration_seconds: i64,
    pub distance_meters: f64,
    pub geometry: Vec<Coord>,
}

pub struct RoadNetwork {
    pub(super) graph: DiGraph<NodeData, EdgeData>,
    pub(super) coord_to_node: HashMap<(i64, i64), NodeIndex>,
}

impl RoadNetwork {
    pub fn new() -> Self {
        Self {
            graph: DiGraph::new(),
            coord_to_node: HashMap::new(),
        }
    }

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

    pub(super) fn add_node_at(&mut self, lat: f64, lng: f64) -> NodeIndex {
        let idx = self.graph.add_node(NodeData { lat, lng });
        let key = coord_key(lat, lng);
        self.coord_to_node.insert(key, idx);
        idx
    }

    pub(super) fn add_edge(&mut self, from: NodeIndex, to: NodeIndex, data: EdgeData) {
        self.graph.add_edge(from, to, data);
    }

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

    pub(super) fn nodes_iter(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.graph
            .node_indices()
            .filter_map(|idx| self.graph.node_weight(idx).map(|n| (n.lat, n.lng)))
    }

    pub(super) fn edges_iter(&self) -> impl Iterator<Item = (usize, usize, f64, f64)> + '_ {
        self.graph.edge_indices().filter_map(|idx| {
            let (from, to) = self.graph.edge_endpoints(idx)?;
            let weight = self.graph.edge_weight(idx)?;
            Some((
                from.index(),
                to.index(),
                weight.travel_time_s,
                weight.distance_m,
            ))
        })
    }

    pub(super) fn graph(&self) -> &DiGraph<NodeData, EdgeData> {
        &self.graph
    }

    pub fn snap_to_road(&self, coord: Coord) -> Option<NodeIndex> {
        self.coord_to_node
            .iter()
            .min_by_key(|((lat_e7, lng_e7), _)| {
                let node_coord = Coord::new(*lat_e7 as f64 / 1e7, *lng_e7 as f64 / 1e7);
                OrderedFloat(haversine_distance(coord, node_coord))
            })
            .map(|(_, &idx)| idx)
    }

    pub fn route(&self, from: Coord, to: Coord) -> Option<RouteResult> {
        let start = self.snap_to_road(from)?;
        let end = self.snap_to_road(to)?;

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

        let geometry: Vec<Coord> = path
            .iter()
            .filter_map(|&idx| self.graph.node_weight(idx).map(|n| n.coord()))
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
            duration_seconds: cost.0.round() as i64,
            distance_meters: distance,
            geometry,
        })
    }

    pub fn node_count(&self) -> usize {
        self.graph.node_count()
    }

    pub fn edge_count(&self) -> usize {
        self.graph.edge_count()
    }
}

impl Default for RoadNetwork {
    fn default() -> Self {
        Self::new()
    }
}
