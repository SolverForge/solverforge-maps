//! Road network graph core types and routing.

use ordered_float::OrderedFloat;
use petgraph::algo::astar;
use petgraph::graph::{DiGraph, NodeIndex};
use rstar::{PointDistance, RTree, RTreeObject, AABB};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use super::coord::Coord;
use super::error::RoutingError;
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
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteResult {
    pub duration_seconds: i64,
    pub distance_meters: f64,
    pub geometry: Vec<Coord>,
}

impl RouteResult {
    /// Simplify the route geometry using Douglas-Peucker algorithm.
    ///
    /// # Arguments
    /// * `tolerance_m` - Tolerance in meters; points within this distance of
    ///   the simplified line are removed.
    pub fn simplify(mut self, tolerance_m: f64) -> Self {
        if self.geometry.len() <= 2 {
            return self;
        }

        self.geometry = douglas_peucker(&self.geometry, tolerance_m);
        self
    }
}

/// A coordinate that has been snapped to the road network.
#[derive(Debug, Clone, Copy, Default)]
pub struct SnappedCoord {
    /// The original coordinate before snapping.
    pub original: Coord,
    /// The snapped coordinate on the road network.
    pub snapped: Coord,
    /// Distance between original and snapped coordinates in meters.
    pub snap_distance_m: f64,
    /// Internal node index (not exposed to users).
    pub(crate) node_index: NodeIndex,
}

/// A point in the R-tree spatial index.
#[derive(Debug, Clone, Copy, Default)]
struct RTreePoint {
    lat: f64,
    lng: f64,
    node_index: NodeIndex,
}

impl RTreeObject for RTreePoint {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point([self.lat, self.lng])
    }
}

impl PointDistance for RTreePoint {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dlat = self.lat - point[0];
        let dlng = self.lng - point[1];
        dlat * dlat + dlng * dlng
    }
}

pub struct RoadNetwork {
    pub(super) graph: DiGraph<NodeData, EdgeData>,
    pub(super) coord_to_node: HashMap<(i64, i64), NodeIndex>,
    spatial_index: Option<RTree<RTreePoint>>,
}

impl RoadNetwork {
    pub fn new() -> Self {
        Self {
            graph: DiGraph::new(),
            coord_to_node: HashMap::new(),
            spatial_index: None,
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
            },
        );
    }

    /// Build the spatial index for fast nearest-neighbor queries.
    /// This is called automatically during graph construction.
    pub(super) fn build_spatial_index(&mut self) {
        let points: Vec<RTreePoint> = self
            .graph
            .node_indices()
            .filter_map(|idx| {
                self.graph.node_weight(idx).map(|n| RTreePoint {
                    lat: n.lat,
                    lng: n.lng,
                    node_index: idx,
                })
            })
            .collect();

        self.spatial_index = Some(RTree::bulk_load(points));
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

    /// Snap a coordinate to the nearest node in the road network.
    ///
    /// Returns `None` if the network is empty.
    pub fn snap_to_road(&self, coord: Coord) -> Option<NodeIndex> {
        self.spatial_index
            .as_ref()?
            .nearest_neighbor(&[coord.lat, coord.lng])
            .map(|p| p.node_index)
    }

    /// Snap a coordinate to the road network with detailed information.
    ///
    /// Returns a `SnappedCoord` containing both original and snapped coordinates,
    /// the snap distance, and an internal node index for routing.
    pub fn snap_to_road_detailed(&self, coord: Coord) -> Result<SnappedCoord, RoutingError> {
        let rtree = self
            .spatial_index
            .as_ref()
            .ok_or(RoutingError::SnapFailed {
                coord,
                nearest_distance_m: None,
            })?;

        let nearest =
            rtree
                .nearest_neighbor(&[coord.lat, coord.lng])
                .ok_or(RoutingError::SnapFailed {
                    coord,
                    nearest_distance_m: None,
                })?;

        let snapped = Coord::new(nearest.lat, nearest.lng);
        let snap_distance_m = haversine_distance(coord, snapped);

        Ok(SnappedCoord {
            original: coord,
            snapped,
            snap_distance_m,
            node_index: nearest.node_index,
        })
    }

    /// Find a route between two coordinates.
    ///
    /// This method snaps the coordinates to the nearest road network nodes
    /// and then finds the shortest path by travel time.
    pub fn route(&self, from: Coord, to: Coord) -> Result<RouteResult, RoutingError> {
        let start_snap = self.snap_to_road_detailed(from)?;
        let end_snap = self.snap_to_road_detailed(to)?;

        self.route_snapped(&start_snap, &end_snap)
    }

    /// Find a route between two already-snapped coordinates.
    pub fn route_snapped(
        &self,
        from: &SnappedCoord,
        to: &SnappedCoord,
    ) -> Result<RouteResult, RoutingError> {
        if from.node_index == to.node_index {
            return Ok(RouteResult {
                duration_seconds: 0,
                distance_meters: 0.0,
                geometry: vec![from.original, to.original],
            });
        }

        let result = astar(
            &self.graph,
            from.node_index,
            |n| n == to.node_index,
            |e| OrderedFloat(e.weight().travel_time_s),
            |_| OrderedFloat(0.0),
        );

        match result {
            Some((cost, path)) => {
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

                Ok(RouteResult {
                    duration_seconds: cost.0.round() as i64,
                    distance_meters: distance,
                    geometry,
                })
            }
            None => Err(RoutingError::NoPath {
                from: from.original,
                to: to.original,
            }),
        }
    }

    /// Find a route optimized for a specific objective.
    pub fn route_with(
        &self,
        from: Coord,
        to: Coord,
        objective: Objective,
    ) -> Result<RouteResult, RoutingError> {
        let start_snap = self.snap_to_road_detailed(from)?;
        let end_snap = self.snap_to_road_detailed(to)?;

        if start_snap.node_index == end_snap.node_index {
            return Ok(RouteResult {
                duration_seconds: 0,
                distance_meters: 0.0,
                geometry: vec![from, to],
            });
        }

        let result = match objective {
            Objective::Time => astar(
                &self.graph,
                start_snap.node_index,
                |n| n == end_snap.node_index,
                |e| OrderedFloat(e.weight().travel_time_s),
                |_| OrderedFloat(0.0),
            ),
            Objective::Distance => astar(
                &self.graph,
                start_snap.node_index,
                |n| n == end_snap.node_index,
                |e| OrderedFloat(e.weight().distance_m),
                |_| OrderedFloat(0.0),
            ),
        };

        match result {
            Some((_, path)) => {
                let geometry: Vec<Coord> = path
                    .iter()
                    .filter_map(|&idx| self.graph.node_weight(idx).map(|n| n.coord()))
                    .collect();

                let mut distance = 0.0;
                let mut time = 0.0;
                for window in path.windows(2) {
                    if let Some(edge) = self.graph.find_edge(window[0], window[1]) {
                        if let Some(weight) = self.graph.edge_weight(edge) {
                            distance += weight.distance_m;
                            time += weight.travel_time_s;
                        }
                    }
                }

                Ok(RouteResult {
                    duration_seconds: time.round() as i64,
                    distance_meters: distance,
                    geometry,
                })
            }
            None => Err(RoutingError::NoPath { from, to }),
        }
    }

    pub fn node_count(&self) -> usize {
        self.graph.node_count()
    }

    pub fn edge_count(&self) -> usize {
        self.graph.edge_count()
    }

    /// Returns the number of strongly connected components in the graph.
    pub fn strongly_connected_components(&self) -> usize {
        petgraph::algo::kosaraju_scc(&self.graph).len()
    }

    /// Returns the fraction of nodes in the largest strongly connected component.
    pub fn largest_component_fraction(&self) -> f64 {
        let sccs = petgraph::algo::kosaraju_scc(&self.graph);
        if sccs.is_empty() {
            return 0.0;
        }
        let largest = sccs.iter().map(|c| c.len()).max().unwrap_or(0);
        let total = self.graph.node_count();
        if total == 0 {
            0.0
        } else {
            largest as f64 / total as f64
        }
    }

    /// Returns true if the graph is strongly connected (single SCC).
    pub fn is_strongly_connected(&self) -> bool {
        self.strongly_connected_components() == 1
    }
}

impl Default for RoadNetwork {
    fn default() -> Self {
        Self::new()
    }
}

/// Optimization objective for routing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Objective {
    /// Minimize travel time (default).
    Time,
    /// Minimize distance.
    Distance,
}

/// Douglas-Peucker line simplification algorithm.
fn douglas_peucker(points: &[Coord], tolerance_m: f64) -> Vec<Coord> {
    if points.len() <= 2 {
        return points.to_vec();
    }

    let first = points[0];
    let last = points[points.len() - 1];

    // Find the point with maximum distance from the line
    let mut max_dist = 0.0;
    let mut max_idx = 0;

    for (i, point) in points.iter().enumerate().skip(1).take(points.len() - 2) {
        let dist = perpendicular_distance(*point, first, last);
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }

    if max_dist > tolerance_m {
        // Recursively simplify
        let mut left = douglas_peucker(&points[..=max_idx], tolerance_m);
        let right = douglas_peucker(&points[max_idx..], tolerance_m);

        // Remove duplicate point at junction
        left.pop();
        left.extend(right);
        left
    } else {
        // All points are within tolerance; keep only endpoints
        vec![first, last]
    }
}

/// Calculate perpendicular distance from a point to a line defined by two points.
fn perpendicular_distance(point: Coord, line_start: Coord, line_end: Coord) -> f64 {
    let dx = line_end.lng - line_start.lng;
    let dy = line_end.lat - line_start.lat;

    // If start and end are the same point, return distance to that point
    let line_length_sq = dx * dx + dy * dy;
    if line_length_sq < f64::EPSILON {
        return haversine_distance(point, line_start);
    }

    // Project point onto line
    let t =
        ((point.lng - line_start.lng) * dx + (point.lat - line_start.lat) * dy) / line_length_sq;
    let t = t.clamp(0.0, 1.0);

    let projected = Coord::new(line_start.lat + t * dy, line_start.lng + t * dx);

    haversine_distance(point, projected)
}
