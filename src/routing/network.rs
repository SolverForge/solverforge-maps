//! Road network graph core types and routing.

use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

use super::algo::{astar, kosaraju_scc};
use super::coord::Coord;
use super::error::RoutingError;
use super::geo::{coord_key, haversine_distance};
use super::graph::{DiGraph, EdgeIdx, NodeIdx};
use super::spatial::{KdTree, Point2D, Segment, SegmentIndex};

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

#[derive(Debug, Clone, Copy, Default)]
pub struct SnappedCoord {
    pub original: Coord,
    pub snapped: Coord,
    pub snap_distance_m: f64,
    pub(crate) node_index: NodeIdx,
}

/// Data stored in the point spatial index.
#[derive(Debug, Clone, Copy)]
struct SpatialPoint {
    node_index: NodeIdx,
}

/// Data stored in the segment spatial index.
#[derive(Debug, Clone, Copy)]
struct SpatialSegment {
    from_node: NodeIdx,
    to_node: NodeIdx,
    edge_index: EdgeIdx,
}

#[derive(Debug, Clone, Copy)]
pub struct EdgeSnappedLocation {
    pub original: Coord,
    pub snapped: Coord,
    pub snap_distance_m: f64,
    pub(crate) edge_index: EdgeIdx,
    pub(crate) position: f64,
    pub(crate) from_node: NodeIdx,
    pub(crate) to_node: NodeIdx,
}

pub struct RoadNetwork {
    pub(super) graph: DiGraph<NodeData, EdgeData>,
    pub(super) coord_to_node: HashMap<(i64, i64), NodeIdx>,
    spatial_index: Option<KdTree<SpatialPoint>>,
    edge_spatial_index: Option<SegmentIndex<SpatialSegment>>,
}

impl RoadNetwork {
    pub fn new() -> Self {
        Self {
            graph: DiGraph::new(),
            coord_to_node: HashMap::new(),
            spatial_index: None,
            edge_spatial_index: None,
        }
    }

    pub(super) fn get_or_create_node(&mut self, lat: f64, lng: f64) -> NodeIdx {
        let key = coord_key(lat, lng);
        if let Some(&idx) = self.coord_to_node.get(&key) {
            idx
        } else {
            let idx = self.graph.add_node(NodeData { lat, lng });
            self.coord_to_node.insert(key, idx);
            idx
        }
    }

    pub(super) fn add_node_at(&mut self, lat: f64, lng: f64) -> NodeIdx {
        let idx = self.graph.add_node(NodeData { lat, lng });
        let key = coord_key(lat, lng);
        self.coord_to_node.insert(key, idx);
        idx
    }

    pub(super) fn add_edge(&mut self, from: NodeIdx, to: NodeIdx, data: EdgeData) {
        self.graph.add_edge(from, to, data);
    }

    pub(super) fn add_edge_by_index(
        &mut self,
        from: usize,
        to: usize,
        travel_time_s: f64,
        distance_m: f64,
    ) {
        let from_idx = NodeIdx::new(from);
        let to_idx = NodeIdx::new(to);
        self.graph.add_edge(
            from_idx,
            to_idx,
            EdgeData {
                travel_time_s,
                distance_m,
            },
        );
    }

    pub(super) fn build_spatial_index(&mut self) {
        // Build point index for nodes
        let points: Vec<(Point2D, SpatialPoint)> = self
            .graph
            .node_indices()
            .filter_map(|node_idx| {
                let node = self.graph.node_weight(node_idx)?;
                Some((
                    Point2D::new(node.lat, node.lng),
                    SpatialPoint {
                        node_index: node_idx,
                    },
                ))
            })
            .collect();

        self.spatial_index = Some(KdTree::from_items(points));

        // Build segment index for edges
        let segments: Vec<(Segment, SpatialSegment)> = self
            .graph
            .edge_indices()
            .filter_map(|edge_idx| {
                let (from_node, to_node) = self.graph.edge_endpoints(edge_idx)?;
                let from_data = self.graph.node_weight(from_node)?;
                let to_data = self.graph.node_weight(to_node)?;
                Some((
                    Segment::new(
                        Point2D::new(from_data.lat, from_data.lng),
                        Point2D::new(to_data.lat, to_data.lng),
                    ),
                    SpatialSegment {
                        from_node,
                        to_node,
                        edge_index: edge_idx,
                    },
                ))
            })
            .collect();

        self.edge_spatial_index = Some(SegmentIndex::bulk_load(segments));
    }

    /// Iterate over all nodes as (lat, lng) pairs.
    pub fn nodes_iter(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.graph
            .node_indices()
            .filter_map(|idx| self.graph.node_weight(idx).map(|n| (n.lat, n.lng)))
    }

    /// Iterate over all edges as (from_idx, to_idx, travel_time_s, distance_m) tuples.
    pub fn edges_iter(&self) -> impl Iterator<Item = (usize, usize, f64, f64)> + '_ {
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

    /// Snap a coordinate to the nearest node in the road network.
    ///
    /// Returns `None` if the network is empty.
    pub fn snap_to_road(&self, coord: Coord) -> Option<NodeIdx> {
        self.spatial_index
            .as_ref()?
            .nearest_neighbor(&Point2D::new(coord.lat, coord.lng))
            .map(|p| p.node_index)
    }

    /// Snap a coordinate to the road network with detailed information.
    ///
    /// Returns a `SnappedCoord` containing both original and snapped coordinates,
    /// the snap distance, and an internal node index for routing.
    pub fn snap_to_road_detailed(&self, coord: Coord) -> Result<SnappedCoord, RoutingError> {
        let tree = self
            .spatial_index
            .as_ref()
            .ok_or(RoutingError::SnapFailed {
                coord,
                nearest_distance_m: None,
            })?;

        let query = Point2D::new(coord.lat, coord.lng);
        let (nearest, _dist_sq) =
            tree.nearest_neighbor_with_distance(&query)
                .ok_or(RoutingError::SnapFailed {
                    coord,
                    nearest_distance_m: None,
                })?;

        // Get the actual coordinates from the graph node
        let node_data =
            self.graph
                .node_weight(nearest.node_index)
                .ok_or(RoutingError::SnapFailed {
                    coord,
                    nearest_distance_m: None,
                })?;

        let snapped = Coord::new(node_data.lat, node_data.lng);
        let snap_distance_m = haversine_distance(coord, snapped);

        Ok(SnappedCoord {
            original: coord,
            snapped,
            snap_distance_m,
            node_index: nearest.node_index,
        })
    }

    /// Snap a coordinate to the nearest road segment (edge-based snapping).
    ///
    /// This is production-grade snapping that projects the coordinate onto the
    /// nearest road segment rather than snapping to the nearest intersection.
    pub fn snap_to_edge(&self, coord: Coord) -> Result<EdgeSnappedLocation, RoutingError> {
        let index = self
            .edge_spatial_index
            .as_ref()
            .ok_or(RoutingError::SnapFailed {
                coord,
                nearest_distance_m: None,
            })?;

        let query = Point2D::new(coord.lat, coord.lng);
        let (segment, seg_data, proj_point, _dist_sq) =
            index
                .nearest_segment(&query)
                .ok_or(RoutingError::SnapFailed {
                    coord,
                    nearest_distance_m: None,
                })?;

        // Compute position along segment
        let (_, position) = segment.project_point(&query);
        let snapped = Coord::new(proj_point.x, proj_point.y);
        let snap_distance_m = haversine_distance(coord, snapped);

        Ok(EdgeSnappedLocation {
            original: coord,
            snapped,
            snap_distance_m,
            edge_index: seg_data.edge_index,
            position,
            from_node: seg_data.from_node,
            to_node: seg_data.to_node,
        })
    }

    /// Find a route between two coordinates.
    ///
    /// This method snaps both coordinates to the nearest road-network nodes,
    /// then runs the public travel-time search over those snapped nodes.
    /// The current implementation passes a zero heuristic to `astar`, so its
    /// behavior is equivalent to Dijkstra's algorithm.
    pub fn route(&self, from: Coord, to: Coord) -> Result<RouteResult, RoutingError> {
        let start_snap = self.snap_to_road_detailed(from)?;
        let end_snap = self.snap_to_road_detailed(to)?;

        self.route_snapped(&start_snap, &end_snap)
    }

    /// Find a route between two edge-snapped locations.
    ///
    /// Use this when start and end should stay on their containing road
    /// segments instead of being snapped all the way to graph nodes.
    pub fn route_edge_snapped(
        &self,
        from: &EdgeSnappedLocation,
        to: &EdgeSnappedLocation,
    ) -> Result<RouteResult, RoutingError> {
        let from_edge = self
            .graph
            .edge_weight(from.edge_index)
            .ok_or(RoutingError::NoPath {
                from: from.original,
                to: to.original,
            })?;
        let to_edge = self
            .graph
            .edge_weight(to.edge_index)
            .ok_or(RoutingError::NoPath {
                from: from.original,
                to: to.original,
            })?;

        // If both snapped to the same edge, check if direct travel is possible
        if from.edge_index == to.edge_index {
            let segment_time = from_edge.travel_time_s;
            let segment_dist = from_edge.distance_m;
            let travel_fraction = (to.position - from.position).abs();
            return Ok(RouteResult {
                duration_seconds: (segment_time * travel_fraction).round() as i64,
                distance_meters: segment_dist * travel_fraction,
                geometry: vec![from.snapped, to.snapped],
            });
        }

        // Cost from snap point to from_node (going backward on edge)
        let cost_to_from_node = from_edge.travel_time_s * from.position;
        // Cost from snap point to to_node (going forward on edge)
        let cost_to_to_node = from_edge.travel_time_s * (1.0 - from.position);

        // Cost from to_edge's from_node to snap point
        let cost_from_dest_from = to_edge.travel_time_s * to.position;
        // Cost from to_edge's to_node to snap point
        let cost_from_dest_to = to_edge.travel_time_s * (1.0 - to.position);

        // Try routing from from_node to both destination edge endpoints
        let mut best_result: Option<(f64, Vec<NodeIdx>, NodeIdx, f64)> = None;

        for &(start_node, start_cost) in &[
            (from.from_node, cost_to_from_node),
            (from.to_node, cost_to_to_node),
        ] {
            for &(end_node, end_cost) in &[
                (to.from_node, cost_from_dest_from),
                (to.to_node, cost_from_dest_to),
            ] {
                if start_node == end_node {
                    let total_cost = start_cost + end_cost;
                    if best_result.is_none() || total_cost < best_result.as_ref().unwrap().0 {
                        best_result = Some((total_cost, vec![start_node], end_node, end_cost));
                    }
                    continue;
                }

                let result = astar(
                    &self.graph,
                    start_node,
                    |n| n == end_node,
                    |e| e.travel_time_s,
                    |_| 0.0,
                );

                if let Some((path_cost, path)) = result {
                    let total_cost = start_cost + path_cost + end_cost;
                    if best_result.is_none() || total_cost < best_result.as_ref().unwrap().0 {
                        best_result = Some((total_cost, path, end_node, end_cost));
                    }
                }
            }
        }

        match best_result {
            Some((total_cost, path, _, _)) => {
                let mut geometry = vec![from.snapped];
                for &idx in &path {
                    if let Some(node) = self.graph.node_weight(idx) {
                        geometry.push(node.coord());
                    }
                }
                geometry.push(to.snapped);

                let mut distance = 0.0;
                distance += from_edge.distance_m * from.position.min(1.0 - from.position);
                for window in path.windows(2) {
                    if let Some(edge) = self.graph.find_edge(window[0], window[1]) {
                        if let Some(weight) = self.graph.edge_weight(edge) {
                            distance += weight.distance_m;
                        }
                    }
                }

                distance += to_edge.distance_m * to.position.min(1.0 - to.position);

                Ok(RouteResult {
                    duration_seconds: total_cost.round() as i64,
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

    /// Find a route between two node-snapped coordinates.
    ///
    /// This expects `SnappedCoord` values produced by `snap_to_road_detailed`
    /// and returns geometry along graph nodes, not projected edge endpoints.
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
            |e| e.travel_time_s,
            |_| 0.0,
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
                    duration_seconds: cost.round() as i64,
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

    /// Find a route between two coordinates with an explicit optimization objective.
    ///
    /// Like `route`, this method snaps to the nearest graph nodes first. The
    /// current public search still uses a zero heuristic for both objectives.
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
                |e| e.travel_time_s,
                |_| 0.0,
            ),
            Objective::Distance => astar(
                &self.graph,
                start_snap.node_index,
                |n| n == end_snap.node_index,
                |e| e.distance_m,
                |_| 0.0,
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

    pub fn strongly_connected_components(&self) -> usize {
        kosaraju_scc(&self.graph).len()
    }

    pub fn largest_component_fraction(&self) -> f64 {
        let sccs = kosaraju_scc(&self.graph);
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

    pub fn is_strongly_connected(&self) -> bool {
        self.strongly_connected_components() == 1
    }

    /// Filter the network to keep only the largest strongly connected component.
    pub fn filter_to_largest_scc(&mut self) {
        let sccs = kosaraju_scc(&self.graph);
        if sccs.len() <= 1 {
            return; // Already connected or empty
        }

        // Find the largest SCC
        let largest_scc: HashSet<NodeIdx> = sccs
            .into_iter()
            .max_by_key(|scc| scc.len())
            .unwrap_or_default()
            .into_iter()
            .collect();

        // Retain only nodes in the largest SCC
        self.graph.retain_nodes(|n, _| largest_scc.contains(&n));

        self.rebuild_coord_to_node();
        self.build_spatial_index();
    }

    fn rebuild_coord_to_node(&mut self) {
        self.coord_to_node.clear();
        for idx in self.graph.node_indices() {
            if let Some(node) = self.graph.node_weight(idx) {
                let key = coord_key(node.lat, node.lng);
                self.coord_to_node.insert(key, idx);
            }
        }
    }
}

impl Default for RoadNetwork {
    fn default() -> Self {
        Self::new()
    }
}

impl RoadNetwork {
    /// Build a network from raw node/edge data (for testing).
    ///
    /// # Arguments
    /// * `nodes` - Slice of (lat, lng) tuples for each node
    /// * `edges` - Slice of (from_idx, to_idx, time_s, dist_m) tuples
    ///
    /// # Example
    /// ```
    /// use solverforge_maps::RoadNetwork;
    ///
    /// let nodes = &[(40.0, -75.0), (39.8, -75.2)];
    /// let edges = &[(0, 1, 60.0, 1000.0), (1, 0, 60.0, 1000.0)];
    /// let network = RoadNetwork::from_test_data(nodes, edges);
    ///
    /// assert_eq!(network.node_count(), 2);
    /// assert_eq!(network.edge_count(), 2);
    /// ```
    pub fn from_test_data(nodes: &[(f64, f64)], edges: &[(usize, usize, f64, f64)]) -> Self {
        let mut network = Self::new();

        // Add all nodes
        for &(lat, lng) in nodes {
            network.add_node_at(lat, lng);
        }

        // Add all edges
        for &(from, to, time_s, dist_m) in edges {
            network.add_edge_by_index(from, to, time_s, dist_m);
        }

        // Build spatial index for snapping
        network.build_spatial_index();

        network
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Objective {
    Time,
    Distance,
}

/// Douglas-Peucker line simplification algorithm.
fn douglas_peucker(points: &[Coord], tolerance_m: f64) -> Vec<Coord> {
    if points.len() <= 2 {
        return points.to_vec();
    }

    let first = points[0];
    let last = points[points.len() - 1];
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
        let mut left = douglas_peucker(&points[..=max_idx], tolerance_m);
        let right = douglas_peucker(&points[max_idx..], tolerance_m);

        left.pop();
        left.extend(right);
        left
    } else {
        vec![first, last]
    }
}

fn perpendicular_distance(point: Coord, line_start: Coord, line_end: Coord) -> f64 {
    let dx = line_end.lng - line_start.lng;
    let dy = line_end.lat - line_start.lat;

    let line_length_sq = dx * dx + dy * dy;
    if line_length_sq < f64::EPSILON {
        return haversine_distance(point, line_start);
    }

    let t =
        ((point.lng - line_start.lng) * dx + (point.lat - line_start.lat) * dy) / line_length_sq;
    let t = t.clamp(0.0, 1.0);

    let projected = Coord::new(line_start.lat + t * dy, line_start.lng + t * dx);

    haversine_distance(point, projected)
}
