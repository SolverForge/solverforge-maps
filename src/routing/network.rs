//! Road network graph core types and routing.

use serde::{Deserialize, Serialize};
use std::collections::HashSet;

use super::algo::{astar, kosaraju_scc};
use super::coord::Coord;
use super::error::RoutingError;
use super::geo::haversine_distance;
use super::graph::{DiGraph, EdgeIdx, NodeIdx};
use super::spatial::{KdTree, Point2D, Segment, SegmentIndex};

/// Path-compressed union-find root lookup for `build_spatial_index`.
fn component_root(parent: &mut [usize], mut root: usize) -> usize {
    let mut probe = root;
    while parent[probe] != probe {
        probe = parent[probe];
    }
    while parent[root] != probe {
        let next = parent[root];
        parent[root] = probe;
        root = next;
    }
    probe
}

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
    pub name: Option<String>,
}

/// One named street along a route, with the distance covered on it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StreetRun {
    pub name: String,
    pub distance_m: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteResult {
    pub duration_seconds: i64,
    pub distance_meters: f64,
    pub geometry: Vec<Coord>,
    /// Consecutive named streets along the route, longest-first order of
    /// traversal. Unnamed road pieces are left out; partial first and last
    /// edges count only the traveled fraction of their street.
    pub streets: Vec<StreetRun>,
}

/// Merge a traversed sequence of (name, distance) pieces into runs.
/// Unnamed pieces (no street tag) contribute nothing.
pub(crate) fn merge_street_runs(
    pieces: impl IntoIterator<Item = (Option<String>, f64)>,
) -> Vec<StreetRun> {
    let mut runs: Vec<StreetRun> = Vec::new();
    for (name, distance) in pieces {
        let Some(name) = name else { continue };
        if distance <= 0.0 {
            continue;
        }
        match runs.last_mut() {
            Some(last) if last.name == name => last.distance_m += distance,
            _ => runs.push(StreetRun {
                name,
                distance_m: distance,
            }),
        }
    }
    runs
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

#[derive(Debug, Clone, Copy)]
struct EdgeTraversal {
    node: NodeIdx,
    time_s: f64,
    distance_m: f64,
}

/// How an itinerary may traverse a snapped location.
///
/// The engine knows each point's role: an origin only needs to leave (`Outgoing`),
/// a destination only needs to arrive (`Incoming`), and every intermediate stop
/// needs both (`Both`). Snapping prefers a node that satisfies the requirement
/// and keeps the plain nearest node when none does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapAccess {
    Incoming,
    Outgoing,
    Both,
}

impl SnapAccess {
    fn allows(&self, incoming: bool, outgoing: bool) -> bool {
        match self {
            SnapAccess::Incoming => incoming,
            SnapAccess::Outgoing => outgoing,
            SnapAccess::Both => incoming && outgoing,
        }
    }
}

pub struct RoadNetwork {
    pub(super) graph: DiGraph<NodeData, EdgeData>,
    spatial_index: Option<KdTree<SpatialPoint>>,
    edge_spatial_index: Option<SegmentIndex<SpatialSegment>>,
    // Per node: whether an incoming and an outgoing edge exist, so access-aware
    // snapping can refuse a node an itinerary could never traverse as required.
    node_access: Option<(Vec<bool>, Vec<bool>)>,
    // Per node: the size of its undirected component. Stub components (parking
    // loops, gated drives) satisfy a direction requirement inside themselves,
    // while no road connects them anywhere; access-aware snapping prefers the
    // candidate with the most graph around it.
    node_component_size: Option<Vec<u32>>,
    max_speed_mps: f64,
}

impl RoadNetwork {
    pub fn new() -> Self {
        Self {
            graph: DiGraph::new(),
            spatial_index: None,
            edge_spatial_index: None,
            node_access: None,
            node_component_size: None,
            max_speed_mps: 0.0,
        }
    }

    pub(super) fn add_node_at(&mut self, lat: f64, lng: f64) -> NodeIdx {
        self.graph.add_node(NodeData { lat, lng })
    }

    pub(super) fn add_edge(&mut self, from: NodeIdx, to: NodeIdx, data: EdgeData) {
        self.record_edge_speed(&data);
        self.graph.add_edge(from, to, data);
    }

    pub fn add_edge_by_index(
        &mut self,
        from: usize,
        to: usize,
        travel_time_s: f64,
        distance_m: f64,
    ) {
        self.add_edge_named_by_index(from, to, travel_time_s, distance_m, None);
    }

    pub(super) fn add_edge_named_by_index(
        &mut self,
        from: usize,
        to: usize,
        travel_time_s: f64,
        distance_m: f64,
        name: Option<String>,
    ) {
        let from_idx = NodeIdx::new(from);
        let to_idx = NodeIdx::new(to);
        let data = EdgeData {
            travel_time_s,
            distance_m,
            name,
        };
        self.record_edge_speed(&data);
        self.graph.add_edge(from_idx, to_idx, data);
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

        // One pass over the edges records, per node, whether an incoming and
        // an outgoing edge exist. The engine knows each point's role, so it
        // can demand the directions its itinerary needs when snapping.
        let node_count = self.graph.node_count();
        if node_count > 0 {
            let mut incoming = vec![false; node_count];
            let mut outgoing = vec![false; node_count];
            for (from, to) in self
                .graph
                .edge_indices()
                .filter_map(|edge| self.graph.edge_endpoints(edge))
            {
                outgoing[from.index()] = true;
                incoming[to.index()] = true;
            }
            self.node_access = Some((incoming, outgoing));

            // Undirected component sizes: a stub can satisfy the direction
            // requirement inside itself, so a parking loop is only told apart
            // from the through network by how much graph surrounds a candidate.
            let mut parent: Vec<usize> = (0..node_count).collect();
            let mut sizes: Vec<usize> = vec![1; node_count];
            for (from, to) in self
                .graph
                .edge_indices()
                .filter_map(|edge| self.graph.edge_endpoints(edge))
            {
                let from_root = component_root(&mut parent, from.index());
                let to_root = component_root(&mut parent, to.index());
                if from_root == to_root {
                    continue;
                }
                // Merge by size, so later finds stay shallow without a rank array.
                if sizes[from_root] < sizes[to_root] {
                    sizes[to_root] += sizes[from_root];
                    sizes[from_root] = 0;
                    parent[from_root] = to_root;
                } else {
                    sizes[from_root] += sizes[to_root];
                    sizes[to_root] = 0;
                    parent[to_root] = from_root;
                }
            }
            let per_node: Vec<u32> = (0..node_count)
                .map(|node_index| {
                    let root = component_root(&mut parent, node_index);
                    u32::try_from(sizes[root]).unwrap_or(u32::MAX)
                })
                .collect();
            self.node_component_size = Some(per_node);
        }
    }

    /// Whether the node satisfies the required traversal direction.
    ///
    /// Graphs without the pass treat every node as usable, which preserves
    /// the plain nearest-node behavior.
    fn allows_access(&self, node_index: NodeIdx, access: SnapAccess) -> bool {
        match self.node_access.as_ref() {
            Some((incoming, outgoing)) => {
                let index = node_index.index();
                access.allows(
                    incoming.get(index).copied().unwrap_or(false),
                    outgoing.get(index).copied().unwrap_or(false),
                )
            }
            None => true,
        }
    }

    /// Size of the undirected component that contains the node.
    fn component_size(&self, node_index: NodeIdx) -> u32 {
        match self.node_component_size.as_ref() {
            Some(per_node) => per_node.get(node_index.index()).copied().unwrap_or(0),
            None => u32::MAX,
        }
    }

    /// Snap with an explicit traversal requirement (see `SnapAccess`).
    ///
    /// Among the nearest candidates the winner satisfies the requirement on
    /// the largest component (the plain nearest wins ties); when none
    /// satisfies the requirement, the plain nearest node is returned.
    pub fn snap_road_index_with_access(
        &self,
        coord: Coord,
        access: SnapAccess,
    ) -> Result<SnappedCoord, RoutingError> {
        let tree = self
            .spatial_index
            .as_ref()
            .ok_or(RoutingError::SnapFailed {
                coord,
                nearest_distance_m: None,
            })?;

        let query = Point2D::new(coord.lat, coord.lng);
        const SNAP_CANDIDATES: usize = 64;
        let candidates = tree.nearest_n_neighbors(&query, SNAP_CANDIDATES);
        // The plain nearest wins ties: only a candidate that satisfies the
        // requirement on a strictly larger component displaces it.
        let mut chosen: Option<(NodeIdx, u32)> = None;
        for (candidate, _dist) in &candidates {
            if !self.allows_access(candidate.node_index, access) {
                continue;
            }
            let size = self.component_size(candidate.node_index);
            if chosen.is_none_or(|(_, best)| size > best) {
                chosen = Some((candidate.node_index, size));
            }
        }
        let chosen_node = match chosen {
            Some((node, _)) => node,
            None => candidates
                .first()
                .map(|(candidate, _)| candidate.node_index)
                .ok_or(RoutingError::SnapFailed {
                    coord,
                    nearest_distance_m: None,
                })?,
        };

        let node_data = self
            .graph
            .node_weight(chosen_node)
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
            node_index: chosen_node,
        })
    }

    fn record_edge_speed(&mut self, edge: &EdgeData) {
        if edge.travel_time_s <= 0.0 || edge.distance_m <= 0.0 {
            return;
        }

        let speed_mps = edge.distance_m / edge.travel_time_s;
        if speed_mps.is_finite() {
            self.max_speed_mps = self.max_speed_mps.max(speed_mps);
        }
    }

    fn distance_lower_bound_between(&self, from: NodeIdx, to: NodeIdx) -> f64 {
        let Some(from_node) = self.graph.node_weight(from) else {
            return 0.0;
        };
        let Some(to_node) = self.graph.node_weight(to) else {
            return 0.0;
        };
        haversine_distance(from_node.coord(), to_node.coord())
    }

    fn time_lower_bound_between(&self, from: NodeIdx, to: NodeIdx) -> f64 {
        if !self.max_speed_mps.is_finite() || self.max_speed_mps <= 0.0 {
            return 0.0;
        }

        self.distance_lower_bound_between(from, to) / self.max_speed_mps
    }

    fn node_path_geometry(&self, path: &[NodeIdx]) -> Vec<Coord> {
        path.iter()
            .filter_map(|&idx| self.graph.node_weight(idx).map(|n| n.coord()))
            .collect()
    }

    fn node_path_metrics(&self, path: &[NodeIdx]) -> (f64, f64) {
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

        (distance, time)
    }

    fn route_result_from_node_path(&self, path: &[NodeIdx], duration_seconds: i64) -> RouteResult {
        let geometry = self.node_path_geometry(path);
        let (distance, _) = self.node_path_metrics(path);
        let pieces = path.windows(2).map(|window| {
            self.graph
                .find_edge(window[0], window[1])
                .and_then(|edge| self.graph.edge_weight(edge))
                .map(|weight| (weight.name.clone(), weight.distance_m))
                .unwrap_or((None, 0.0))
        });

        RouteResult {
            duration_seconds,
            distance_meters: distance,
            geometry,
            streets: merge_street_runs(pieces),
        }
    }

    /// Iterate over all nodes as (lat, lng) pairs.
    pub fn nodes_iter(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.graph
            .node_indices()
            .filter_map(|idx| self.graph.node_weight(idx).map(|n| (n.lat, n.lng)))
    }

    /// Iterate over all edges as (from_idx, to_idx, travel_time_s, distance_m)
    /// tuples. Names are available through [`Self::edges_with_names_iter`].
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

    /// Iterate over all edges with street names as
    /// (from_idx, to_idx, travel_time_s, distance_m, name) tuples.
    pub fn edges_with_names_iter(
        &self,
    ) -> impl Iterator<Item = (usize, usize, f64, f64, Option<String>)> + '_ {
        self.graph.edge_indices().filter_map(|idx| {
            let (from, to) = self.graph.edge_endpoints(idx)?;
            let weight = self.graph.edge_weight(idx)?;
            Some((
                from.index(),
                to.index(),
                weight.travel_time_s,
                weight.distance_m,
                weight.name.clone(),
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
        let start_snap = self.snap_road_index_with_access(from, SnapAccess::Outgoing)?;
        let end_snap = self.snap_road_index_with_access(to, SnapAccess::Incoming)?;

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

        // Same-edge travel is only valid in the edge's forward direction.
        if from.edge_index == to.edge_index {
            if to.position < from.position {
                return Err(RoutingError::NoPath {
                    from: from.original,
                    to: to.original,
                });
            }

            let segment_time = from_edge.travel_time_s;
            let segment_dist = from_edge.distance_m;
            let travel_fraction = to.position - from.position;
            return Ok(RouteResult {
                duration_seconds: (segment_time * travel_fraction).round() as i64,
                distance_meters: segment_dist * travel_fraction,
                geometry: vec![from.snapped, to.snapped],
                streets: merge_street_runs([(
                    from_edge.name.clone(),
                    segment_dist * travel_fraction,
                )]),
            });
        }

        let start_exit = EdgeTraversal {
            node: from.to_node,
            time_s: from_edge.travel_time_s * (1.0 - from.position),
            distance_m: from_edge.distance_m * (1.0 - from.position),
        };
        let end_entry = EdgeTraversal {
            node: to.from_node,
            time_s: to_edge.travel_time_s * to.position,
            distance_m: to_edge.distance_m * to.position,
        };

        let best_result = if start_exit.node == end_entry.node {
            Some((start_exit.time_s + end_entry.time_s, vec![start_exit.node]))
        } else {
            astar(
                &self.graph,
                start_exit.node,
                |n| n == end_entry.node,
                |e| e.travel_time_s,
                |n| self.time_lower_bound_between(n, end_entry.node),
            )
            .map(|(path_cost, path)| (start_exit.time_s + path_cost + end_entry.time_s, path))
        };

        match best_result {
            Some((total_cost, path)) => {
                let mut geometry = vec![from.snapped];
                for coord in self.node_path_geometry(&path) {
                    if geometry.last().copied() != Some(coord) {
                        geometry.push(coord);
                    }
                }
                if geometry.last().copied() != Some(to.snapped) {
                    geometry.push(to.snapped);
                }

                let (path_distance, _) = self.node_path_metrics(&path);
                let distance = start_exit.distance_m + path_distance + end_entry.distance_m;
                let mut pieces = Vec::new();
                pieces.push((from_edge.name.clone(), start_exit.distance_m));
                for window in path.windows(2) {
                    if let Some(edge) = self.graph.find_edge(window[0], window[1]) {
                        if let Some(weight) = self.graph.edge_weight(edge) {
                            pieces.push((weight.name.clone(), weight.distance_m));
                        }
                    }
                }
                pieces.push((to_edge.name.clone(), end_entry.distance_m));

                Ok(RouteResult {
                    duration_seconds: total_cost.round() as i64,
                    distance_meters: distance,
                    geometry,
                    streets: merge_street_runs(pieces),
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
            // Stay on the network: a line between the raw points may cross land or water.
            return Ok(RouteResult {
                duration_seconds: 0,
                distance_meters: 0.0,
                geometry: vec![from.snapped, to.snapped],
                streets: Vec::new(),
            });
        }

        let result = astar(
            &self.graph,
            from.node_index,
            |n| n == to.node_index,
            |e| e.travel_time_s,
            |n| self.time_lower_bound_between(n, to.node_index),
        );

        match result {
            Some((cost, path)) => Ok(self.route_result_from_node_path(&path, cost.round() as i64)),
            None => Err(RoutingError::NoPath {
                from: from.original,
                to: to.original,
            }),
        }
    }

    /// Find a route between two coordinates with an explicit optimization objective.
    ///
    /// Like `route`, this method snaps to the nearest graph nodes first and
    /// uses admissible straight-line lower bounds for both objectives.
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
                geometry: vec![start_snap.snapped, end_snap.snapped],
                streets: Vec::new(),
            });
        }

        let result = match objective {
            Objective::Time => astar(
                &self.graph,
                start_snap.node_index,
                |n| n == end_snap.node_index,
                |e| e.travel_time_s,
                |n| self.time_lower_bound_between(n, end_snap.node_index),
            ),
            Objective::Distance => astar(
                &self.graph,
                start_snap.node_index,
                |n| n == end_snap.node_index,
                |e| e.distance_m,
                |n| self.distance_lower_bound_between(n, end_snap.node_index),
            ),
        };

        match result {
            Some((_, path)) => {
                let (distance, time) = self.node_path_metrics(&path);
                Ok(RouteResult {
                    duration_seconds: time.round() as i64,
                    distance_meters: distance,
                    geometry: self.node_path_geometry(&path),
                    streets: merge_street_runs(path.windows(2).map(|window| {
                        self.graph
                            .find_edge(window[0], window[1])
                            .and_then(|edge| self.graph.edge_weight(edge))
                            .map(|weight| (weight.name.clone(), weight.distance_m))
                            .unwrap_or((None, 0.0))
                    })),
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

        self.build_spatial_index();
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
    /// For networks with street names use [`Self::from_test_data_named`].
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

    /// Build a network from test data with named edges.
    ///
    /// # Arguments
    /// * `nodes` - Slice of (lat, lng) tuples
    /// * `edges` - Slice of (from_idx, to_idx, time_s, dist_m, name) tuples
    ///
    /// # Example
    /// ```
    /// use solverforge_maps::RoadNetwork;
    ///
    /// let nodes = &[(40.0, -75.0), (39.8, -75.2)];
    /// let edges = &[(0, 1, 60.0, 1000.0, "Market Street")];
    /// let network = RoadNetwork::from_test_data_named(nodes, edges);
    ///
    /// assert_eq!(network.edge_count(), 1);
    /// ```
    pub fn from_test_data_named(
        nodes: &[(f64, f64)],
        edges: &[(usize, usize, f64, f64, &str)],
    ) -> Self {
        let mut network = Self::new();

        for &(lat, lng) in nodes {
            network.add_node_at(lat, lng);
        }

        for &(from, to, time_s, dist_m, name) in edges {
            network.add_edge_named_by_index(from, to, time_s, dist_m, Some(name.to_string()));
        }

        network.build_spatial_index();

        network
    }
}

#[cfg(test)]
#[path = "network_tests.rs"]
mod tests;

#[cfg(test)]
#[path = "network_name_tests.rs"]
mod name_tests;

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
