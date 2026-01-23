//! Matrix computation for road networks.

use ordered_float::OrderedFloat;
use petgraph::algo::dijkstra;
use rayon::prelude::*;
use std::collections::HashMap;
use tokio::sync::mpsc::Sender;

use super::coord::Coord;
use super::network::{EdgeSnappedLocation, RoadNetwork, SnappedCoord};
use super::progress::RoutingProgress;

/// Value used to indicate unreachable pairs in the matrix.
pub const UNREACHABLE: i64 = i64::MAX;

/// A travel time matrix with metadata and analysis methods.
#[derive(Debug, Clone, Default)]
pub struct TravelTimeMatrix {
    data: Vec<i64>,
    size: usize,
    locations: Vec<SnappedCoord>,
}

impl TravelTimeMatrix {
    pub(crate) fn new(data: Vec<i64>, size: usize, locations: Vec<SnappedCoord>) -> Self {
        debug_assert_eq!(data.len(), size * size);
        debug_assert_eq!(locations.len(), size);
        Self {
            data,
            size,
            locations,
        }
    }

    /// Get the travel time from one location to another.
    ///
    /// Returns `None` if indices are out of bounds.
    /// Returns `Some(UNREACHABLE)` if the pair is not reachable.
    #[inline]
    pub fn get(&self, from: usize, to: usize) -> Option<i64> {
        if from < self.size && to < self.size {
            Some(self.data[from * self.size + to])
        } else {
            None
        }
    }

    /// Check if a pair is reachable.
    #[inline]
    pub fn is_reachable(&self, from: usize, to: usize) -> bool {
        self.get(from, to)
            .map(|v| v != UNREACHABLE)
            .unwrap_or(false)
    }

    /// Get the matrix size (number of locations).
    #[inline]
    pub fn size(&self) -> usize {
        self.size
    }

    /// Get the snapped locations.
    #[inline]
    pub fn locations(&self) -> &[SnappedCoord] {
        &self.locations
    }

    /// Get a row of the matrix as a slice.
    #[inline]
    pub fn row(&self, i: usize) -> Option<&[i64]> {
        if i < self.size {
            let start = i * self.size;
            Some(&self.data[start..start + self.size])
        } else {
            None
        }
    }

    /// Get the minimum travel time (excluding diagonal and unreachable).
    pub fn min(&self) -> Option<i64> {
        let mut min_val = None;
        for i in 0..self.size {
            for j in 0..self.size {
                if i == j {
                    continue;
                }
                let val = self.data[i * self.size + j];
                if val != UNREACHABLE {
                    min_val = Some(min_val.map(|m: i64| m.min(val)).unwrap_or(val));
                }
            }
        }
        min_val
    }

    /// Get the maximum travel time (excluding diagonal and unreachable).
    pub fn max(&self) -> Option<i64> {
        let mut max_val = None;
        for i in 0..self.size {
            for j in 0..self.size {
                if i == j {
                    continue;
                }
                let val = self.data[i * self.size + j];
                if val != UNREACHABLE {
                    max_val = Some(max_val.map(|m: i64| m.max(val)).unwrap_or(val));
                }
            }
        }
        max_val
    }

    /// Get the mean travel time (excluding diagonal and unreachable).
    pub fn mean(&self) -> Option<f64> {
        let mut sum = 0i64;
        let mut count = 0usize;
        for i in 0..self.size {
            for j in 0..self.size {
                if i == j {
                    continue;
                }
                let val = self.data[i * self.size + j];
                if val != UNREACHABLE {
                    sum = sum.saturating_add(val);
                    count += 1;
                }
            }
        }
        if count > 0 {
            Some(sum as f64 / count as f64)
        } else {
            None
        }
    }

    /// Get the fraction of reachable pairs (excluding diagonal).
    pub fn reachability_ratio(&self) -> f64 {
        if self.size <= 1 {
            return 1.0;
        }
        let total_pairs = self.size * (self.size - 1);
        let reachable = self.count_reachable();
        reachable as f64 / total_pairs as f64
    }

    /// Count reachable pairs (excluding diagonal).
    fn count_reachable(&self) -> usize {
        let mut count = 0;
        for i in 0..self.size {
            for j in 0..self.size {
                if i != j && self.data[i * self.size + j] != UNREACHABLE {
                    count += 1;
                }
            }
        }
        count
    }

    /// Get all unreachable pairs as (from_idx, to_idx).
    pub fn unreachable_pairs(&self) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();
        for i in 0..self.size {
            for j in 0..self.size {
                if i != j && self.data[i * self.size + j] == UNREACHABLE {
                    pairs.push((i, j));
                }
            }
        }
        pairs
    }

    /// Get raw data as a flat slice (row-major order).
    pub fn as_slice(&self) -> &[i64] {
        &self.data
    }
}

impl RoadNetwork {
    /// Compute the travel time matrix using per-source Dijkstra with rayon parallelism.
    ///
    /// This is production-grade: O(n × Dijkstra) instead of O(n² × A*).
    /// Each source runs Dijkstra once to get costs to ALL nodes, then O(1) lookup per destination.
    pub async fn compute_matrix(
        &self,
        locations: &[Coord],
        progress: Option<&Sender<RoutingProgress>>,
    ) -> TravelTimeMatrix {
        let n = locations.len();
        if n == 0 {
            return TravelTimeMatrix::new(vec![], 0, vec![]);
        }

        // Snap all locations to edges (production-grade)
        let edge_snapped: Vec<Option<EdgeSnappedLocation>> = locations
            .iter()
            .map(|&coord| self.snap_to_edge(coord).ok())
            .collect();

        // Build SnappedCoord for API compatibility
        let snapped: Vec<SnappedCoord> = locations
            .iter()
            .zip(edge_snapped.iter())
            .map(|(&coord, edge_snap)| {
                if let Some(es) = edge_snap {
                    SnappedCoord {
                        original: coord,
                        snapped: es.snapped,
                        snap_distance_m: es.snap_distance_m,
                        node_index: es.from_node,
                    }
                } else {
                    SnappedCoord {
                        original: coord,
                        snapped: coord,
                        snap_distance_m: f64::INFINITY,
                        node_index: petgraph::graph::NodeIndex::new(0),
                    }
                }
            })
            .collect();

        // Compute rows in parallel via rayon - each row runs Dijkstra from source endpoints
        let graph = &self.graph;
        let rows: Vec<Vec<i64>> = (0..n)
            .into_par_iter()
            .map(|i| {
                let mut row = vec![0i64; n];

                let Some(from) = &edge_snapped[i] else {
                    for (j, cell) in row.iter_mut().enumerate() {
                        if i != j {
                            *cell = UNREACHABLE;
                        }
                    }
                    return row;
                };

                let from_edge = match graph.edge_weight(from.edge_index) {
                    Some(e) => e,
                    None => {
                        for (j, cell) in row.iter_mut().enumerate() {
                            if i != j {
                                *cell = UNREACHABLE;
                            }
                        }
                        return row;
                    }
                };

                // Run Dijkstra from BOTH endpoints of source edge (to ALL nodes)
                let costs_a = dijkstra(graph, from.from_node, None, |e| {
                    OrderedFloat(e.weight().travel_time_s)
                });
                let costs_b = dijkstra(graph, from.to_node, None, |e| {
                    OrderedFloat(e.weight().travel_time_s)
                });

                // Offset from snap point to each endpoint
                let off_a = from_edge.travel_time_s * from.position;
                let off_b = from_edge.travel_time_s * (1.0 - from.position);

                for j in 0..n {
                    if i == j {
                        continue;
                    }

                    let Some(to) = &edge_snapped[j] else {
                        row[j] = UNREACHABLE;
                        continue;
                    };

                    let to_edge = match graph.edge_weight(to.edge_index) {
                        Some(e) => e,
                        None => {
                            row[j] = UNREACHABLE;
                            continue;
                        }
                    };

                    let to_off_a = to_edge.travel_time_s * to.position;
                    let to_off_b = to_edge.travel_time_s * (1.0 - to.position);

                    // Best of 4 combinations: (from endpoint) -> (to endpoint)
                    let mut best = f64::MAX;
                    if let Some(&c) = costs_a.get(&to.from_node) {
                        best = best.min(off_a + c.0 + to_off_a);
                    }
                    if let Some(&c) = costs_a.get(&to.to_node) {
                        best = best.min(off_a + c.0 + to_off_b);
                    }
                    if let Some(&c) = costs_b.get(&to.from_node) {
                        best = best.min(off_b + c.0 + to_off_a);
                    }
                    if let Some(&c) = costs_b.get(&to.to_node) {
                        best = best.min(off_b + c.0 + to_off_b);
                    }

                    row[j] = if best == f64::MAX {
                        UNREACHABLE
                    } else {
                        best.round() as i64
                    };
                }

                row
            })
            .collect();

        // Progress update after matrix computation
        if let Some(tx) = progress {
            let _ = tx
                .send(RoutingProgress::ComputingMatrix {
                    percent: 80,
                    row: n,
                    total: n,
                })
                .await;
        }

        let data: Vec<i64> = rows.into_iter().flatten().collect();
        TravelTimeMatrix::new(data, n, snapped)
    }

    pub async fn compute_geometries(
        &self,
        locations: &[Coord],
        progress: Option<&Sender<RoutingProgress>>,
    ) -> HashMap<(usize, usize), Vec<Coord>> {
        let n = locations.len();
        let total_pairs = n * (n - 1);
        let mut geometries = HashMap::new();
        let mut pair_count = 0usize;

        for i in 0..n {
            for j in 0..n {
                if i == j {
                    continue;
                }
                if let Ok(result) = self.route(locations[i], locations[j]) {
                    geometries.insert((i, j), result.geometry);
                }
                pair_count += 1;

                if let Some(tx) = progress {
                    if pair_count.is_multiple_of(10) || pair_count == total_pairs {
                        let percent = 80 + (pair_count * 15 / total_pairs.max(1)) as u8;
                        let _ = tx
                            .send(RoutingProgress::ComputingGeometries {
                                percent,
                                pair: pair_count,
                                total: total_pairs,
                            })
                            .await;
                    }
                }
            }
        }

        geometries
    }
}
