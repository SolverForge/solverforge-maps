//! Matrix computation for road networks.

use ordered_float::OrderedFloat;
use petgraph::algo::dijkstra;
use rayon::prelude::*;
use std::collections::HashMap;
use tokio::sync::mpsc::Sender;

use super::coord::Coord;
use super::network::{RoadNetwork, SnappedCoord};
use super::progress::RoutingProgress;

/// Value used to indicate unreachable pairs in the matrix.
pub const UNREACHABLE: i64 = i64::MAX;

/// A travel time matrix with metadata and analysis methods.
#[derive(Debug, Clone)]
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
    pub async fn compute_matrix(
        &self,
        locations: &[Coord],
        progress: Option<&Sender<RoutingProgress>>,
    ) -> TravelTimeMatrix {
        let n = locations.len();
        let mut data = vec![0i64; n * n];

        // Snap all locations
        let snapped: Vec<SnappedCoord> = locations
            .iter()
            .map(|&coord| {
                self.snap_to_road_detailed(coord).unwrap_or_else(|_| {
                    // Fallback: create a "virtual" snapped coord for error handling
                    SnappedCoord {
                        original: coord,
                        snapped: coord,
                        snap_distance_m: f64::INFINITY,
                        node_index: petgraph::graph::NodeIndex::new(0),
                    }
                })
            })
            .collect();

        let nodes: Vec<_> = locations
            .iter()
            .map(|&coord| self.snap_to_road(coord))
            .collect();

        for i in 0..n {
            if let Some(from_node) = nodes[i] {
                let costs = dijkstra(self.graph(), from_node, None, |e| {
                    OrderedFloat(e.weight().travel_time_s)
                });

                for j in 0..n {
                    if i == j {
                        continue;
                    }
                    if let Some(to_node) = nodes[j] {
                        if let Some(cost) = costs.get(&to_node) {
                            data[i * n + j] = cost.0.round() as i64;
                        } else {
                            data[i * n + j] = UNREACHABLE;
                        }
                    } else {
                        data[i * n + j] = UNREACHABLE;
                    }
                }
            } else {
                for j in 0..n {
                    if i != j {
                        data[i * n + j] = UNREACHABLE;
                    }
                }
            }

            if let Some(tx) = progress {
                let percent = 50 + ((i + 1) * 30 / n) as u8;
                let _ = tx
                    .send(RoutingProgress::ComputingMatrix {
                        percent,
                        row: i + 1,
                        total: n,
                    })
                    .await;
            }
        }

        TravelTimeMatrix::new(data, n, snapped)
    }

    /// Compute the travel time matrix using parallel row computation.
    ///
    /// Uses rayon for parallel processing, computing each row independently.
    /// This is significantly faster than the sequential version for large matrices.
    pub fn compute_matrix_parallel(&self, locations: &[Coord]) -> TravelTimeMatrix {
        let n = locations.len();
        if n == 0 {
            return TravelTimeMatrix::new(vec![], 0, vec![]);
        }

        // Snap all locations
        let snapped: Vec<SnappedCoord> = locations
            .iter()
            .map(|&coord| {
                self.snap_to_road_detailed(coord)
                    .unwrap_or_else(|_| SnappedCoord {
                        original: coord,
                        snapped: coord,
                        snap_distance_m: f64::INFINITY,
                        node_index: petgraph::graph::NodeIndex::new(0),
                    })
            })
            .collect();

        let nodes: Vec<_> = locations
            .iter()
            .map(|&coord| self.snap_to_road(coord))
            .collect();

        // Compute rows in parallel
        let rows: Vec<Vec<i64>> = (0..n)
            .into_par_iter()
            .map(|i| {
                let mut row = vec![0i64; n];

                if let Some(from_node) = nodes[i] {
                    let costs = dijkstra(self.graph(), from_node, None, |e| {
                        OrderedFloat(e.weight().travel_time_s)
                    });

                    for j in 0..n {
                        if i == j {
                            continue;
                        }
                        if let Some(to_node) = nodes[j] {
                            if let Some(cost) = costs.get(&to_node) {
                                row[j] = cost.0.round() as i64;
                            } else {
                                row[j] = UNREACHABLE;
                            }
                        } else {
                            row[j] = UNREACHABLE;
                        }
                    }
                } else {
                    for (j, cell) in row.iter_mut().enumerate() {
                        if i != j {
                            *cell = UNREACHABLE;
                        }
                    }
                }

                row
            })
            .collect();

        // Flatten rows into single vector
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

    pub fn compute_matrix_sync(&self, locations: &[Coord]) -> TravelTimeMatrix {
        let n = locations.len();
        let mut data = vec![0i64; n * n];

        // Snap all locations
        let snapped: Vec<SnappedCoord> = locations
            .iter()
            .map(|&coord| {
                self.snap_to_road_detailed(coord)
                    .unwrap_or_else(|_| SnappedCoord {
                        original: coord,
                        snapped: coord,
                        snap_distance_m: f64::INFINITY,
                        node_index: petgraph::graph::NodeIndex::new(0),
                    })
            })
            .collect();

        let nodes: Vec<_> = locations
            .iter()
            .map(|&coord| self.snap_to_road(coord))
            .collect();

        for i in 0..n {
            if let Some(from_node) = nodes[i] {
                let costs = dijkstra(self.graph(), from_node, None, |e| {
                    OrderedFloat(e.weight().travel_time_s)
                });

                for j in 0..n {
                    if i == j {
                        continue;
                    }
                    if let Some(to_node) = nodes[j] {
                        if let Some(cost) = costs.get(&to_node) {
                            data[i * n + j] = cost.0.round() as i64;
                        } else {
                            data[i * n + j] = UNREACHABLE;
                        }
                    } else {
                        data[i * n + j] = UNREACHABLE;
                    }
                }
            } else {
                for j in 0..n {
                    if i != j {
                        data[i * n + j] = UNREACHABLE;
                    }
                }
            }
        }

        TravelTimeMatrix::new(data, n, snapped)
    }

    pub fn compute_geometries_sync(
        &self,
        locations: &[Coord],
    ) -> HashMap<(usize, usize), Vec<Coord>> {
        let n = locations.len();
        let mut geometries = HashMap::new();

        for i in 0..n {
            for j in 0..n {
                if i == j {
                    continue;
                }
                if let Ok(result) = self.route(locations[i], locations[j]) {
                    geometries.insert((i, j), result.geometry);
                }
            }
        }

        geometries
    }
}
