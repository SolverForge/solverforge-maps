//! Matrix computation for road networks.

use rayon::prelude::*;
use std::collections::HashMap;
use tokio::sync::mpsc::Sender;

use super::algo::dijkstra;
use super::coord::Coord;
use super::graph::NodeIdx;
use super::network::{RoadNetwork, SnappedCoord};
use super::progress::RoutingProgress;

pub const UNREACHABLE: i64 = i64::MAX;

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

    #[inline]
    pub fn is_reachable(&self, from: usize, to: usize) -> bool {
        self.get(from, to)
            .map(|v| v != UNREACHABLE)
            .unwrap_or(false)
    }

    #[inline]
    pub fn size(&self) -> usize {
        self.size
    }

    #[inline]
    pub fn locations(&self) -> &[SnappedCoord] {
        &self.locations
    }

    #[inline]
    pub fn row(&self, i: usize) -> Option<&[i64]> {
        if i < self.size {
            let start = i * self.size;
            Some(&self.data[start..start + self.size])
        } else {
            None
        }
    }

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

    pub fn reachability_ratio(&self) -> f64 {
        if self.size <= 1 {
            return 1.0;
        }
        let total_pairs = self.size * (self.size - 1);
        let reachable = self.count_reachable();
        reachable as f64 / total_pairs as f64
    }

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
        if n == 0 {
            return TravelTimeMatrix::new(vec![], 0, vec![]);
        }

        let node_snapped: Vec<Option<SnappedCoord>> = locations
            .iter()
            .map(|&coord| self.snap_to_road_detailed(coord).ok())
            .collect();

        let snapped_locations: Vec<SnappedCoord> = locations
            .iter()
            .zip(node_snapped.iter())
            .map(|(&coord, edge_snap)| {
                if let Some(snapped) = edge_snap {
                    *snapped
                } else {
                    SnappedCoord {
                        original: coord,
                        snapped: coord,
                        snap_distance_m: f64::INFINITY,
                        node_index: NodeIdx::new(0),
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

                let Some(from) = &node_snapped[i] else {
                    for (j, cell) in row.iter_mut().enumerate() {
                        if i != j {
                            *cell = UNREACHABLE;
                        }
                    }
                    return row;
                };

                let costs = dijkstra(graph, from.node_index, None, |e| e.travel_time_s);

                for j in 0..n {
                    if i == j {
                        continue;
                    }

                    let Some(to) = &node_snapped[j] else {
                        row[j] = UNREACHABLE;
                        continue;
                    };

                    row[j] = if let Some(&best) = costs.get(&to.node_index) {
                        best.round() as i64
                    } else {
                        UNREACHABLE
                    };
                }

                row
            })
            .collect();

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
        TravelTimeMatrix::new(data, n, snapped_locations)
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
