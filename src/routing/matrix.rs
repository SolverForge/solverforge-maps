//! Matrix computation for road networks.

use ordered_float::OrderedFloat;
use petgraph::algo::dijkstra;
use std::collections::HashMap;
use tokio::sync::mpsc::Sender;

use super::coord::Coord;
use super::geo::{haversine_distance, DEFAULT_SPEED_MPS};
use super::network::RoadNetwork;
use super::progress::RoutingProgress;

pub type TravelTimeMatrix = Vec<Vec<i64>>;

impl RoadNetwork {
    pub async fn compute_matrix(
        &self,
        locations: &[Coord],
        progress: Option<&Sender<RoutingProgress>>,
    ) -> TravelTimeMatrix {
        let n = locations.len();
        let mut matrix = vec![vec![0i64; n]; n];

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
                            matrix[i][j] = cost.0.round() as i64;
                        } else {
                            let dist = haversine_distance(locations[i], locations[j]);
                            matrix[i][j] = (dist / DEFAULT_SPEED_MPS).round() as i64;
                        }
                    }
                }
            } else {
                for j in 0..n {
                    if i == j {
                        continue;
                    }
                    let dist = haversine_distance(locations[i], locations[j]);
                    matrix[i][j] = (dist / DEFAULT_SPEED_MPS).round() as i64;
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

        matrix
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
                if let Some(result) = self.route(locations[i], locations[j]) {
                    geometries.insert((i, j), result.geometry);
                }
                pair_count += 1;

                if let Some(tx) = progress {
                    if pair_count % 10 == 0 || pair_count == total_pairs {
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
        let mut matrix = vec![vec![0i64; n]; n];

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
                            matrix[i][j] = cost.0.round() as i64;
                        } else {
                            let dist = haversine_distance(locations[i], locations[j]);
                            matrix[i][j] = (dist / DEFAULT_SPEED_MPS).round() as i64;
                        }
                    }
                }
            } else {
                for j in 0..n {
                    if i == j {
                        continue;
                    }
                    let dist = haversine_distance(locations[i], locations[j]);
                    matrix[i][j] = (dist / DEFAULT_SPEED_MPS).round() as i64;
                }
            }
        }

        matrix
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
                if let Some(result) = self.route(locations[i], locations[j]) {
                    geometries.insert((i, j), result.geometry);
                }
            }
        }

        geometries
    }
}
