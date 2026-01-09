//! Matrix computation for road networks.

use ordered_float::OrderedFloat;
use petgraph::algo::dijkstra;
use std::collections::HashMap;

use super::geo::{haversine_distance, DEFAULT_SPEED_MPS};
use super::network::RoadNetwork;

impl RoadNetwork {
    /// Computes route geometries for all location pairs.
    pub fn compute_all_geometries(
        &self,
        locations: &[(f64, f64)],
    ) -> HashMap<(usize, usize), Vec<(f64, f64)>> {
        self.compute_all_geometries_with_progress(locations, |_, _| {})
    }

    /// Computes route geometries with row-level progress callback.
    ///
    /// # Example
    ///
    /// ```
    /// # use solverforge_maps::RoadNetwork;
    /// let network = RoadNetwork::new();
    /// let locations = vec![(39.95, -75.16), (39.96, -75.17)];
    /// let mut progress_calls = 0;
    /// let geometries = network.compute_all_geometries_with_progress(&locations, |row, total| {
    ///     progress_calls += 1;
    ///     assert!(row < total);
    /// });
    /// assert_eq!(progress_calls, 2); // One call per source location
    /// ```
    pub fn compute_all_geometries_with_progress<F>(
        &self,
        locations: &[(f64, f64)],
        mut on_row_complete: F,
    ) -> HashMap<(usize, usize), Vec<(f64, f64)>>
    where
        F: FnMut(usize, usize),
    {
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
            on_row_complete(i, n);
        }

        geometries
    }

    /// Computes all-pairs travel time matrix for given locations.
    pub fn compute_matrix(&self, locations: &[(f64, f64)]) -> Vec<Vec<i64>> {
        self.compute_matrix_with_progress(locations, |_, _| {})
    }

    /// Computes all-pairs travel time matrix with row-level progress callback.
    ///
    /// # Example
    ///
    /// ```
    /// # use solverforge_maps::RoadNetwork;
    /// let network = RoadNetwork::new();
    /// let locations = vec![(39.95, -75.16), (39.96, -75.17)];
    /// let mut progress_calls = 0;
    /// let matrix = network.compute_matrix_with_progress(&locations, |row, total| {
    ///     progress_calls += 1;
    ///     assert!(row < total);
    /// });
    /// assert_eq!(progress_calls, 2); // One call per row
    /// assert_eq!(matrix.len(), 2);
    /// ```
    pub fn compute_matrix_with_progress<F>(
        &self,
        locations: &[(f64, f64)],
        mut on_row_complete: F,
    ) -> Vec<Vec<i64>>
    where
        F: FnMut(usize, usize),
    {
        let n = locations.len();
        let mut matrix = vec![vec![0i64; n]; n];

        let nodes: Vec<_> = locations
            .iter()
            .map(|&(lat, lng)| self.snap_to_road(lat, lng))
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
                            let dist = haversine_distance(
                                locations[i].0,
                                locations[i].1,
                                locations[j].0,
                                locations[j].1,
                            );
                            matrix[i][j] = (dist / DEFAULT_SPEED_MPS).round() as i64;
                        }
                    }
                }
            } else {
                for j in 0..n {
                    if i == j {
                        continue;
                    }
                    let dist = haversine_distance(
                        locations[i].0,
                        locations[i].1,
                        locations[j].0,
                        locations[j].1,
                    );
                    matrix[i][j] = (dist / DEFAULT_SPEED_MPS).round() as i64;
                }
            }

            on_row_complete(i, n);
        }

        matrix
    }
}
