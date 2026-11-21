//! Slower roads for a period of the day: the same network with some edges
//! taking longer to drive.

use std::fmt;

use super::bbox::BoundingBox;
use super::config::RoadClass;
use super::coord::Coord;
use super::network::{EdgeData, RoadNetwork};

/// Roads that take `factor` times as long to drive: every road of the listed
/// classes (every road when none is listed), only where they lie in `area`
/// when one is given. An edge lies in an area when its midpoint does.
#[derive(Debug, Clone, PartialEq)]
pub struct Slowdown {
    factor: f64,
    roads: Vec<RoadClass>,
    area: Option<BoundingBox>,
}

/// A slowdown factor that is not a finite number above zero.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlowdownError {
    pub factor: f64,
}

impl fmt::Display for SlowdownError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "slowdown factor {} is not a finite number above zero",
            self.factor
        )
    }
}

impl std::error::Error for SlowdownError {}

impl Slowdown {
    /// Every road taking `factor` times as long: 1.5 is half as long again,
    /// 0.8 is faster than usual.
    pub fn new(factor: f64) -> Result<Self, SlowdownError> {
        if !factor.is_finite() || factor <= 0.0 {
            return Err(SlowdownError { factor });
        }
        Ok(Self {
            factor,
            roads: Vec::new(),
            area: None,
        })
    }

    /// Only roads of these classes.
    pub fn roads(mut self, roads: impl IntoIterator<Item = RoadClass>) -> Self {
        self.roads = roads.into_iter().collect();
        self
    }

    /// Only roads inside `area`.
    pub fn area(mut self, area: BoundingBox) -> Self {
        self.area = Some(area);
        self
    }

    pub fn factor(&self) -> f64 {
        self.factor
    }

    /// Whether an edge of `class` with its midpoint at `midpoint` is slowed.
    pub fn applies(&self, class: RoadClass, midpoint: Coord) -> bool {
        (self.roads.is_empty() || self.roads.contains(&class))
            && self.area.is_none_or(|area| area.contains(midpoint))
    }
}

impl RoadNetwork {
    /// This network with every edge's travel time multiplied by the factors of
    /// the slowdowns that apply to it; slowdowns that overlap multiply. Nodes,
    /// distances, names, classes and connectivity stay the same, so places snap
    /// where they do on this network and every route still exists.
    pub fn slowed(&self, slowdowns: &[Slowdown]) -> RoadNetwork {
        let mut slowed = RoadNetwork::new();
        for idx in self.graph.node_indices() {
            if let Some(node) = self.graph.node_weight(idx) {
                slowed.add_node_at(node.lat, node.lng);
            }
        }
        for idx in self.graph.edge_indices() {
            let (Some((from, to)), Some(edge)) =
                (self.graph.edge_endpoints(idx), self.graph.edge_weight(idx))
            else {
                continue;
            };
            let (Some(a), Some(b)) = (self.graph.node_weight(from), self.graph.node_weight(to))
            else {
                continue;
            };
            let midpoint = Coord::new((a.lat + b.lat) / 2.0, (a.lng + b.lng) / 2.0);
            let factor: f64 = slowdowns
                .iter()
                .filter(|s| s.applies(edge.road_class, midpoint))
                .map(Slowdown::factor)
                .product();
            slowed.add_edge(
                from,
                to,
                EdgeData {
                    travel_time_s: edge.travel_time_s * factor,
                    distance_m: edge.distance_m,
                    name: edge.name.clone(),
                    road_class: edge.road_class,
                },
            );
        }
        slowed.build_spatial_index();
        slowed
    }
}

#[cfg(test)]
#[path = "traffic_tests.rs"]
mod tests;
