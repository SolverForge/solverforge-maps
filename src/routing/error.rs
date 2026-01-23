//! Error types for routing operations.

use super::coord::{Coord, CoordError};

#[derive(Debug)]
pub enum RoutingError {
    Network(String),
    Parse(String),
    Io(std::io::Error),
    SnapFailed {
        coord: Coord,
        nearest_distance_m: Option<f64>,
    },
    NoPath {
        from: Coord,
        to: Coord,
    },
    InvalidCoordinate {
        error: CoordError,
    },
    Cancelled,
}

impl std::fmt::Display for RoutingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RoutingError::Network(msg) => write!(f, "Network error: {}", msg),
            RoutingError::Parse(msg) => write!(f, "Parse error: {}", msg),
            RoutingError::Io(e) => write!(f, "I/O error: {}", e),
            RoutingError::SnapFailed {
                coord,
                nearest_distance_m,
            } => match nearest_distance_m {
                Some(dist) => write!(
                    f,
                    "Failed to snap coordinate ({}, {}) to road network; nearest node is {:.1}m away",
                    coord.lat, coord.lng, dist
                ),
                None => write!(
                    f,
                    "Failed to snap coordinate ({}, {}) to road network; no nodes in network",
                    coord.lat, coord.lng
                ),
            },
            RoutingError::NoPath { from, to } => write!(
                f,
                "No path found from ({}, {}) to ({}, {})",
                from.lat, from.lng, to.lat, to.lng
            ),
            RoutingError::InvalidCoordinate { error } => {
                write!(f, "Invalid coordinate: {}", error)
            }
            RoutingError::Cancelled => write!(f, "Operation was cancelled"),
        }
    }
}

impl std::error::Error for RoutingError {}

impl From<std::io::Error> for RoutingError {
    fn from(e: std::io::Error) -> Self {
        RoutingError::Io(e)
    }
}

impl From<CoordError> for RoutingError {
    fn from(e: CoordError) -> Self {
        RoutingError::InvalidCoordinate { error: e }
    }
}
