//! Error types for routing operations.

use std::fmt;

use super::coord::Coord;

#[derive(Debug, Clone, PartialEq)]
pub enum CoordError {
    LatOutOfRange { value: f64 },
    LngOutOfRange { value: f64 },
    LatNaN,
    LngNaN,
    LatInfinite { value: f64 },
    LngInfinite { value: f64 },
}

impl fmt::Display for CoordError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CoordError::LatOutOfRange { value } => {
                write!(f, "latitude {} out of valid range [-90, 90]", value)
            }
            CoordError::LngOutOfRange { value } => {
                write!(f, "longitude {} out of valid range [-180, 180]", value)
            }
            CoordError::LatNaN => write!(f, "latitude is NaN"),
            CoordError::LngNaN => write!(f, "longitude is NaN"),
            CoordError::LatInfinite { value } => write!(f, "latitude {} is infinite", value),
            CoordError::LngInfinite { value } => write!(f, "longitude {} is infinite", value),
        }
    }
}

impl std::error::Error for CoordError {}

#[derive(Debug, Clone, PartialEq)]
pub enum BBoxError {
    MinLatGreaterThanMax { min: f64, max: f64 },
    MinLngGreaterThanMax { min: f64, max: f64 },
    LatOutOfRange { value: f64 },
    LngOutOfRange { value: f64 },
    NaN { field: &'static str },
    Infinite { field: &'static str, value: f64 },
}

impl fmt::Display for BBoxError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BBoxError::MinLatGreaterThanMax { min, max } => {
                write!(f, "min_lat {} is greater than max_lat {}", min, max)
            }
            BBoxError::MinLngGreaterThanMax { min, max } => {
                write!(f, "min_lng {} is greater than max_lng {}", min, max)
            }
            BBoxError::LatOutOfRange { value } => {
                write!(f, "latitude {} out of valid range [-90, 90]", value)
            }
            BBoxError::LngOutOfRange { value } => {
                write!(f, "longitude {} out of valid range [-180, 180]", value)
            }
            BBoxError::NaN { field } => write!(f, "{} is NaN", field),
            BBoxError::Infinite { field, value } => write!(f, "{} {} is infinite", field, value),
        }
    }
}

impl std::error::Error for BBoxError {}

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

impl fmt::Display for RoutingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
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
