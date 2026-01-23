//! Local OSM road routing using Overpass API and petgraph.

mod bbox;
mod cache;
mod config;
mod coord;
mod error;
mod fetch;
mod geo;
mod matrix;
mod network;
mod osm;
mod progress;

pub use bbox::{BBoxError, BoundingBox};
pub use cache::{CacheStats, NetworkRef};
pub use config::{NetworkConfig, SpeedProfile};
pub use coord::{Coord, CoordError};
pub use error::RoutingError;
pub use matrix::{TravelTimeMatrix, UNREACHABLE};
pub use network::{Objective, RoadNetwork, RouteResult, SnappedCoord};
pub use progress::RoutingProgress;

pub use geo::haversine_distance;

pub type RoutingResult<T> = Result<T, RoutingError>;
