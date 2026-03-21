//! Local OSM road routing using Overpass API.

mod algo;
mod bbox;
mod cache;
mod config;
mod coord;
mod error;
mod fetch;
mod geo;
mod graph;
mod matrix;
mod network;
mod osm;
mod progress;
mod spatial;

pub use bbox::BoundingBox;
pub use cache::{CacheStats, NetworkRef};
pub use config::{ConnectivityPolicy, NetworkConfig, SpeedProfile};
pub use coord::Coord;
pub use error::{BBoxError, CoordError, RoutingError};
pub use matrix::{TravelTimeMatrix, UNREACHABLE};
pub use network::{Objective, RoadNetwork, RouteResult, SnappedCoord};
pub use progress::RoutingProgress;

pub use geo::haversine_distance;

pub type RoutingResult<T> = Result<T, RoutingError>;
