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

pub use bbox::BoundingBox;
pub use cache::NetworkRef;
pub use config::{NetworkConfig, SpeedProfile};
pub use coord::Coord;
pub use error::RoutingError;
pub use matrix::TravelTimeMatrix;
pub use network::{RoadNetwork, RouteResult};
pub use progress::RoutingProgress;

pub use geo::haversine_distance;

pub type RoutingResult<T> = Result<T, RoutingError>;
