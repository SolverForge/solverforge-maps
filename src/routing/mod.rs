//! Local OSM road routing using Overpass API and petgraph.
//!
//! Downloads OpenStreetMap road network data via Overpass API,
//! builds a graph locally, and computes shortest paths with Dijkstra.
//! Results are cached in memory (per-process) and `.osm_cache/` (persistent).

mod bbox;
mod cache;
mod error;
mod fetch;
mod geo;
mod matrix;
mod network;
mod osm;

pub use bbox::BoundingBox;
pub use error::RoutingError;
pub use geo::haversine_distance;
pub use network::{RoadNetwork, RouteResult};
