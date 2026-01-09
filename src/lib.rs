//! Generic map and routing utilities for VRP and similar problems.
//!
//! This crate provides:
//! - OSM road network loading via Overpass API
//! - Shortest path routing with Dijkstra/A*
//! - Travel time matrix computation
//! - Google Polyline encoding/decoding for route visualization
//!
//! # Quick Start
//!
//! ```no_run
//! use solverforge_maps::{BoundingBox, RoadNetwork};
//!
//! # async fn example() -> Result<(), Box<dyn std::error::Error>> {
//! // Define area and load road network
//! let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);
//! let network = RoadNetwork::load_or_fetch(&bbox).await?;
//!
//! // Compute route
//! if let Some(route) = network.route((39.95, -75.16), (39.96, -75.17)) {
//!     println!("Duration: {} seconds", route.duration_seconds);
//! }
//!
//! // Compute travel time matrix
//! let locations = vec![(39.95, -75.16), (39.96, -75.17)];
//! let matrix = network.compute_matrix(&locations);
//! # Ok(())
//! # }
//! ```

pub mod geometry;
pub mod routing;

pub use geometry::{decode_polyline, encode_polyline, EncodedSegment};
pub use routing::{haversine_distance, BoundingBox, RoadNetwork, RouteResult, RoutingError};
