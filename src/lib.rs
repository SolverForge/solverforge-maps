//! Generic map and routing utilities for VRP and similar problems.
//!
//! # Quick Start
//!
//! ```no_run
//! use solverforge_maps::{BoundingBox, Coord, NetworkConfig, RoadNetwork, RoutingResult};
//!
//! #[tokio::main]
//! async fn main() -> RoutingResult<()> {
//!     let locations = vec![
//!         Coord::new(39.95, -75.16),
//!         Coord::new(39.96, -75.17),
//!     ];
//!
//!     let bbox = BoundingBox::from_coords(&locations).expand(0.1);
//!     let config = NetworkConfig::default();
//!
//!     let network = RoadNetwork::load_or_fetch(&bbox, &config, None).await?;
//!     let matrix = network.compute_matrix(&locations, None).await;
//!     let route = network.route(locations[0], locations[1])?;
//!
//!     println!("Matrix size: {}", matrix.size());
//!     println!("Route: {:?}", route);
//!     Ok(())
//! }
//! ```

pub mod geometry;
pub mod routing;

pub use geometry::{decode_polyline, encode_polyline, EncodedSegment};
pub use routing::{
    haversine_distance, BBoxError, BoundingBox, CacheStats, Coord, CoordError, NetworkConfig,
    NetworkRef, Objective, RoadNetwork, RouteResult, RoutingError, RoutingProgress, RoutingResult,
    SnappedCoord, SpeedProfile, TravelTimeMatrix, UNREACHABLE,
};
