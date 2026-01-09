# solverforge-maps

Generic map and routing utilities for Vehicle Routing Problems (VRP) and similar optimization problems.

## Features

- **OSM Road Network**: Download and cache OpenStreetMap road data via Overpass API
- **Shortest Path Routing**: A*/Dijkstra routing with travel times and distances
- **Travel Time Matrix**: Compute all-pairs travel times for location sets
- **Route Geometry**: Full road-following geometries for visualization
- **Polyline Encoding**: Google Polyline Algorithm for efficient route transmission
- **Three-Tier Caching**: In-memory, file-based, and API caching for performance

## Installation

```toml
[dependencies]
solverforge-maps = "0.1"
tokio = { version = "1", features = ["full"] }
```

## Quick Start

```rust
use solverforge_maps::{BoundingBox, RoadNetwork};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Define area of interest (Philadelphia)
    let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);

    // Load road network (downloads from OSM if not cached)
    let network = RoadNetwork::load_or_fetch(&bbox).await?;

    println!("Loaded {} nodes, {} edges", network.node_count(), network.edge_count());

    // Compute route between two points
    let from = (39.95, -75.16);
    let to = (39.96, -75.17);

    if let Some(route) = network.route(from, to) {
        println!("Duration: {} seconds", route.duration_seconds);
        println!("Distance: {:.0} meters", route.distance_meters);
        println!("Geometry: {} points", route.geometry.len());
    }

    Ok(())
}
```

## API Reference

### Road Network

#### Creating a Network

```rust
use solverforge_maps::{BoundingBox, RoadNetwork, RoutingError};
use std::sync::Arc;

// Create empty network (for testing)
let network = RoadNetwork::new();

// Load from OSM with caching (recommended)
let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);
let network: Arc<RoadNetwork> = RoadNetwork::load_or_fetch(&bbox).await?;

// Download fresh from OSM (no caching)
let network: RoadNetwork = RoadNetwork::from_bbox(&bbox).await?;
```

#### Single Route

```rust
use solverforge_maps::RouteResult;

let from = (39.95, -75.16);  // (latitude, longitude)
let to = (39.96, -75.17);

if let Some(route) = network.route(from, to) {
    // Travel time in seconds
    let duration: i64 = route.duration_seconds;

    // Distance in meters
    let distance: f64 = route.distance_meters;

    // Full geometry as (lat, lng) pairs
    let geometry: Vec<(f64, f64)> = route.geometry;
}
```

#### Travel Time Matrix

Compute all-pairs travel times for a set of locations:

```rust
let locations = vec![
    (39.95, -75.16),  // Depot
    (39.96, -75.17),  // Customer 1
    (39.94, -75.15),  // Customer 2
];

// Simple computation
let matrix: Vec<Vec<i64>> = network.compute_matrix(&locations);
// matrix[i][j] = travel time in seconds from location i to j

// With progress callback (for large matrices)
let matrix = network.compute_matrix_with_progress(&locations, |row, total| {
    println!("Computing row {}/{}", row + 1, total);
});
```

#### Route Geometries

Get full road-following geometries for all location pairs:

```rust
use std::collections::HashMap;

let locations = vec![
    (39.95, -75.16),
    (39.96, -75.17),
    (39.94, -75.15),
];

// Simple computation
let geometries: HashMap<(usize, usize), Vec<(f64, f64)>> =
    network.compute_all_geometries(&locations);

// geometries[(0, 1)] = route geometry from location 0 to location 1

// With progress callback
let geometries = network.compute_all_geometries_with_progress(&locations, |row, total| {
    println!("Computing geometries for location {}/{}", row + 1, total);
});
```

#### Network Info

```rust
let nodes: usize = network.node_count();
let edges: usize = network.edge_count();
```

### Bounding Box

```rust
use solverforge_maps::BoundingBox;

// Create bounding box
let bbox = BoundingBox::new(
    39.9,   // min_lat
    -75.2,  // min_lng
    40.0,   // max_lat
    -75.1,  // max_lng
);

// Expand by 10% on each side (useful for edge cases)
let expanded = bbox.expand(0.1);
```

### Polyline Encoding

Encode/decode route geometries using [Google Polyline Algorithm](https://developers.google.com/maps/documentation/utilities/polylinealgorithm):

```rust
use solverforge_maps::{encode_polyline, decode_polyline};

// Encode coordinates to compact string
let coords = vec![(38.5, -120.2), (40.7, -120.95), (43.252, -126.453)];
let encoded: String = encode_polyline(&coords);
// encoded = "_p~iF~ps|U_ulLnnqC_mqNvxq`@"

// Decode back to coordinates
let decoded: Vec<(f64, f64)> = decode_polyline(&encoded);
// Precision: 5 decimal places (~1 meter)
```

### Haversine Distance

Calculate great-circle distance between two points:

```rust
use solverforge_maps::routing::haversine_distance;

let dist_meters = haversine_distance(
    39.9526, -75.1635,  // Point 1 (lat, lng)
    39.9496, -75.1503,  // Point 2 (lat, lng)
);
```

### Error Handling

```rust
use solverforge_maps::RoutingError;

match RoadNetwork::load_or_fetch(&bbox).await {
    Ok(network) => { /* use network */ }
    Err(RoutingError::Network(msg)) => {
        eprintln!("Network error: {}", msg);
    }
    Err(RoutingError::Parse(msg)) => {
        eprintln!("Failed to parse OSM data: {}", msg);
    }
    Err(RoutingError::Io(e)) => {
        eprintln!("I/O error: {}", e);
    }
    Err(RoutingError::NoRoute) => {
        eprintln!("No route found");
    }
}
```

## Caching

The library uses three-tier caching for performance:

1. **In-Memory Cache**: Instant lookup for repeated requests in the same process
2. **File Cache**: Persists to `.osm_cache/` directory, survives restarts
3. **Overpass API**: Downloads fresh data when cache misses (~5-30 seconds)

Cache files are automatically invalidated when the format version changes.

To clear the cache:

```bash
rm -rf .osm_cache/
```

## Highway Speed Defaults

Travel times are computed using these default speeds:

| Highway Type | Speed |
|-------------|-------|
| motorway | 100 km/h |
| trunk | 80 km/h |
| primary | 60 km/h |
| secondary | 50 km/h |
| tertiary | 40 km/h |
| residential | 30 km/h |
| unclassified | 30 km/h |
| service | 20 km/h |
| living_street | 10 km/h |

## Types Reference

### RouteResult

```rust
pub struct RouteResult {
    /// Travel time in seconds
    pub duration_seconds: i64,
    /// Distance in meters
    pub distance_meters: f64,
    /// Full route geometry as (lat, lng) pairs
    pub geometry: Vec<(f64, f64)>,
}
```

### BoundingBox

```rust
pub struct BoundingBox {
    pub min_lat: f64,
    pub min_lng: f64,
    pub max_lat: f64,
    pub max_lng: f64,
}
```

### RoutingError

```rust
pub enum RoutingError {
    Network(String),      // HTTP/network failure
    Parse(String),        // Invalid OSM data
    Io(std::io::Error),   // File system error
    NoRoute,              // No path exists
}
```

### EncodedSegment

For API responses with encoded polylines:

```rust
use solverforge_maps::geometry::EncodedSegment;

let segment = EncodedSegment::new(
    0,                    // entity_idx (e.g., vehicle index)
    "Vehicle A",          // entity_name
    &route.geometry,      // coordinates
);

// segment.polyline contains the encoded string
// segment.point_count contains the number of points
```

## License

Apache-2.0
