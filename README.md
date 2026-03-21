# solverforge-maps

Generic map and routing utilities for Vehicle Routing Problems (VRP) and similar optimization problems.

## Features

- **OSM Road Network**: Download and cache OpenStreetMap road data via Overpass API
- **K-D Tree Spatial Indexing**: Nearest-node and nearest-segment snapping on the road network
- **Shortest Path Routing**: Dijkstra-style shortest paths for time and distance objectives
- **Travel Time Matrix**: Compute all-pairs travel times with parallel computation
- **Route Geometry**: Node-snapped and edge-snapped route geometries with Douglas-Peucker simplification
- **Polyline Encoding**: Google Polyline Algorithm for efficient route transmission
- **Input Validation**: Fail-fast coordinate and bounding box validation
- **Cache Management**: In-memory and file-based caching with inspection and eviction
- **Graph Analysis**: Connectivity analysis for debugging routing failures
- **Zero-Erasure Design**: No `Arc`, no `Box<dyn>`, concrete types throughout

## Installation

```toml
[dependencies]
solverforge-maps = "1.0"
tokio = { version = "1", features = ["full"] }
```

## Quick Start

```rust
use solverforge_maps::{BoundingBox, Coord, NetworkConfig, RoadNetwork, RoutingResult};

#[tokio::main]
async fn main() -> RoutingResult<()> {
    let locations = vec![
        Coord::try_new(39.95, -75.16)?,
        Coord::try_new(39.96, -75.17)?,
    ];

    let bbox = BoundingBox::from_coords(&locations).expand_for_routing(&locations);
    let config = NetworkConfig::default();

    let network = RoadNetwork::load_or_fetch(&bbox, &config, None).await?;
    let matrix = network.compute_matrix(&locations, None).await;
    let route = network.route(locations[0], locations[1])?; // snaps both points to nearest nodes

    println!("Matrix size: {}", matrix.size());
    println!("Route duration: {} seconds", route.duration_seconds);
    Ok(())
}
```

---

## API Reference

### Coord

Geographic coordinate with latitude and longitude. Validates input on construction.

```rust
use solverforge_maps::{Coord, CoordError};

// Use try_new for external or user-provided input
let coord = Coord::try_new(39.95, -75.16)?;

// Fallible construction
let coord: Result<Coord, CoordError> = Coord::try_new(91.0, -75.16);
assert!(matches!(coord, Err(CoordError::LatOutOfRange { .. })));

// Coord::new is still fine for trusted literals that are guaranteed valid
let trusted_coord = Coord::new(39.95, -75.16);

// Distance calculation
let other = Coord::try_new(39.96, -75.17)?;
let distance_meters = coord.distance_to(other);

// Tuple conversion
let from_tuple: Result<Coord, CoordError> = (39.95, -75.16).try_into();
let to_tuple: (f64, f64) = coord.into();
```

#### CoordError

```rust
pub enum CoordError {
    LatOutOfRange { value: f64 },   // lat < -90 or > 90
    LngOutOfRange { value: f64 },   // lng < -180 or > 180
    LatNaN,
    LngNaN,
    LatInfinite { value: f64 },
    LngInfinite { value: f64 },
}
```

---

### BoundingBox

Rectangular geographic region. Validates that min < max on construction.

```rust
use solverforge_maps::{BoundingBox, BBoxError, Coord};

// Use try_new when bounds come from external input
let bbox = BoundingBox::try_new(39.9, -75.2, 40.0, -75.1)?;

// Fallible construction
let bbox: Result<BoundingBox, BBoxError> = BoundingBox::try_new(40.0, -75.2, 39.9, -75.1);
assert!(matches!(bbox, Err(BBoxError::MinLatGreaterThanMax { .. })));

// From coordinates
let locations = vec![
    Coord::try_new(39.95, -75.16)?,
    Coord::try_new(39.96, -75.17)?,
];
let bbox = BoundingBox::from_coords(&locations);

// Expansion methods
let expanded = bbox.expand(0.1);                      // Expand by ratio
let expanded = bbox.expand_meters(1000.0);            // Expand by meters
let expanded = bbox.expand_for_routing(&locations);   // Smart expansion (1.4x detour factor)

// Queries
let center: Coord = bbox.center();
let contains: bool = bbox.contains(Coord::try_new(39.95, -75.15)?);
```

`BoundingBox::new` remains appropriate when the bounds are compile-time literals
or otherwise already validated before construction.

#### BBoxError

```rust
pub enum BBoxError {
    MinLatGreaterThanMax { min: f64, max: f64 },
    MinLngGreaterThanMax { min: f64, max: f64 },
    LatOutOfRange { value: f64 },
    LngOutOfRange { value: f64 },
    NaN { field: &'static str },
    Infinite { field: &'static str, value: f64 },
}
```

---

### NetworkConfig

Configuration for network loading and routing.

```rust
use solverforge_maps::{NetworkConfig, SpeedProfile};
use std::time::Duration;

// Defaults
let config = NetworkConfig::default();

// Builder pattern
let config = NetworkConfig::new()
    .overpass_url("https://overpass-api.de/api/interpreter")
    .cache_dir("/tmp/osm_cache")
    .connect_timeout(Duration::from_secs(30))
    .read_timeout(Duration::from_secs(120))
    .speed_profile(SpeedProfile::default());
```

---

### SpeedProfile

Speed profiles for different road types.

```rust
use solverforge_maps::SpeedProfile;

let profile = SpeedProfile::default();

// Get speed in meters per second (maxspeed tag, highway type)
let speed_mps = profile.speed_mps(None, "motorway");       // ~27.78 m/s (100 km/h default)
let speed_mps = profile.speed_mps(Some("50"), "motorway"); // ~13.89 m/s (50 km/h from tag)
```

| Highway Type | Default Speed |
|-------------|---------------|
| motorway | 100 km/h |
| trunk | 80 km/h |
| primary | 60 km/h |
| secondary | 50 km/h |
| tertiary | 40 km/h |
| residential | 30 km/h |
| unclassified | 30 km/h |
| service | 20 km/h |
| living_street | 10 km/h |

---

### RoadNetwork

Core routing engine built from OSM data.

#### Loading a Network

```rust
use solverforge_maps::{BoundingBox, NetworkConfig, RoadNetwork, NetworkRef};

let bbox = BoundingBox::try_new(39.9, -75.2, 40.0, -75.1)?;
let config = NetworkConfig::default();

// Load from cache or fetch from API (returns cached reference)
let network: NetworkRef = RoadNetwork::load_or_fetch(&bbox, &config, None).await?;

// Always fetch fresh (bypasses cache)
let network: RoadNetwork = RoadNetwork::fetch(&bbox, &config, None).await?;
```

`NetworkRef` is a zero-cost guard that provides access to the cached `RoadNetwork`. It implements `Deref<Target = RoadNetwork>` so all methods are available directly.

#### Routing

`route` and `route_with` snap both endpoints to the nearest graph nodes before
searching. They currently call the shared `astar` implementation with a zero
heuristic, so the public search behavior is equivalent to Dijkstra's algorithm.
If you need geometry that starts and ends on the containing road segments rather
than at snapped nodes, use `snap_to_edge` with `route_edge_snapped`.

```rust
use solverforge_maps::{Coord, Objective, RouteResult, RoutingError};

let from = Coord::try_new(39.95, -75.16)?;
let to = Coord::try_new(39.96, -75.17)?;

// Route by minimum travel time (default). Endpoints are snapped to nearest nodes.
let route: Result<RouteResult, RoutingError> = network.route(from, to);

// Route with specific objective. Public search still uses a zero heuristic today.
let route = network.route_with(from, to, Objective::Time)?;     // Minimize time
let route = network.route_with(from, to, Objective::Distance)?; // Minimize distance

// Access route data
println!("Duration: {} seconds", route.duration_seconds);
println!("Distance: {:.0} meters", route.distance_meters);
println!("Geometry: {} points", route.geometry.len());

// Simplify geometry for transmission (Douglas-Peucker algorithm)
let simplified = route.simplify(10.0);  // tolerance in meters
```

#### Edge-Snapped Routing

Use edge snapping when you want the returned geometry to begin and end on the
nearest road segments instead of the nearest graph nodes.

```rust
use solverforge_maps::{Coord, RouteResult, RoutingError};

let from = Coord::try_new(39.95, -75.16)?;
let to = Coord::try_new(39.96, -75.17)?;

let from_edge = network.snap_to_edge(from)?;
let to_edge = network.snap_to_edge(to)?;
let route: Result<RouteResult, RoutingError> = network.route_edge_snapped(&from_edge, &to_edge);
```

#### Coordinate Snapping

```rust
use solverforge_maps::{Coord, SnappedCoord, RoutingError};

let coord = Coord::try_new(39.95, -75.16)?;

// Simple snap (returns None if network is empty)
let node = network.snap_to_road(coord);

// Detailed snap with distance information
let snapped: Result<SnappedCoord, RoutingError> = network.snap_to_road_detailed(coord);

if let Ok(snap) = snapped {
    println!("Original: {:?}", snap.original);
    println!("Snapped: {:?}", snap.snapped);
    println!("Snap distance: {:.1} meters", snap.snap_distance_m);
}

// Route between pre-snapped node locations (more efficient for repeated routing)
let from_snap = network.snap_to_road_detailed(from)?;
let to_snap = network.snap_to_road_detailed(to)?;
let route = network.route_snapped(&from_snap, &to_snap)?;
```

#### Network Statistics

```rust
let nodes: usize = network.node_count();
let edges: usize = network.edge_count();

// Connectivity analysis (useful for debugging routing failures)
let components: usize = network.strongly_connected_components();
let largest_fraction: f64 = network.largest_component_fraction();
let is_connected: bool = network.is_strongly_connected();

println!("Network has {} SCCs", components);
println!("Largest component contains {:.1}% of nodes", largest_fraction * 100.0);
```

---

### TravelTimeMatrix

Travel time matrix with metadata and analysis methods.

```rust
use solverforge_maps::{Coord, TravelTimeMatrix, UNREACHABLE};

let locations = vec![
    Coord::try_new(39.95, -75.16)?,
    Coord::try_new(39.96, -75.17)?,
    Coord::try_new(39.94, -75.15)?,
];

// Compute matrix (async, parallel via rayon internally)
let matrix: TravelTimeMatrix = network.compute_matrix(&locations, None).await;

// Access travel times
let time: Option<i64> = matrix.get(0, 1);           // From location 0 to 1
let row: Option<&[i64]> = matrix.row(0);            // All times from location 0
let reachable: bool = matrix.is_reachable(0, 1);    // Check if pair is reachable

// Matrix metadata
let size: usize = matrix.size();                     // Number of locations
let locations: &[SnappedCoord] = matrix.locations(); // Snapped coordinates

// Statistics (excluding diagonal and unreachable pairs)
let min: Option<i64> = matrix.min();
let max: Option<i64> = matrix.max();
let mean: Option<f64> = matrix.mean();

// Reachability analysis
let ratio: f64 = matrix.reachability_ratio();                 // Fraction reachable
let unreachable: Vec<(usize, usize)> = matrix.unreachable_pairs();

// Raw data access
let data: &[i64] = matrix.as_slice();  // Row-major flat array
```

The constant `UNREACHABLE` (`i64::MAX`) indicates pairs with no path.

---

### Route Geometries

Compute geometries for all location pairs.

```rust
use solverforge_maps::Coord;
use std::collections::HashMap;

let locations = vec![
    Coord::new(39.95, -75.16),
    Coord::new(39.96, -75.17),
];

// Async computation
let geometries: HashMap<(usize, usize), Vec<Coord>> =
    network.compute_geometries(&locations, None).await;

// Access specific route geometry
if let Some(route_geom) = geometries.get(&(0, 1)) {
    println!("Route 0->1 has {} points", route_geom.len());
}
```

---

### Cache Management

Inspect and manage the network cache.

```rust
use solverforge_maps::{RoadNetwork, CacheStats, BoundingBox};

// Get cache statistics
let stats: CacheStats = RoadNetwork::cache_stats().await;
println!("Cached networks: {}", stats.networks_cached);
println!("Total nodes: {}", stats.total_nodes);
println!("Total edges: {}", stats.total_edges);
println!("Memory: {} bytes", stats.memory_bytes);
println!("Hits: {}, Misses: {}", stats.hits, stats.misses);

// List cached regions
let regions: Vec<BoundingBox> = RoadNetwork::cached_regions().await;

// Evict specific region
let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);
let evicted: bool = RoadNetwork::evict(&bbox).await;

// Clear entire cache
RoadNetwork::clear_cache().await;
```

---

### Progress Reporting

Stream progress updates during long operations.

```rust
use solverforge_maps::{BoundingBox, NetworkConfig, RoadNetwork, RoutingProgress};
use tokio::sync::mpsc;

let (tx, mut rx) = mpsc::channel::<RoutingProgress>(100);

tokio::spawn(async move {
    while let Some(progress) = rx.recv().await {
        match progress {
            RoutingProgress::CheckingCache { percent } => {
                println!("[{:3}%] Checking cache...", percent);
            }
            RoutingProgress::DownloadingNetwork { percent, bytes } => {
                println!("[{:3}%] Downloading... {} bytes", percent, bytes);
            }
            RoutingProgress::BuildingGraph { percent } => {
                println!("[{:3}%] Building graph...", percent);
            }
            RoutingProgress::ComputingMatrix { percent, row, total } => {
                println!("[{:3}%] Computing matrix row {}/{}", percent, row, total);
            }
            RoutingProgress::Complete => {
                println!("[100%] Done");
            }
            _ => {}
        }
    }
});

let network = RoadNetwork::load_or_fetch(&bbox, &config, Some(&tx)).await?;
let matrix = network.compute_matrix(&locations, Some(&tx)).await;
```

#### RoutingProgress Variants

| Variant | Description |
|---------|-------------|
| `CheckingCache { percent }` | Checking in-memory and file caches |
| `DownloadingNetwork { percent, bytes }` | Downloading from Overpass API |
| `ParsingOsm { percent, nodes, edges }` | Parsing OSM JSON response |
| `BuildingGraph { percent }` | Building routing graph |
| `ComputingMatrix { percent, row, total }` | Computing travel time matrix |
| `ComputingGeometries { percent, pair, total }` | Computing route geometries |
| `Complete` | Operation finished |

---

### Polyline Encoding

Encode/decode route geometries using [Google Polyline Algorithm](https://developers.google.com/maps/documentation/utilities/polylinealgorithm).

```rust
use solverforge_maps::{encode_polyline, decode_polyline, Coord};

let coords = vec![
    Coord::new(38.5, -120.2),
    Coord::new(40.7, -120.95),
    Coord::new(43.252, -126.453),
];

let encoded: String = encode_polyline(&coords);
let decoded: Vec<Coord> = decode_polyline(&encoded);
```

---

### Haversine Distance

Calculate great-circle distance between two coordinates.

```rust
use solverforge_maps::{haversine_distance, Coord};

let a = Coord::new(39.9526, -75.1635);
let b = Coord::new(39.9496, -75.1503);

let distance_meters = haversine_distance(a, b);

// Or use the method on Coord
let distance_meters = a.distance_to(b);
```

---

### Error Handling

```rust
use solverforge_maps::{RoutingError, RoutingResult};

fn example() -> RoutingResult<()> {
    Ok(())
}

match network.route(from, to) {
    Ok(route) => { /* use route */ }
    Err(RoutingError::SnapFailed { coord, nearest_distance_m }) => {
        eprintln!("Could not snap {:?} to road network", coord);
        if let Some(dist) = nearest_distance_m {
            eprintln!("Nearest node was {} meters away", dist);
        }
    }
    Err(RoutingError::NoPath { from, to }) => {
        eprintln!("No path exists from {:?} to {:?}", from, to);
    }
    Err(RoutingError::InvalidCoordinate { error }) => {
        eprintln!("Invalid coordinate: {}", error);
    }
    Err(RoutingError::Network(msg)) => {
        eprintln!("Network error: {}", msg);
    }
    Err(RoutingError::Parse(msg)) => {
        eprintln!("Parse error: {}", msg);
    }
    Err(RoutingError::Io(e)) => {
        eprintln!("I/O error: {}", e);
    }
    Err(RoutingError::Cancelled) => {
        eprintln!("Operation cancelled");
    }
}
```

---

## Types Summary

### Structs

| Type | Description |
|------|-------------|
| `Coord` | Geographic coordinate (lat, lng) |
| `BoundingBox` | Rectangular geographic region |
| `NetworkConfig` | Configuration for network loading |
| `SpeedProfile` | Road type speed mapping |
| `RoadNetwork` | Core routing graph |
| `NetworkRef` | Zero-cost reference to cached network |
| `RouteResult` | Single route with geometry |
| `SnappedCoord` | Coordinate snapped to road network |
| `TravelTimeMatrix` | N x N travel time matrix |
| `CacheStats` | Cache statistics and metrics |
| `EncodedSegment` | Pre-encoded route segment |

### Enums

| Type | Description |
|------|-------------|
| `CoordError` | Coordinate validation errors |
| `BBoxError` | Bounding box validation errors |
| `RoutingError` | Routing operation errors |
| `RoutingProgress` | Progress reporting events |
| `Objective` | Routing objective (Time, Distance) |

### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `UNREACHABLE` | `i64::MAX` | Sentinel for unreachable pairs |

---

## Caching

Three-tier caching for performance:

1. **In-Memory Cache**: Instant lookup for repeated requests
2. **File Cache**: Persists to `cache_dir` (default `.osm_cache/`)
3. **Overpass API**: Downloads fresh data on cache miss

```bash
# Clear file cache
rm -rf .osm_cache/
```

```rust
// Or programmatically
RoadNetwork::clear_cache().await;
```

---

## License

Apache-2.0
