# solverforge-maps

Generic map and routing utilities for Vehicle Routing Problems (VRP) and similar optimization problems.

## Features

- **OSM Road Network**: Download and cache OpenStreetMap road data via Overpass API
- **Shortest Path Routing**: A*/Dijkstra routing with travel times and distances
- **Travel Time Matrix**: Compute all-pairs travel times for location sets
- **Route Geometry**: Full road-following geometries for visualization
- **Polyline Encoding**: Google Polyline Algorithm for efficient route transmission
- **Three-Tier Caching**: In-memory, file-based, and API caching for performance
- **Zero-Erasure Design**: No `Arc`, no `Box<dyn>`, concrete types throughout

## Installation

```toml
[dependencies]
solverforge-maps = "0.1"
tokio = { version = "1", features = ["full"] }
```

## Quick Start

```rust
use solverforge_maps::{BoundingBox, Coord, NetworkConfig, RoadNetwork, RoutingResult};

#[tokio::main]
async fn main() -> RoutingResult<()> {
    let locations = vec![
        Coord::new(39.95, -75.16),
        Coord::new(39.96, -75.17),
    ];

    let bbox = BoundingBox::from_coords(&locations).expand(0.1);
    let config = NetworkConfig::default();

    let network = RoadNetwork::load_or_fetch(&bbox, &config, None).await?;
    let matrix = network.compute_matrix(&locations, None).await;
    let route = network.route(locations[0], locations[1]);

    println!("Matrix: {:?}", matrix);
    println!("Route: {:?}", route);
    Ok(())
}
```

## API Reference

### Coord

Geographic coordinate with latitude and longitude.

```rust
use solverforge_maps::Coord;

let coord = Coord::new(39.95, -75.16);
let other = Coord::new(39.96, -75.17);

let distance_meters = coord.distance_to(other);

let from_tuple: Coord = (39.95, -75.16).into();
```

### BoundingBox

Rectangular geographic region.

```rust
use solverforge_maps::{BoundingBox, Coord};

let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);

let locations = vec![
    Coord::new(39.95, -75.16),
    Coord::new(39.96, -75.17),
];
let bbox = BoundingBox::from_coords(&locations);

let expanded = bbox.expand(0.1);

let center = bbox.center();

let contains = bbox.contains(Coord::new(39.95, -75.15));
```

### NetworkConfig

Configuration for network loading and routing.

```rust
use solverforge_maps::{NetworkConfig, SpeedProfile};
use std::path::PathBuf;
use std::time::Duration;

let config = NetworkConfig::default();

let config = NetworkConfig::new()
    .overpass_url("https://overpass-api.de/api/interpreter")
    .cache_dir(PathBuf::from(".osm_cache"))
    .connect_timeout(Duration::from_secs(30))
    .read_timeout(Duration::from_secs(120))
    .speed_profile(SpeedProfile::Default)
    .highway_types(vec!["motorway", "trunk", "primary", "secondary", "tertiary", "residential"]);
```

### SpeedProfile

Speed profiles for different road types.

```rust
use solverforge_maps::SpeedProfile;

let profile = SpeedProfile::Default;

let profile = SpeedProfile::Custom {
    motorway_kmh: 110.0,
    trunk_kmh: 90.0,
    primary_kmh: 70.0,
    secondary_kmh: 60.0,
    tertiary_kmh: 50.0,
    residential_kmh: 35.0,
    service_kmh: 25.0,
    default_kmh: 30.0,
};

let speed_mps = profile.speed_mps("motorway");
```

### RoadNetwork

Core routing engine built from OSM data.

#### Loading a Network

```rust
use solverforge_maps::{BoundingBox, NetworkConfig, RoadNetwork, NetworkRef};

let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);
let config = NetworkConfig::default();

let network: NetworkRef = RoadNetwork::load_or_fetch(&bbox, &config, None).await?;

let network: RoadNetwork = RoadNetwork::fetch(&bbox, &config, None).await?;
```

`NetworkRef` is a zero-cost guard that provides access to the cached `RoadNetwork`. It implements `Deref<Target = RoadNetwork>` so all `RoadNetwork` methods are available directly.

#### Single Route

```rust
use solverforge_maps::{Coord, RouteResult};

let from = Coord::new(39.95, -75.16);
let to = Coord::new(39.96, -75.17);

if let Some(route) = network.route(from, to) {
    println!("Duration: {} seconds", route.duration_seconds);
    println!("Distance: {:.0} meters", route.distance_meters);
    println!("Geometry: {} points", route.geometry.len());
}
```

#### Travel Time Matrix

```rust
use solverforge_maps::{Coord, TravelTimeMatrix};

let locations = vec![
    Coord::new(39.95, -75.16),
    Coord::new(39.96, -75.17),
    Coord::new(39.94, -75.15),
];

let matrix: TravelTimeMatrix = network.compute_matrix(&locations, None).await;

let matrix: TravelTimeMatrix = network.compute_matrix_sync(&locations);
```

`TravelTimeMatrix` is `Vec<Vec<i64>>` where `matrix[i][j]` is travel time in seconds from location `i` to location `j`. Returns `-1` for unreachable pairs.

#### Route Geometries

```rust
use solverforge_maps::Coord;
use std::collections::HashMap;

let locations = vec![
    Coord::new(39.95, -75.16),
    Coord::new(39.96, -75.17),
];

let geometries: HashMap<(usize, usize), Vec<Coord>> =
    network.compute_geometries(&locations, None).await;

let route_0_to_1: &Vec<Coord> = &geometries[&(0, 1)];
```

#### Network Statistics

```rust
let nodes: usize = network.node_count();
let edges: usize = network.edge_count();
```

### Progress Reporting

Stream progress updates during long operations.

```rust
use solverforge_maps::{BoundingBox, Coord, NetworkConfig, RoadNetwork, RoutingProgress};
use tokio::sync::mpsc;

let (tx, mut rx) = mpsc::channel::<RoutingProgress>(100);

tokio::spawn(async move {
    while let Some(progress) = rx.recv().await {
        let (phase, message) = progress.phase_message();
        println!("[{:3}%] {}: {}", progress.percent(), phase, message);
        if let Some(detail) = progress.detail() {
            println!("       {}", detail);
        }
    }
});

let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);
let config = NetworkConfig::default();

let network = RoadNetwork::load_or_fetch(&bbox, &config, Some(&tx)).await?;

let locations = vec![
    Coord::new(39.95, -75.16),
    Coord::new(39.96, -75.17),
];
let matrix = network.compute_matrix(&locations, Some(&tx)).await;
```

#### RoutingProgress Phases

| Phase | Description |
|-------|-------------|
| `CheckingCache` | Checking in-memory and file caches |
| `DownloadingNetwork` | Downloading from Overpass API |
| `ParsingOsm` | Parsing OSM JSON response |
| `BuildingGraph` | Building routing graph |
| `ComputingMatrix` | Computing travel time matrix |
| `ComputingGeometries` | Computing route geometries |
| `EncodingGeometries` | Encoding geometries to polylines |
| `Complete` | Operation finished |

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

### EncodedSegment

Pre-encoded route segment for API responses.

```rust
use solverforge_maps::{EncodedSegment, Coord};

let coords = vec![
    Coord::new(39.95, -75.16),
    Coord::new(39.96, -75.17),
];

let segment = EncodedSegment::new(0, "Vehicle A", &coords);

println!("Entity: {} (idx {})", segment.entity_name, segment.entity_idx);
println!("Polyline: {}", segment.polyline);
println!("Points: {}", segment.point_count);
```

### Haversine Distance

Calculate great-circle distance between two coordinates.

```rust
use solverforge_maps::{haversine_distance, Coord};

let a = Coord::new(39.9526, -75.1635);
let b = Coord::new(39.9496, -75.1503);

let distance_meters = haversine_distance(a, b);

let distance_meters = a.distance_to(b);
```

### Error Handling

```rust
use solverforge_maps::{RoutingError, RoutingResult};

fn example() -> RoutingResult<()> {
    // ... operations that may fail
    Ok(())
}

match RoadNetwork::load_or_fetch(&bbox, &config, None).await {
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

## Types Reference

### Core Types

```rust
pub struct Coord {
    pub lat: f64,
    pub lng: f64,
}

pub struct BoundingBox {
    pub min_lat: f64,
    pub min_lng: f64,
    pub max_lat: f64,
    pub max_lng: f64,
}

pub struct NetworkConfig {
    pub overpass_url: String,
    pub cache_dir: PathBuf,
    pub connect_timeout: Duration,
    pub read_timeout: Duration,
    pub speed_profile: SpeedProfile,
    pub highway_types: Vec<&'static str>,
}

pub struct RoadNetwork { /* routing graph */ }

pub struct NetworkRef { /* zero-cost cache guard */ }

pub struct RouteResult {
    pub duration_seconds: i64,
    pub distance_meters: f64,
    pub geometry: Vec<Coord>,
}

pub struct EncodedSegment {
    pub entity_idx: usize,
    pub entity_name: String,
    pub polyline: String,
    pub point_count: usize,
}
```

### Enums

```rust
pub enum SpeedProfile {
    Default,
    Custom {
        motorway_kmh: f64,
        trunk_kmh: f64,
        primary_kmh: f64,
        secondary_kmh: f64,
        tertiary_kmh: f64,
        residential_kmh: f64,
        service_kmh: f64,
        default_kmh: f64,
    },
}

pub enum RoutingProgress {
    CheckingCache { percent: u8 },
    DownloadingNetwork { percent: u8, bytes: usize },
    ParsingOsm { percent: u8, nodes: usize, edges: usize },
    BuildingGraph { percent: u8 },
    ComputingMatrix { percent: u8, row: usize, total: usize },
    ComputingGeometries { percent: u8, pair: usize, total: usize },
    EncodingGeometries { percent: u8 },
    Complete,
}

pub enum RoutingError {
    Network(String),
    Parse(String),
    Io(std::io::Error),
    NoRoute,
}
```

### Type Aliases

```rust
pub type RoutingResult<T> = Result<T, RoutingError>;
pub type TravelTimeMatrix = Vec<Vec<i64>>;
```

## Caching

Three-tier caching for performance:

1. **In-Memory Cache**: Instant lookup for repeated requests in the same process
2. **File Cache**: Persists to configured `cache_dir` (default `.osm_cache/`), survives restarts
3. **Overpass API**: Downloads fresh data when cache misses

Cache files are automatically invalidated when the format version changes.

```bash
rm -rf .osm_cache/
```

## Default Highway Speeds

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

Use `SpeedProfile::Custom` to override these defaults.

## License

Apache-2.0
