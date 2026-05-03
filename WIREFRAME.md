# SolverForge Maps Wireframe

## Purpose

This crate provides reusable map, geometry, road-network, and routing primitives for SolverForge planning applications. It is a library-first repository, so the "wireframe" is the expected developer-facing surface and data flow rather than a visual UI.

## Public Surface

```text
consumer crate
  |
  v
solverforge_maps::{Coord, BoundingBox, NetworkConfig, RoadNetwork}
  |
  +-- geometry helpers
  |     +-- haversine_distance
  |     +-- encode_polyline / decode_polyline
  |
  +-- routing results
        +-- RouteResult
        +-- TravelTimeMatrix
        +-- SnappedCoord
        +-- RoutingProgress
```

## Routing Flow

```text
input coordinates
  |
  v
BoundingBox::from_coords(...).expand(...)
  |
  v
RoadNetwork::load_or_fetch(...)
  |
  +-- cache lookup in .osm_cache/
  +-- fetch OSM data when needed
  +-- parse and index network graph
  |
  v
route, route_with, route_snapped, or compute_matrix
  |
  v
RouteResult or TravelTimeMatrix
```

## TravelTimeMatrix Surface

`TravelTimeMatrix` is an `N x N` matrix over snapped input locations.

- `get(from, to)` returns fastest travel time seconds.
- `distance_meters(from, to)` returns meters along that same fastest-time path.
- `row(i)` and `row_distances(i)` expose per-origin slices.
- `UNREACHABLE` marks unreachable pairs; diagonal cells stay `0`.

## Repository Layout

```text
src/lib.rs                 public exports
src/geometry.rs            polyline and geometry helpers
src/routing/               graph, OSM, cache, snapping, routes, matrix
tests/integration.rs       public API contract tests
tests/live_integration.rs  external-service checks
examples/                  runnable examples
README.md                  user-facing guide
AGENTS.md                  contributor guide
CHANGELOG.md               release notes
```

## Verification Paths

Normal local validation should use `make lint` and `cargo test --quiet`. Live routing-service validation is separate and runs through `make test-live`.
