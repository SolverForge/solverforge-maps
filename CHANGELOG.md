# Changelog

All notable changes to this project will be documented in this file. See [commit-and-tag-version](https://github.com/absolute-version/commit-and-tag-version) for commit guidelines.

## [2.3.2](https://github.com/SolverForge/solverforge-maps/compare/v2.3.1...v2.3.2) (2026-11-06)

### Bug Fixes

* **routing:** `SnapAccess` is re-exported at the crate root as well, not only under `routing`.

## [2.3.1](https://github.com/SolverForge/solverforge-maps/compare/v2.3.0...v2.3.1) (2026-11-06)

### Bug Fixes

* **routing:** a stop whose nearest graph node lies on a one-directional fragment no longer makes every ordering infeasible. `compute_matrix_with_access()` assigns each location the traversal directions its itinerary role requires (`Both` for a round trip's origin and stops, `Outgoing` for a pure start, `Incoming` for a stop the itinerary may finish at, `Incoming` for a fixed end), and snapping prefers the nearest node that satisfies them, keeping the plain nearest node when none does. Plain `compute_matrix()` and `snap_to_road_detailed()` keep their exact nearest behavior; `route()` now treats its start as leaving and its end as arriving.

### Features

* **routing:** `SnapAccess` and `RoadNetwork::snap_road_index_with_access()` expose the access-aware snapping; the road network records, per node, whether an incoming and an outgoing edge exist.

## [2.3.0](https://github.com/SolverForge/solverforge-maps/compare/v2.2.0...v2.3.0) (2026-11-06)

### Features

* **routing:** street names are parsed from OSM ways and carried by graph edges; every route now reports its named street runs ([`streets`](https://docs.rs/solverforge-maps)). Consecutive edges on the same street merge into one run with its distance; unnamed road pieces are left out; partial first and last edges count only the traveled fraction. The road cache moves to version 7, so networks cached under the old rules are rebuilt.

* **network:** `edges_with_names_iter()` exposes edges together with their street names; `from_test_data_named()` builds named synthetic networks for tests.

## [2.2.0](https://github.com/SolverForge/solverforge-maps/compare/v2.1.5...v2.2.0) (2026-11-06)

### Bug Fixes

* **routing:** build a car-legal road graph ([9f040c4](https://github.com/SolverForge/solverforge-maps/commit/9f040c4)).
  * Default roads include the `motorway_link`, `trunk_link`, `primary_link`, `secondary_link` and `tertiary_link` ramps; without them motorways could not be entered or left.
  * Roundabouts (`junction=roundabout`/`circular`) and motorways are one-way unless tagged otherwise; `oneway=true`, `reverse` and `reversible` follow OSM.
  * Roads closed to cars by `motorcar`, `motor_vehicle`, `vehicle` or `access` (most specific wins) are left out; `destination` stays open.
  * Graph nodes are OSM nodes: two OSM nodes at one position, such as a bridge over a road vertex, no longer become a junction.
  * A route whose two points snap to the same node returns that node instead of a straight line between the raw points.
  * The cache version moves to 6, so networks cached under the old rules are rebuilt.

### Refactoring

* **algo:** remove the unused single-metric Dijkstra ([6d97a07](https://github.com/SolverForge/solverforge-maps/commit/6d97a07)).

## [2.1.5](https://github.com/SolverForge/solverforge-maps/compare/v2.1.4...v2.1.5) (2026-11-03)

### Bug Fixes

* **fetch:** identify the crate release in the Overpass User-Agent ([fa35491](https://github.com/SolverForge/solverforge-maps/commit/fa35491)). Requests sent the stale `SolverForge/0.5.0`; they now send `solverforge-maps/<version> (+<repository>)`, as the Overpass usage policy requires.

## [2.1.4](///compare/v2.1.3...v2.1.4) (2026-05-03)


### Features

* **matrix:** expose route distances in travel-time matrix 8fb1c8d

## [2.1.4](https://github.com/SolverForge/solverforge-maps/compare/v2.1.3...v2.1.4) (2026-05-03)

### Features

* expose route distance meters on `TravelTimeMatrix` alongside travel times

## [2.1.3](///compare/v2.1.2...v2.1.3) (2026-04-12)

## [2.1.2](https://github.com/SolverForge/solverforge-maps/compare/v2.1.1...v2.1.2) (2026-04-12)

### Chores

* sync Cargo release metadata so crates.io publication succeeds
* fail release automation when the tag, manifest, and lockfile drift apart

## [2.1.1](///compare/v2.1.0...v2.1.1) (2026-04-03)


### Bug Fixes

* count contention waiters as memory hits 291550a

## [2.1.0](///compare/v2.0.1...v2.1.0) (2026-03-27)


### Features

* expose precise cache-layer metrics a8a6929

## [2.1.0](https://github.com/SolverForge/solverforge-maps/compare/v2.0.1...v2.1.0) (2026-03-26)

### Features

* expose precise cache-layer metrics for `load_or_fetch`
* run live external-service integration checks from local Makefile validation

## [2.0.1](https://github.com/SolverForge/solverforge-maps/compare/v2.0.0...v2.0.1) (2026-03-21)

### Chores

* fix README example formatting and clean up release metadata

## 1.0.0 (2026-01-24)


### Features

* derive Default for Coord, TravelTimeMatrix, SnappedCoord, RTreePoint 87bb9bf
* enhanced RoutingError with detailed variants 53c549d
* import solverforge-maps library f49fb0c
* input validation for Coord and BoundingBox b284e64
* R-tree spatial indexing, type-safe routing, and API improvements 3ab2dd7
* **routing:** implement dynamic speed from OSM maxspeed tag 002a2e6
* zero-erasure API redesign 9821023
