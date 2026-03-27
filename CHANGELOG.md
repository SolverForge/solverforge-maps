# Changelog

All notable changes to this project will be documented in this file. See [commit-and-tag-version](https://github.com/absolute-version/commit-and-tag-version) for commit guidelines.

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
