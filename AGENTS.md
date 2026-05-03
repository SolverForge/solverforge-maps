# Repository Guidelines

## Project Structure & Module Organization

`solverforge-maps` is a Rust library crate. Public exports are centralized in `src/lib.rs`, with routing APIs under `src/routing/` and geometry helpers in `src/geometry.rs`. Route loading, caching, matrix generation, graph utilities, snapping, and OSM parsing each live in focused routing modules such as `network.rs`, `matrix.rs`, `graph.rs`, `spatial.rs`, `cache.rs`, and `osm.rs`. Integration coverage lives in `tests/integration.rs`; live external-service checks are in `tests/live_integration.rs`. The runnable demo is `examples/live_network_visualization.rs`. Build and release workflows are in `Makefile` and `.github/workflows/`.

## Build, Test, and Development Commands

- `make build`: compile the crate in debug mode.
- `make test-quick`: run doctests, library tests, and integration tests without live external calls.
- `cargo test --quiet`: run the normal local test suite.
- `make test-live`: run live integration tests with `SOLVERFORGE_RUN_LIVE_TESTS=1`.
- `make lint`: run `cargo fmt --all -- --check` and `cargo clippy -- -D warnings`.
- `make ci-local`: simulate the main local CI path.
- `cargo run --example live_network_visualization`: run the included visualization example.

## Coding Style & Naming Conventions

Use standard Rust 2021 style and keep code `rustfmt` clean. Prefer clear module-local helpers over broad abstractions. Public types use `PascalCase`, functions and modules use `snake_case`, and constants use `SCREAMING_SNAKE_CASE` such as `UNREACHABLE`. Preserve the crate’s fail-fast validation style and explicit error enums rather than silently defaulting invalid routing inputs.

## Testing Guidelines

Add focused tests near the behavior being changed: module tests in `src/routing/*_tests.rs` for internal routing logic, and `tests/integration.rs` for public API contracts. Name tests by observable behavior, for example `matrix_distance_keeps_unreachable_sentinel`. Do not make normal tests depend on network services; put those checks in `tests/live_integration.rs` and run them through `make test-live`.

## Commit & Pull Request Guidelines

Use conventional commits matching the existing history, such as `feat(matrix): expose route distances in travel-time matrix`, `fix(cache): ...`, `refactor(tests): ...`, or `chore(release): ...`. Pull requests should describe the behavior change, list verification commands, link related issues when applicable, and call out live-test or release implications. For release work, keep `Cargo.toml`, `Cargo.lock`, and `CHANGELOG.md` aligned; do not create tags unless the release flow explicitly requires it.

## Security & Configuration Tips

Do not commit generated caches or local artifacts from `.osm_cache/` or `target/`. Live tests may contact external routing or map services, so keep credentials and service-specific configuration outside the repository.
