//! Consolidated integration tests for solverforge-maps.

use std::path::PathBuf;
use std::time::Duration;

use solverforge_maps::{
    decode_polyline, encode_polyline, haversine_distance, BBoxError, BoundingBox, Coord,
    CoordError, NetworkConfig, RoadNetwork, RouteResult, RoutingError, SpeedProfile, UNREACHABLE,
};

mod types {
    use super::*;

    mod coord {
        use super::*;

        #[test]
        fn new() {
            let coord = Coord::new(39.95, -75.16);
            assert!((coord.lat - 39.95).abs() < f64::EPSILON);
            assert!((coord.lng - (-75.16)).abs() < f64::EPSILON);
        }

        #[test]
        fn try_new_valid() {
            let coord = Coord::try_new(39.95, -75.16).unwrap();
            assert!((coord.lat - 39.95).abs() < f64::EPSILON);
            assert!((coord.lng - (-75.16)).abs() < f64::EPSILON);
        }

        #[test]
        fn try_new_invalid_lat() {
            let result = Coord::try_new(91.0, -75.16);
            assert!(matches!(result, Err(CoordError::LatOutOfRange { .. })));
        }

        #[test]
        fn try_new_invalid_lng() {
            let result = Coord::try_new(39.95, 181.0);
            assert!(matches!(result, Err(CoordError::LngOutOfRange { .. })));
        }

        #[test]
        fn try_new_nan() {
            let result = Coord::try_new(f64::NAN, -75.16);
            assert!(matches!(result, Err(CoordError::LatNaN)));
        }

        #[test]
        fn try_new_infinite() {
            let result = Coord::try_new(f64::INFINITY, -75.16);
            assert!(matches!(result, Err(CoordError::LatInfinite { .. })));
        }
    }

    mod bbox {
        use super::*;

        #[test]
        fn expand() {
            let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);
            let expanded = bbox.expand(0.1);
            assert!(expanded.min_lat < bbox.min_lat);
            assert!(expanded.max_lat > bbox.max_lat);
        }

        #[test]
        fn expand_meters() {
            let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);
            let expanded = bbox.expand_meters(1000.0);
            assert!(expanded.min_lat < bbox.min_lat);
            assert!(expanded.max_lat > bbox.max_lat);
        }

        #[test]
        fn expand_for_routing() {
            let locations = vec![Coord::new(39.95, -75.16), Coord::new(39.96, -75.17)];
            let bbox = BoundingBox::from_coords(&locations);
            let expanded = bbox.expand_for_routing(&locations);
            assert!(expanded.min_lat < bbox.min_lat);
        }

        #[test]
        fn from_coords() {
            let coords = vec![
                Coord::new(39.95, -75.16),
                Coord::new(39.96, -75.17),
                Coord::new(39.94, -75.15),
            ];
            let bbox = BoundingBox::from_coords(&coords);
            assert!((bbox.min_lat - 39.94).abs() < f64::EPSILON);
            assert!((bbox.max_lat - 39.96).abs() < f64::EPSILON);
            assert!((bbox.min_lng - (-75.17)).abs() < f64::EPSILON);
            assert!((bbox.max_lng - (-75.15)).abs() < f64::EPSILON);
        }

        #[test]
        fn contains() {
            let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);
            assert!(bbox.contains(Coord::new(39.95, -75.15)));
            assert!(!bbox.contains(Coord::new(40.1, -75.15)));
        }

        #[test]
        fn center() {
            let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.0);
            let center = bbox.center();
            assert!((center.lat - 39.95).abs() < f64::EPSILON);
            assert!((center.lng - (-75.1)).abs() < f64::EPSILON);
        }

        #[test]
        fn try_new_valid() {
            let bbox = BoundingBox::try_new(39.9, -75.2, 40.0, -75.1);
            assert!(bbox.is_ok());
        }

        #[test]
        fn try_new_min_greater_than_max() {
            let bbox = BoundingBox::try_new(40.0, -75.2, 39.9, -75.1);
            assert!(matches!(bbox, Err(BBoxError::MinLatGreaterThanMax { .. })));
        }
    }

    mod config {
        use super::*;

        #[test]
        fn default_config() {
            let config = NetworkConfig::default();
            assert_eq!(
                config.overpass_url,
                "https://overpass-api.de/api/interpreter"
            );
            assert_eq!(config.cache_dir, PathBuf::from(".osm_cache"));
        }

        #[test]
        fn builder_pattern() {
            let config = NetworkConfig::new()
                .overpass_url("https://custom.api/interpreter")
                .cache_dir("/tmp/cache")
                .connect_timeout(Duration::from_secs(60));

            assert_eq!(config.overpass_url, "https://custom.api/interpreter");
            assert_eq!(config.cache_dir, PathBuf::from("/tmp/cache"));
            assert_eq!(config.connect_timeout, Duration::from_secs(60));
        }

        #[test]
        fn speed_profile() {
            let profile = SpeedProfile::default();
            let motorway_mps = profile.speed_mps(None, "motorway");
            assert!((motorway_mps - 27.78).abs() < 0.1);

            let maxspeed_mps = profile.speed_mps(Some("50"), "motorway");
            assert!((maxspeed_mps - 13.889).abs() < 0.1);
        }
    }
}

mod geo {
    use super::*;

    #[test]
    fn haversine() {
        let a = Coord::new(39.9526, -75.1635);
        let b = Coord::new(39.9496, -75.1503);
        let dist = haversine_distance(a, b);
        assert!((dist - 1200.0).abs() < 100.0);
    }

    #[test]
    fn haversine_same_point() {
        let a = Coord::new(39.95, -75.16);
        let dist = haversine_distance(a, a);
        assert!(dist.abs() < f64::EPSILON);
    }

    #[test]
    fn encode_decode_roundtrip() {
        let coords = vec![
            Coord::new(38.5, -120.2),
            Coord::new(40.7, -120.95),
            Coord::new(43.252, -126.453),
        ];
        let encoded = encode_polyline(&coords);
        let decoded = decode_polyline(&encoded);

        assert_eq!(decoded.len(), coords.len());
        for (orig, dec) in coords.iter().zip(decoded.iter()) {
            assert!((orig.lat - dec.lat).abs() < 0.00001);
            assert!((orig.lng - dec.lng).abs() < 0.00001);
        }
    }

    #[test]
    fn empty_coords() {
        let encoded = encode_polyline(&[]);
        assert!(encoded.is_empty());
        let decoded = decode_polyline("");
        assert!(decoded.is_empty());
    }
}

mod routing {
    use super::*;

    mod network {
        use super::*;

        #[test]
        fn empty_network() {
            let network = RoadNetwork::new();
            assert_eq!(network.node_count(), 0);
            assert_eq!(network.edge_count(), 0);
        }

        #[test]
        fn snap_to_road_empty() {
            let network = RoadNetwork::new();
            assert!(network.snap_to_road(Coord::new(39.95, -75.16)).is_none());
        }

        #[test]
        fn snap_to_road_detailed_empty() {
            let network = RoadNetwork::new();
            let result = network.snap_to_road_detailed(Coord::new(39.95, -75.16));
            assert!(matches!(result, Err(RoutingError::SnapFailed { .. })));
        }

        #[test]
        fn route_empty_network() {
            let network = RoadNetwork::new();
            let result = network.route(Coord::new(39.95, -75.16), Coord::new(39.96, -75.17));
            assert!(matches!(result, Err(RoutingError::SnapFailed { .. })));
        }

        #[test]
        fn connectivity_empty() {
            let network = RoadNetwork::new();
            assert_eq!(network.strongly_connected_components(), 0);
            assert!((network.largest_component_fraction() - 0.0).abs() < f64::EPSILON);
        }
    }

    mod route_simplify {
        use super::*;

        #[test]
        fn simplify_short_route() {
            let route = RouteResult {
                duration_seconds: 100,
                distance_meters: 1000.0,
                geometry: vec![Coord::new(39.95, -75.16), Coord::new(39.96, -75.17)],
            };

            let simplified = route.simplify(10.0);
            assert_eq!(simplified.geometry.len(), 2);
        }

        #[test]
        fn simplify_preserves_endpoints() {
            let first = Coord::new(39.95, -75.16);
            let last = Coord::new(39.96, -75.17);
            let route = RouteResult {
                duration_seconds: 100,
                distance_meters: 1000.0,
                geometry: vec![first, Coord::new(39.955, -75.165), last],
            };

            let simplified = route.simplify(10000.0);
            assert!(simplified.geometry.len() >= 2);
            assert_eq!(simplified.geometry[0], first);
            assert_eq!(simplified.geometry[simplified.geometry.len() - 1], last);
        }
    }

    mod verification {
        use super::*;

        /// Philadelphia bounding box (cached data exists)
        fn philadelphia_bbox() -> BoundingBox {
            BoundingBox::new(39.946, -75.174, 39.962, -75.150)
        }

        /// City Hall area
        fn city_hall() -> Coord {
            Coord::new(39.9526, -75.1635)
        }

        /// Waterfront area
        fn waterfront() -> Coord {
            Coord::new(39.9496, -75.1503)
        }

        /// Market Street West (for straight segment test)
        fn market_st_west() -> Coord {
            Coord::new(39.9526, -75.1700)
        }

        /// Verify every point in the route geometry is within a small tolerance of a network node.
        #[tokio::test]
        async fn geometry_points_lie_on_network() {
            let bbox = philadelphia_bbox();
            let config = NetworkConfig::default();

            let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
                .await
                .expect("Failed to load Philadelphia network");

            let route = network
                .route(city_hall(), waterfront())
                .expect("Failed to compute route");

            // Every geometry point should snap to within 50m of a network node
            let snap_threshold_m = 50.0;
            for (i, point) in route.geometry.iter().enumerate() {
                let snap_result = network.snap_to_road_detailed(*point);
                assert!(
                    snap_result.is_ok(),
                    "Geometry point {} ({}, {}) failed to snap",
                    i,
                    point.lat,
                    point.lng
                );
                let snap = snap_result.unwrap();
                assert!(
                    snap.snap_distance_m < snap_threshold_m,
                    "Geometry point {} is {}m from network (threshold: {}m)",
                    i,
                    snap.snap_distance_m,
                    snap_threshold_m
                );
            }
        }

        /// Use a known straight road segment where distance is predictable.
        /// Market Street in Philadelphia is mostly straight east-west.
        #[tokio::test]
        async fn known_straight_segment() {
            let bbox = philadelphia_bbox();
            let config = NetworkConfig::default();

            let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
                .await
                .expect("Failed to load Philadelphia network");

            // Two points on Market Street, ~500m apart
            let start = market_st_west();
            let end = city_hall();

            let route = network.route(start, end).expect("Failed to compute route");

            let straight_line_distance = haversine_distance(start, end);

            // Route distance should be within reasonable bounds of straight-line distance
            // Urban areas with one-way streets may require up to 2x detours
            let ratio = route.distance_meters / straight_line_distance;
            assert!(
                (0.9..=2.5).contains(&ratio),
                "Route distance ({:.0}m) should be close to straight-line ({:.0}m), ratio: {:.2}",
                route.distance_meters,
                straight_line_distance,
                ratio
            );

            // Sanity: straight-line is ~550m, route should be in reasonable range
            assert!(
                straight_line_distance > 400.0 && straight_line_distance < 700.0,
                "Unexpected straight-line distance: {:.0}m",
                straight_line_distance
            );
        }

        /// Verify that routing from A to B actually reaches B.
        #[tokio::test]
        async fn roundtrip_snap_route_reaches_destination() {
            let bbox = philadelphia_bbox();
            let config = NetworkConfig::default();

            let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
                .await
                .expect("Failed to load Philadelphia network");

            let start_raw = city_hall();
            let end_raw = waterfront();

            // Snap both points
            let start_snap = network
                .snap_to_road_detailed(start_raw)
                .expect("Failed to snap start");
            let end_snap = network
                .snap_to_road_detailed(end_raw)
                .expect("Failed to snap end");

            // Compute route
            let route = network
                .route(start_raw, end_raw)
                .expect("Failed to compute route");

            assert!(
                route.geometry.len() >= 2,
                "Route geometry should have at least 2 points"
            );

            let route_start = route.geometry.first().unwrap();
            let route_end = route.geometry.last().unwrap();

            // Route endpoints should be within 50m of snapped points
            let endpoint_threshold_m = 50.0;

            let start_distance = haversine_distance(*route_start, start_snap.snapped);
            assert!(
                start_distance < endpoint_threshold_m,
                "Route start ({}, {}) is {}m from snapped start ({}, {}), threshold: {}m",
                route_start.lat,
                route_start.lng,
                start_distance,
                start_snap.snapped.lat,
                start_snap.snapped.lng,
                endpoint_threshold_m
            );

            let end_distance = haversine_distance(*route_end, end_snap.snapped);
            assert!(
                end_distance < endpoint_threshold_m,
                "Route end ({}, {}) is {}m from snapped end ({}, {}), threshold: {}m",
                route_end.lat,
                route_end.lng,
                end_distance,
                end_snap.snapped.lat,
                end_snap.snapped.lng,
                endpoint_threshold_m
            );
        }

        /// Multiple sanity assertions: distance bounds, speed range, geometry continuity.
        #[tokio::test]
        async fn route_sanity_checks() {
            let bbox = philadelphia_bbox();
            let config = NetworkConfig::default();

            let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
                .await
                .expect("Failed to load Philadelphia network");

            // Test multiple routes
            let waypoint_pairs = [
                (city_hall(), waterfront()),
                (market_st_west(), waterfront()),
                (market_st_west(), city_hall()),
            ];

            for (start, end) in waypoint_pairs {
                let route = network.route(start, end).expect("Failed to compute route");

                let straight_line = haversine_distance(start, end);

                // 4a: Route distance >= straight-line distance (with small tolerance)
                assert!(
                    route.distance_meters >= straight_line * 0.99,
                    "Route distance ({:.0}m) should be >= straight-line ({:.0}m)",
                    route.distance_meters,
                    straight_line
                );

                // 4b: Duration proportional to distance (reasonable speed range)
                let speed_mps = route.distance_meters / route.duration_seconds as f64;
                assert!(
                    speed_mps > 1.0,
                    "Speed ({:.1} m/s = {:.1} km/h) too slow",
                    speed_mps,
                    speed_mps * 3.6
                );
                assert!(
                    speed_mps < 35.0,
                    "Speed ({:.1} m/s = {:.1} km/h) too fast",
                    speed_mps,
                    speed_mps * 3.6
                );

                // 4c: Geometry is continuous (no teleportation)
                for (i, window) in route.geometry.windows(2).enumerate() {
                    let gap = haversine_distance(window[0], window[1]);
                    assert!(
                        gap < 500.0,
                        "Geometry gap at index {} too large: {:.0}m",
                        i,
                        gap
                    );
                }

                // 4d: Geometry distance ≈ reported distance
                let geom_distance: f64 = route
                    .geometry
                    .windows(2)
                    .map(|w| haversine_distance(w[0], w[1]))
                    .sum();
                let ratio = geom_distance / route.distance_meters;
                assert!(
                    ratio > 0.9 && ratio < 1.1,
                    "Geometry distance ({:.0}m) should match reported ({:.0}m), ratio: {:.2}",
                    geom_distance,
                    route.distance_meters,
                    ratio
                );
            }
        }
    }

    mod osrm_comparison {
        use super::*;
        use serde::Deserialize;

        /// Philadelphia bounding box (cached data exists)
        fn philadelphia_bbox() -> BoundingBox {
            BoundingBox::new(39.946, -75.174, 39.962, -75.150)
        }

        /// City Hall area
        fn city_hall() -> Coord {
            Coord::new(39.9526, -75.1635)
        }

        /// Waterfront area
        fn waterfront() -> Coord {
            Coord::new(39.9496, -75.1503)
        }

        #[derive(Deserialize)]
        struct OsrmResponse {
            code: String,
            routes: Vec<OsrmRoute>,
        }

        #[derive(Deserialize)]
        struct OsrmRoute {
            distance: f64,
            duration: f64,
            geometry: String,
        }

        async fn query_osrm(from: Coord, to: Coord) -> Option<OsrmRoute> {
            let url = format!(
                "https://router.project-osrm.org/route/v1/driving/{},{};{},{}?overview=full&geometries=polyline",
                from.lng, from.lat, to.lng, to.lat
            );
            let resp: OsrmResponse = reqwest::get(&url).await.ok()?.json().await.ok()?;
            if resp.code == "Ok" {
                resp.routes.into_iter().next()
            } else {
                None
            }
        }

        /// Compare route distances between solverforge and OSRM.
        /// Tolerance: ±25% (accounts for different graph construction, speed profiles,
        /// and road selection algorithms)
        #[tokio::test]
        async fn distance_matches_osrm() {
            let bbox = philadelphia_bbox();
            let config = NetworkConfig::default();

            let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
                .await
                .expect("Failed to load Philadelphia network");

            let start = city_hall();
            let end = waterfront();

            let route = network.route(start, end).expect("Failed to compute route");

            let Some(osrm_route) = query_osrm(start, end).await else {
                eprintln!("OSRM unavailable, skipping test");
                return;
            };

            let ratio = route.distance_meters / osrm_route.distance;
            let tolerance = 0.25;

            assert!(
                (ratio - 1.0).abs() <= tolerance,
                "Distance mismatch: solverforge={:.0}m, OSRM={:.0}m, ratio={:.2} (tolerance: ±{:.0}%)",
                route.distance_meters,
                osrm_route.distance,
                ratio,
                tolerance * 100.0
            );
        }

        /// Compare travel times between solverforge and OSRM.
        /// Tolerance: ±35% (OSRM uses different speed profiles, traffic data,
        /// and turn costs)
        #[tokio::test]
        async fn duration_matches_osrm() {
            let bbox = philadelphia_bbox();
            let config = NetworkConfig::default();

            let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
                .await
                .expect("Failed to load Philadelphia network");

            let start = city_hall();
            let end = waterfront();

            let route = network.route(start, end).expect("Failed to compute route");

            let Some(osrm_route) = query_osrm(start, end).await else {
                eprintln!("OSRM unavailable, skipping test");
                return;
            };

            let ratio = route.duration_seconds as f64 / osrm_route.duration;
            let tolerance = 0.35;

            assert!(
                (ratio - 1.0).abs() <= tolerance,
                "Duration mismatch: solverforge={:.0}s, OSRM={:.0}s, ratio={:.2} (tolerance: ±{:.0}%)",
                route.duration_seconds,
                osrm_route.duration,
                ratio,
                tolerance * 100.0
            );
        }

        /// Verify route geometries follow similar paths.
        /// Tolerance: 250m deviation allowed (different snapping, road selection,
        /// and one-way street handling)
        #[tokio::test]
        async fn geometry_similar_to_osrm() {
            let bbox = philadelphia_bbox();
            let config = NetworkConfig::default();

            let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
                .await
                .expect("Failed to load Philadelphia network");

            let start = city_hall();
            let end = waterfront();

            let route = network.route(start, end).expect("Failed to compute route");

            let Some(osrm_route) = query_osrm(start, end).await else {
                eprintln!("OSRM unavailable, skipping test");
                return;
            };

            let osrm_geometry = decode_polyline(&osrm_route.geometry);
            if osrm_geometry.is_empty() {
                eprintln!("OSRM returned empty geometry, skipping test");
                return;
            }

            let max_deviation_m = 250.0;

            // Sample points along solverforge route and check distance to OSRM route
            let sample_step = (route.geometry.len() / 10).max(1);
            for (i, point) in route.geometry.iter().enumerate().step_by(sample_step) {
                // Find nearest point on OSRM route
                let min_distance = osrm_geometry
                    .iter()
                    .map(|osrm_point| haversine_distance(*point, *osrm_point))
                    .fold(f64::INFINITY, f64::min);

                assert!(
                    min_distance < max_deviation_m,
                    "Geometry deviation at point {}: {:.0}m from OSRM route (threshold: {:.0}m)",
                    i,
                    min_distance,
                    max_deviation_m
                );
            }
        }
    }
}

mod matrix {
    use super::*;

    #[tokio::test]
    async fn empty() {
        let network = RoadNetwork::new();
        let locations: Vec<Coord> = vec![];
        let matrix = network.compute_matrix(&locations, None).await;
        assert_eq!(matrix.size(), 0);
    }

    #[tokio::test]
    async fn single_location() {
        let network = RoadNetwork::new();
        let locations = vec![Coord::new(39.95, -75.16)];
        let matrix = network.compute_matrix(&locations, None).await;
        assert_eq!(matrix.size(), 1);
        assert_eq!(matrix.get(0, 0), Some(0));
    }

    #[tokio::test]
    async fn two_locations() {
        let network = RoadNetwork::new();
        let locations = vec![Coord::new(39.95, -75.16), Coord::new(39.96, -75.17)];
        let matrix = network.compute_matrix(&locations, None).await;
        assert_eq!(matrix.size(), 2);
        assert_eq!(matrix.get(0, 0), Some(0));
        assert_eq!(matrix.get(1, 1), Some(0));
        // Unreachable since network is empty
        assert_eq!(matrix.get(0, 1), Some(UNREACHABLE));
        assert_eq!(matrix.get(1, 0), Some(UNREACHABLE));
    }

    #[tokio::test]
    async fn matrix_methods() {
        let network = RoadNetwork::new();
        let locations = vec![Coord::new(39.95, -75.16)];
        let matrix = network.compute_matrix(&locations, None).await;

        assert_eq!(matrix.size(), 1);
        assert!(matrix.row(0).is_some());
        assert!(matrix.row(1).is_none());
        assert!(matrix.locations().len() == 1);
    }
}

mod visual {
    use solverforge_maps::{BoundingBox, NetworkConfig, RoadNetwork};
    use textplots::{Chart, Plot, Shape};

    struct Location {
        name: &'static str,
        bbox: BoundingBox,
    }

    fn locations() -> Vec<Location> {
        vec![
            Location {
                name: "Philadelphia (City Hall area)",
                bbox: BoundingBox::new(39.946, -75.174, 39.962, -75.150),
            },
            Location {
                name: "Dragoncello (Poggio Rusco, IT)",
                // Small hamlet in Lombardy
                bbox: BoundingBox::new(44.978, 11.095, 44.986, 11.108),
            },
            Location {
                name: "Clusone (Lombardy, IT)",
                // Historic town in the Alps
                bbox: BoundingBox::new(45.882, 9.940, 45.895, 9.960),
            },
        ]
    }

    fn plot_network(name: &str, network: &RoadNetwork, bbox: &BoundingBox) {
        // Collect nodes and edges from the network
        let nodes: Vec<(f64, f64)> = network.nodes_iter().collect();
        let edges: Vec<(usize, usize, f64, f64)> = network.edges_iter().collect();

        if nodes.is_empty() {
            println!("\n{}: No data", name);
            return;
        }

        // Collect line segments for plotting
        let mut segments: Vec<(f32, f32)> = Vec::new();
        let mut seen = std::collections::HashSet::new();
        for &(from, to, _, _) in &edges {
            let key = (from.min(to), from.max(to));
            if seen.insert(key) && from < nodes.len() && to < nodes.len() {
                let (lat1, lng1) = nodes[from];
                let (lat2, lng2) = nodes[to];
                segments.push((lng1 as f32, lat1 as f32));
                segments.push((lng2 as f32, lat2 as f32));
                segments.push((f32::NAN, f32::NAN));
            }
        }

        // Intersection points
        let intersections: Vec<(f32, f32)> = nodes
            .iter()
            .map(|(lat, lng)| (*lng as f32, *lat as f32))
            .collect();

        println!("\n{}", name);
        println!(
            "{} nodes, {} edges",
            network.node_count(),
            network.edge_count()
        );

        let x_min = bbox.min_lng as f32;
        let x_max = bbox.max_lng as f32;

        Chart::new(180, 60, x_min, x_max)
            .lineplot(&Shape::Lines(&segments))
            .lineplot(&Shape::Points(&intersections))
            .nice();
    }

    #[tokio::test]
    async fn road_network_visualization() {
        let config = NetworkConfig::default();

        for loc in locations() {
            print!("\nFetching {}...", loc.name);
            match RoadNetwork::load_or_fetch(&loc.bbox, &config, None).await {
                Ok(network_ref) => {
                    println!(" done");
                    plot_network(loc.name, &network_ref, &loc.bbox);
                }
                Err(e) => {
                    println!(" failed: {}", e);
                }
            }
        }
    }
}
