//! Consolidated integration tests for solverforge-maps.

use std::path::PathBuf;
use std::time::Duration;

use solverforge_maps::{
    decode_polyline, encode_polyline, haversine_distance, BBoxError, BoundingBox,
    ConnectivityPolicy, Coord, CoordError, NetworkConfig, RoadNetwork, RouteResult, RoutingError,
    SpeedProfile, UNREACHABLE,
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
                .connect_timeout(Duration::from_secs(60))
                .connectivity_policy(ConnectivityPolicy::LargestStronglyConnectedComponent);

            assert_eq!(config.overpass_url, "https://custom.api/interpreter");
            assert_eq!(config.cache_dir, PathBuf::from("/tmp/cache"));
            assert_eq!(config.connect_timeout, Duration::from_secs(60));
            assert_eq!(
                config.connectivity_policy,
                ConnectivityPolicy::LargestStronglyConnectedComponent
            );
        }

        #[test]
        fn speed_profile() {
            let profile = SpeedProfile::default();
            let motorway_mps = profile.speed_mps(None, "motorway");
            assert!((motorway_mps - 27.78).abs() < 0.1);

            let maxspeed_mps = profile.speed_mps(Some("50"), "motorway");
            assert!((maxspeed_mps - 13.889).abs() < 0.1);
        }

        #[test]
        fn default_connectivity_policy_keeps_all_components() {
            let config = NetworkConfig::default();
            assert_eq!(config.connectivity_policy, ConnectivityPolicy::KeepAll);
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

        #[test]
        fn largest_scc_filter_is_opt_in() {
            let mut network = RoadNetwork::from_test_data(
                &[(0.0, 0.0), (0.0, 1.0), (10.0, 10.0), (10.0, 11.0)],
                &[
                    (0, 1, 10.0, 100.0),
                    (1, 0, 10.0, 100.0),
                    (2, 3, 10.0, 100.0),
                ],
            );

            assert_eq!(network.node_count(), 4);
            assert_eq!(network.strongly_connected_components(), 3);

            network.filter_to_largest_scc();

            assert_eq!(network.node_count(), 2);
            assert_eq!(network.edge_count(), 2);
            assert_eq!(network.strongly_connected_components(), 1);
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

    mod edge_snapped {
        use super::*;

        #[test]
        fn same_edge_forward_uses_directed_partial_distance() {
            let network =
                RoadNetwork::from_test_data(&[(0.0, 0.0), (0.0, 1.0)], &[(0, 1, 100.0, 1000.0)]);
            let from = network
                .snap_to_edge(Coord::new(0.0, 0.2))
                .expect("Failed to snap start");
            let to = network
                .snap_to_edge(Coord::new(0.0, 0.8))
                .expect("Failed to snap end");

            let route = network
                .route_edge_snapped(&from, &to)
                .expect("Expected forward travel on same edge");

            assert_eq!(route.duration_seconds, 60);
            assert!((route.distance_meters - 600.0).abs() < 1e-6);
            assert_eq!(route.geometry, vec![from.snapped, to.snapped]);
        }

        #[test]
        fn same_edge_reverse_returns_no_path() {
            let network =
                RoadNetwork::from_test_data(&[(0.0, 0.0), (0.0, 1.0)], &[(0, 1, 100.0, 1000.0)]);
            let from = network
                .snap_to_edge(Coord::new(0.0, 0.8))
                .expect("Failed to snap start");
            let to = network
                .snap_to_edge(Coord::new(0.0, 0.2))
                .expect("Failed to snap end");

            let result = network.route_edge_snapped(&from, &to);
            assert!(matches!(result, Err(RoutingError::NoPath { .. })));
        }

        #[test]
        fn reverse_only_edge_respects_edge_orientation() {
            let network =
                RoadNetwork::from_test_data(&[(0.0, 0.0), (0.0, 1.0)], &[(1, 0, 100.0, 1000.0)]);
            let from = network
                .snap_to_edge(Coord::new(0.0, 0.8))
                .expect("Failed to snap start");
            let to = network
                .snap_to_edge(Coord::new(0.0, 0.2))
                .expect("Failed to snap end");

            let route = network
                .route_edge_snapped(&from, &to)
                .expect("Expected travel in reverse edge direction");

            assert_eq!(route.duration_seconds, 60);
            assert!((route.distance_meters - 600.0).abs() < 1e-6);
            assert_eq!(route.geometry, vec![from.snapped, to.snapped]);
        }

        #[test]
        fn cross_edge_route_counts_selected_partials() {
            let network = RoadNetwork::from_test_data(
                &[(0.0, 0.0), (0.0, 1.0), (0.0, 2.0)],
                &[(0, 1, 100.0, 1000.0), (1, 2, 200.0, 2000.0)],
            );
            let from = network
                .snap_to_edge(Coord::new(0.0, 0.25))
                .expect("Failed to snap start");
            let to = network
                .snap_to_edge(Coord::new(0.0, 1.5))
                .expect("Failed to snap end");

            let route = network
                .route_edge_snapped(&from, &to)
                .expect("Expected route across adjacent directed edges");

            assert_eq!(route.duration_seconds, 175);
            assert!((route.distance_meters - 1750.0).abs() < 1e-6);
            assert_eq!(
                route.geometry,
                vec![from.snapped, Coord::new(0.0, 1.0), to.snapped]
            );
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

    #[tokio::test]
    async fn matrix_matches_single_route_node_snapping() {
        let nodes = &[(0.0, 0.0), (0.0, 1.0), (0.0, 2.0)];
        let edges = &[
            (0, 1, 100.0, 1000.0),
            (1, 0, 100.0, 1000.0),
            (1, 2, 100.0, 1000.0),
            (2, 1, 100.0, 1000.0),
        ];
        let network = RoadNetwork::from_test_data(nodes, edges);

        let start = Coord::new(0.0, 0.4);
        let end = Coord::new(0.0, 1.6);
        let locations = vec![start, end];

        let matrix = network.compute_matrix(&locations, None).await;
        let route = network.route(start, end).expect("Failed to compute route");

        assert_eq!(matrix.get(0, 1), Some(route.duration_seconds));
        assert_eq!(matrix.locations()[0].snapped, Coord::new(0.0, 0.0));
        assert_eq!(matrix.locations()[1].snapped, Coord::new(0.0, 2.0));
    }
}
