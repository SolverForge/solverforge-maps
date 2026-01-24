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
            let motorway_mps = profile.speed_mps("motorway");
            assert!((motorway_mps - 27.78).abs() < 0.1);
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
                    plot_network(loc.name, &*network_ref, &loc.bbox);
                }
                Err(e) => {
                    println!(" failed: {}", e);
                }
            }
        }
    }
}
