//! Integration tests for the routing module.

use std::path::PathBuf;
use std::time::Duration;

use solverforge_maps::{
    decode_polyline, encode_polyline, haversine_distance, BoundingBox, Coord, NetworkConfig,
    RoadNetwork, SpeedProfile,
};

mod coord {
    use super::*;

    #[test]
    fn new() {
        let coord = Coord::new(39.95, -75.16);
        assert!((coord.lat - 39.95).abs() < f64::EPSILON);
        assert!((coord.lng - (-75.16)).abs() < f64::EPSILON);
    }

    #[test]
    fn from_tuple() {
        let coord: Coord = (39.95, -75.16).into();
        assert!((coord.lat - 39.95).abs() < f64::EPSILON);
        assert!((coord.lng - (-75.16)).abs() < f64::EPSILON);
    }

    #[test]
    fn to_tuple() {
        let coord = Coord::new(39.95, -75.16);
        let tuple: (f64, f64) = coord.into();
        assert!((tuple.0 - 39.95).abs() < f64::EPSILON);
        assert!((tuple.1 - (-75.16)).abs() < f64::EPSILON);
    }

    #[test]
    fn distance() {
        let a = Coord::new(39.9526, -75.1635);
        let b = Coord::new(39.9496, -75.1503);
        let dist = a.distance_to(b);
        assert!((dist - 1200.0).abs() < 100.0);
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
}

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
}

mod matrix {
    use super::*;

    #[test]
    fn empty() {
        let network = RoadNetwork::new();
        let locations: Vec<Coord> = vec![];
        let matrix = network.compute_matrix_sync(&locations);
        assert!(matrix.is_empty());
    }

    #[test]
    fn single_location() {
        let network = RoadNetwork::new();
        let locations = vec![Coord::new(39.95, -75.16)];
        let matrix = network.compute_matrix_sync(&locations);
        assert_eq!(matrix.len(), 1);
        assert_eq!(matrix[0][0], 0);
    }

    #[test]
    fn two_locations() {
        let network = RoadNetwork::new();
        let locations = vec![Coord::new(39.95, -75.16), Coord::new(39.96, -75.17)];
        let matrix = network.compute_matrix_sync(&locations);
        assert_eq!(matrix.len(), 2);
        assert_eq!(matrix[0][0], 0);
        assert_eq!(matrix[1][1], 0);
        assert!(matrix[0][1] > 0);
        assert!(matrix[1][0] > 0);
    }
}

mod geometry {
    use super::*;

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
    fn known_encoding() {
        let coords = vec![
            Coord::new(38.5, -120.2),
            Coord::new(40.7, -120.95),
            Coord::new(43.252, -126.453),
        ];
        let encoded = encode_polyline(&coords);
        assert!(!encoded.is_empty());
        let decoded = decode_polyline(&encoded);
        assert_eq!(decoded.len(), 3);
    }

    #[test]
    fn empty_coords() {
        let encoded = encode_polyline(&[]);
        assert!(encoded.is_empty());
        let decoded = decode_polyline("");
        assert!(decoded.is_empty());
    }

    #[test]
    fn single_point() {
        let coords = vec![Coord::new(0.0, 0.0)];
        let encoded = encode_polyline(&coords);
        let decoded = decode_polyline(&encoded);
        assert_eq!(decoded.len(), 1);
        assert!(decoded[0].lat.abs() < 0.00001);
        assert!(decoded[0].lng.abs() < 0.00001);
    }
}
