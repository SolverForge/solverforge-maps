//! Opt-in live integration tests for solverforge-maps.
//!
//! These tests hit public network services and mutable upstream map data.
//! Run them explicitly with:
//!
//! `cargo test --test live_integration -- --ignored`

use serde::Deserialize;
use solverforge_maps::{
    decode_polyline, haversine_distance, BoundingBox, Coord, NetworkConfig, RoadNetwork,
};
use textplots::{Chart, Plot, Shape};

fn philadelphia_bbox() -> BoundingBox {
    BoundingBox::new(39.946, -75.174, 39.962, -75.150)
}

fn city_hall() -> Coord {
    Coord::new(39.9526, -75.1635)
}

fn waterfront() -> Coord {
    Coord::new(39.9496, -75.1503)
}

fn market_st_west() -> Coord {
    Coord::new(39.9526, -75.1700)
}

mod verification {
    use super::*;

    /// Verify every point in the route geometry is within a small tolerance of a network node.
    #[tokio::test]
    #[ignore = "requires live Overpass data"]
    async fn geometry_points_lie_on_network() {
        let bbox = philadelphia_bbox();
        let config = NetworkConfig::default();

        let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
            .await
            .expect("Failed to load Philadelphia network");

        let route = network
            .route(city_hall(), waterfront())
            .expect("Failed to compute route");

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
    #[tokio::test]
    #[ignore = "requires live Overpass data"]
    async fn known_straight_segment() {
        let bbox = philadelphia_bbox();
        let config = NetworkConfig::default();

        let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
            .await
            .expect("Failed to load Philadelphia network");

        let start = market_st_west();
        let end = city_hall();

        let route = network.route(start, end).expect("Failed to compute route");
        let straight_line_distance = haversine_distance(start, end);
        let ratio = route.distance_meters / straight_line_distance;

        assert!(
            (0.9..=2.5).contains(&ratio),
            "Route distance ({:.0}m) should be close to straight-line ({:.0}m), ratio: {:.2}",
            route.distance_meters,
            straight_line_distance,
            ratio
        );

        assert!(
            straight_line_distance > 400.0 && straight_line_distance < 700.0,
            "Unexpected straight-line distance: {:.0}m",
            straight_line_distance
        );
    }

    /// Verify that routing from A to B actually reaches B.
    #[tokio::test]
    #[ignore = "requires live Overpass data"]
    async fn roundtrip_snap_route_reaches_destination() {
        let bbox = philadelphia_bbox();
        let config = NetworkConfig::default();

        let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
            .await
            .expect("Failed to load Philadelphia network");

        let start_raw = city_hall();
        let end_raw = waterfront();

        let start_snap = network
            .snap_to_road_detailed(start_raw)
            .expect("Failed to snap start");
        let end_snap = network
            .snap_to_road_detailed(end_raw)
            .expect("Failed to snap end");
        let route = network
            .route(start_raw, end_raw)
            .expect("Failed to compute route");

        assert!(
            route.geometry.len() >= 2,
            "Route geometry should have at least 2 points"
        );

        let route_start = route.geometry.first().unwrap();
        let route_end = route.geometry.last().unwrap();
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
    #[ignore = "requires live Overpass data"]
    async fn route_sanity_checks() {
        let bbox = philadelphia_bbox();
        let config = NetworkConfig::default();

        let network = RoadNetwork::load_or_fetch(&bbox, &config, None)
            .await
            .expect("Failed to load Philadelphia network");

        let waypoint_pairs = [
            (city_hall(), waterfront()),
            (market_st_west(), waterfront()),
            (market_st_west(), city_hall()),
        ];

        for (start, end) in waypoint_pairs {
            let route = network.route(start, end).expect("Failed to compute route");
            let straight_line = haversine_distance(start, end);

            assert!(
                route.distance_meters >= straight_line * 0.99,
                "Route distance ({:.0}m) should be >= straight-line ({:.0}m)",
                route.distance_meters,
                straight_line
            );

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

            for (i, window) in route.geometry.windows(2).enumerate() {
                let gap = haversine_distance(window[0], window[1]);
                assert!(
                    gap < 500.0,
                    "Geometry gap at index {} too large: {:.0}m",
                    i,
                    gap
                );
            }

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

    #[tokio::test]
    #[ignore = "requires live Overpass and OSRM services"]
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

    #[tokio::test]
    #[ignore = "requires live Overpass and OSRM services"]
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

    #[tokio::test]
    #[ignore = "requires live Overpass and OSRM services"]
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
        let sample_step = (route.geometry.len() / 10).max(1);
        for (i, point) in route.geometry.iter().enumerate().step_by(sample_step) {
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

mod visual {
    use super::*;

    struct Location {
        name: &'static str,
        bbox: BoundingBox,
    }

    fn locations() -> Vec<Location> {
        vec![
            Location {
                name: "Philadelphia (City Hall area)",
                bbox: philadelphia_bbox(),
            },
            Location {
                name: "Dragoncello (Poggio Rusco, IT)",
                bbox: BoundingBox::new(44.978, 11.095, 44.986, 11.108),
            },
            Location {
                name: "Clusone (Lombardy, IT)",
                bbox: BoundingBox::new(45.882, 9.940, 45.895, 9.960),
            },
        ]
    }

    fn plot_network(name: &str, network: &RoadNetwork, bbox: &BoundingBox) {
        let nodes: Vec<(f64, f64)> = network.nodes_iter().collect();
        let edges: Vec<(usize, usize, f64, f64)> = network.edges_iter().collect();

        if nodes.is_empty() {
            println!("\n{}: No data", name);
            return;
        }

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
    #[ignore = "requires live Overpass data"]
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
