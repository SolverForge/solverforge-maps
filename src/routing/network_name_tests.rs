//! Street-name behavior: per-edge names survive into merged route runs.

use super::RoadNetwork;
use crate::routing::Coord;
use crate::routing::StreetRun;

#[test]
fn route_streets_merge_consecutive_same_name_edges() {
    let nodes = &[(0.0, 0.0), (0.0, 0.01), (0.0, 0.02), (0.0, 0.03)];
    let mut network = RoadNetwork::new();
    for &(lat, lng) in nodes {
        network.add_node_at(lat, lng);
    }
    // The first two edges share one street; the third does not.
    network.add_edge_named_by_index(0, 1, 60.0, 1000.0, Some("Corso Como".into()));
    network.add_edge_named_by_index(1, 2, 60.0, 1000.0, Some("Corso Como".into()));
    network.add_edge_named_by_index(2, 3, 60.0, 1000.0, Some("Via Sarpri".into()));
    network.build_spatial_index();

    let result = network
        .route(Coord::new(0.0, 0.0), Coord::new(0.0, 0.03))
        .expect("route through three edges should exist");
    assert_eq!(
        result.streets,
        vec![
            StreetRun {
                name: "Corso Como".into(),
                distance_m: 2000.0
            },
            StreetRun {
                name: "Via Sarpri".into(),
                distance_m: 1000.0
            },
        ]
    );
}

#[test]
fn route_streets_skip_unnamed_edges() {
    let mut network = RoadNetwork::new();
    for &(lat, lng) in &[(0.0, 0.0), (0.0, 0.01), (0.0, 0.02)] {
        network.add_node_at(lat, lng);
    }
    network.add_edge_named_by_index(0, 1, 60.0, 500.0, None);
    network.add_edge_named_by_index(1, 2, 60.0, 1000.0, Some("Via Sarpri".into()));
    network.build_spatial_index();

    let result = network
        .route(Coord::new(0.0, 0.0), Coord::new(0.0, 0.02))
        .expect("route over an unnamed edge should exist");
    // The unnamed first piece contributes nothing; only the named street is listed.
    assert_eq!(
        result.streets,
        vec![StreetRun {
            name: "Via Sarpri".into(),
            distance_m: 1000.0
        }]
    );
}

#[test]
fn route_streets_count_partial_edges() {
    let mut network = RoadNetwork::new();
    for &(lat, lng) in &[(0.0, 0.0), (0.0, 0.01)] {
        network.add_node_at(lat, lng);
    }
    network.add_edge_named_by_index(0, 1, 60.0, 2000.0, Some("Strada Lunga".into()));
    network.build_spatial_index();

    // Both points snap onto the same named edge; the traveled fraction of its
    // street is the run's distance, in meters.
    let result = network
        .route(Coord::new(0.0, 0.0), Coord::new(0.0, 0.01))
        .expect("partial-edge route should exist");
    let total = result.streets.iter().map(|run| run.distance_m).sum::<f64>();
    assert!(
        (total - result.distance_meters).abs() < 0.5,
        "{:?}",
        result.streets
    );
    assert_eq!(
        result.streets,
        vec![StreetRun {
            name: "Strada Lunga".into(),
            distance_m: result.distance_meters
        }]
    );
}
