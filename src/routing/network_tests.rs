use super::{NodeIdx, Objective, RoadNetwork};
use crate::routing::Coord;

#[test]
fn time_routing_uses_non_zero_admissible_heuristic() {
    let nodes = &[(0.0, 0.0), (0.0, 0.01), (0.01, 0.0), (0.01, 0.01)];
    let edges = &[
        (0, 1, 200.0, 1_200.0),
        (1, 3, 200.0, 1_200.0),
        (0, 2, 50.0, 1_200.0),
        (2, 3, 50.0, 1_200.0),
        (1, 0, 200.0, 1_200.0),
        (3, 1, 200.0, 1_200.0),
        (2, 0, 50.0, 1_200.0),
        (3, 2, 50.0, 1_200.0),
    ];
    let network = RoadNetwork::from_test_data(nodes, edges);

    let result = network
        .route(Coord::new(0.0, 0.0), Coord::new(0.01, 0.01))
        .expect("time route should exist");

    assert_eq!(result.duration_seconds, 100);
    assert_eq!(result.distance_meters, 2_400.0);
    assert_eq!(
        result.geometry,
        vec![
            Coord::new(0.0, 0.0),
            Coord::new(0.01, 0.0),
            Coord::new(0.01, 0.01),
        ]
    );
    assert!(network.time_lower_bound_between(NodeIdx(0), NodeIdx(3)) > 0.0);
}

#[test]
fn distance_routing_uses_non_zero_admissible_heuristic() {
    let nodes = &[(0.0, 0.0), (0.0, 0.02), (0.01, 0.0), (0.01, 0.02)];
    let edges = &[
        (0, 1, 40.0, 3_000.0),
        (1, 3, 40.0, 3_000.0),
        (0, 2, 90.0, 900.0),
        (2, 3, 90.0, 900.0),
        (1, 0, 40.0, 3_000.0),
        (3, 1, 40.0, 3_000.0),
        (2, 0, 90.0, 900.0),
        (3, 2, 90.0, 900.0),
    ];
    let network = RoadNetwork::from_test_data(nodes, edges);

    let result = network
        .route_with(
            Coord::new(0.0, 0.0),
            Coord::new(0.01, 0.02),
            Objective::Distance,
        )
        .expect("distance route should exist");

    assert_eq!(result.distance_meters, 1_800.0);
    assert_eq!(result.duration_seconds, 180);
    assert_eq!(
        result.geometry,
        vec![
            Coord::new(0.0, 0.0),
            Coord::new(0.01, 0.0),
            Coord::new(0.01, 0.02),
        ]
    );
    assert!(network.distance_lower_bound_between(NodeIdx(0), NodeIdx(3)) > 0.0);
}
