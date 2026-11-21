use super::*;
use crate::routing::SnapAccess;

// A triangle of two-way roads: 0 - 1 is a primary road, 1 - 2 residential,
// 0 - 2 a motorway detour that is slower than the primary + residential route
// until the city streets slow down.
fn network() -> RoadNetwork {
    let nodes = &[(45.00, 7.00), (45.00, 7.01), (45.01, 7.01)];
    let roads = [
        (0, 1, 60.0, 800.0, RoadClass::Primary),
        (1, 2, 60.0, 1_100.0, RoadClass::Residential),
        (0, 2, 150.0, 3_000.0, RoadClass::Motorway),
    ];
    let edges: Vec<_> = roads
        .iter()
        .flat_map(|&(a, b, s, m, class)| [(a, b, s, m, class), (b, a, s, m, class)])
        .collect();
    RoadNetwork::from_test_data_classed(nodes, &edges)
}

async fn seconds(network: &RoadNetwork) -> (i64, i64) {
    let places = [Coord::new(45.00, 7.00), Coord::new(45.01, 7.01)];
    let matrix = network
        .compute_matrix_with_access(&places, &[SnapAccess::Both; 2], None)
        .await;
    (
        matrix.get(0, 1).expect("in the matrix"),
        matrix.distance_meters(0, 1).expect("in the matrix"),
    )
}

#[tokio::test]
async fn no_slowdown_keeps_every_travel_time() {
    assert_eq!(seconds(&network().slowed(&[])).await, (120, 1_900));
}

#[tokio::test]
async fn a_slowdown_of_a_road_class_reroutes_onto_the_faster_class() {
    let city = Slowdown::new(2.0)
        .expect("a factor")
        .roads([RoadClass::Primary, RoadClass::Residential]);
    // City streets now take 240 s; the motorway's 150 s wins, over its 3 km.
    assert_eq!(seconds(&network().slowed(&[city])).await, (150, 3_000));
}

#[tokio::test]
async fn a_slowdown_applies_only_to_roads_inside_its_area() {
    let elsewhere = Slowdown::new(3.0)
        .expect("a factor")
        .area(BoundingBox::new(46.0, 8.0, 46.1, 8.1));
    assert_eq!(seconds(&network().slowed(&[elsewhere])).await, (120, 1_900));
    // Only the primary road's midpoint (45.00, 7.005) lies in this area.
    let here = Slowdown::new(1.5)
        .expect("a factor")
        .area(BoundingBox::new(44.99, 7.0, 45.004, 7.008));
    assert_eq!(seconds(&network().slowed(&[here])).await, (150, 1_900));
}

#[tokio::test]
async fn overlapping_slowdowns_multiply() {
    let all = Slowdown::new(1.25).expect("a factor");
    let primary = Slowdown::new(2.0)
        .expect("a factor")
        .roads([RoadClass::Primary]);
    // Primary 60 * 1.25 * 2 = 150, residential 60 * 1.25 = 75: 225 s, against
    // the motorway's 150 * 1.25 = 187.5.
    assert_eq!(
        seconds(&network().slowed(&[all, primary])).await,
        (188, 3_000)
    );
}

#[test]
fn a_factor_must_be_a_finite_number_above_zero() {
    for factor in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(Slowdown::new(factor).is_err(), "{factor} accepted");
    }
    assert!(Slowdown::new(0.8).is_ok());
}

#[test]
fn road_classes_follow_the_highway_tag_and_their_links() {
    assert_eq!(
        RoadClass::from_highway("motorway_link"),
        RoadClass::Motorway
    );
    assert_eq!(
        RoadClass::from_highway("living_street"),
        RoadClass::LivingStreet
    );
    assert_eq!(RoadClass::from_highway("track"), RoadClass::Other);
    for class in RoadClass::ALL {
        assert_eq!(RoadClass::parse(class.as_str()), Some(class));
    }
}
