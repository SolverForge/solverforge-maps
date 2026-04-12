use super::*;

#[test]
fn segment_index_returns_none_when_empty() {
    let index: SegmentIndex<usize> = SegmentIndex::bulk_load(vec![]);
    assert!(index.nearest_segment(&Point2D::new(0.0, 0.0)).is_none());
}

#[test]
fn segment_index_matches_bruteforce_on_large_input() {
    let segments: Vec<(Segment, usize)> = (0..256)
        .map(|i| {
            let y = i as f64 * 10.0;
            (
                Segment::new(Point2D::new(0.0, y), Point2D::new(5.0, y + 1.0)),
                i,
            )
        })
        .collect();
    let index = SegmentIndex::bulk_load(segments.clone());
    let query = Point2D::new(2.25, 1234.4);

    let indexed = index
        .nearest_segment(&query)
        .expect("expected nearest segment");

    let brute_force = segments
        .iter()
        .map(|(segment, data)| {
            let (projection, _) = segment.project_point(&query);
            (
                *data,
                projection,
                query.distance_squared(&projection),
                *segment,
            )
        })
        .min_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
        .expect("expected brute-force result");

    assert_eq!(*indexed.1, brute_force.0);
    assert_eq!(*indexed.0, brute_force.3);
    assert!((indexed.2.x - brute_force.1.x).abs() < 1e-9);
    assert!((indexed.2.y - brute_force.1.y).abs() < 1e-9);
    assert!((indexed.3 - brute_force.2).abs() < 1e-9);
}
