//! Zero-erasure spatial index implementation using K-D Trees.

/// 2D point for spatial queries.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Point2D {
    pub x: f64,
    pub y: f64,
}

impl Point2D {
    #[inline]
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Squared Euclidean distance to another point.
    #[inline]
    pub fn distance_squared(&self, other: &Point2D) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }
}

/// K-D Tree node for 2D points.
struct KdNode<T> {
    point: Point2D,
    data: T,
    left: Option<usize>,
    right: Option<usize>,
}

/// K-D Tree for efficient 2D nearest neighbor queries.
///
/// Zero-erasure architecture: nodes stored in a flat Vec, indices instead of pointers.
pub struct KdTree<T> {
    nodes: Vec<KdNode<T>>,
    root: Option<usize>,
}

impl<T> KdTree<T> {
    /// Create an empty K-D Tree.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            root: None,
        }
    }

    /// Find the nearest neighbor to a query point.
    ///
    /// Returns a reference to the data associated with the nearest point,
    /// or `None` if the tree is empty.
    pub fn nearest_neighbor(&self, query: &Point2D) -> Option<&T> {
        self.nearest_neighbor_with_distance(query)
            .map(|(data, _)| data)
    }

    /// Find the nearest neighbor with its squared distance.
    pub fn nearest_neighbor_with_distance(&self, query: &Point2D) -> Option<(&T, f64)> {
        let root = self.root?;
        let mut best: Option<(usize, f64)> = None;

        self.search_nearest(root, query, 0, &mut best);

        best.map(|(idx, dist)| (&self.nodes[idx].data, dist))
    }

    fn search_nearest(
        &self,
        node_idx: usize,
        query: &Point2D,
        depth: usize,
        best: &mut Option<(usize, f64)>,
    ) {
        let node = &self.nodes[node_idx];
        let dist = query.distance_squared(&node.point);

        match best {
            Some((_, best_dist)) if dist < *best_dist => {
                *best = Some((node_idx, dist));
            }
            None => {
                *best = Some((node_idx, dist));
            }
            _ => {}
        }

        // Determine which side of the splitting plane the query is on
        let axis = depth % 2;
        let query_val = if axis == 0 { query.x } else { query.y };
        let node_val = if axis == 0 {
            node.point.x
        } else {
            node.point.y
        };
        let diff = query_val - node_val;

        // Search the closer side first
        let (first, second) = if diff <= 0.0 {
            (node.left, node.right)
        } else {
            (node.right, node.left)
        };

        if let Some(child) = first {
            self.search_nearest(child, query, depth + 1, best);
        }

        // Check if we need to search the other side
        // Only if the splitting plane is closer than our current best
        let plane_dist = diff * diff;
        if let Some((_, best_dist)) = best {
            if plane_dist < *best_dist {
                if let Some(child) = second {
                    self.search_nearest(child, query, depth + 1, best);
                }
            }
        }
    }
}

impl<T> Default for KdTree<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// Build K-D Tree from items using an index-based approach.
///
/// This is a helper to properly construct the tree with ownership.
impl<T> KdTree<T> {
    /// Build from a vector of items, taking ownership.
    pub fn from_items(items: Vec<(Point2D, T)>) -> Self {
        if items.is_empty() {
            return Self::new();
        }

        // Convert to indexed items for sorting
        let mut indexed: Vec<(usize, Point2D)> = items
            .iter()
            .enumerate()
            .map(|(i, (p, _))| (i, *p))
            .collect();

        let mut tree = Self {
            nodes: Vec::with_capacity(items.len()),
            root: None,
        };

        // We'll store items in a Vec and extract by index
        let mut items: Vec<Option<(Point2D, T)>> = items.into_iter().map(Some).collect();

        tree.root = tree.build_from_indexed(&mut indexed, &mut items, 0);
        tree
    }

    fn build_from_indexed(
        &mut self,
        indexed: &mut [(usize, Point2D)],
        items: &mut [Option<(Point2D, T)>],
        depth: usize,
    ) -> Option<usize> {
        if indexed.is_empty() {
            return None;
        }

        let axis = depth % 2;

        indexed.sort_by(|a, b| {
            let va = if axis == 0 { a.1.x } else { a.1.y };
            let vb = if axis == 0 { b.1.x } else { b.1.y };
            va.partial_cmp(&vb).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mid = indexed.len() / 2;
        let (left_slice, rest) = indexed.split_at_mut(mid);
        let (mid_item, right_slice) = rest.split_first_mut().expect("not empty");

        // Extract the item at the median index
        let orig_idx = mid_item.0;
        let (point, data) = items[orig_idx].take().expect("item already taken");

        let left = self.build_from_indexed(left_slice, items, depth + 1);
        let right = self.build_from_indexed(right_slice, items, depth + 1);

        let node_idx = self.nodes.len();
        self.nodes.push(KdNode {
            point,
            data,
            left,
            right,
        });

        Some(node_idx)
    }
}

/// Segment in 2D space.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Segment {
    pub from: Point2D,
    pub to: Point2D,
}

impl Segment {
    #[inline]
    pub fn new(from: Point2D, to: Point2D) -> Self {
        Self { from, to }
    }

    /// Compute the centroid of the segment.
    #[inline]
    pub fn centroid(&self) -> Point2D {
        Point2D {
            x: (self.from.x + self.to.x) / 2.0,
            y: (self.from.y + self.to.y) / 2.0,
        }
    }

    /// Project a point onto this segment and return the closest point on the segment.
    ///
    /// Returns (projected_point, t) where t is the position along the segment [0, 1].
    pub fn project_point(&self, point: &Point2D) -> (Point2D, f64) {
        let dx = self.to.x - self.from.x;
        let dy = self.to.y - self.from.y;
        let len_sq = dx * dx + dy * dy;

        if len_sq < f64::EPSILON {
            // Degenerate segment (zero length)
            return (self.from, 0.0);
        }

        // Project point onto line, clamped to [0, 1]
        let t = ((point.x - self.from.x) * dx + (point.y - self.from.y) * dy) / len_sq;
        let t = t.clamp(0.0, 1.0);

        let proj = Point2D {
            x: self.from.x + t * dx,
            y: self.from.y + t * dy,
        };

        (proj, t)
    }
}

#[derive(Debug, Copy, Clone)]
struct BoundingBox2D {
    min: Point2D,
    max: Point2D,
}

impl BoundingBox2D {
    fn from_segment(segment: &Segment) -> Self {
        Self {
            min: Point2D::new(
                segment.from.x.min(segment.to.x),
                segment.from.y.min(segment.to.y),
            ),
            max: Point2D::new(
                segment.from.x.max(segment.to.x),
                segment.from.y.max(segment.to.y),
            ),
        }
    }

    fn union(self, other: Self) -> Self {
        Self {
            min: Point2D::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
            max: Point2D::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
        }
    }

    fn distance_squared_to_point(&self, point: &Point2D) -> f64 {
        let dx = if point.x < self.min.x {
            self.min.x - point.x
        } else if point.x > self.max.x {
            point.x - self.max.x
        } else {
            0.0
        };
        let dy = if point.y < self.min.y {
            self.min.y - point.y
        } else if point.y > self.max.y {
            point.y - self.max.y
        } else {
            0.0
        };
        dx * dx + dy * dy
    }
}

struct SegmentNode<T> {
    segment: Segment,
    data: T,
    bounds: BoundingBox2D,
    left: Option<usize>,
    right: Option<usize>,
}

/// Spatial index for line segments.
///
/// Uses a branch-and-bound K-D tree over segment centroids with exact subtree bounds.
pub struct SegmentIndex<T> {
    nodes: Vec<SegmentNode<T>>,
    root: Option<usize>,
}

impl<T> SegmentIndex<T> {
    /// Build a segment index from a list of segments with associated data.
    pub fn bulk_load(segments: Vec<(Segment, T)>) -> Self {
        let mut indexed: Vec<(usize, Point2D)> = segments
            .iter()
            .enumerate()
            .map(|(i, (segment, _))| (i, segment.centroid()))
            .collect();
        let mut items: Vec<Option<(Segment, T)>> = segments.into_iter().map(Some).collect();
        let mut nodes = Vec::with_capacity(items.len());
        let root = Self::build_nodes(&mut indexed, &mut items, 0, &mut nodes);
        Self { nodes, root }
    }

    /// Find the nearest segment to a query point.
    ///
    /// Returns the segment, associated data, projected point on segment, and squared distance.
    pub fn nearest_segment(&self, query: &Point2D) -> Option<(&Segment, &T, Point2D, f64)> {
        let root = self.root?;
        let mut best: Option<(usize, Point2D, f64)> = None;
        self.search_nearest(root, query, 0, &mut best);
        let (idx, projection, dist) = best?;
        let node = &self.nodes[idx];
        Some((&node.segment, &node.data, projection, dist))
    }

    fn build_nodes(
        indexed: &mut [(usize, Point2D)],
        items: &mut [Option<(Segment, T)>],
        depth: usize,
        nodes: &mut Vec<SegmentNode<T>>,
    ) -> Option<usize> {
        if indexed.is_empty() {
            return None;
        }

        let axis = depth % 2;
        indexed.sort_by(|a, b| {
            let va = if axis == 0 { a.1.x } else { a.1.y };
            let vb = if axis == 0 { b.1.x } else { b.1.y };
            va.partial_cmp(&vb).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mid = indexed.len() / 2;
        let (left_slice, rest) = indexed.split_at_mut(mid);
        let (mid_item, right_slice) = rest.split_first_mut().expect("not empty");
        let left = Self::build_nodes(left_slice, items, depth + 1, nodes);
        let right = Self::build_nodes(right_slice, items, depth + 1, nodes);

        let (segment, data) = items[mid_item.0].take().expect("item already taken");
        let bounds = BoundingBox2D::from_segment(&segment);
        let idx = nodes.len();
        nodes.push(SegmentNode {
            segment,
            data,
            bounds,
            left,
            right,
        });

        let mut bounds = BoundingBox2D::from_segment(&nodes[idx].segment);
        if let Some(left_idx) = left {
            bounds = bounds.union(nodes[left_idx].bounds);
        }
        if let Some(right_idx) = right {
            bounds = bounds.union(nodes[right_idx].bounds);
        }
        nodes[idx].bounds = bounds;

        Some(idx)
    }

    fn search_nearest(
        &self,
        node_idx: usize,
        query: &Point2D,
        depth: usize,
        best: &mut Option<(usize, Point2D, f64)>,
    ) {
        let node = &self.nodes[node_idx];
        let (projection, _) = node.segment.project_point(query);
        let dist = query.distance_squared(&projection);

        match best {
            Some((_, _, best_dist)) if dist < *best_dist => {
                *best = Some((node_idx, projection, dist));
            }
            None => {
                *best = Some((node_idx, projection, dist));
            }
            _ => {}
        }

        let axis = depth % 2;
        let centroid = node.segment.centroid();
        let query_val = if axis == 0 { query.x } else { query.y };
        let node_val = if axis == 0 { centroid.x } else { centroid.y };
        let (first, second) = if query_val <= node_val {
            (node.left, node.right)
        } else {
            (node.right, node.left)
        };

        if let Some(child) = first {
            self.search_child(child, query, depth + 1, best);
        }
        if let Some(child) = second {
            self.search_child(child, query, depth + 1, best);
        }
    }

    fn search_child(
        &self,
        child_idx: usize,
        query: &Point2D,
        depth: usize,
        best: &mut Option<(usize, Point2D, f64)>,
    ) {
        if let Some((_, _, best_dist)) = best {
            let child_dist = self.nodes[child_idx]
                .bounds
                .distance_squared_to_point(query);
            if child_dist > *best_dist {
                return;
            }
        }

        self.search_nearest(child_idx, query, depth, best);
    }
}

#[cfg(test)]
mod tests {
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
}
