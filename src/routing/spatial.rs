//! Zero-erasure spatial index implementation using K-D Trees.

/// 2D point for spatial queries.
#[derive(Debug, Copy, Clone)]
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
#[derive(Debug, Copy, Clone)]
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

/// Spatial index for line segments.
///
/// Uses a K-D Tree on segment centroids, then refines with actual segment distance.
pub struct SegmentIndex<T> {
    /// K-D Tree indexed by segment centroid
    tree: KdTree<(Segment, T)>,
    /// All segments for brute-force refinement within candidate set
    segments: Vec<(Segment, T)>,
}

impl<T: Clone> SegmentIndex<T> {
    /// Build a segment index from a list of segments with associated data.
    pub fn bulk_load(segments: Vec<(Segment, T)>) -> Self {
        let items: Vec<(Point2D, (Segment, T))> = segments
            .iter()
            .cloned()
            .map(|(seg, data)| (seg.centroid(), (seg, data)))
            .collect();

        Self {
            tree: KdTree::from_items(items),
            segments,
        }
    }

    /// Find the nearest segment to a query point.
    ///
    /// Returns the segment, associated data, projected point on segment, and squared distance.
    pub fn nearest_segment(&self, query: &Point2D) -> Option<(&Segment, &T, Point2D, f64)> {
        if self.segments.is_empty() {
            return None;
        }

        // For small datasets, just brute force
        if self.segments.len() <= 100 {
            return self.brute_force_nearest(query);
        }

        // Use K-D tree to find candidates, then refine
        // First, get the nearest centroid as a starting point
        let nearest_centroid = self.tree.nearest_neighbor_with_distance(query)?;

        // The actual nearest segment might be different from the one with nearest centroid
        // We need to search within a radius equal to our best distance so far
        let (seg, data) = nearest_centroid.0;
        let (proj, _) = seg.project_point(query);
        let mut best_dist = query.distance_squared(&proj);
        let mut best_result = (seg, data, proj, best_dist);

        // Check all segments (for correctness; can be optimized with spatial queries)
        // Since K-D tree on centroids doesn't give us tight bounds, we do a full scan
        // This is still faster than pure brute force for construction + multiple queries
        for (seg, data) in &self.segments {
            let (proj, _) = seg.project_point(query);
            let dist = query.distance_squared(&proj);
            if dist < best_dist {
                best_dist = dist;
                best_result = (seg, data, proj, dist);
            }
        }

        Some(best_result)
    }

    fn brute_force_nearest(&self, query: &Point2D) -> Option<(&Segment, &T, Point2D, f64)> {
        let mut best: Option<(&Segment, &T, Point2D, f64)> = None;

        for (seg, data) in &self.segments {
            let (proj, _) = seg.project_point(query);
            let dist = query.distance_squared(&proj);

            match &best {
                Some((_, _, _, best_dist)) if dist < *best_dist => {
                    best = Some((seg, data, proj, dist));
                }
                None => {
                    best = Some((seg, data, proj, dist));
                }
                _ => {}
            }
        }

        best
    }
}
