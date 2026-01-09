//! Bounding box for geographic queries.

/// Bounding box for OSM queries.
#[derive(Debug, Clone, Copy)]
pub struct BoundingBox {
    pub min_lat: f64,
    pub min_lng: f64,
    pub max_lat: f64,
    pub max_lng: f64,
}

impl BoundingBox {
    /// Creates a new bounding box.
    pub fn new(min_lat: f64, min_lng: f64, max_lat: f64, max_lng: f64) -> Self {
        Self {
            min_lat,
            min_lng,
            max_lat,
            max_lng,
        }
    }

    /// Expands the bounding box by a factor (e.g., 0.1 = 10% on each side).
    pub fn expand(&self, factor: f64) -> Self {
        let lat_range = self.max_lat - self.min_lat;
        let lng_range = self.max_lng - self.min_lng;
        let lat_pad = lat_range * factor;
        let lng_pad = lng_range * factor;

        Self {
            min_lat: self.min_lat - lat_pad,
            min_lng: self.min_lng - lng_pad,
            max_lat: self.max_lat + lat_pad,
            max_lng: self.max_lng + lng_pad,
        }
    }

    /// Returns a cache key for this bounding box.
    pub(crate) fn cache_key(&self) -> String {
        format!(
            "{:.4}_{:.4}_{:.4}_{:.4}",
            self.min_lat, self.min_lng, self.max_lat, self.max_lng
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bbox_expand() {
        let bbox = BoundingBox::new(39.9, -75.2, 40.0, -75.1);
        let expanded = bbox.expand(0.1);
        assert!(expanded.min_lat < bbox.min_lat);
        assert!(expanded.max_lat > bbox.max_lat);
    }
}
