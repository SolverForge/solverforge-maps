//! Bounding box for geographic queries.

use serde::{Deserialize, Serialize};

use super::coord::Coord;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min_lat: f64,
    pub min_lng: f64,
    pub max_lat: f64,
    pub max_lng: f64,
}

impl BoundingBox {
    pub const fn new(min_lat: f64, min_lng: f64, max_lat: f64, max_lng: f64) -> Self {
        Self {
            min_lat,
            min_lng,
            max_lat,
            max_lng,
        }
    }

    pub fn from_coords(coords: &[Coord]) -> Self {
        assert!(
            !coords.is_empty(),
            "Cannot create BoundingBox from empty coords"
        );

        let mut min_lat = f64::MAX;
        let mut max_lat = f64::MIN;
        let mut min_lng = f64::MAX;
        let mut max_lng = f64::MIN;

        for coord in coords {
            min_lat = min_lat.min(coord.lat);
            max_lat = max_lat.max(coord.lat);
            min_lng = min_lng.min(coord.lng);
            max_lng = max_lng.max(coord.lng);
        }

        Self {
            min_lat,
            min_lng,
            max_lat,
            max_lng,
        }
    }

    pub fn expand(self, factor: f64) -> Self {
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

    pub fn center(&self) -> Coord {
        Coord::new(
            (self.min_lat + self.max_lat) / 2.0,
            (self.min_lng + self.max_lng) / 2.0,
        )
    }

    pub fn contains(&self, coord: Coord) -> bool {
        coord.lat >= self.min_lat
            && coord.lat <= self.max_lat
            && coord.lng >= self.min_lng
            && coord.lng <= self.max_lng
    }

    pub(crate) fn cache_key(&self) -> String {
        format!(
            "{:.4}_{:.4}_{:.4}_{:.4}",
            self.min_lat, self.min_lng, self.max_lat, self.max_lng
        )
    }
}
