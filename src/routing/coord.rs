//! Coordinate type for geographic locations.

use serde::{Deserialize, Serialize};

/// A geographic coordinate with latitude and longitude.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Coord {
    pub lat: f64,
    pub lng: f64,
}

impl Coord {
    #[inline]
    pub const fn new(lat: f64, lng: f64) -> Self {
        Self { lat, lng }
    }

    #[inline]
    pub fn distance_to(self, other: Coord) -> f64 {
        super::geo::haversine_distance(self, other)
    }
}

impl From<(f64, f64)> for Coord {
    #[inline]
    fn from((lat, lng): (f64, f64)) -> Self {
        Self { lat, lng }
    }
}

impl From<Coord> for (f64, f64) {
    #[inline]
    fn from(coord: Coord) -> Self {
        (coord.lat, coord.lng)
    }
}
