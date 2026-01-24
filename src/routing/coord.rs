//! Coordinate type for geographic locations.

use serde::{Deserialize, Serialize};

use super::error::CoordError;

#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Coord {
    pub lat: f64,
    pub lng: f64,
}

impl Coord {
    /// Creates a new coordinate, panicking on invalid input.
    ///
    /// # Panics
    ///
    /// Panics if:
    /// - `lat` or `lng` is NaN
    /// - `lat` or `lng` is infinite
    /// - `lat` is outside [-90, 90]
    /// - `lng` is outside [-180, 180]
    #[inline]
    pub fn new(lat: f64, lng: f64) -> Self {
        match Self::try_new(lat, lng) {
            Ok(coord) => coord,
            Err(e) => panic!("invalid coordinate: {}", e),
        }
    }

    /// Attempts to create a new coordinate with validation.
    ///
    /// Returns an error if:
    /// - `lat` or `lng` is NaN
    /// - `lat` or `lng` is infinite
    /// - `lat` is outside [-90, 90]
    /// - `lng` is outside [-180, 180]
    #[inline]
    pub fn try_new(lat: f64, lng: f64) -> Result<Self, CoordError> {
        if lat.is_nan() {
            return Err(CoordError::LatNaN);
        }

        if lng.is_nan() {
            return Err(CoordError::LngNaN);
        }

        if lat.is_infinite() {
            return Err(CoordError::LatInfinite { value: lat });
        }

        if lng.is_infinite() {
            return Err(CoordError::LngInfinite { value: lng });
        }

        if !(-90.0..=90.0).contains(&lat) {
            return Err(CoordError::LatOutOfRange { value: lat });
        }

        if !(-180.0..=180.0).contains(&lng) {
            return Err(CoordError::LngOutOfRange { value: lng });
        }

        Ok(Self { lat, lng })
    }

    #[inline]
    pub fn distance_to(self, other: Coord) -> f64 {
        super::geo::haversine_distance(self, other)
    }
}

impl TryFrom<(f64, f64)> for Coord {
    type Error = CoordError;

    #[inline]
    fn try_from((lat, lng): (f64, f64)) -> Result<Self, Self::Error> {
        Self::try_new(lat, lng)
    }
}

impl From<Coord> for (f64, f64) {
    #[inline]
    fn from(coord: Coord) -> Self {
        (coord.lat, coord.lng)
    }
}
