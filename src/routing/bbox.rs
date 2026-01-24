//! Bounding box for geographic queries.

use serde::{Deserialize, Serialize};

use super::coord::Coord;
use super::error::BBoxError;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min_lat: f64,
    pub min_lng: f64,
    pub max_lat: f64,
    pub max_lng: f64,
}

impl BoundingBox {
    /// Creates a new bounding box, panicking on invalid input.
    ///
    /// # Panics
    ///
    /// Panics if:
    /// - Any value is NaN or infinite
    /// - `min_lat > max_lat`
    /// - `min_lng > max_lng`
    /// - Latitude values are outside [-90, 90]
    /// - Longitude values are outside [-180, 180]
    pub fn new(min_lat: f64, min_lng: f64, max_lat: f64, max_lng: f64) -> Self {
        match Self::try_new(min_lat, min_lng, max_lat, max_lng) {
            Ok(bbox) => bbox,
            Err(e) => panic!("invalid bounding box: {}", e),
        }
    }

    pub fn try_new(
        min_lat: f64,
        min_lng: f64,
        max_lat: f64,
        max_lng: f64,
    ) -> Result<Self, BBoxError> {
        // Check NaN
        if min_lat.is_nan() {
            return Err(BBoxError::NaN { field: "min_lat" });
        }
        if min_lng.is_nan() {
            return Err(BBoxError::NaN { field: "min_lng" });
        }
        if max_lat.is_nan() {
            return Err(BBoxError::NaN { field: "max_lat" });
        }
        if max_lng.is_nan() {
            return Err(BBoxError::NaN { field: "max_lng" });
        }

        // Check infinite
        if min_lat.is_infinite() {
            return Err(BBoxError::Infinite {
                field: "min_lat",
                value: min_lat,
            });
        }
        if min_lng.is_infinite() {
            return Err(BBoxError::Infinite {
                field: "min_lng",
                value: min_lng,
            });
        }
        if max_lat.is_infinite() {
            return Err(BBoxError::Infinite {
                field: "max_lat",
                value: max_lat,
            });
        }
        if max_lng.is_infinite() {
            return Err(BBoxError::Infinite {
                field: "max_lng",
                value: max_lng,
            });
        }

        // Check latitude range
        if !(-90.0..=90.0).contains(&min_lat) {
            return Err(BBoxError::LatOutOfRange { value: min_lat });
        }
        if !(-90.0..=90.0).contains(&max_lat) {
            return Err(BBoxError::LatOutOfRange { value: max_lat });
        }

        // Check longitude range
        if !(-180.0..=180.0).contains(&min_lng) {
            return Err(BBoxError::LngOutOfRange { value: min_lng });
        }
        if !(-180.0..=180.0).contains(&max_lng) {
            return Err(BBoxError::LngOutOfRange { value: max_lng });
        }

        // Check min < max
        if min_lat > max_lat {
            return Err(BBoxError::MinLatGreaterThanMax {
                min: min_lat,
                max: max_lat,
            });
        }
        if min_lng > max_lng {
            return Err(BBoxError::MinLngGreaterThanMax {
                min: min_lng,
                max: max_lng,
            });
        }

        Ok(Self {
            min_lat,
            min_lng,
            max_lat,
            max_lng,
        })
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

    pub fn expand_meters(self, meters: f64) -> Self {
        let lat_deg = meters / 111_320.0;
        let center_lat = (self.min_lat + self.max_lat) / 2.0;
        let lng_deg = meters / (111_320.0 * center_lat.to_radians().cos());

        Self {
            min_lat: self.min_lat - lat_deg,
            min_lng: self.min_lng - lng_deg,
            max_lat: self.max_lat + lat_deg,
            max_lng: self.max_lng + lng_deg,
        }
    }

    pub fn expand_for_routing(self, locations: &[Coord]) -> Self {
        const DETOUR_FACTOR: f64 = 1.4;

        if locations.len() < 2 {
            return self;
        }

        let mut max_distance: f64 = 0.0;
        for i in 0..locations.len() {
            for j in (i + 1)..locations.len() {
                let dist = locations[i].distance_to(locations[j]);
                max_distance = max_distance.max(dist);
            }
        }

        let expansion = max_distance * (DETOUR_FACTOR - 1.0) / 2.0;
        self.expand_meters(expansion)
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
