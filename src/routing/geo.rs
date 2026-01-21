//! Geographic utility functions.

use super::coord::Coord;

pub const DEFAULT_SPEED_MPS: f64 = 50.0 * 1000.0 / 3600.0;

pub fn coord_key(lat: f64, lng: f64) -> (i64, i64) {
    ((lat * 1e7).round() as i64, (lng * 1e7).round() as i64)
}

pub fn haversine_distance(a: Coord, b: Coord) -> f64 {
    const R: f64 = 6_371_000.0;

    let lat1_rad = a.lat.to_radians();
    let lat2_rad = b.lat.to_radians();
    let dlat = (b.lat - a.lat).to_radians();
    let dlng = (b.lng - a.lng).to_radians();

    let h = (dlat / 2.0).sin().powi(2)
        + lat1_rad.cos() * lat2_rad.cos() * (dlng / 2.0).sin().powi(2);
    let c = 2.0 * h.sqrt().asin();

    R * c
}
