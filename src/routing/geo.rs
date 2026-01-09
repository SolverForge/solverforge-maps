//! Geographic utility functions.

/// Default driving speed in m/s (50 km/h = 13.89 m/s).
pub const DEFAULT_SPEED_MPS: f64 = 50.0 * 1000.0 / 3600.0;

/// Converts coordinates to a hash key (7 decimal places precision).
pub fn coord_key(lat: f64, lng: f64) -> (i64, i64) {
    ((lat * 1e7).round() as i64, (lng * 1e7).round() as i64)
}

/// Returns speed in m/s for a highway type.
pub fn get_speed_for_highway(highway: &str) -> f64 {
    let kmh = match highway {
        "motorway" | "motorway_link" => 100.0,
        "trunk" | "trunk_link" => 80.0,
        "primary" | "primary_link" => 60.0,
        "secondary" | "secondary_link" => 50.0,
        "tertiary" | "tertiary_link" => 40.0,
        "residential" => 30.0,
        "unclassified" => 30.0,
        "service" => 20.0,
        "living_street" => 10.0,
        _ => 30.0,
    };
    kmh * 1000.0 / 3600.0
}

/// Haversine distance between two points in meters.
pub fn haversine_distance(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f64 {
    const R: f64 = 6_371_000.0; // Earth radius in meters

    let lat1_rad = lat1.to_radians();
    let lat2_rad = lat2.to_radians();
    let dlat = (lat2 - lat1).to_radians();
    let dlng = (lng2 - lng1).to_radians();

    let a = (dlat / 2.0).sin().powi(2)
        + lat1_rad.cos() * lat2_rad.cos() * (dlng / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();

    R * c
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haversine_distance() {
        // Philadelphia City Hall to Liberty Bell (~500m)
        let dist = haversine_distance(39.9526, -75.1635, 39.9496, -75.1503);
        assert!((dist - 1200.0).abs() < 100.0); // Approximately 1.2 km
    }

    #[test]
    fn test_coord_key() {
        let key = coord_key(39.9526, -75.1635);
        assert_eq!(key, (399526000, -751635000));
    }
}
