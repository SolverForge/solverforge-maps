//! Speed parsing for OSM maxspeed tags.

/// Parses OSM maxspeed tag into meters per second.
///
/// Supported formats:
/// - Numeric: "50" (assumes km/h)
/// - Explicit units: "60 km/h", "30 mph"
/// - Special: "walk" (~5 km/h), "none"/"unlimited" (returns None)
pub(crate) fn parse_maxspeed(value: &str) -> Option<f64> {
    let value = value.trim();

    match value.to_lowercase().as_str() {
        "walk" => return Some(5.0 * 1000.0 / 3600.0),
        "none" | "unlimited" => return None,
        _ => {}
    }

    let (num_str, is_mph) = if let Some(s) = value.strip_suffix("mph") {
        (s.trim(), true)
    } else if let Some(s) = value.strip_suffix("km/h") {
        (s.trim(), false)
    } else if let Some(s) = value.strip_suffix("kmh") {
        (s.trim(), false)
    } else {
        (value, false)
    };

    let kmh: f64 = num_str.parse().ok()?;
    if kmh <= 0.0 || kmh > 300.0 {
        return None;
    }

    let kmh = if is_mph { kmh * 1.60934 } else { kmh };
    Some(kmh * 1000.0 / 3600.0)
}
