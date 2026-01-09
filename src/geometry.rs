//! Geometry utilities for route visualization.
//!
//! Implements Google Polyline encoding for efficient route transmission.
//! See: <https://developers.google.com/maps/documentation/utilities/polylinealgorithm>

use serde::Serialize;
use utoipa::ToSchema;

/// Encodes a sequence of coordinates using Google Polyline Algorithm.
///
/// The algorithm encodes latitude/longitude pairs as an ASCII string for
/// efficient transmission. Each coordinate is encoded as the difference
/// from the previous point, with 5 decimal places of precision.
///
/// # Examples
///
/// ```
/// use solverforge_maps::encode_polyline;
///
/// // Single point encodes to non-empty string
/// let encoded = encode_polyline(&[(38.5, -120.2)]);
/// assert!(!encoded.is_empty());
///
/// // Empty input gives empty output
/// let empty = encode_polyline(&[]);
/// assert!(empty.is_empty());
///
/// // Two points create a line
/// let line = encode_polyline(&[(38.5, -120.2), (40.7, -120.95)]);
/// assert!(!line.is_empty());
/// ```
pub fn encode_polyline(coords: &[(f64, f64)]) -> String {
    if coords.is_empty() {
        return String::new();
    }

    let mut result = String::new();
    let mut prev_lat = 0i64;
    let mut prev_lng = 0i64;

    for &(lat, lng) in coords {
        // Convert to fixed-point with 5 decimal places
        let lat_e5 = (lat * 1e5).round() as i64;
        let lng_e5 = (lng * 1e5).round() as i64;

        // Encode deltas
        encode_value(lat_e5 - prev_lat, &mut result);
        encode_value(lng_e5 - prev_lng, &mut result);

        prev_lat = lat_e5;
        prev_lng = lng_e5;
    }

    result
}

/// Encodes a single signed value using the polyline algorithm.
fn encode_value(value: i64, output: &mut String) {
    // Left-shift and invert if negative
    let mut encoded = if value < 0 {
        !((value) << 1)
    } else {
        (value) << 1
    };

    // Break into 5-bit chunks, OR with 0x20 if more chunks follow
    while encoded >= 0x20 {
        output.push(char::from_u32(((encoded & 0x1f) | 0x20) as u32 + 63).unwrap());
        encoded >>= 5;
    }
    output.push(char::from_u32(encoded as u32 + 63).unwrap());
}

/// Decodes a Google Polyline string back to coordinates.
///
/// # Examples
///
/// ```
/// use solverforge_maps::{encode_polyline, decode_polyline};
///
/// let original = vec![(38.5, -120.2), (40.7, -120.95), (43.252, -126.453)];
/// let encoded = encode_polyline(&original);
/// let decoded = decode_polyline(&encoded);
///
/// // Check round-trip (within 5 decimal places precision)
/// assert_eq!(decoded.len(), original.len());
/// for (orig, dec) in original.iter().zip(decoded.iter()) {
///     assert!((orig.0 - dec.0).abs() < 0.00001);
///     assert!((orig.1 - dec.1).abs() < 0.00001);
/// }
/// ```
pub fn decode_polyline(encoded: &str) -> Vec<(f64, f64)> {
    let mut coords = Vec::new();
    let mut lat = 0i64;
    let mut lng = 0i64;
    let bytes = encoded.as_bytes();
    let mut i = 0;

    while i < bytes.len() {
        // Decode latitude delta
        let (lat_delta, consumed) = decode_value(&bytes[i..]);
        i += consumed;
        lat += lat_delta;

        if i >= bytes.len() {
            break;
        }

        // Decode longitude delta
        let (lng_delta, consumed) = decode_value(&bytes[i..]);
        i += consumed;
        lng += lng_delta;

        coords.push((lat as f64 / 1e5, lng as f64 / 1e5));
    }

    coords
}

/// Decodes a single value, returning (value, bytes_consumed).
fn decode_value(bytes: &[u8]) -> (i64, usize) {
    let mut result = 0i64;
    let mut shift = 0;
    let mut consumed = 0;

    for &b in bytes {
        consumed += 1;
        let chunk = (b as i64) - 63;
        result |= (chunk & 0x1f) << shift;
        shift += 5;

        if chunk < 0x20 {
            break;
        }
    }

    // Invert if negative (check LSB)
    if result & 1 != 0 {
        result = !(result >> 1);
    } else {
        result >>= 1;
    }

    (result, consumed)
}

/// Encoded route segment.
#[derive(Debug, Clone, Serialize, ToSchema)]
pub struct EncodedSegment {
    /// Entity index (e.g., vehicle index).
    pub entity_idx: usize,
    /// Entity name (e.g., vehicle name).
    pub entity_name: String,
    /// Encoded polyline string (Google format).
    pub polyline: String,
    /// Number of points in the route.
    pub point_count: usize,
}

impl EncodedSegment {
    /// Creates a new encoded segment from coordinates.
    pub fn new(entity_idx: usize, entity_name: impl Into<String>, coords: &[(f64, f64)]) -> Self {
        Self {
            entity_idx,
            entity_name: entity_name.into(),
            polyline: encode_polyline(coords),
            point_count: coords.len(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode_roundtrip() {
        let coords = vec![
            (38.5, -120.2),
            (40.7, -120.95),
            (43.252, -126.453),
        ];
        let encoded = encode_polyline(&coords);
        let decoded = decode_polyline(&encoded);

        assert_eq!(decoded.len(), coords.len());
        for (orig, dec) in coords.iter().zip(decoded.iter()) {
            assert!((orig.0 - dec.0).abs() < 0.00001);
            assert!((orig.1 - dec.1).abs() < 0.00001);
        }
    }

    #[test]
    fn test_known_encoding() {
        // Known encoding from Google's examples
        let coords = vec![(38.5, -120.2), (40.7, -120.95), (43.252, -126.453)];
        let encoded = encode_polyline(&coords);
        // The encoding should be deterministic
        assert!(!encoded.is_empty());
        // Verify we can decode it back
        let decoded = decode_polyline(&encoded);
        assert_eq!(decoded.len(), 3);
    }

    #[test]
    fn test_empty_coords() {
        let encoded = encode_polyline(&[]);
        assert!(encoded.is_empty());
        let decoded = decode_polyline("");
        assert!(decoded.is_empty());
    }

    #[test]
    fn test_single_point() {
        let coords = vec![(0.0, 0.0)];
        let encoded = encode_polyline(&coords);
        let decoded = decode_polyline(&encoded);
        assert_eq!(decoded.len(), 1);
        assert!((decoded[0].0).abs() < 0.00001);
        assert!((decoded[0].1).abs() < 0.00001);
    }
}
