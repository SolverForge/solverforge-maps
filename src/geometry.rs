//! Google Polyline encoding/decoding for route visualization.

use serde::Serialize;
use utoipa::ToSchema;

use crate::routing::Coord;

/// Encodes coordinates using Google Polyline Algorithm.
///
/// # Examples
///
/// ```
/// use solverforge_maps::{encode_polyline, Coord};
///
/// let encoded = encode_polyline(&[Coord::new(38.5, -120.2)]);
/// assert!(!encoded.is_empty());
///
/// let empty = encode_polyline(&[]);
/// assert!(empty.is_empty());
/// ```
pub fn encode_polyline(coords: &[Coord]) -> String {
    if coords.is_empty() {
        return String::new();
    }

    let mut result = String::new();
    let mut prev_lat = 0i64;
    let mut prev_lng = 0i64;

    for coord in coords {
        let lat_e5 = (coord.lat * 1e5).round() as i64;
        let lng_e5 = (coord.lng * 1e5).round() as i64;

        encode_value(lat_e5 - prev_lat, &mut result);
        encode_value(lng_e5 - prev_lng, &mut result);

        prev_lat = lat_e5;
        prev_lng = lng_e5;
    }

    result
}

fn encode_value(value: i64, output: &mut String) {
    let mut encoded = if value < 0 {
        !((value) << 1)
    } else {
        (value) << 1
    };

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
/// use solverforge_maps::{encode_polyline, decode_polyline, Coord};
///
/// let original = vec![Coord::new(38.5, -120.2), Coord::new(40.7, -120.95)];
/// let encoded = encode_polyline(&original);
/// let decoded = decode_polyline(&encoded);
///
/// assert_eq!(decoded.len(), original.len());
/// ```
pub fn decode_polyline(encoded: &str) -> Vec<Coord> {
    let mut coords = Vec::new();
    let mut lat = 0i64;
    let mut lng = 0i64;
    let bytes = encoded.as_bytes();
    let mut i = 0;

    while i < bytes.len() {
        let (lat_delta, consumed) = decode_value(&bytes[i..]);
        i += consumed;
        lat += lat_delta;

        if i >= bytes.len() {
            break;
        }

        let (lng_delta, consumed) = decode_value(&bytes[i..]);
        i += consumed;
        lng += lng_delta;

        coords.push(Coord::new(lat as f64 / 1e5, lng as f64 / 1e5));
    }

    coords
}

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

    if result & 1 != 0 {
        result = !(result >> 1);
    } else {
        result >>= 1;
    }

    (result, consumed)
}

#[derive(Debug, Clone, Serialize, ToSchema)]
pub struct EncodedSegment {
    pub entity_idx: usize,
    pub entity_name: String,
    pub polyline: String,
    pub point_count: usize,
}

impl EncodedSegment {
    pub fn new(entity_idx: usize, entity_name: impl Into<String>, coords: &[Coord]) -> Self {
        Self {
            entity_idx,
            entity_name: entity_name.into(),
            polyline: encode_polyline(coords),
            point_count: coords.len(),
        }
    }
}
