//! OpenStreetMap data structures for Overpass API.

use serde::Deserialize;

/// Response from Overpass API.
#[derive(Debug, Deserialize)]
pub struct OverpassResponse {
    pub elements: Vec<OsmElement>,
}

/// An OSM element (node or way).
#[derive(Debug, Deserialize)]
pub struct OsmElement {
    #[serde(rename = "type")]
    pub elem_type: String,
    pub id: i64,
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub nodes: Option<Vec<i64>>,
    pub tags: Option<OsmTags>,
}

/// OSM tags for an element.
#[derive(Debug, Deserialize)]
pub struct OsmTags {
    pub highway: Option<String>,
    pub oneway: Option<String>,
    /// Maxspeed tag (for future use with dynamic speed calculation).
    #[allow(dead_code)]
    pub maxspeed: Option<String>,
}
