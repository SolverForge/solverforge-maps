//! OpenStreetMap data structures for Overpass API.

use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct OverpassResponse {
    pub elements: Vec<OsmElement>,
}

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

#[derive(Debug, Default, Deserialize)]
pub struct OsmTags {
    pub highway: Option<String>,
    pub oneway: Option<String>,
    pub maxspeed: Option<String>,
    pub junction: Option<String>,
    pub access: Option<String>,
    pub vehicle: Option<String>,
    pub motor_vehicle: Option<String>,
    pub motorcar: Option<String>,
}

/// Directions in which a car may travel along a way, relative to its node order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TravelDirection {
    Both,
    Forward,
    Backward,
    None,
}

/// Access values that close a road to private cars. `destination` stays open:
/// it only forbids through traffic, and a stop on that road is a destination.
const CLOSED_TO_CARS: &[&str] = &[
    "no",
    "private",
    "agricultural",
    "forestry",
    "emergency",
    "psv",
    "customers",
    "delivery",
];

impl OsmTags {
    /// The most specific access tag decides: motorcar, then motor_vehicle,
    /// then vehicle, then access.
    pub fn open_to_cars(&self) -> bool {
        let decisive = [
            &self.motorcar,
            &self.motor_vehicle,
            &self.vehicle,
            &self.access,
        ]
        .into_iter()
        .find_map(|tag| tag.as_deref());
        !decisive.is_some_and(|value| CLOSED_TO_CARS.contains(&value))
    }

    pub fn travel_direction(&self) -> TravelDirection {
        match self.oneway.as_deref() {
            Some("yes" | "true" | "1") => TravelDirection::Forward,
            Some("-1" | "reverse") => TravelDirection::Backward,
            Some("no" | "false" | "0") => TravelDirection::Both,
            // Direction changes during the day: no single direction can be promised.
            Some("reversible" | "alternating") => TravelDirection::None,
            _ if self.implicitly_one_way() => TravelDirection::Forward,
            _ => TravelDirection::Both,
        }
    }

    /// OSM implies oneway=yes on roundabouts and motorways.
    fn implicitly_one_way(&self) -> bool {
        matches!(self.junction.as_deref(), Some("roundabout" | "circular"))
            || matches!(self.highway.as_deref(), Some("motorway" | "motorway_link"))
    }
}
