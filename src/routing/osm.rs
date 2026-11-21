//! OpenStreetMap data structures for Overpass API.

use serde::Deserialize;

use super::config::TravelMode;

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
    pub name: Option<String>,
    pub highway: Option<String>,
    pub oneway: Option<String>,
    pub maxspeed: Option<String>,
    pub junction: Option<String>,
    pub access: Option<String>,
    pub vehicle: Option<String>,
    pub motor_vehicle: Option<String>,
    pub motorcar: Option<String>,
    pub bicycle: Option<String>,
    pub foot: Option<String>,
    #[serde(rename = "oneway:bicycle")]
    pub oneway_bicycle: Option<String>,
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

    /// Whether a traveller in `mode` may use the way. A bicycle follows its
    /// `bicycle`, `vehicle` and `access` tags and rides a footway or pedestrian
    /// street only where `bicycle` allows it; a walker follows `foot` and
    /// `access` and never walks a motorway or trunk road.
    pub fn open_to(&self, mode: TravelMode) -> bool {
        let allows = |tag: &Option<String>| {
            matches!(tag.as_deref(), Some("yes" | "designated" | "permissive"))
        };
        let closed = |tags: &[&Option<String>]| {
            tags.iter()
                .find_map(|tag| tag.as_deref())
                .is_some_and(|value| CLOSED_TO_CARS.contains(&value))
        };
        let highway = self.highway.as_deref().unwrap_or("");
        match mode {
            TravelMode::Car => self.open_to_cars(),
            TravelMode::Bicycle => {
                if matches!(highway, "footway" | "pedestrian") && !allows(&self.bicycle) {
                    return false;
                }
                !closed(&[&self.bicycle, &self.vehicle, &self.access])
            }
            TravelMode::Foot => {
                !highway.starts_with("motorway")
                    && !highway.starts_with("trunk")
                    && !closed(&[&self.foot, &self.access])
            }
        }
    }

    /// Directions a traveller in `mode` may follow the way: a walker either
    /// way; a bicycle as cars do, unless `oneway:bicycle=no` opens the way
    /// against the traffic.
    pub fn travel_direction_for(&self, mode: TravelMode) -> TravelDirection {
        match mode {
            TravelMode::Car => self.travel_direction(),
            TravelMode::Foot => TravelDirection::Both,
            TravelMode::Bicycle => match self.oneway_bicycle.as_deref() {
                Some("no" | "false" | "0") => TravelDirection::Both,
                _ => match self.travel_direction() {
                    // A reversible car lane is still ridden, in both directions.
                    TravelDirection::None => TravelDirection::Both,
                    direction => direction,
                },
            },
        }
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
