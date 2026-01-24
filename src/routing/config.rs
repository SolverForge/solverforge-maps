//! Configuration for road network operations.

use std::path::PathBuf;
use std::time::Duration;

#[derive(Debug, Clone)]
pub struct SpeedProfile {
    pub motorway: f64,
    pub trunk: f64,
    pub primary: f64,
    pub secondary: f64,
    pub tertiary: f64,
    pub residential: f64,
    pub unclassified: f64,
    pub service: f64,
    pub living_street: f64,
    pub default: f64,
}

impl Default for SpeedProfile {
    fn default() -> Self {
        Self {
            motorway: 100.0,
            trunk: 80.0,
            primary: 60.0,
            secondary: 50.0,
            tertiary: 40.0,
            residential: 30.0,
            unclassified: 30.0,
            service: 20.0,
            living_street: 10.0,
            default: 30.0,
        }
    }
}

impl SpeedProfile {
    pub fn speed_mps(&self, maxspeed: Option<&str>, highway: &str) -> f64 {
        let kmh = maxspeed
            .and_then(|v| self.parse_maxspeed(v))
            .unwrap_or_else(|| self.highway_speed(highway));
        kmh * 1000.0 / 3600.0
    }

    fn highway_speed(&self, highway: &str) -> f64 {
        match highway {
            "motorway" | "motorway_link" => self.motorway,
            "trunk" | "trunk_link" => self.trunk,
            "primary" | "primary_link" => self.primary,
            "secondary" | "secondary_link" => self.secondary,
            "tertiary" | "tertiary_link" => self.tertiary,
            "residential" => self.residential,
            "unclassified" => self.unclassified,
            "service" => self.service,
            "living_street" => self.living_street,
            _ => self.default,
        }
    }

    fn parse_maxspeed(&self, value: &str) -> Option<f64> {
        let value = value.trim();

        match value.to_lowercase().as_str() {
            "walk" => return Some(5.0),
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

        Some(if is_mph { kmh * 1.60934 } else { kmh })
    }
}

#[derive(Debug, Clone)]
pub struct NetworkConfig {
    pub overpass_url: String,
    pub cache_dir: PathBuf,
    pub connect_timeout: Duration,
    pub read_timeout: Duration,
    pub speed_profile: SpeedProfile,
    pub highway_types: Vec<&'static str>,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            overpass_url: "https://overpass-api.de/api/interpreter".to_string(),
            cache_dir: PathBuf::from(".osm_cache"),
            connect_timeout: Duration::from_secs(30),
            read_timeout: Duration::from_secs(180),
            speed_profile: SpeedProfile::default(),
            highway_types: vec![
                "motorway",
                "trunk",
                "primary",
                "secondary",
                "tertiary",
                "residential",
                "unclassified",
                "service",
                "living_street",
            ],
        }
    }
}

impl NetworkConfig {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn overpass_url(mut self, url: impl Into<String>) -> Self {
        self.overpass_url = url.into();
        self
    }

    pub fn cache_dir(mut self, path: impl Into<PathBuf>) -> Self {
        self.cache_dir = path.into();
        self
    }

    pub fn connect_timeout(mut self, timeout: Duration) -> Self {
        self.connect_timeout = timeout;
        self
    }

    pub fn read_timeout(mut self, timeout: Duration) -> Self {
        self.read_timeout = timeout;
        self
    }

    pub fn speed_profile(mut self, profile: SpeedProfile) -> Self {
        self.speed_profile = profile;
        self
    }

    pub fn highway_types(mut self, types: Vec<&'static str>) -> Self {
        self.highway_types = types;
        self
    }

    pub(crate) fn highway_regex(&self) -> String {
        format!("^({})$", self.highway_types.join("|"))
    }
}
