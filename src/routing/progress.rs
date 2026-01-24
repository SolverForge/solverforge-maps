//! Progress events for routing operations.

use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "phase", rename_all = "snake_case")]
pub enum RoutingProgress {
    CheckingCache {
        percent: u8,
    },
    DownloadingNetwork {
        percent: u8,
        bytes: usize,
    },
    ParsingOsm {
        percent: u8,
        nodes: usize,
        edges: usize,
    },
    BuildingGraph {
        percent: u8,
    },
    ComputingMatrix {
        percent: u8,
        row: usize,
        total: usize,
    },
    ComputingGeometries {
        percent: u8,
        pair: usize,
        total: usize,
    },
    EncodingGeometries {
        percent: u8,
    },
    Complete,
}

impl RoutingProgress {
    pub fn percent(&self) -> u8 {
        match self {
            Self::CheckingCache { percent } => *percent,
            Self::DownloadingNetwork { percent, .. } => *percent,
            Self::ParsingOsm { percent, .. } => *percent,
            Self::BuildingGraph { percent } => *percent,
            Self::ComputingMatrix { percent, .. } => *percent,
            Self::ComputingGeometries { percent, .. } => *percent,
            Self::EncodingGeometries { percent } => *percent,
            Self::Complete => 100,
        }
    }

    pub fn phase_message(&self) -> (&'static str, &'static str) {
        match self {
            Self::CheckingCache { .. } => ("cache", "Checking cache..."),
            Self::DownloadingNetwork { .. } => ("network", "Downloading road network..."),
            Self::ParsingOsm { .. } => ("parsing", "Parsing OSM data..."),
            Self::BuildingGraph { .. } => ("building", "Building routing graph..."),
            Self::ComputingMatrix { .. } => ("matrix", "Computing travel times..."),
            Self::ComputingGeometries { .. } => ("geometry", "Computing route geometries..."),
            Self::EncodingGeometries { .. } => ("encoding", "Encoding geometries..."),
            Self::Complete => ("complete", "Ready!"),
        }
    }

    pub fn detail(&self) -> Option<String> {
        match self {
            Self::DownloadingNetwork { bytes, .. } if *bytes > 0 => {
                Some(format!("{} KB downloaded", bytes / 1024))
            }
            Self::ParsingOsm { nodes, edges, .. } => {
                Some(format!("{} nodes, {} edges", nodes, edges))
            }
            Self::ComputingMatrix { row, total, .. } => Some(format!("Row {}/{}", row, total)),
            Self::ComputingGeometries { pair, total, .. } => {
                Some(format!("Pair {}/{}", pair, total))
            }
            _ => None,
        }
    }
}
