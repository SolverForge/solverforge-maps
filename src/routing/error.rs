//! Error types for routing operations.

/// Error type for routing operations.
#[derive(Debug)]
pub enum RoutingError {
    /// Network request failed.
    Network(String),
    /// Failed to parse OSM data.
    Parse(String),
    /// I/O error.
    Io(std::io::Error),
    /// No route found.
    NoRoute,
}

impl std::fmt::Display for RoutingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RoutingError::Network(msg) => write!(f, "Network error: {}", msg),
            RoutingError::Parse(msg) => write!(f, "Parse error: {}", msg),
            RoutingError::Io(e) => write!(f, "I/O error: {}", e),
            RoutingError::NoRoute => write!(f, "No route found"),
        }
    }
}

impl std::error::Error for RoutingError {}

impl From<std::io::Error> for RoutingError {
    fn from(e: std::io::Error) -> Self {
        RoutingError::Io(e)
    }
}
