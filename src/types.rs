#[derive(Debug)]

pub struct DroneInput {
    pub roll: f64,     // radians
    pub pitch: f64,    // radians
    pub yaw_rate: f64, // radians/sec (held constant)
}

/// Flattened state vector: [x, y, vx, vy]
pub type State = Vec<f64>;

/// Model-specific control vector
pub type Control = Vec<f64>;
