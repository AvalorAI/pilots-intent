use nalgebra::SVector;

/// Fixed-size state vector with compile-time dimension safety.
pub type State<const N: usize> = SVector<f64, N>;

/// Fixed-size control vector with compile-time dimension safety.
pub type Control<const M: usize> = SVector<f64, M>;

/// Pilot stick inputs expressed in radians and rad/s.
/// Body frame is x-forward, y-right, z-down. Positive yaw is clockwise when viewed from above.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DroneInput {
    pub roll_rad: f64,
    pub pitch_rad: f64,
    pub yaw_rate_rps: f64,
}
