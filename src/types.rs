/// Pilot stick inputs expressed in radians and rad/s.
/// Body frame is x-forward, y-right, z-down. Positive yaw is clockwise when viewed from above.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DroneInput {
    pub roll_rad: f64,     // radians, right wing down is positive
    pub pitch_rad: f64,    // radians, nose up is positive
    pub yaw_rate_rps: f64, // radians per second, clockwise is positive
}

/// Minimal trait for states that can be integrated with time-marching methods.
/// The derivative has the same shape as the state.
pub trait IntegrableState: Clone {
    fn add_scaled(&self, derivative: &Self, scale: f64) -> Self;
}

/// Optional helper for anything that can be projected into a 2D plot.
pub trait Position2D {
    fn position(&self) -> (f64, f64);
}

/// States that can be viewed as dense vectors for matrix-based solvers/analysis.
pub trait StateVector: IntegrableState {
    fn to_dvector(&self) -> nalgebra::DVector<f64>;
    fn from_dvector(v: nalgebra::DVector<f64>) -> Self;
}
