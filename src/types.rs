/// Pilot stick inputs expressed in radians and rad/s.
/// Body frame is x-forward, y-right, z-down. Positive yaw is clockwise when viewed from above.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DroneInput {
    pub roll_rad: f64,     // radians, right wing down is positive
    pub pitch_rad: f64,    // radians, nose up is positive
    pub yaw_rate_rps: f64, // radians per second, clockwise is positive
}

/// Model-specific control vector for the simple quadcopter: body-frame accelerations and yaw rate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Control {
    pub ax_body_mps2: f64,
    pub ay_body_mps2: f64,
    pub yaw_rate_rps: f64,
}

/// Flattened planar NED state: [North, East, V_North, V_East, Yaw]
/// Yaw is radians, 0 faces North, positive is clockwise toward East.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub north_m: f64,
    pub east_m: f64,
    pub v_north_mps: f64,
    pub v_east_mps: f64,
    pub yaw_rad: f64,
}

impl State {
    pub fn new(north_m: f64, east_m: f64, v_north_mps: f64, v_east_mps: f64, yaw_rad: f64) -> Self {
        Self {
            north_m,
            east_m,
            v_north_mps,
            v_east_mps,
            yaw_rad,
        }
    }

    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0, 0.0)
    }

    pub fn xy(&self) -> (f64, f64) {
        (self.north_m, self.east_m)
    }

    pub fn ensure_finite(&self) {
        assert!(self.north_m.is_finite(), "north must be finite");
        assert!(self.east_m.is_finite(), "east must be finite");
        assert!(self.v_north_mps.is_finite(), "v_north must be finite");
        assert!(self.v_east_mps.is_finite(), "v_east must be finite");
        assert!(self.yaw_rad.is_finite(), "yaw must be finite");
    }
}

/// Minimal trait for states that can be integrated with explicit methods.
pub trait IntegrableState: Clone {
    fn add_scaled(&self, derivative: &Self, scale: f64) -> Self;
}

impl IntegrableState for State {
    fn add_scaled(&self, derivative: &Self, scale: f64) -> Self {
        Self {
            north_m: self.north_m + scale * derivative.north_m,
            east_m: self.east_m + scale * derivative.east_m,
            v_north_mps: self.v_north_mps + scale * derivative.v_north_mps,
            v_east_mps: self.v_east_mps + scale * derivative.v_east_mps,
            yaw_rad: self.yaw_rad + scale * derivative.yaw_rad,
        }
    }
}
