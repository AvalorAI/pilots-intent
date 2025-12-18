use crate::{
    traits::{Dynamics, GRAVITY, LinearizableDynamics},
    types::{DroneInput, IntegrableState, Position2D, StateVector},
};

/// Planar NED quadcopter model using hover small-angle thrust and linear drag.
/// Body frame: x-forward, y-right, z-down. Yaw = 0 faces North, positive clockwise.
pub struct SimpleQuadcopter {
    pub drag: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimpleQuadState {
    pub north_m: f64,
    pub east_m: f64,
    pub v_north_mps: f64,
    pub v_east_mps: f64,
    pub yaw_rad: f64,
}

impl SimpleQuadState {
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

    pub fn ensure_finite(&self) {
        assert!(self.north_m.is_finite(), "north must be finite");
        assert!(self.east_m.is_finite(), "east must be finite");
        assert!(self.v_north_mps.is_finite(), "v_north must be finite");
        assert!(self.v_east_mps.is_finite(), "v_east must be finite");
        assert!(self.yaw_rad.is_finite(), "yaw must be finite");
    }
}

impl IntegrableState for SimpleQuadState {
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

impl Position2D for SimpleQuadState {
    fn position(&self) -> (f64, f64) {
        (self.north_m, self.east_m)
    }
}

impl StateVector for SimpleQuadState {
    fn to_dvector(&self) -> nalgebra::DVector<f64> {
        nalgebra::DVector::from_vec(vec![
            self.north_m,
            self.east_m,
            self.v_north_mps,
            self.v_east_mps,
            self.yaw_rad,
        ])
    }

    fn from_dvector(v: nalgebra::DVector<f64>) -> Self {
        assert!(
            v.len() == 5,
            "SimpleQuadState expects 5 elements (north, east, v_north, v_east, yaw)"
        );
        Self {
            north_m: v[0],
            east_m: v[1],
            v_north_mps: v[2],
            v_east_mps: v[3],
            yaw_rad: v[4],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimpleQuadControl {
    pub ax_body_mps2: f64,
    pub ay_body_mps2: f64,
    pub yaw_rate_rps: f64,
}

impl Dynamics for SimpleQuadcopter {
    type State = SimpleQuadState;
    type Control = SimpleQuadControl;

    fn input_to_control(&self, input: &DroneInput) -> Self::Control {
        // Limit tilt to avoid tan() blowing up near ±90°
        const MAX_TILT_RAD: f64 = std::f64::consts::FRAC_PI_2 * 0.95;
        let pitch = input.pitch_rad.clamp(-MAX_TILT_RAD, MAX_TILT_RAD);
        let roll = input.roll_rad.clamp(-MAX_TILT_RAD, MAX_TILT_RAD);

        // Small-angle hover approximation: a_forward ≈ g * tan(pitch)
        // Body frame: x-forward, y-right, z-down
        SimpleQuadControl {
            ax_body_mps2: GRAVITY * pitch.tan(),
            ay_body_mps2: -GRAVITY * roll.tan(),
            yaw_rate_rps: input.yaw_rate_rps,
        }
    }

    fn derivative(&self, _t: f64, state: &Self::State, control: &Self::Control) -> Self::State {
        state.ensure_finite();

        let c = state.yaw_rad.cos();
        let s = state.yaw_rad.sin();

        // Rotate body accelerations into NED (x = North, y = East)
        let ax_n = control.ax_body_mps2 * c - control.ay_body_mps2 * s;
        let ay_e = control.ax_body_mps2 * s + control.ay_body_mps2 * c;

        // Linear drag in N/E directions
        let dv_n = ax_n - self.drag * state.v_north_mps;
        let dv_e = ay_e - self.drag * state.v_east_mps;

        SimpleQuadState {
            north_m: state.v_north_mps,    // north_dot
            east_m: state.v_east_mps,      // east_dot
            v_north_mps: dv_n,             // v_north_dot
            v_east_mps: dv_e,              // v_east_dot
            yaw_rad: control.yaw_rate_rps, // yaw_dot
        }
    }

    fn validate_state(&self, state: &Self::State) {
        state.ensure_finite();
    }
}

impl LinearizableDynamics for SimpleQuadcopter {
    fn jacobian(
        &self,
        _t: f64,
        state: &Self::State,
        control: &Self::Control,
    ) -> nalgebra::DMatrix<f64> {
        let c = state.yaw_rad.cos();
        let s = state.yaw_rad.sin();

        // Partials of rotated accelerations w.r.t yaw
        let dax_dyaw = -control.ax_body_mps2 * s - control.ay_body_mps2 * c;
        let day_dyaw = control.ax_body_mps2 * c - control.ay_body_mps2 * s;

        nalgebra::DMatrix::from_row_slice(
            5,
            5,
            &[
                0.0, 0.0, 1.0, 0.0, 0.0, // d(north_dot)/d(state)
                0.0, 0.0, 0.0, 1.0, 0.0, // d(east_dot)/d(state)
                0.0, 0.0, -self.drag, 0.0, dax_dyaw, // d(v_n_dot)/d(yaw)
                0.0, 0.0, 0.0, -self.drag, day_dyaw, // d(v_e_dot)/d(yaw)
                0.0, 0.0, 0.0, 0.0, 0.0, // d(yaw_dot)/d(state)
            ],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> SimpleQuadcopter {
        SimpleQuadcopter { drag: 0.1 }
    }

    #[test]
    fn pitch_forward_pushes_north_when_yaw_zero() {
        let input = DroneInput {
            roll_rad: 0.0,
            pitch_rad: 10f64.to_radians(),
            yaw_rate_rps: 0.0,
        };

        let control = model().input_to_control(&input);
        let state = SimpleQuadState::zero();
        let dx = model().derivative(0.0, &state, &control);

        assert!(dx.v_north_mps > 0.0);
        assert!(dx.v_east_mps.abs() < 1e-9);
    }

    #[test]
    fn roll_right_pushes_east_when_yaw_zero() {
        let input = DroneInput {
            roll_rad: 5f64.to_radians(),
            pitch_rad: 0.0,
            yaw_rate_rps: 0.0,
        };

        let control = model().input_to_control(&input);
        let state = SimpleQuadState::zero();
        let dx = model().derivative(0.0, &state, &control);

        assert!(dx.v_east_mps > 0.0);
        assert!(dx.v_north_mps.abs() < 1e-9);
    }

    #[test]
    fn yaw_rotates_body_frame_into_ned() {
        let input = DroneInput {
            roll_rad: 0.0,
            pitch_rad: 10f64.to_radians(),
            yaw_rate_rps: 0.0,
        };
        let control = model().input_to_control(&input);

        // 90 deg yaw should rotate forward accel into East
        let state = SimpleQuadState::new(0.0, 0.0, 0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let dx = model().derivative(0.0, &state, &control);

        assert!(dx.v_east_mps > 0.0);
        assert!(dx.v_north_mps.abs() < 1e-9);
    }

    #[test]
    fn yaw_rate_propagates_heading() {
        let input = DroneInput {
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rate_rps: 1.0,
        };
        let control = model().input_to_control(&input);
        let state = SimpleQuadState::zero();
        let dx = model().derivative(0.0, &state, &control);

        assert_eq!(dx.yaw_rad, 1.0);
    }
}
