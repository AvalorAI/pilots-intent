use crate::{
    traits::{Dynamics, GRAVITY},
    types::{Control, DroneInput, State},
};

/// Planar NED quadcopter model using hover small-angle thrust and linear drag.
/// Body frame: x-forward, y-right, z-down. Yaw = 0 faces North, positive clockwise.
pub struct SimpleQuadcopter {
    pub drag: f64,
}

impl Dynamics for SimpleQuadcopter {
    type State = State;
    type Control = Control;

    fn input_to_control(&self, input: &DroneInput) -> Control {
        // Limit tilt to avoid tan() blowing up near ±90°
        const MAX_TILT_RAD: f64 = std::f64::consts::FRAC_PI_2 * 0.95;
        let pitch = input.pitch_rad.clamp(-MAX_TILT_RAD, MAX_TILT_RAD);
        let roll = input.roll_rad.clamp(-MAX_TILT_RAD, MAX_TILT_RAD);

        // Small-angle hover approximation: a_forward ≈ g * tan(pitch)
        // Body frame: x-forward, y-right, z-down
        Control {
            ax_body_mps2: GRAVITY * pitch.tan(),
            ay_body_mps2: GRAVITY * roll.tan(),
            yaw_rate_rps: input.yaw_rate_rps,
        }
    }

    fn f(&self, state: &State, control: &Control) -> State {
        state.ensure_finite();

        let c = state.yaw_rad.cos();
        let s = state.yaw_rad.sin();

        // Rotate body accelerations into NED (x = North, y = East)
        let ax_n = control.ax_body_mps2 * c - control.ay_body_mps2 * s;
        let ay_e = control.ax_body_mps2 * s + control.ay_body_mps2 * c;

        // Linear drag in N/E directions
        let dv_n = ax_n - self.drag * state.v_north_mps;
        let dv_e = ay_e - self.drag * state.v_east_mps;

        State {
            north_m: state.v_north_mps,    // north_dot
            east_m: state.v_east_mps,      // east_dot
            v_north_mps: dv_n,             // v_north_dot
            v_east_mps: dv_e,              // v_east_dot
            yaw_rad: control.yaw_rate_rps, // yaw_dot
        }
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
        let state = State::zero();
        let dx = model().f(&state, &control);

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
        let state = State::zero();
        let dx = model().f(&state, &control);

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
        let state = State::new(0.0, 0.0, 0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let dx = model().f(&state, &control);

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
        let state = State::zero();
        let dx = model().f(&state, &control);

        assert_eq!(dx.yaw_rad, 1.0);
    }
}
