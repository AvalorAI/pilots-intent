use crate::{
    traits::{Dynamics, LinearizableDynamics, GRAVITY},
    types::{Control, DroneInput, State},
};

/// Planar NED quadcopter model using hover small-angle thrust and linear drag.
///
/// State indices: `[north_m, east_m, v_north_mps, v_east_mps, yaw_rad]`
/// Control indices: `[ax_body_mps2, ay_body_mps2, yaw_rate_rps]`
pub struct SimpleQuadcopter {
    pub drag: f64,
}

impl Dynamics<5, 3> for SimpleQuadcopter {
    type Input = DroneInput;

    fn input_to_control(&self, input: &Self::Input) -> Control<3> {
        const MAX_TILT_RAD: f64 = std::f64::consts::FRAC_PI_2 * 0.95;
        let pitch = input.pitch_rad.clamp(-MAX_TILT_RAD, MAX_TILT_RAD);
        let roll = input.roll_rad.clamp(-MAX_TILT_RAD, MAX_TILT_RAD);

        Control::<3>::new(
            GRAVITY * pitch.tan(),
            GRAVITY * roll.tan(),
            input.yaw_rate_rps,
        )
    }

    fn f(&self, state: &State<5>, control: &Control<3>) -> State<5> {
        let v_north = state[2];
        let v_east = state[3];
        let yaw = state[4];

        let ax_body = control[0];
        let ay_body = control[1];
        let yaw_rate = control[2];

        let (s, c) = yaw.sin_cos();

        // Rotate body accelerations into NED (x = North, y = East)
        let ax_n = ax_body * c - ay_body * s;
        let ay_e = ax_body * s + ay_body * c;

        State::<5>::new(
            v_north,                    // north_dot
            v_east,                     // east_dot
            ax_n - self.drag * v_north, // v_north_dot
            ay_e - self.drag * v_east,  // v_east_dot
            yaw_rate,                   // yaw_dot
        )
    }
}

impl LinearizableDynamics<5, 3> for SimpleQuadcopter {
    fn jacobian(
        &self,
        state: &State<5>,
        control: &Control<3>,
    ) -> nalgebra::DMatrix<f64> {
        let yaw = state[4];
        let ax_body = control[0];
        let ay_body = control[1];

        let (s, c) = yaw.sin_cos();

        let dax_dyaw = -ax_body * s - ay_body * c;
        let day_dyaw = ax_body * c - ay_body * s;

        nalgebra::DMatrix::from_row_slice(
            5,
            5,
            &[
                0.0, 0.0, 1.0, 0.0, 0.0,          // d(north_dot)/d(state)
                0.0, 0.0, 0.0, 1.0, 0.0,          // d(east_dot)/d(state)
                0.0, 0.0, -self.drag, 0.0, dax_dyaw, // d(v_n_dot)/d(state)
                0.0, 0.0, 0.0, -self.drag, day_dyaw, // d(v_e_dot)/d(state)
                0.0, 0.0, 0.0, 0.0, 0.0,          // d(yaw_dot)/d(state)
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

    fn zero_state() -> State<5> {
        State::<5>::zeros()
    }

    #[test]
    fn pitch_forward_pushes_north_when_yaw_zero() {
        let input = DroneInput {
            roll_rad: 0.0,
            pitch_rad: 10f64.to_radians(),
            yaw_rate_rps: 0.0,
        };

        let control = model().input_to_control(&input);
        let dx = model().f(&zero_state(), &control);

        assert!(dx[2] > 0.0, "v_north should increase");
        assert!(dx[3].abs() < 1e-9, "v_east should be ~0");
    }

    #[test]
    fn roll_right_pushes_east_when_yaw_zero() {
        let input = DroneInput {
            roll_rad: 5f64.to_radians(),
            pitch_rad: 0.0,
            yaw_rate_rps: 0.0,
        };

        let control = model().input_to_control(&input);
        let dx = model().f(&zero_state(), &control);

        assert!(dx[3] > 0.0, "v_east should increase");
        assert!(dx[2].abs() < 1e-9, "v_north should be ~0");
    }

    #[test]
    fn yaw_rotates_body_frame_into_ned() {
        let input = DroneInput {
            roll_rad: 0.0,
            pitch_rad: 10f64.to_radians(),
            yaw_rate_rps: 0.0,
        };
        let control = model().input_to_control(&input);

        // 90 deg yaw: forward accel should become East
        let state = State::<5>::new(0.0, 0.0, 0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let dx = model().f(&state, &control);

        assert!(dx[3] > 0.0, "v_east should increase");
        assert!(dx[2].abs() < 1e-9, "v_north should be ~0");
    }

    #[test]
    fn yaw_rate_propagates_heading() {
        let input = DroneInput {
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rate_rps: 1.0,
        };
        let control = model().input_to_control(&input);
        let dx = model().f(&zero_state(), &control);

        assert_eq!(dx[4], 1.0);
    }
}
