use crate::{
    traits::{Dynamics, GRAVITY},
    types::{Control, DroneInput, State},
};

pub struct SimpleQuadcopter {
    pub drag: f64,
}

impl Dynamics for SimpleQuadcopter {
    fn input_to_control(&self, input: &DroneInput) -> Control {
        // Body-frame accelerations
        // Pitch > 0 = Forward (X); Roll > 0 = Right (Y)
        let ax_body = GRAVITY * input.pitch.tan();
        let ay_body = GRAVITY * input.roll.tan();

        // [ax_body, ay_body, yaw_rate]
        vec![ax_body, ay_body]
    }

    fn f(&self, state: &State, control: &Control) -> State {
        let vx = state[2];
        let vy = state[3];
        let yaw = state[4];

        let ax_body = control[0];
        let ay_body = control[1];

        let theta = -(yaw + std::f64::consts::FRAC_PI_2);

        // Rotate body acceleration into NED world frame
        let ax_world = ax_body * theta.cos() - ay_body * theta.sin();
        let ay_world = ax_body * theta.sin() + ay_body * theta.cos();

        // 2. Apply drag in the direction of world velocity
        let dvx = ax_world - (self.drag * vx);
        let dvy = ay_world - (self.drag * vy);

        vec![
            vx,  // x_dot
            vy,  // y_dot
            dvx, // vx_dot
            dvy, // vy_dot
            0.0, // yaw_rate, // yaw_dot
        ]
    }
}
