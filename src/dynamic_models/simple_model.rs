use crate::{
    traits::{Dynamics, GRAVITY},
    types::{Control, DroneInput, State},
};

pub struct SimpleModel {
    pub drag: f64,
}

impl Dynamics for SimpleModel {
    fn input_to_control(&self, input: &DroneInput) -> Control {
        // Body-frame accelerations
        // Pitch > 0 = Forward (X); Roll > 0 = Right (Y)
        let ax_body = GRAVITY * input.pitch.tan();
        let ay_body = -GRAVITY * input.roll.tan();

        // [ax_body, ay_body, yaw_rate]
        vec![ax_body, ay_body, input.yaw_rate]
    }

    fn f(&self, state: &State, control: &Control) -> State {
        let vx = state[2];
        let vy = state[3];
        let yaw = state[4];

        let ax_body = control[0];
        let ay_body = control[1];
        let yaw_rate = control[2];

        // 1. Rotate body acceleration into world frame
        let ax_world = ax_body * yaw.cos() - ay_body * yaw.sin();
        let ay_world = ax_body * yaw.sin() + ay_body * yaw.cos();

        // 2. Apply drag in the direction of world velocity
        let dvx = ax_world - (self.drag * vx);
        let dvy = ay_world - (self.drag * vy);

        vec![
            vx,       // x_dot
            vy,       // y_dot
            dvx,      // vx_dot
            dvy,      // vy_dot
            yaw_rate, // yaw_dot
        ]
    }
}
