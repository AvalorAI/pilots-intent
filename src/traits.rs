use crate::types::{Control, DroneInput, State};

pub const GRAVITY: f64 = 9.81;

pub trait Dynamics {
    fn input_to_control(&self, input: &DroneInput) -> Control;

    fn f(&self, state: &State, control: &Control) -> State;

    fn dfdx(&self, _state: &State, _control: &Control) -> Vec<Vec<f64>> {
        unimplemented!("Analytical Jacobian not implemented for this model");
    }
}

pub trait Stepper {
    fn step(&self, model: &dyn Dynamics, control: &Control, state: &State, dt: f64) -> State;
}
