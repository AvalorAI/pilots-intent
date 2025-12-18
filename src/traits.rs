use crate::types::{Control, DroneInput, State};

pub const GRAVITY: f64 = 9.81;

pub trait Dynamics {
    fn input_to_control(&self, input: &DroneInput) -> Control;

    fn f(&self, state: &State, control: &Control) -> State;
}

pub trait Stepper {
    fn step(&self, model: &dyn Dynamics, control: &Control, state: &State, dt: f64) -> State;
}
