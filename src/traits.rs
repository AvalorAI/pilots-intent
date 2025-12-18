use crate::types::{DroneInput, IntegrableState};

pub const GRAVITY: f64 = 9.81;

/// Core interface a dynamic model must implement.
/// All states are assumed to be expressed in the NED frame (North-East-Down).
pub trait Dynamics {
    type State: IntegrableState;
    type Control: Clone;

    fn input_to_control(&self, input: &DroneInput) -> Self::Control;

    fn f(&self, state: &Self::State, control: &Self::Control) -> Self::State;

    fn dfdx(&self, _state: &Self::State, _control: &Self::Control) -> Vec<Vec<f64>> {
        unimplemented!("Analytical Jacobian not implemented for this model");
    }
}

pub trait Stepper<M: Dynamics> {
    fn step(&self, model: &M, state: &M::State, control: &M::Control, dt: f64) -> M::State;
}
