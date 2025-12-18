use crate::types::{DroneInput, IntegrableState};

pub const GRAVITY: f64 = 9.81;

/// Core interface a dynamic model must implement.
/// All states are assumed to be expressed in the NED frame (North-East-Down).
pub trait Dynamics {
    type State: IntegrableState;
    type Control: Clone;

    /// Map user input into model-specific control.
    fn input_to_control(&self, input: &DroneInput) -> Self::Control;

    /// Time derivative: dx/dt = f(t, x, u)
    fn derivative(&self, t: f64, state: &Self::State, control: &Self::Control) -> Self::State;

    /// Optional validation hook for states.
    fn validate_state(&self, _state: &Self::State) {}

    fn dfdx(&self, _state: &Self::State, _control: &Self::Control) -> Vec<Vec<f64>> {
        unimplemented!("Analytical Jacobian not implemented for this model");
    }
}

/// Optional extension for models that can provide a dense Jacobian for implicit
/// or stability-aware solvers.
pub trait LinearizableDynamics: Dynamics {
    fn jacobian(
        &self,
        t: f64,
        state: &Self::State,
        control: &Self::Control,
    ) -> nalgebra::DMatrix<f64>;
}

pub trait Stepper<M: Dynamics> {
    fn step(
        &mut self,
        model: &M,
        t: f64,
        state: &M::State,
        control: &M::Control,
        dt: f64,
    ) -> M::State;
}
