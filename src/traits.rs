use crate::types::{Control, State};

pub const GRAVITY: f64 = 9.81;

/// Result of a single integration step, with optional error estimate for adaptive solvers.
#[derive(Debug, Clone, Copy)]
pub struct StepResult<const N: usize> {
    pub state: State<N>,
    pub error_estimate: Option<f64>,
}

/// Core dynamics interface. All states expressed in the NED frame.
pub trait Dynamics<const N: usize, const M: usize> {
    type Input;

    /// Map user input into model-specific control vector.
    fn input_to_control(&self, input: &Self::Input) -> Control<M>;

    /// Time derivative: dx/dt = f(x, u).
    fn f(&self, state: &State<N>, control: &Control<M>) -> State<N>;
}

/// Extension for models that provide an analytical Jacobian for implicit solvers.
pub trait LinearizableDynamics<const N: usize, const M: usize>: Dynamics<N, M> {
    fn jacobian(
        &self,
        state: &State<N>,
        control: &Control<M>,
    ) -> nalgebra::DMatrix<f64>;
}

/// Time-marching integrator, monomorphized over the model for zero-cost dispatch.
pub trait Stepper<const N: usize, const M: usize, Model: Dynamics<N, M>> {
    fn step(
        &self,
        model: &Model,
        state: &State<N>,
        control: &Control<M>,
        dt: f64,
    ) -> StepResult<N>;
}

/// Feedback controller producing control vectors from state.
pub trait Controller<const N: usize, const M: usize> {
    fn control(&self, t: f64, state: &State<N>) -> Control<M>;
}

/// Holds a fixed control vector, ignoring time and state.
pub struct ConstantControl<const M: usize> {
    pub value: Control<M>,
}

impl<const N: usize, const M: usize> Controller<N, M> for ConstantControl<M> {
    fn control(&self, _t: f64, _state: &State<N>) -> Control<M> {
        self.value
    }
}

/// Step-by-step observer with early-exit support. Return false to stop integration.
pub trait Observer<const N: usize> {
    fn observe(&mut self, step: usize, t: f64, state: &State<N>) -> bool;
}

/// Default observer that stores every state.
pub struct StoreAll<const N: usize> {
    pub states: Vec<State<N>>,
}

impl<const N: usize> StoreAll<N> {
    pub fn with_capacity(cap: usize) -> Self {
        Self {
            states: Vec::with_capacity(cap),
        }
    }
}

impl<const N: usize> Observer<N> for StoreAll<N> {
    fn observe(&mut self, _step: usize, _t: f64, state: &State<N>) -> bool {
        self.states.push(*state);
        true
    }
}
