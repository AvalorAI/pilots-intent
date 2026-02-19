use crate::traits::{Dynamics, StepResult, Stepper};
use crate::types::{Control, State};

/// Classic fixed-step Runge-Kutta 4 integrator.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rk4;

impl<const N: usize, const M: usize, Model: Dynamics<N, M>> Stepper<N, M, Model> for Rk4 {
    fn step(
        &self,
        model: &Model,
        state: &State<N>,
        control: &Control<M>,
        dt: f64,
    ) -> StepResult<N> {
        debug_assert!(dt.is_finite() && dt > 0.0, "dt must be finite and > 0");

        let half_dt = 0.5 * dt;

        let k1 = model.f(state, control);
        let s2 = *state + k1 * half_dt;

        let k2 = model.f(&s2, control);
        let s3 = *state + k2 * half_dt;

        let k3 = model.f(&s3, control);
        let s4 = *state + k3 * dt;

        let k4 = model.f(&s4, control);

        let new_state = *state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
        StepResult {
            state: new_state,
            error_estimate: None,
        }
    }
}
