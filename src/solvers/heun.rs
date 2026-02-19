use crate::traits::{Dynamics, StepResult, Stepper};
use crate::types::{Control, State};

/// Heun's method (explicit RK2, improved Euler).
///
/// ```text
/// k1 = f(x_n,  u)
/// k2 = f(x_n + dt·k1,  u)
/// x_{n+1} = x_n + dt/2 · (k1 + k2)
/// ```
///
/// Second-order accurate, two function evaluations per step.
#[derive(Clone, Copy, Debug, Default)]
pub struct Heun;

impl<const N: usize, const M: usize, Model: Dynamics<N, M>> Stepper<N, M, Model> for Heun {
    fn step(
        &self,
        model: &Model,
        state: &State<N>,
        control: &Control<M>,
        dt: f64,
    ) -> StepResult<N> {
        debug_assert!(dt.is_finite() && dt > 0.0, "dt must be finite and > 0");

        let k1 = model.f(state, control);
        let s_tilde = *state + k1 * dt;
        let k2 = model.f(&s_tilde, control);

        StepResult {
            state: *state + (k1 + k2) * (0.5 * dt),
            error_estimate: None,
        }
    }
}
