use crate::traits::{Dynamics, MultiStepper, StepResult};
use crate::types::{Control, State};

/// Adams-Bashforth 2-step explicit multi-step method.
///
/// ```text
/// x_{n+1} = x_n + dt/2 · (3·f_n − f_{n−1})
/// ```
///
/// Second-order accurate. Requires one previous derivative evaluation
/// (bootstrapped by a single-step method during startup).
#[derive(Clone, Copy, Debug, Default)]
pub struct Ab2;

impl<const N: usize, const M: usize, Model: Dynamics<N, M>> MultiStepper<N, M, Model> for Ab2 {
    fn history_len(&self) -> usize {
        2
    }

    fn step(
        &self,
        _model: &Model,
        state: &State<N>,
        f_history: &[State<N>],
        _control: &Control<M>,
        dt: f64,
    ) -> StepResult<N> {
        debug_assert!(dt.is_finite() && dt > 0.0);
        debug_assert_eq!(f_history.len(), 2);

        let f_prev = &f_history[0]; // f_{n-1}
        let f_curr = &f_history[1]; // f_n

        StepResult {
            state: *state + (*f_curr * 3.0 - *f_prev) * (dt / 2.0),
            error_estimate: None,
        }
    }
}
