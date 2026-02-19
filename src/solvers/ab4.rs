use crate::traits::{Dynamics, MultiStepper, StepResult};
use crate::types::{Control, State};

/// Adams-Bashforth 4-step explicit multi-step method.
///
/// ```text
/// x_{n+1} = x_n + dt/24 · (55·f_n − 59·f_{n−1} + 37·f_{n−2} − 9·f_{n−3})
/// ```
///
/// Fourth-order accurate. Requires three previous derivative evaluations
/// (bootstrapped by a single-step method during startup).
#[derive(Clone, Copy, Debug, Default)]
pub struct Ab4;

impl<const N: usize, const M: usize, Model: Dynamics<N, M>> MultiStepper<N, M, Model> for Ab4 {
    fn history_len(&self) -> usize {
        4
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
        debug_assert_eq!(f_history.len(), 4);

        let f3 = &f_history[0]; // f_{n-3}
        let f2 = &f_history[1]; // f_{n-2}
        let f1 = &f_history[2]; // f_{n-1}
        let f0 = &f_history[3]; // f_n

        let weighted = *f0 * 55.0 - *f1 * 59.0 + *f2 * 37.0 - *f3 * 9.0;

        StepResult {
            state: *state + weighted * (dt / 24.0),
            error_estimate: None,
        }
    }
}
