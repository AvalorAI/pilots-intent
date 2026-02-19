use crate::traits::{Dynamics, StepResult, Stepper};
use crate::types::{Control, State};

#[derive(Clone, Copy, Debug, Default)]
pub struct ForwardEuler;

impl ForwardEuler {
    pub fn stability(z: num_complex::Complex<f64>) -> num_complex::Complex<f64> {
        num_complex::Complex::new(1.0, 0.0) + z
    }
}

impl<const N: usize, const M: usize, Model: Dynamics<N, M>> Stepper<N, M, Model> for ForwardEuler {
    fn step(
        &self,
        model: &Model,
        state: &State<N>,
        control: &Control<M>,
        dt: f64,
    ) -> StepResult<N> {
        debug_assert!(dt.is_finite(), "dt must be finite");
        debug_assert!(dt > 0.0, "dt must be > 0");

        let dx = model.f(state, control);
        StepResult {
            state: *state + dx * dt,
            error_estimate: None,
        }
    }
}
