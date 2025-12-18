use crate::{
    traits::{Dynamics, Stepper},
    types::{Control, State},
};

#[derive(Clone, Copy, Debug, Default)]
pub struct ForwardEuler;

impl ForwardEuler {
    pub fn stability(z: num_complex::Complex<f64>) -> num_complex::Complex<f64> {
        num_complex::Complex::new(1.0, 0.0) + z
    }
}

impl Stepper for ForwardEuler {
    fn step(&self, model: &dyn Dynamics, state: &State, control: &Control, dt: f64) -> State {
        assert!(dt.is_finite(), "dt must be finite");
        assert!(dt > 0.0, "dt must be > 0");

        let dx = model.f(state, control);
        assert_eq!(dx.len(), state.len(), "State dimension mismatch");

        state
            .iter()
            .zip(dx.iter())
            .map(|(x, dx)| x + dt * dx)
            .collect()
    }
}
