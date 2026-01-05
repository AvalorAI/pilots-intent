use crate::{
    traits::{Dynamics, Stepper},
    types::IntegrableState,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct ForwardEuler;

impl ForwardEuler {
    pub fn stability(z: num_complex::Complex<f64>) -> num_complex::Complex<f64> {
        num_complex::Complex::new(1.0, 0.0) + z
    }
}

impl<M: Dynamics> Stepper<M> for ForwardEuler {
    fn step(
        &mut self,
        model: &M,
        t: f64,
        state: &M::State,
        control: &M::Control,
        dt: f64,
    ) -> M::State {
        assert!(dt.is_finite(), "dt must be finite");
        assert!(dt > 0.0, "dt must be > 0");

        let dx = model.derivative(t, state, control);
        state.add_scaled(&dx, dt)
    }
}
