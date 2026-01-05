use crate::{
    traits::{Dynamics, Stepper},
    types::IntegrableState,
};

/// Classic fixed-step Runge-Kutta 4 integrator.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rk4;

impl<M: Dynamics> Stepper<M> for Rk4 {
    fn step(
        &mut self,
        model: &M,
        t: f64,
        state: &M::State,
        control: &M::Control,
        dt: f64,
    ) -> M::State {
        assert!(dt.is_finite() && dt > 0.0, "dt must be finite and > 0");

        let half_dt = 0.5 * dt;

        let k1 = model.derivative(t, state, control);
        let state_k2 = state.add_scaled(&k1, half_dt);

        let k2 = model.derivative(t + half_dt, &state_k2, control);
        let state_k3 = state.add_scaled(&k2, half_dt);

        let k3 = model.derivative(t + half_dt, &state_k3, control);
        let state_k4 = state.add_scaled(&k3, dt);

        let k4 = model.derivative(t + dt, &state_k4, control);

        // x_{n+1} = x_n + dt/6 * (k1 + 2k2 + 2k3 + k4)
        let dt_over_6 = dt / 6.0;
        let incr = k1
            .add_scaled(&k2, 2.0)
            .add_scaled(&k3, 2.0)
            .add_scaled(&k4, 1.0);

        state.add_scaled(&incr, dt_over_6)
    }
}
