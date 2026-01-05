use nalgebra::{DMatrix, DVector};

use crate::{
    traits::{LinearizableDynamics, Stepper},
    types::StateVector,
};

use super::newton::{NewtonOpts, newton};

/// Backward Euler implicit integrator using Newton's method.
#[derive(Clone, Debug)]
pub struct BackwardEuler {
    pub newton_opts: NewtonOpts,
}

impl Default for BackwardEuler {
    fn default() -> Self {
        Self {
            newton_opts: NewtonOpts::default(),
        }
    }
}

impl<M> Stepper<M> for BackwardEuler
where
    M: LinearizableDynamics,
    M::State: StateVector,
{
    fn step(
        &mut self,
        model: &M,
        t: f64,
        state: &M::State,
        control: &M::Control,
        dt: f64,
    ) -> M::State {
        assert!(dt.is_finite() && dt > 0.0, "dt must be finite and > 0");

        let u_prev = state.to_dvector();
        let m = u_prev.len();
        assert!(m > 0, "state dimension must be > 0");

        // F(x) = x - u_i - dt * f(t+dt, x)
        let f_newton = |x: &DVector<f64>| -> DVector<f64> {
            let x_state = M::State::from_dvector(x.clone());
            let fx = model.derivative(t + dt, &x_state, control);
            let fx_vec = fx.to_dvector();
            x - &u_prev - fx_vec * dt
        };

        // J(x) = I - dt * df/dx
        let j_newton = |x: &DVector<f64>| -> DMatrix<f64> {
            let x_state = M::State::from_dvector(x.clone());
            let j = model.jacobian(t + dt, &x_state, control);
            assert!(
                j.nrows() == m && j.ncols() == m,
                "jacobian must be square with dimension matching the state"
            );
            DMatrix::<f64>::identity(m, m) - j * dt
        };

        let (x_next, _) = newton(f_newton, j_newton, u_prev.clone(), self.newton_opts);
        M::State::from_dvector(x_next)
    }
}

impl BackwardEuler {
    pub fn stability(z: num_complex::Complex<f64>) -> num_complex::Complex<f64> {
        num_complex::Complex::new(1.0, 0.0) / (num_complex::Complex::new(1.0, 0.0) - z)
    }
}
