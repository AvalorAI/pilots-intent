use nalgebra::{DMatrix, DVector};

use crate::{
    traits::{LinearizableDynamics, StepResult, Stepper},
    types::{Control, State},
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

impl<const N: usize, const M: usize, Model> Stepper<N, M, Model> for BackwardEuler
where
    Model: LinearizableDynamics<N, M>,
{
    fn step(
        &self,
        model: &Model,
        state: &State<N>,
        control: &Control<M>,
        dt: f64,
    ) -> StepResult<N> {
        assert!(dt.is_finite() && dt > 0.0, "dt must be finite and > 0");

        let u_prev = DVector::from_column_slice(state.as_slice());

        // F(x) = x - u_i - dt * f(x)
        let f_newton = |x: &DVector<f64>| -> DVector<f64> {
            let x_state = State::<N>::from_column_slice(x.as_slice());
            let fx = model.f(&x_state, control);
            let fx_vec = DVector::from_column_slice(fx.as_slice());
            x - &u_prev - fx_vec * dt
        };

        // J(x) = I - dt * df/dx
        let j_newton = |x: &DVector<f64>| -> DMatrix<f64> {
            let x_state = State::<N>::from_column_slice(x.as_slice());
            let j = model.jacobian(&x_state, control);
            assert!(
                j.nrows() == N && j.ncols() == N,
                "jacobian must be square with dimension matching the state"
            );
            DMatrix::<f64>::identity(N, N) - j * dt
        };

        let (x_next, _) = newton(f_newton, j_newton, u_prev.clone(), self.newton_opts);
        StepResult {
            state: State::<N>::from_column_slice(x_next.as_slice()),
            error_estimate: None,
        }
    }
}

impl BackwardEuler {
    pub fn stability(z: num_complex::Complex<f64>) -> num_complex::Complex<f64> {
        num_complex::Complex::new(1.0, 0.0) / (num_complex::Complex::new(1.0, 0.0) - z)
    }
}
