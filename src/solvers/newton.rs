use nalgebra::{DMatrix, DVector};

#[derive(Clone, Copy, Debug)]
pub struct NewtonOpts {
    pub iter_max: usize,
    pub min_error: f64,
}

impl Default for NewtonOpts {
    fn default() -> Self {
        Self {
            iter_max: 15,
            min_error: 1e-10,
        }
    }
}

/// Newton method for vector root finding: solve `F(x) = 0`.
/// Returns (solution, iterate history).
pub fn newton<F, J>(
    f: F,
    dfdx: J,
    x0: DVector<f64>,
    opts: NewtonOpts,
) -> (DVector<f64>, Vec<DVector<f64>>)
where
    F: Fn(&DVector<f64>) -> DVector<f64>,
    J: Fn(&DVector<f64>) -> DMatrix<f64>,
{
    let mut x_hist = Vec::with_capacity(opts.iter_max);
    let mut x = x0;
    x_hist.push(x.clone());

    for _ in 1..opts.iter_max {
        let fx = f(&x);
        let max_abs = fx.iter().fold(0.0_f64, |acc, &v| acc.max(v.abs()));
        if max_abs < opts.min_error {
            break;
        }

        let jx = dfdx(&x);
        let delta = jx
            .lu()
            .solve(&fx)
            .expect("Newton: Jacobian is singular / solve failed");

        x = x - delta;
        x_hist.push(x.clone());
    }

    (x, x_hist)
}
