use std::time::{Duration, Instant};

use crate::{
    traits::{Dynamics, Stepper},
    types::{DroneInput, State},
};

#[derive(Debug, Clone)]
pub struct Prediction {
    pub states: Vec<State>, // x(0), x(dt), ..., x(t_final)
    pub t_final: f64,
    pub cpu_time: Duration,
}

impl Prediction {
    pub fn n(&self) -> usize {
        self.states.len().saturating_sub(1)
    }

    pub fn dt(&self) -> f64 {
        if self.n() == 0 {
            0.0
        } else {
            self.t_final / self.n() as f64
        }
    }
}

/// Predict future states assuming constant input over the horizon
pub fn predict<M, S>(
    input: &DroneInput,
    initial_state: State,
    model: &M,
    solver: &S,
    t_final: f64,
    n: usize,
) -> Prediction
where
    M: Dynamics,
    S: Stepper,
{
    assert!(n > 0, "n must be > 0");
    assert!(t_final.is_finite() && t_final > 0.0);

    let dt = t_final / n as f64;
    let start = Instant::now();

    // Input → control (held constant)
    let control = model.input_to_control(input);

    let mut states = Vec::with_capacity(n + 1);
    let mut state = initial_state;

    states.push(state.clone());

    for _ in 0..n {
        state = solver.step(model, &state, &control, dt);
        states.push(state.clone());
    }

    Prediction {
        states,
        t_final,
        cpu_time: start.elapsed(),
    }
}
