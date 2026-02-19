use std::time::{Duration, Instant};

use crate::{
    traits::{Dynamics, StepResult, Stepper},
    types::{Control, State},
};

#[derive(Debug, Clone)]
pub struct Prediction<const N: usize, const M: usize> {
    pub states: Vec<State<N>>,
    pub control: Control<M>,
    pub t0: f64,
    pub t_final: f64,
    cpu_time: Duration,
}

impl<const N: usize, const M: usize> Prediction<N, M> {
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

    pub fn cpu_time(&self) -> Duration {
        self.cpu_time
    }

    pub fn cpu_seconds(&self) -> f64 {
        self.cpu_time.as_secs_f64()
    }

    pub fn t_at(&self, i: usize) -> f64 {
        self.t0 + (i as f64) * self.dt()
    }
}

/// Predict future states assuming constant control over the horizon.
pub fn predict_constant<const N: usize, const M: usize, Model, Sol>(
    model: &Model,
    stepper: &Sol,
    initial_state: State<N>,
    control: Control<M>,
    t0: f64,
    t_final: f64,
    steps: usize,
) -> Prediction<N, M>
where
    Model: Dynamics<N, M>,
    Sol: Stepper<N, M, Model>,
{
    assert!(steps > 0, "steps must be > 0");
    assert!(
        t_final.is_finite() && t_final > 0.0,
        "t_final must be finite and > 0"
    );

    let dt = t_final / steps as f64;
    let start = Instant::now();

    let mut states = Vec::with_capacity(steps + 1);
    let mut state = initial_state;

    states.push(state);

    for _ in 0..steps {
        let StepResult {
            state: next, ..
        } = stepper.step(model, &state, &control, dt);
        state = next;
        states.push(state);
    }

    Prediction {
        states,
        control,
        t0,
        t_final,
        cpu_time: start.elapsed(),
    }
}
