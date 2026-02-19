use std::collections::VecDeque;
use std::time::{Duration, Instant};

use crate::{
    traits::{Dynamics, MultiStepper, StepResult, Stepper},
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

/// Predict future states using a multi-step method with single-step bootstrap.
///
/// The first `history_len - 1` steps use `bootstrap` (typically RK4) to build
/// up the derivative history, then the remaining steps use the multi-step `stepper`.
pub fn predict_constant_multistep<const N: usize, const M: usize, Model, Boot, Sol>(
    model: &Model,
    bootstrap: &Boot,
    stepper: &Sol,
    initial_state: State<N>,
    control: Control<M>,
    t0: f64,
    t_final: f64,
    steps: usize,
) -> Prediction<N, M>
where
    Model: Dynamics<N, M>,
    Boot: Stepper<N, M, Model>,
    Sol: MultiStepper<N, M, Model>,
{
    assert!(steps > 0, "steps must be > 0");
    assert!(
        t_final.is_finite() && t_final > 0.0,
        "t_final must be finite and > 0"
    );

    let k = stepper.history_len();
    assert!(
        steps >= k,
        "need at least {k} steps for a {k}-step method"
    );

    let dt = t_final / steps as f64;
    let start = Instant::now();

    let mut states = Vec::with_capacity(steps + 1);
    let mut state = initial_state;
    states.push(state);

    // Ring buffer of recent derivative evaluations, oldest first.
    let mut f_ring: VecDeque<State<N>> = VecDeque::with_capacity(k);
    f_ring.push_back(model.f(&state, &control));

    // Bootstrap phase: use single-step method to fill the derivative history.
    let bootstrap_steps = k - 1;
    for _ in 0..bootstrap_steps {
        let StepResult { state: next, .. } = bootstrap.step(model, &state, &control, dt);
        state = next;
        states.push(state);
        f_ring.push_back(model.f(&state, &control));
    }

    // Multi-step phase.
    let f_buf_len = k;
    for _ in bootstrap_steps..steps {
        let history: Vec<State<N>> = f_ring.iter().copied().collect();
        let StepResult { state: next, .. } = stepper.step(model, &state, &history, &control, dt);
        state = next;
        states.push(state);

        // Maintain the ring buffer.
        if f_ring.len() == f_buf_len {
            f_ring.pop_front();
        }
        f_ring.push_back(model.f(&state, &control));
    }

    Prediction {
        states,
        control,
        t0,
        t_final,
        cpu_time: start.elapsed(),
    }
}
