use std::time::{Duration, Instant};

use crate::{
    traits::{Dynamics, Stepper},
    types::DroneInput,
};

#[derive(Debug, Clone)]
pub struct Prediction<S, U> {
    pub states: Vec<S>,
    pub control: U,
    pub t0: f64,
    pub t_final: f64,
    cpu_time: Duration,
}

impl<S, U> Prediction<S, U> {
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

/// Predict future states assuming constant input over the horizon
pub fn predict<M, S>(
    input: &DroneInput,
    initial_state: M::State,
    model: &M,
    solver: &mut S,
    t0: f64,
    t_final: f64,
    steps: usize,
) -> Prediction<M::State, M::Control>
where
    M: Dynamics,
    S: Stepper<M>,
{
    assert!(steps > 0, "steps must be > 0");
    assert!(
        t_final.is_finite() && t_final > 0.0,
        "t_final must be finite and > 0"
    );

    let dt = t_final / steps as f64;
    let start = Instant::now();

    let control = model.input_to_control(input);

    let mut states = Vec::with_capacity(steps + 1);
    let mut state = initial_state;

    states.push(state.clone());

    for i in 0..steps {
        let t = t0 + i as f64 * dt;
        model.validate_state(&state);
        state = solver.step(model, t, &state, &control, dt);
        states.push(state.clone());
    }

    Prediction {
        states,
        control,
        t0,
        t_final,
        cpu_time: start.elapsed(),
    }
}
