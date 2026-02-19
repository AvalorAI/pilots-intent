# pilots-intent

Predict quadcopter trajectories from pilot stick inputs. Const-generic Rust library with compile-time dimension safety and zero-cost solver dispatch.

Given a drone's current state and pilot input (roll, pitch, yaw rate), simulate the future flight path under constant control using pluggable numerical integrators.

## Quick start

```rust
use pilots_intent::{
    dynamic_models::SimpleQuadcopter,
    plot::plot_xy,
    predict::predict_constant,
    solvers::ForwardEuler,
    traits::Dynamics,
    types::{DroneInput, State},
};

let input = DroneInput {
    roll_rad: 20f64.to_radians(),
    pitch_rad: 10f64.to_radians(),
    yaw_rate_rps: 0.0,
};

let model = SimpleQuadcopter { drag: 0.0 };
let stepper = ForwardEuler;
let control = model.input_to_control(&input);

let prediction = predict_constant(
    &model,
    &stepper,
    State::<5>::new(0.0, 0.0, 5.0, 1.0, 0.0),
    control,
    0.0,     // t0
    10.0,    // t_final
    30_000,  // steps
);

plot_xy(&prediction, "plot_output.png");
```

## Architecture

State dimension `N` and control dimension `M` are const-generic parameters throughout the library. All traits monomorphize over the concrete model, so there is no dynamic dispatch.

### Core traits

| Trait | Parameters | Purpose |
|-------|-----------|---------|
| `Dynamics<N, M>` | State dim, control dim | ODE right-hand side `dx/dt = f(x, u)` + input mapping |
| `LinearizableDynamics<N, M>` | (extends Dynamics) | Adds analytical Jacobian `df/dx` for implicit solvers |
| `Stepper<N, M, Model>` | Dims + model type | Single integration step, returns `StepResult<N>` |
| `Controller<N, M>` | State dim, control dim | Feedback controller `u(t, x)` |
| `Observer<N>` | State dim | Step-by-step callback with early-exit support |

### Type aliases

```rust
type State<const N: usize>   = SVector<f64, N>;  // nalgebra fixed-size
type Control<const M: usize> = SVector<f64, M>;
```

## Models

### SimpleQuadcopter

Planar NED quadcopter with hover small-angle thrust approximation and linear drag.

- **State (N=5):** `[north_m, east_m, v_north_mps, v_east_mps, yaw_rad]`
- **Control (M=3):** `[ax_body_mps2, ay_body_mps2, yaw_rate_rps]`
- **Input:** `DroneInput { roll_rad, pitch_rad, yaw_rate_rps }`

Body-to-NED rotation via yaw angle. Positive pitch = nose down (forward acceleration). Positive roll = right tilt (rightward acceleration). Implements `LinearizableDynamics` for implicit solvers.

## Solvers

| Solver | Order | Type | Requires |
|--------|-------|------|----------|
| `ForwardEuler` | 1 | Explicit | `Dynamics` |
| `Rk4` | 4 | Explicit | `Dynamics` |
| `BackwardEuler` | 1 | Implicit (Newton) | `LinearizableDynamics` |

All solvers implement `Stepper<N, M, Model>` and include a `stability(z)` function for stability region analysis.

## Visualization

```rust
use pilots_intent::plot::*;

// XY trajectory with start/end markers
plot_xy(&prediction, "trajectory.png");

// Single state component over time
plot_component(&prediction, 4, "Yaw [deg]", "yaw.png", |v| v.to_degrees());

// Solver stability region in the complex plane
plot_stability_region(ForwardEuler::stability, -3.0, 1.0, -2.0, 2.0, 500, "stability.png");

// Eigenvalue locus of df/dx·dt along the trajectory
plot_eigvals(&prediction, &model, "eigvals.png");
```

## Frame conventions

- **NED frame:** North = +x, East = +y, Down = +z
- **Body frame:** x-forward, y-right, z-down
- **Yaw:** 0 = facing North, positive = clockwise from above

## Dependencies

- [nalgebra](https://nalgebra.org/) — fixed-size linear algebra (`SVector`, `DMatrix`)
- [num-complex](https://crates.io/crates/num-complex) — complex arithmetic for stability analysis
- [plotters](https://plotters-rs.github.io/) — PNG rendering
