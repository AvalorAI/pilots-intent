use pilots_intent::{
    dynamic_models::SimpleQuadcopter,
    plot::plot_xy,
    predict::{predict_constant, predict_constant_multistep},
    solvers::{Ab2, Ab4, ForwardEuler, Heun, Midpoint, Rk4},
    traits::Dynamics,
    types::{DroneInput, State},
};

fn main() {
    let input = DroneInput {
        roll_rad: 20f64.to_radians(),
        pitch_rad: 10f64.to_radians(),
        yaw_rate_rps: 0.0,
    };

    let t_final = 10.0;
    let steps = 30_000;

    // NED planar state: North, East, V_North, V_East, Yaw
    let initial_state = State::<5>::new(
        0.0,  // north [m]
        0.0,  // east [m]
        5.0,  // v_north [m/s]
        1.0,  // v_east [m/s]
        20.0, // yaw [rad] (0 = facing North)
    );

    let model = SimpleQuadcopter { drag: 0.0 };
    let control = model.input_to_control(&input);

    // --- Single-step methods ---

    let p = predict_constant(&model, &ForwardEuler, initial_state, control, 0.0, t_final, steps);
    println!("{:20} — {:.3?}", "Forward Euler", p.cpu_time());

    let p = predict_constant(&model, &Heun, initial_state, control, 0.0, t_final, steps);
    println!("{:20} — {:.3?}", "Heun (RK2)", p.cpu_time());

    let p = predict_constant(&model, &Midpoint, initial_state, control, 0.0, t_final, steps);
    println!("{:20} — {:.3?}", "Midpoint (RK2)", p.cpu_time());

    let p = predict_constant(&model, &Rk4, initial_state, control, 0.0, t_final, steps);
    println!("{:20} — {:.3?}", "RK4", p.cpu_time());

    // --- Multi-step methods (bootstrapped with RK4) ---

    let p = predict_constant_multistep(&model, &Rk4, &Ab2, initial_state, control, 0.0, t_final, steps);
    println!("{:20} — {:.3?}", "AB2 (multi-step)", p.cpu_time());

    let p = predict_constant_multistep(&model, &Rk4, &Ab4, initial_state, control, 0.0, t_final, steps);
    println!("{:20} — {:.3?}", "AB4 (multi-step)", p.cpu_time());

    // Plot RK4 as reference
    let p = predict_constant(&model, &Rk4, initial_state, control, 0.0, t_final, steps);
    plot_xy(&p, "plot_output.png");
}
