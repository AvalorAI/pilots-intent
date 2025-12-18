use pilots_intent::{
    dynamic_models::simple_model::SimpleModel,
    plot::plot_xy,
    predict::predict,
    solver::ForwardEuler,
    types::{DroneInput, State},
};

fn main() {
    // 1. Pilot input now includes yaw_rate to handle constant turns
    let input = DroneInput {
        pitch: 20f64.to_radians(), // Realistic forward tilt
        roll: 0f64.to_radians(),   // No lateral tilt
                                   // yaw_rate: -0.0,            // Constant slow turn (radians/sec)
    };

    let t_final = 10.0;
    let steps = 30_000;

    // 2. State expanded to 5D: [x, y, vx, vy, yaw]
    // We assume the drone starts at (0,0) moving with current momentum,
    // and facing North (yaw = 0.0).
    let initial_state: State = vec![
        0.0, // x
        0.0, // y
        0.0, // vx (Current momentum West)
        0.0, // vy (Current momentum North)
        0.0, // yaw (Facing direction)
    ];

    // 3. Simple planar dynamics model
    // Added a small drag value to prevent the drone from reaching
    // impossible speeds over a 10s horizon.
    let model = SimpleModel { drag: 0.1 };

    let solver = ForwardEuler;

    // The prediction logic remains the same, but 'state' inside is now 5D
    let prediction = predict(&input, initial_state, &model, &solver, t_final, steps);

    println!("Computation time: {:?}", prediction.cpu_time);

    // This will now plot the 2D path resulting from the 5D integration
    plot_xy(&prediction, "plot_output.png")
}
