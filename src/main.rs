use pilots_intent::{
    dynamic_models::SimpleQuadcopter,
    plot::plot_xy,
    predict::predict,
    solvers::ForwardEuler,
    types::{DroneInput, State},
};

fn main() {
    let input = DroneInput {
        roll: 20f64.to_radians(),
        pitch: 0f64.to_radians(),
    };

    let t_final = 10.0;
    let steps = 30_000;

    // 2. State expanded to 5D: [x, y, vx, vy, yaw]
    // We assume the drone starts at (0,0) moving with current momentum,
    // and facing North (yaw = 0.0).
    let initial_state: State = vec![
        0.0,  // x
        0.0,  // y
        0.0,  // vx (Current momentum West)
        0.0,  // vy (Current momentum North)
        3.14, // yaw (Facing direction)
    ];

    let model = SimpleQuadcopter { drag: 0.1 };
    let prediction = predict(&input, initial_state, &model, &ForwardEuler, t_final, steps);

    println!("Computation time: {:?}", prediction.cpu_time());

    plot_xy(&prediction, "plot_output.png")
}
