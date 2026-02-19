use pilots_intent::{
    dynamic_models::SimpleQuadcopter,
    plot::plot_xy,
    predict::predict_constant,
    solvers::ForwardEuler,
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
    let stepper = ForwardEuler;
    let control = model.input_to_control(&input);

    let prediction = predict_constant(
        &model,
        &stepper,
        initial_state,
        control,
        0.0,
        t_final,
        steps,
    );

    println!("Computation time: {:?}", prediction.cpu_time());

    plot_xy(&prediction, "plot_output.png")
}
