use plotters::prelude::*;
use std::path::Path;

use crate::predict::Prediction;

pub fn plot_xy<P: AsRef<Path>>(prediction: &Prediction, filename: P) {
    let states = &prediction.states;
    assert!(states.len() >= 2, "need at least 2 states");

    // Extract (x, y)
    let mut points = Vec::with_capacity(states.len());
    let mut x_min = f64::INFINITY;
    let mut x_max = f64::NEG_INFINITY;
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;

    for s in states {
        let x = s[0];
        let y = s[1];

        x_min = x_min.min(x);
        x_max = x_max.max(x);
        y_min = y_min.min(y);
        y_max = y_max.max(y);

        points.push((x, y));
    }

    // Padding so line is not glued to border
    let pad_x = ((x_max - x_min).abs() * 0.1).max(1e-3);
    let pad_y = ((y_max - y_min).abs() * 0.1).max(1e-3);

    let root = BitMapBackend::new(filename.as_ref(), (900, 900)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .margin(20)
        .caption("Predicted Pilot Intent (XY)", ("sans-serif", 28))
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(
            (x_min - pad_x)..(x_max + pad_x),
            (y_min - pad_y)..(y_max + pad_y),
        )
        .unwrap();

    chart
        .configure_mesh()
        .x_desc("x [m]")
        .y_desc("y [m]")
        .draw()
        .unwrap();

    // Trajectory
    chart
        .draw_series(LineSeries::new(points.clone(), &BLUE))
        .unwrap();

    // Start point
    chart
        .draw_series(std::iter::once(Circle::new(points[0], 4, GREEN.filled())))
        .unwrap();

    // End point
    chart
        .draw_series(std::iter::once(Circle::new(
            *points.last().unwrap(),
            4,
            RED.filled(),
        )))
        .unwrap();

    root.present().unwrap();
}
