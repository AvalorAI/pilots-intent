use plotters::prelude::*;
use std::path::Path;

use num_complex::Complex;

use crate::{
    predict::Prediction,
    traits::LinearizableDynamics,
    types::{Position2D, StateVector},
};

pub fn plot_xy<S, U, P>(prediction: &Prediction<S, U>, filename: P)
where
    S: Position2D,
    P: AsRef<Path>,
{
    let states = &prediction.states;
    assert!(states.len() >= 2, "need at least 2 states");

    // Extract (x, y)
    let mut points = Vec::with_capacity(states.len());
    let mut x_min = f64::INFINITY;
    let mut x_max = f64::NEG_INFINITY;
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;

    for s in states {
        let (x, y) = s.position();

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
        .x_desc("North [m]")
        .y_desc("East [m]")
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

/// Plot a scalar component (by index) of a StateVector over time.
pub fn plot_component<S, U, P>(
    prediction: &Prediction<S, U>,
    component: usize,
    y_desc: &str,
    filename: P,
    to_display: impl Fn(f64) -> f64,
) where
    S: StateVector,
    P: AsRef<Path>,
{
    assert!(!prediction.states.is_empty(), "states must not be empty");

    let mut series = Vec::with_capacity(prediction.states.len());
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;

    for (i, s) in prediction.states.iter().enumerate() {
        let v = s.to_dvector();
        assert!(
            component < v.len(),
            "component {component} out of bounds for state of dim {}",
            v.len()
        );
        let t = prediction.t_at(i);
        let y = to_display(v[component]);
        y_min = y_min.min(y);
        y_max = y_max.max(y);
        series.push((t, y));
    }

    let pad = ((y_max - y_min).abs() * 0.05).max(1e-9);
    let y_range = (y_min - pad)..(y_max + pad);

    let root = BitMapBackend::new(filename.as_ref(), (1200, 700)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .margin(20)
        .caption(format!("State[{component}] vs t"), ("sans-serif", 28))
        .x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(prediction.t0..(prediction.t0 + prediction.t_final), y_range)
        .unwrap();

    chart
        .configure_mesh()
        .x_desc("t [s]")
        .y_desc(y_desc)
        .draw()
        .unwrap();

    chart.draw_series(LineSeries::new(series, &BLUE)).unwrap();

    root.present().unwrap();
}

/// Plot absolute stability region for a one-step method given its stability function R(z).
pub fn plot_stability_region<P: AsRef<Path>, R>(
    stabfn: R,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    n: usize,
    filename: P,
) where
    R: Fn(Complex<f64>) -> Complex<f64>,
{
    assert!(n >= 2, "n must be >= 2");
    assert!(x_max > x_min && y_max > y_min, "invalid ranges");

    let size = (900u32, 900u32);
    let root = BitMapBackend::new(filename.as_ref(), size).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .margin(20)
        .caption("Stability region (|R(z)| ≤ 1)", ("sans-serif", 28))
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(x_min..x_max, y_min..y_max)
        .unwrap();

    chart
        .configure_mesh()
        .x_desc("Re")
        .y_desc("Im")
        .draw()
        .unwrap();

    let dx = (x_max - x_min) / ((n - 1) as f64);
    let dy = (y_max - y_min) / ((n - 1) as f64);

    for ix in 0..(n - 1) {
        for iy in 0..(n - 1) {
            let x0 = x_min + (ix as f64) * dx;
            let x1 = x0 + dx;
            let y0 = y_min + (iy as f64) * dy;
            let y1 = y0 + dy;

            let xc = 0.5 * (x0 + x1);
            let yc = 0.5 * (y0 + y1);
            let z = Complex::new(xc, yc);

            let stable = stabfn(z).norm() <= 1.0;
            let style = if stable {
                WHITE.filled()
            } else {
                GREEN.filled()
            };

            chart
                .draw_series(std::iter::once(Rectangle::new([(x0, y0), (x1, y1)], style)))
                .unwrap();
        }
    }

    chart
        .draw_series([
            PathElement::new(vec![(0.0, y_min), (0.0, y_max)], &BLACK),
            PathElement::new(vec![(x_min, 0.0), (x_max, 0.0)], &BLACK),
        ])
        .unwrap();

    root.present().unwrap();
}

/// Plot eigenvalues of df/dx · dt along the trajectory for linearizable models.
pub fn plot_eigvals<M, P>(
    prediction: &Prediction<M::State, M::Control>,
    model: &M,
    filename: P,
) -> Vec<Vec<Complex<f64>>>
where
    M: LinearizableDynamics,
    M::State: StateVector,
    P: AsRef<Path>,
{
    assert!(!prediction.states.is_empty(), "states must not be empty");
    let m = prediction.states[0].to_dvector().len();
    assert!(m > 0, "state dimension must be > 0");

    let dt = prediction.dt();
    let mut eigs: Vec<Vec<Complex<f64>>> = Vec::with_capacity(prediction.states.len());

    let mut x_min = f64::INFINITY;
    let mut x_max = f64::NEG_INFINITY;
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;

    for (i, state) in prediction.states.iter().enumerate() {
        let t = prediction.t_at(i);
        let j = model.jacobian(t, state, &prediction.control);
        assert!(
            j.nrows() == m && j.ncols() == m,
            "jacobian must be square with dimension matching the state"
        );

        let lam = j.complex_eigenvalues();
        let mut row = Vec::with_capacity(m);
        for k in 0..m {
            let z = lam[k] * dt;
            x_min = x_min.min(z.re);
            x_max = x_max.max(z.re);
            y_min = y_min.min(z.im);
            y_max = y_max.max(z.im);
            row.push(z);
        }
        eigs.push(row);
    }

    if x_min == x_max {
        x_min -= 1.0;
        x_max += 1.0;
    }
    if y_min == y_max {
        y_min -= 1.0;
        y_max += 1.0;
    }

    let x_pad = ((x_max - x_min).abs() * 0.05).max(1e-9);
    let y_pad = ((y_max - y_min).abs() * 0.05).max(1e-9);

    let x_range = (x_min - x_pad)..(x_max + x_pad);
    let y_range = (y_min - y_pad)..(y_max + y_pad);

    let root = BitMapBackend::new(filename.as_ref(), (900, 900)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let mut chart = ChartBuilder::on(&root)
        .margin(20)
        .caption("Eigenvalues of df/dx · dt", ("sans-serif", 28))
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(x_range, y_range)
        .unwrap();

    chart
        .configure_mesh()
        .x_desc("Re")
        .y_desc("Im")
        .draw()
        .unwrap();

    let x0 = x_min - x_pad;
    let x1 = x_max + x_pad;
    let y0 = y_min - y_pad;
    let y1 = y_max + y_pad;

    chart
        .draw_series([
            PathElement::new(vec![(0.0, y0), (0.0, y1)], &BLACK),
            PathElement::new(vec![(x0, 0.0), (x1, 0.0)], &BLACK),
        ])
        .unwrap();

    for row in &eigs {
        for z in row {
            chart
                .draw_series(std::iter::once(Circle::new((z.re, z.im), 2, BLUE.filled())))
                .unwrap();
        }
    }

    root.present().unwrap();
    eigs
}
