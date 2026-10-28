//! Runs the four-stage pinned pipeline and prints its report, first as fast as it
//! can go and then with the sensor paced at 30 Hz.
//!
//! Usage: `cargo run --release -p tpt-t-urban-sim --example pipeline -- [frames]`.
//! The default of 120 frames keeps the moving object inside the 40 m sensor range.

use std::time::Duration;

use tpt_t_urban_sim::pipeline::{PipelineReport, run, run_paced};

fn print_report(label: &str, report: &PipelineReport) {
    println!("{label}");
    println!("  frames received:        {}", report.frames);
    println!("  out of order:           {}", report.out_of_order);
    println!("  tracks at the end:      {}", report.final_tracks);
    println!(
        "  prediction error (1 s): {:.2} m",
        report.prediction_error_m
    );
    println!(
        "  sensor to planner:      mean {:.1} µs, worst {:.1} µs",
        report.mean_latency_us, report.max_latency_us
    );
    println!(
        "  sensor to actuator:     mean {:.1} µs, worst {:.1} µs",
        report.mean_actuation_us, report.max_actuation_us
    );
    println!("  AEB edge ticks:         {}", report.aeb_ticks);
    println!("  peak brake:             {} ‰", report.max_brake_permille);
    println!("  peak steer:             {} cdeg", report.max_steer_cdeg);
    println!(
        "  peak planned decel:     {} mm/s²",
        report.max_planned_decel_mm_s2
    );
    println!("  edge ticks:             {}", report.edge_ticks);
    println!(
        "  worst tick overrun:     {:.1} µs",
        report.max_tick_overrun_us
    );
    for (name, pin) in ["sensor", "perception", "planner", "edge"]
        .iter()
        .zip(report.pinned)
    {
        println!("  pin {name:<11}         {pin:?}");
    }
}

fn main() {
    let frames: u32 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(120);
    print_report("unpaced", &run(frames, 0.1));
    print_report(
        "paced at 30 Hz",
        &run_paced(frames, 1.0 / 30.0, Duration::from_micros(33_333)),
    );
    println!("(host timing, not target hardware)");
}
