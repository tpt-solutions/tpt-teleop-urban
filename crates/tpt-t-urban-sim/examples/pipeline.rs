//! Runs the three-stage pinned pipeline and prints its report.
//!
//! Usage: `cargo run --release -p tpt-t-urban-sim --example pipeline -- [frames]`.
//! The default of 120 frames keeps the moving object inside the 40 m sensor range.

use tpt_t_urban_sim::pipeline::run;

fn main() {
    let frames: u32 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(120);
    let report = run(frames, 0.1);
    println!("frames received:      {}", report.frames);
    println!("out of order:         {}", report.out_of_order);
    println!("tracks at the end:    {}", report.final_tracks);
    println!("mean latency:         {:.1} µs", report.mean_latency_us);
    println!("worst latency:        {:.1} µs", report.max_latency_us);
    for (name, pin) in ["sensor", "perception", "consumer"]
        .iter()
        .zip(report.pinned)
    {
        println!("pin {name:<11}        {pin:?}");
    }
    println!("(host timing, not target hardware)");
}
