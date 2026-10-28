//! Runs the seeded crossing sweep and prints the summary, with every guarded
//! collision on an avoidable crossing.
//!
//! Usage: `cargo run --release -p tpt-t-urban-sim --example sweep -- [seed] [count]`

use tpt_t_urban_sim::sweep;

fn main() {
    let mut args = std::env::args().skip(1);
    let seed: u32 = args
        .next()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0xC0FF_EE01);
    let count: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(400);

    let report = sweep(seed, count);
    println!("seed {seed:#010x}, {count} crossings");
    println!("  avoidable:           {}", report.avoidable);
    println!("  guarded collisions:  {}", report.guarded_collisions);
    println!("  unguarded collisions: {}", report.unguarded_collisions);
    println!("  avoidable but hit:   {}", report.failures.len());
    for (scenario, outcome) in &report.failures {
        println!("    {scenario:?}");
        println!("      -> {outcome:?}");
    }
}
