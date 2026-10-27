//! Writes the guarded pedestrian-crossing scenario as an SVG on stdout.
//!
//! ```text
//! cargo run -p tpt-t-urban-sim --example render_crossing > crossing.svg
//! ```

use tpt_t_urban_sim::render::crossing_svg;
use tpt_t_urban_sim::{Scenario, run_crossing_traced};

fn main() {
    let mut trace = Vec::new();
    let outcome = run_crossing_traced(&Scenario::default(), |p| trace.push(p));
    eprintln!("{outcome:?}");
    print!("{}", crossing_svg(&trace, (0.0, 50.0), (-8.0, 8.0)));
}
