//! Writes the crossing scenario as an interactive HTML viewer:
//! `cargo run -p tpt-t-urban-sim --bin visualizer -- [out.html]`.
//!
//! The page holds the frames from `tpt_t_urban_sim::viz` inline. Open it in a
//! browser: Play advances the run, and the slider scrubs it. It needs no window
//! toolkit, so the crate stays within the MIT/BSD/ISC dependency policy.

use std::path::PathBuf;

use tpt_t_urban_sim::render::viewer_html;
use tpt_t_urban_sim::scenario::Scenario;
use tpt_t_urban_sim::viz::{bounds, frames};

fn main() -> std::io::Result<()> {
    let out = std::env::args()
        .nth(1)
        .map_or_else(|| PathBuf::from("crossing.html"), PathBuf::from);
    let frames = frames(&Scenario::default());
    std::fs::write(&out, viewer_html(&frames, bounds(&frames)))?;
    println!("wrote {} ({} frames)", out.display(), frames.len());
    Ok(())
}
