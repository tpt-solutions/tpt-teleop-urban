//! Headless urban simulator, scenario injection, and egui visualizer.
//!
//! - [`plant`]: kinematic bicycle model of the vehicle.
//! - [`scenario`]: a pedestrian crossing scenario that runs the whole chain:
//!   noisy measurements → IMM prediction → guardian → vehicle.
//!
//! Not implemented yet: 3D city rendering, sensor raycasting, V2X traffic
//! injection, and the egui visualizer. Those are Phase 15 items in `todo.md`.

pub mod plant;
pub mod scenario;

pub use plant::Bicycle;
pub use scenario::{Outcome, Scenario, run_crossing};
