//! Headless urban simulator, scenario injection, and egui visualizer.
//!
//! - [`plant`]: kinematic bicycle model of the vehicle.
//! - [`scenario`]: a pedestrian crossing scenario that runs the whole chain:
//!   noisy measurements → IMM prediction → guardian → vehicle.
//! - [`mod@sweep`]: seeded sweep over many crossings, checking the guardian against
//!   the crossings that were avoidable.
//! - [`drive`]: closed-loop route driving, with the guardian watching the
//!   planner heartbeat and handing over to the pull-over MRM on a stall.
//! - [`lidar`]: 2D LiDAR by ray casting against boxes, with range noise.
//! - [`gnss`]: GNSS model with noise, multipath jumps, and denial, for driving
//!   the localization source selector.
//!
//! Not implemented yet: 3D city rendering, sensor raycasting, V2X traffic
//! injection, and the egui visualizer. Those are Phase 15 items in `todo.md`.

pub mod drive;
pub mod gnss;
pub mod journal;
pub mod lidar;
pub mod mrm;
pub mod plant;
pub mod render;
pub mod scenario;
pub mod sweep;

pub use gnss::{Condition, Gnss, Window};
pub use plant::Bicycle;
pub use scenario::{Outcome, Scenario, TracePoint, run_crossing, run_crossing_traced};
pub use sweep::{SweepReport, sweep};
