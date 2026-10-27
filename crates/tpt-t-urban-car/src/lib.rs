//! Trajectory planning, speed profiling, and kinematic control for 4-wheel vehicles.
//!
//! - [`profile`]: path curvature and the speed profile along it.
//! - [`control`]: pure pursuit path tracking.
//! - [`glosa`]: green-light speed advisory from SPAT timing.
//!
//! Not implemented yet: dynamic re-routing and emergency-vehicle preemption.
//! Those need a road graph and SRM input.

pub mod control;
pub mod glosa;
pub mod profile;

pub use control::{Pose, pure_pursuit};
pub use glosa::{Advice, Context, Signal, advise};
pub use profile::{Limits, Point, curvature, speed_profile};
