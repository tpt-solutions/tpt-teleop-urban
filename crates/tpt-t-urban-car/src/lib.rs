//! Trajectory planning, speed profiling, and kinematic control for 4-wheel vehicles.
//!
//! - [`profile`]: path curvature and the speed profile along it.
//! - [`control`]: pure pursuit path tracking.
//! - [`glosa`]: green-light speed advisory from SPAT timing.
//! - [`routing`]: fastest route over a road graph, replanned on closures.
//! - [`pull_over`]: shoulder pull-over path and its lateral-acceleration bound,
//!   for a minimal risk maneuver.
//! - [`preemption`]: the response to an emergency-vehicle alert, pulling over or
//!   stopping. The alert can come from any source; SRM decoding is not done.

pub mod control;
pub mod glosa;
pub mod preemption;
pub mod profile;
pub mod pull_over;
pub mod routing;

pub use control::{Pose, pure_pursuit};
pub use glosa::{Advice, Context, Signal, advise};
pub use profile::{Limits, Point, curvature, speed_profile};
pub use routing::{Edge, RoadGraph, Route};
