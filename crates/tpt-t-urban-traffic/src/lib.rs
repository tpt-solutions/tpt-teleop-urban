//! Infrastructure-side intersection management, congestion optimization, and SRM priority routing.
//!
//! - [`controller`]: fixed-time signal controller with yellow and all-red
//!   clearance, and emergency-vehicle preemption driven by SRM.
//! - [`demand`]: queued-vehicle counts per approach, from vehicle positions and
//!   velocities, as the demand input of the optimizer.
//! - [`optimize`]: demand-proportional green split for one intersection, and
//!   corridor offsets for a green wave across several.
//!
//! Not implemented yet: SPAT/MAP broadcast, which needs the V2X message encoders
//! in `tpt-t-urban-v2x`.

pub mod controller;
pub mod demand;
pub mod optimize;

pub use controller::{Controller, Interval, Output};
pub use optimize::{allocate_greens, corridor_offsets};
