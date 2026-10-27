//! Infrastructure-side intersection management, congestion optimization, and SRM priority routing.
//!
//! - [`controller`]: fixed-time signal controller with yellow and all-red
//!   clearance, and emergency-vehicle preemption driven by SRM.
//!
//! Not implemented yet: congestion optimization across intersections, and
//! SPAT/MAP broadcast, which need the V2X message encoders in
//! `tpt-t-urban-v2x`.

pub mod controller;

pub use controller::{Controller, Interval, Output};
