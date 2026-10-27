//! Last-mile sidewalk bot control: curb mounting, pedestrian yielding, and secure compartments.
//!
//! - [`mount`]: curb approach, alignment, and ultrasonic verification.
//! - [`yield_ped`]: pedestrian right-of-way with hold-off.
//! - [`compartment`]: package lock, gated by curb verification, rest, and a
//!   constant-time code check.
//!
//! The jaywalker-reaction hookup (consuming `tpt-t-urban-predict` cones) needs
//! the integration in Phase 16, and is not done here.

pub mod compartment;
pub mod mount;
pub mod yield_ped;

pub use compartment::{Compartment, Lock, Refusal, hash_code};
pub use mount::{Mount, MountAction, MountState};
pub use yield_ped::{Decision, Pedestrian, Yielder};
