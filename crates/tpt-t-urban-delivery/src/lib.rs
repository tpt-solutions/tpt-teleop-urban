//! Last-mile sidewalk bot control: curb mounting, pedestrian yielding, and secure compartments.
//!
//! - [`mount`]: curb approach, alignment, and ultrasonic verification.
//! - [`yield_ped`]: pedestrian right-of-way with hold-off.
//! - [`reaction`]: turns a pedestrian's predicted cone into a yield and a smooth,
//!   bounded deceleration, ahead of the guardian's AEB backstop.
//! - [`compartment`]: package lock, gated by curb verification, rest, and a
//!   constant-time code check.
//!

pub mod compartment;
pub mod mount;
pub mod reaction;
pub mod sidewalk;
pub mod yield_ped;

pub use compartment::{Compartment, Lock, Refusal, hash_code};
pub use mount::{Mount, MountAction, MountState};
pub use yield_ped::{Decision, Pedestrian, Yielder};
