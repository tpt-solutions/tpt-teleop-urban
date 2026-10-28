//! Bare-metal/RTOS actuation layer: steer-by-wire, brake-by-wire, throttle, and watchdog.
//!
//! This crate is `no_std`. It builds for embedded targets such as
//! `thumbv7em-none-eabihf`.
//!
//! - [`actuator`]: clamps and rate-limits steering, brake and throttle commands.
//! - [`watchdog`]: software watchdog with a latched expiry.
//! - [`failover`]: dual-channel steering failover with requalification.
//! - [`frame`]: the 8-byte actuation payload with a CRC-8, for the bus.
//!
//! Hardware bring-up (Phase 18) and the real-time task layout (Phase 13) are not
//! part of this crate.

#![cfg_attr(not(test), no_std)]
#![deny(unsafe_op_in_unsafe_fn)]

pub mod actuator;
pub mod failover;
pub mod frame;
pub mod watchdog;

pub use actuator::{Actuation, Limiter};
pub use failover::{Channel, Failover, NoChannel};
pub use watchdog::{State as WatchdogState, Watchdog};
