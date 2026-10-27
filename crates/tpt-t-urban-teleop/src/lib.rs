//! Urban vehicle side of tpt-teleop's control protocol.
//!
//! - [`adapter`]: turns the upstream `ControlCommand` into actuator commands,
//!   gated by the upstream mode machine, traffic law, and actuator limits, and
//!   redacts video before it leaves the vehicle.
//!
//! Built on tpt-teleop's `tpt-t-core` (pinned git dependency). The
//! `DomainTeleopInterface` trait in the bridge spec is not in upstream yet, so
//! the adapter exposes the same operations as plain methods.

pub mod adapter;

pub use adapter::{CommandError, TeleopAdapter};
pub use tpt_t_core::ser::ControlCommand;
pub use tpt_t_core::{Mode, StateMachine};
