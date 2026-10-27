//! Urban vehicle side of tpt-teleop's control protocol.
//!
//! - [`adapter`]: turns the upstream `ControlCommand` into actuator commands,
//!   gated by the upstream mode machine, traffic law, and actuator limits, and
//!   redacts video before it leaves the vehicle.
//!
//! [`TeleopAdapter`] implements `DomainTeleopInterface` from the upstream
//! `tpt-t-domain-bridge` crate. Wrap it in a `DomainSession` to get the
//! handover lifecycle.

pub mod adapter;
pub mod handover;

pub use adapter::{AXIS_STEERING, AXIS_THROTTLE, CommandError, TeleopAdapter};
pub use handover::{assistance_reason, clear_mrm, ready_to_return};
pub use tpt_t_core::ser::ControlCommand;
pub use tpt_t_core::{Mode, StateMachine};
