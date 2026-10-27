//! Adapter implementing tpt-teleop's `DomainTeleopInterface` for urban vehicles.
//!
//! - [`state`]: the universal teleop safety state machine.
//! - [`adapter`]: operator command mapping, gated by mode, traffic law and
//!   actuator limits, and video redaction before it reaches the operator.
//!
//! Pending: the `DomainTeleopInterface` trait comes from `tpt-t-domain-bridge`,
//! which lives in the `tpt-teleop` repository. This crate does not yet depend on
//! it, so the adapter is not the trait impl yet.

pub mod adapter;
pub mod state;

pub use adapter::{CommandError, ControlCommand, TeleopAdapter};
pub use state::{Event, Mode, transition};
