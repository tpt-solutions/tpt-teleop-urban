//! Deterministic traffic-law FSM with signed justifications and a hash-chained audit ledger.
//!
//! - [`decision`]: stop-sign and speed-limit FSM. Each decision has a reason.
//! - [`ledger`]: append-only ledger. Entries are hash-chained and Ed25519-signed.
//!
//! Implements Phase 6 of `todo.md`. Durable storage and the full rule set are
//! not implemented yet.

pub mod decision;
pub mod ledger;

pub use decision::{ApproachState, Decision, Evaluation, Observation, evaluate};
pub use ledger::{Entry, Ledger, VerifyError, verify_chain};
