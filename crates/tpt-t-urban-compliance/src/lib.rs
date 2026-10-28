//! Deterministic traffic-law FSM with signed justifications and a hash-chained audit ledger.
//!
//! - [`decision`]: stop-sign and speed-limit FSM. Each decision has a reason.
//! - [`ledger`]: append-only ledger. Entries are hash-chained and Ed25519-signed.
//! - [`store`]: file-backed ledger. Each append is synced before it returns, and
//!   a torn tail from a power cut is cut off on open.
//!
//! Implements Phase 6 of `todo.md`. The full rule set is not implemented yet.

pub mod decision;
pub mod journal;
pub mod ledger;
pub mod store;

pub use decision::{
    ApproachState, Decision, Evaluation, Observation, REASONS, evaluate, reason_code, reason_text,
};
pub use journal::Record;
pub use ledger::{Entry, Ledger, VerifyError, verify_chain};
pub use store::{DurableLedger, Recovery, StoreError};
