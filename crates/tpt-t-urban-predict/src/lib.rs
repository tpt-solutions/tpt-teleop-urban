//! SIMD-accelerated IMM Kalman prediction of pedestrian, cyclist, and vehicle trajectories.
//!
//! - [`imm`]: two-model IMM filter and trajectory cone. This is the scalar
//!   reference implementation, and it always builds on stable.
//! - [`linalg`]: fixed-size matrix helpers.
//!
//! The spec calls for a `portable_simd` kernel for 100+ agents. It is a
//! nightly-only item and is not in this build yet. Phase 5 in `todo.md` tracks it.

pub mod imm;
pub mod linalg;

pub use imm::{ConePoint, Estimate, Imm};
