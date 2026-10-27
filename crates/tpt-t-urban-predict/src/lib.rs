//! SIMD-accelerated IMM Kalman prediction of pedestrian, cyclist, and vehicle trajectories.
//!
//! - [`imm`]: two-model IMM filter and trajectory cone. Scalar reference, builds
//!   on stable.
//! - [`batch`]: structure-of-arrays constant-velocity propagation over many
//!   agents. Scalar on stable, eight-wide `portable_simd` kernel with the `simd`
//!   feature.
//! - [`linalg`]: fixed-size matrix helpers.
//!
//! The `simd` feature needs a nightly toolchain. Build it with
//! `cargo +nightly test -p tpt-t-urban-predict --features simd`. CI runs that
//! job. The stable build does not enable the feature.

#![cfg_attr(feature = "simd", feature(portable_simd))]

pub mod batch;
pub mod imm;
pub mod linalg;

#[cfg(feature = "simd")]
pub use batch::propagate_cv_simd;
pub use batch::{AgentsSoa, propagate_cv_scalar};
pub use imm::{ConePoint, Estimate, Imm};
