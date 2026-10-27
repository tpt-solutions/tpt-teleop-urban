//! Zero-copy C-V2X / DSRC parsing for BSM, MAP, SPAT, and SRM messages.
//!
//! - [`uper`]: X.691 unaligned PER primitives. Verified against the standard.
//! - [`bsm`]: BSM Part I core data. The field ranges are NOT verified against
//!   SAE J2735, see the module docs.
//!
//! MAP, SPAT and SRM are not implemented yet (Phase 3 in `todo.md`).

pub mod bsm;
pub mod uper;
