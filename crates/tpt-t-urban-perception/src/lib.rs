//! Multi-sensor fusion, 3D object detection, tracking, and GNSS anomaly handling.
//!
//! - [`gnss`]: kinematic GNSS jump detection and LiDAR fallback with hysteresis.
//! - [`curb`]: curb edge detection from a lateral height profile.
//!
//! Not yet implemented: multi-sensor fusion (needs sensor models and data),
//! 3D detection, and tracking. These are Phase 4 items in `todo.md`.

pub mod curb;
pub mod gnss;

pub use curb::{Curb, detect_curb};
pub use gnss::{Fix, Source, SourceSelector};
