//! Multi-sensor fusion, 3D object detection, tracking, and GNSS anomaly handling.
//!
//! - [`gnss`]: kinematic GNSS jump detection and LiDAR fallback with hysteresis.
//! - [`curb`]: curb edge detection from a lateral height profile.
//! - [`fusion`]: inverse-variance fusion of two position estimates.
//!
//! Not yet implemented: 3D object detection and tracking. Those need sensor
//! data and a detector model, which this repository does not have.

pub mod curb;
pub mod fusion;
pub mod gnss;

pub use curb::{Curb, detect_curb};
pub use fusion::{Estimate, fuse};
pub use gnss::{Fix, Source, SourceSelector};
