//! Multi-sensor fusion, 3D object detection, tracking, and GNSS anomaly handling.
//!
//! - [`gnss`]: kinematic GNSS jump detection and LiDAR fallback with hysteresis.
//! - [`curb`]: curb edge detection from a lateral height profile.
//! - [`fusion`]: inverse-variance fusion of two position estimates.
//! - [`localize`]: LiDAR scan matching (2D ICP) against a known static map.
//! - [`detect`]: groups LiDAR points into objects. Two-dimensional, no classes.
//! - [`track`]: associates detections across frames and estimates velocity.
//!
//! Not yet implemented: 3D detection and classification. Those need sensor
//! data and a detector model, which this repository does not have.

pub mod curb;
pub mod detect;
pub mod fusion;
pub mod gnss;
pub mod localize;
pub mod track;

pub use curb::{Curb, detect_curb};
pub use fusion::{Estimate, fuse};
pub use gnss::{Fix, Source, SourceSelector};
