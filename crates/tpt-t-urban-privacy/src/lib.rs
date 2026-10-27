//! On-the-fly face and license-plate blurring in the zero-copy frame buffer.
//!
//! - [`frame`]: grayscale frame buffer and the in-place region blur.
//! - [`redact`]: the redaction gate. Only a [`redact::RedactedFrame`] can reach
//!   a [`redact::Sink`].
//!
//! Detection is not implemented here. The face and plate detector is a trained
//! model, and it is not in this repository. The redaction path takes detected
//! rectangles as input, so the detector can be plugged in later.

pub mod frame;
pub mod redact;

pub use frame::{Frame, FrameError, Rect};
pub use redact::{RedactedFrame, Redactor, Sink, changed_outside_detections};
