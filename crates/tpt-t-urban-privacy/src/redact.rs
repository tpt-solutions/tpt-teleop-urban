//! Redaction gate: the only route from a raw frame to a sink.
//!
//! [`Redactor::redact`] consumes a raw [`Frame`] and returns a [`RedactedFrame`].
//! `Sink::accept` takes only `RedactedFrame`, so the type system stops a raw
//! frame from reaching disk or the teleop link. A frame that was never
//! redacted cannot be passed to a sink.

use crate::frame::{Frame, FrameError, Rect};

/// A frame whose detected regions have been blurred. Only [`Redactor`] builds one.
#[derive(Debug)]
pub struct RedactedFrame<'a> {
    frame: Frame<'a>,
}

impl RedactedFrame<'_> {
    #[must_use]
    pub fn pixels(&self) -> &[u8] {
        self.frame.pixels()
    }

    #[must_use]
    pub const fn width(&self) -> usize {
        self.frame.width()
    }

    #[must_use]
    pub const fn height(&self) -> usize {
        self.frame.height()
    }
}

/// Destination for redacted frames: disk, the recorder, or the teleop link.
pub trait Sink {
    fn accept(&mut self, frame: &RedactedFrame<'_>);
}

/// Blurs detected PII regions before a frame leaves the capture path.
#[derive(Clone, Copy, Debug)]
pub struct Redactor {
    radius: usize,
}

impl Redactor {
    /// Blur radius in pixels. Larger radii hide more detail and cost more time.
    #[must_use]
    pub const fn new(radius: usize) -> Self {
        Self { radius }
    }

    /// Scratch bytes [`Redactor::redact`] needs for `detections`: the largest
    /// clipped region. The caller allocates this once, outside the hot path.
    #[must_use]
    pub fn scratch_len(detections: &[Rect], width: usize, height: usize) -> usize {
        detections
            .iter()
            .filter_map(|r| r.clip(width, height))
            .map(|r| r.area())
            .max()
            .unwrap_or(0)
    }

    /// Blurs every detected region of `frame` in place and returns the redacted
    /// frame. Regions outside the frame are ignored.
    pub fn redact<'a>(
        &self,
        mut frame: Frame<'a>,
        detections: &[Rect],
        scratch: &mut [u8],
    ) -> Result<RedactedFrame<'a>, FrameError> {
        for rect in detections {
            frame.blur_region(*rect, self.radius, scratch)?;
        }
        Ok(RedactedFrame { frame })
    }
}

/// Visualizer hook: returns `true` if any pixel outside every detection differs
/// from `before`. A correct redaction returns `false`. The blur must touch only
/// the detected regions.
#[must_use]
pub fn changed_outside_detections(
    before: &[u8],
    after: &RedactedFrame<'_>,
    detections: &[Rect],
) -> bool {
    let width = after.width();
    let height = after.height();
    (0..height).any(|y| {
        (0..width).any(|x| {
            let idx = y * width + x;
            let inside = detections.iter().any(|r| r.contains(x, y));
            !inside && before[idx] != after.pixels()[idx]
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn redaction_changes_only_detected_regions() {
        let (w, h) = (32, 24);
        let original: Vec<u8> = (0..w * h).map(|i| ((i * 7) % 251) as u8).collect();
        let mut px = original.clone();
        let detections = [
            Rect {
                x: 2,
                y: 2,
                w: 8,
                h: 8,
            },
            Rect {
                x: 20,
                y: 10,
                w: 6,
                h: 5,
            },
        ];
        let mut scratch = vec![0u8; Redactor::scratch_len(&detections, w, h)];
        let frame = Frame::new(&mut px, w, h).unwrap();
        let redacted = Redactor::new(2)
            .redact(frame, &detections, &mut scratch)
            .unwrap();

        assert!(!changed_outside_detections(
            &original,
            &redacted,
            &detections
        ));
        assert_ne!(redacted.pixels(), original.as_slice());
    }

    #[test]
    fn detection_partly_off_frame_is_clipped_not_rejected() {
        let (w, h) = (16, 16);
        let mut px = vec![10u8; w * h];
        let detections = [Rect {
            x: 12,
            y: 12,
            w: 10,
            h: 10,
        }];
        let mut scratch = vec![0u8; Redactor::scratch_len(&detections, w, h)];
        let frame = Frame::new(&mut px, w, h).unwrap();
        assert!(
            Redactor::new(1)
                .redact(frame, &detections, &mut scratch)
                .is_ok()
        );
    }

    struct Recorder {
        frames: usize,
        last_len: usize,
    }

    impl Sink for Recorder {
        fn accept(&mut self, frame: &RedactedFrame<'_>) {
            self.frames += 1;
            self.last_len = frame.pixels().len();
        }
    }

    #[test]
    fn only_redacted_frames_reach_the_sink() {
        let (w, h) = (8, 8);
        let mut px = vec![0u8; w * h];
        let mut scratch = vec![0u8; 0];
        let redacted = Redactor::new(1)
            .redact(Frame::new(&mut px, w, h).unwrap(), &[], &mut scratch)
            .unwrap();
        let mut sink = Recorder {
            frames: 0,
            last_len: 0,
        };
        sink.accept(&redacted);
        assert_eq!((sink.frames, sink.last_len), (1, 64));
    }
}
