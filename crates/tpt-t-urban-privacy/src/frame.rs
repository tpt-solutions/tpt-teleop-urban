//! Frame buffers and the in-place region blur.
//!
//! The blur is three passes of a separable box filter, which approximates a
//! Gaussian. It runs in place on the caller's frame. The only scratch memory is
//! a caller-provided slice, so the blur never allocates.

/// Why a frame could not be built or blurred.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// `pixels.len()` does not equal `width * height`.
    SizeMismatch,
    /// The scratch slice is smaller than the region it must hold.
    ScratchTooSmall,
}

/// Axis-aligned rectangle in pixel coordinates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rect {
    pub x: usize,
    pub y: usize,
    pub w: usize,
    pub h: usize,
}

impl Rect {
    /// This rectangle clipped to a `width` × `height` frame, or `None` if the
    /// overlap is empty.
    #[must_use]
    pub fn clip(self, width: usize, height: usize) -> Option<Rect> {
        if self.x >= width || self.y >= height {
            return None;
        }
        let w = self.w.min(width - self.x);
        let h = self.h.min(height - self.y);
        (w > 0 && h > 0).then_some(Rect {
            x: self.x,
            y: self.y,
            w,
            h,
        })
    }

    /// Whether pixel `(x, y)` lies inside this rectangle.
    #[must_use]
    pub const fn contains(&self, x: usize, y: usize) -> bool {
        x >= self.x && x < self.x + self.w && y >= self.y && y < self.y + self.h
    }

    /// Pixels in the rectangle.
    #[must_use]
    pub const fn area(&self) -> usize {
        self.w * self.h
    }
}

/// A grayscale frame borrowed from the capture ring buffer.
#[derive(Debug)]
pub struct Frame<'a> {
    pixels: &'a mut [u8],
    width: usize,
    height: usize,
}

impl<'a> Frame<'a> {
    /// Wraps `pixels` as a `width` × `height` frame, row-major.
    pub fn new(pixels: &'a mut [u8], width: usize, height: usize) -> Result<Self, FrameError> {
        if pixels.len() != width * height {
            return Err(FrameError::SizeMismatch);
        }
        Ok(Self {
            pixels,
            width,
            height,
        })
    }

    #[must_use]
    pub const fn width(&self) -> usize {
        self.width
    }

    #[must_use]
    pub const fn height(&self) -> usize {
        self.height
    }

    #[must_use]
    pub fn pixels(&self) -> &[u8] {
        self.pixels
    }

    /// Blurs `rect` (clipped to the frame) in place. `scratch` must hold at least
    /// the clipped area. Returns the clipped rectangle that was blurred, or
    /// `None` if nothing overlapped the frame.
    pub fn blur_region(
        &mut self,
        rect: Rect,
        radius: usize,
        scratch: &mut [u8],
    ) -> Result<Option<Rect>, FrameError> {
        let Some(region) = rect.clip(self.width, self.height) else {
            return Ok(None);
        };
        if scratch.len() < region.area() {
            return Err(FrameError::ScratchTooSmall);
        }
        for _ in 0..3 {
            self.box_pass_horizontal(region, radius, &mut scratch[..region.area()]);
            self.box_pass_vertical(region, radius, &scratch[..region.area()]);
        }
        Ok(Some(region))
    }

    /// Horizontal box filter: frame region -> scratch, clamped at region edges.
    fn box_pass_horizontal(&self, r: Rect, radius: usize, out: &mut [u8]) {
        for row in 0..r.h {
            let base = (r.y + row) * self.width + r.x;
            let line = &self.pixels[base..base + r.w];
            for col in 0..r.w {
                let lo = col.saturating_sub(radius);
                let hi = (col + radius).min(r.w - 1);
                let sum: u32 = line[lo..=hi].iter().map(|&v| u32::from(v)).sum();
                out[row * r.w + col] = (sum / (hi - lo + 1) as u32) as u8;
            }
        }
    }

    /// Vertical box filter: scratch -> frame region, clamped at region edges.
    fn box_pass_vertical(&mut self, r: Rect, radius: usize, src: &[u8]) {
        for col in 0..r.w {
            for row in 0..r.h {
                let lo = row.saturating_sub(radius);
                let hi = (row + radius).min(r.h - 1);
                let sum: u32 = (lo..=hi).map(|k| u32::from(src[k * r.w + col])).sum();
                self.pixels[(r.y + row) * self.width + r.x + col] =
                    (sum / (hi - lo + 1) as u32) as u8;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gradient(width: usize, height: usize) -> Vec<u8> {
        (0..width * height).map(|i| (i % 256) as u8).collect()
    }

    #[test]
    fn rejects_mismatched_buffer() {
        let mut px = vec![0u8; 10];
        assert_eq!(
            Frame::new(&mut px, 4, 4).unwrap_err(),
            FrameError::SizeMismatch
        );
    }

    #[test]
    fn rect_clipping() {
        let r = Rect {
            x: 6,
            y: 6,
            w: 10,
            h: 10,
        };
        assert_eq!(
            r.clip(8, 8),
            Some(Rect {
                x: 6,
                y: 6,
                w: 2,
                h: 2
            })
        );
        assert_eq!(
            Rect {
                x: 9,
                y: 0,
                w: 2,
                h: 2
            }
            .clip(8, 8),
            None
        );
    }

    #[test]
    fn blur_of_uniform_region_is_unchanged() {
        let mut px = vec![90u8; 64];
        let mut frame = Frame::new(&mut px, 8, 8).unwrap();
        let mut scratch = vec![0u8; 64];
        frame
            .blur_region(
                Rect {
                    x: 2,
                    y: 2,
                    w: 4,
                    h: 4,
                },
                2,
                &mut scratch,
            )
            .unwrap();
        assert!(frame.pixels().iter().all(|&v| v == 90));
    }

    #[test]
    fn blur_reduces_contrast_inside_only() {
        let (w, h) = (16, 16);
        let original = gradient(w, h);
        let mut px = original.clone();
        let rect = Rect {
            x: 4,
            y: 4,
            w: 8,
            h: 8,
        };
        let mut scratch = vec![0u8; rect.area()];
        let mut frame = Frame::new(&mut px, w, h).unwrap();
        frame.blur_region(rect, 2, &mut scratch).unwrap();

        let spread = |vals: &[u8]| vals.iter().max().unwrap() - vals.iter().min().unwrap();
        let inside: Vec<u8> = (0..h)
            .flat_map(|y| (0..w).map(move |x| (x, y)))
            .filter(|&(x, y)| rect.contains(x, y))
            .map(|(x, y)| px[y * w + x])
            .collect();
        let inside_orig: Vec<u8> = (0..h)
            .flat_map(|y| (0..w).map(move |x| (x, y)))
            .filter(|&(x, y)| rect.contains(x, y))
            .map(|(x, y)| original[y * w + x])
            .collect();
        assert!(spread(&inside) < spread(&inside_orig));

        for y in 0..h {
            for x in 0..w {
                if !rect.contains(x, y) {
                    assert_eq!(px[y * w + x], original[y * w + x], "pixel ({x},{y})");
                }
            }
        }
    }

    #[test]
    fn scratch_must_fit_the_region() {
        let mut px = vec![0u8; 64];
        let mut frame = Frame::new(&mut px, 8, 8).unwrap();
        let mut scratch = vec![0u8; 3];
        assert_eq!(
            frame.blur_region(
                Rect {
                    x: 0,
                    y: 0,
                    w: 4,
                    h: 4
                },
                1,
                &mut scratch
            ),
            Err(FrameError::ScratchTooSmall)
        );
    }
}
