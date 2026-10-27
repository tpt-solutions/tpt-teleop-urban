//! Curb edge detection from a lateral height profile.
//!
//! A LiDAR scan across the sidewalk gives height against lateral offset. The
//! curb is the first place where the height steps up by more than the curb
//! height threshold. The ultrasonic mounting check in `tpt-t-urban-delivery`
//! confirms the result at the wheel.

/// Height step that counts as a curb: 80 mm.
pub const CURB_STEP_MM: i32 = 80;

/// One LiDAR sample: lateral offset from the vehicle in millimetres, and the
/// ground height at that offset in millimetres.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sample {
    pub lateral_mm: i32,
    pub height_mm: i32,
}

/// A detected curb edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Curb {
    /// Lateral offset of the top edge.
    pub lateral_mm: i32,
    /// Height difference across the edge.
    pub step_mm: i32,
}

/// Finds the first upward height step of at least [`CURB_STEP_MM`] between
/// consecutive samples, scanning in the order given.
///
/// Samples must be ordered by lateral offset. Samples that are not ordered by
/// lateral offset are skipped.
#[must_use]
pub fn detect_curb(profile: &[Sample]) -> Option<Curb> {
    profile.windows(2).find_map(|pair| {
        let (a, b) = (pair[0], pair[1]);
        if b.lateral_mm <= a.lateral_mm {
            return None;
        }
        let step = b.height_mm - a.height_mm;
        (step >= CURB_STEP_MM).then_some(Curb {
            lateral_mm: b.lateral_mm,
            step_mm: step,
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(lateral_mm: i32, height_mm: i32) -> Sample {
        Sample {
            lateral_mm,
            height_mm,
        }
    }

    #[test]
    fn finds_the_top_edge_of_a_curb() {
        let profile = [s(0, 0), s(100, 0), s(200, 0), s(300, 120), s(400, 120)];
        assert_eq!(
            detect_curb(&profile),
            Some(Curb {
                lateral_mm: 300,
                step_mm: 120
            })
        );
    }

    #[test]
    fn flat_road_has_no_curb() {
        let profile = [s(0, 0), s(100, 5), s(200, 0)];
        assert_eq!(detect_curb(&profile), None);
    }

    #[test]
    fn step_down_is_not_a_curb() {
        let profile = [s(0, 150), s(100, 0)];
        assert_eq!(detect_curb(&profile), None);
    }

    #[test]
    fn step_at_exactly_the_threshold_counts() {
        let profile = [s(0, 0), s(100, CURB_STEP_MM)];
        assert!(detect_curb(&profile).is_some());
    }

    #[test]
    fn unordered_samples_are_skipped() {
        let profile = [s(300, 0), s(100, 200)];
        assert_eq!(detect_curb(&profile), None);
    }
}
