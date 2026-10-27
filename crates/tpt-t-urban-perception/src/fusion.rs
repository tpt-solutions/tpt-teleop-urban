//! Position fusion: combines GNSS and LiDAR estimates by inverse variance.
//!
//! Each source reports a position and a variance. The fused estimate is the
//! variance-weighted mean, and its variance is the reciprocal of the summed
//! inverse variances, which is never larger than either input. This is the
//! static (one-step) form of a Kalman measurement update, and it keeps the
//! detector's "drop GNSS on anomaly" from being all-or-nothing: a degraded GNSS
//! fix still contributes, but with less weight.
//!
//! Units: metres for position, m² for variance. Floats are used here, since the
//! fusion is not on the bit-exact path. The anomaly gate in [`crate::gnss`] keeps
//! integer arithmetic.

/// Position estimate with its variance (m²) per axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimate {
    pub x: f32,
    pub y: f32,
    pub variance: f32,
}

/// Fuses two independent estimates. Returns `None` when both variances are
/// non-positive or non-finite, since no weighting is then defined.
#[must_use]
pub fn fuse(a: Estimate, b: Estimate) -> Option<Estimate> {
    let wa = weight(a.variance)?;
    let wb = weight(b.variance)?;
    let total = wa + wb;
    Some(Estimate {
        x: (wa * a.x + wb * b.x) / total,
        y: (wa * a.y + wb * b.y) / total,
        variance: 1.0 / total,
    })
}

fn weight(variance: f32) -> Option<f32> {
    if variance.is_finite() && variance > 0.0 {
        Some(1.0 / variance)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn est(x: f32, y: f32, variance: f32) -> Estimate {
        Estimate { x, y, variance }
    }

    #[test]
    fn equal_variances_average_the_positions() {
        let f = fuse(est(0.0, 0.0, 1.0), est(2.0, 4.0, 1.0)).unwrap();
        assert!((f.x - 1.0).abs() < 1e-6 && (f.y - 2.0).abs() < 1e-6);
        assert!((f.variance - 0.5).abs() < 1e-6);
    }

    #[test]
    fn the_more_certain_source_dominates() {
        // A degraded GNSS fix (variance 25 m², a 5 m multipath error) against a precise LiDAR fix.
        let f = fuse(est(5.0, 0.0, 25.0), est(0.0, 0.0, 0.01)).unwrap();
        assert!(f.x.abs() < 0.01, "x {}", f.x);
    }

    #[test]
    fn fused_variance_is_never_worse_than_either_input() {
        let f = fuse(est(0.0, 0.0, 0.04), est(0.0, 0.0, 4.0)).unwrap();
        assert!(f.variance < 0.04);
    }

    #[test]
    fn invalid_variance_is_rejected() {
        assert_eq!(fuse(est(0.0, 0.0, 0.0), est(0.0, 0.0, 1.0)), None);
        assert_eq!(fuse(est(0.0, 0.0, f32::NAN), est(0.0, 0.0, 1.0)), None);
    }
}
