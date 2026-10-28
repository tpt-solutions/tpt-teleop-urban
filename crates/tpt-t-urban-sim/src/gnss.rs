//! GNSS model for urban canyons: noise, multipath jumps, and signal denial.
//!
//! The model is a schedule of [`Window`]s over time. Outside every window the fix
//! is clean, with noise only. Inside a window the condition applies:
//!
//! - [`Condition::Denied`]: no fix at all, as in a tunnel or a deep canyon.
//! - [`Condition::Multipath`]: the fix is offset by a fixed amount, as when a
//!   reflection off a building face is the strongest signal. The offset is what
//!   makes the reported position jump.
//!
//! Units are millimetres and milliseconds, the same as `tpt_t_urban_perception::gnss`.

use tpt_t_urban_perception::gnss::Fix;

/// What the signal does during a window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Condition {
    /// Noise only.
    Clean,
    /// No fix is produced.
    Denied,
    /// The fix is offset by `(dx_mm, dy_mm)` from the true position.
    Multipath { dx_mm: i64, dy_mm: i64 },
}

/// A stretch of time with one condition. `start_ms` is included, `end_ms` is not.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window {
    pub start_ms: u64,
    pub end_ms: u64,
    pub condition: Condition,
}

/// GNSS receiver with a fixed schedule of conditions.
pub struct Gnss<'a> {
    windows: &'a [Window],
    noise_mm: i64,
    state: u32,
}

impl<'a> Gnss<'a> {
    /// A receiver following `windows`, with noise of up to `noise_mm` per axis.
    /// `seed` must be non-zero.
    #[must_use]
    pub fn new(windows: &'a [Window], noise_mm: i64, seed: u32) -> Self {
        Self {
            windows,
            noise_mm,
            state: if seed == 0 { 0x9E37_79B9 } else { seed },
        }
    }

    /// The condition at `t_ms`. Where windows overlap, the first one listed wins.
    #[must_use]
    pub fn condition_at(&self, t_ms: u64) -> Condition {
        self.windows
            .iter()
            .find(|w| (w.start_ms..w.end_ms).contains(&t_ms))
            .map_or(Condition::Clean, |w| w.condition)
    }

    /// One fix at `t_ms` for a vehicle truly at `(truth_x_mm, truth_y_mm)`, or
    /// `None` if the signal is denied.
    pub fn fix(&mut self, t_ms: u64, truth_x_mm: i64, truth_y_mm: i64) -> Option<Fix> {
        let (bias_x, bias_y) = match self.condition_at(t_ms) {
            Condition::Clean => (0, 0),
            Condition::Denied => return None,
            Condition::Multipath { dx_mm, dy_mm } => (dx_mm, dy_mm),
        };
        let x_mm = truth_x_mm + bias_x + self.noise();
        let y_mm = truth_y_mm + bias_y + self.noise();
        Some(Fix { x_mm, y_mm, t_ms })
    }

    /// Noise in `[-noise_mm, noise_mm]` from an xorshift stream.
    fn noise(&mut self) -> i64 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        let unit = f64::from(self.state) / f64::from(u32::MAX);
        ((unit * 2.0 - 1.0) * self.noise_mm as f64).round() as i64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn denied_window_produces_no_fix_and_clean_time_does() {
        let windows = [Window {
            start_ms: 1_000,
            end_ms: 2_000,
            condition: Condition::Denied,
        }];
        let mut gnss = Gnss::new(&windows, 0, 1);
        assert!(gnss.fix(500, 0, 0).is_some());
        assert!(gnss.fix(1_500, 0, 0).is_none());
        assert!(gnss.fix(2_000, 0, 0).is_some(), "end is exclusive");
    }

    #[test]
    fn multipath_offsets_the_fix_by_the_configured_amount() {
        let windows = [Window {
            start_ms: 0,
            end_ms: 1_000,
            condition: Condition::Multipath {
                dx_mm: 5_000,
                dy_mm: -1_000,
            },
        }];
        let mut gnss = Gnss::new(&windows, 0, 1);
        let fix = gnss.fix(10, 100_000, 200_000).unwrap();
        assert_eq!((fix.x_mm, fix.y_mm), (105_000, 199_000));
    }

    #[test]
    fn noise_stays_within_its_bound() {
        let mut gnss = Gnss::new(&[], 50, 7);
        for t in 0..1_000 {
            let fix = gnss.fix(t, 0, 0).unwrap();
            assert!(fix.x_mm.abs() <= 50 && fix.y_mm.abs() <= 50);
        }
    }
}
