//! GNSS anomaly detection and localization source selection.
//!
//! GNSS multipath in urban canyons can jump the reported position by metres
//! in a single fix. A jump is an anomaly when it implies a speed no vehicle can
//! reach. The selector then switches to LiDAR localization, and returns to GNSS
//! only after a run of consistent fixes.
//!
//! All units are integers: millimetres and milliseconds.

/// A GNSS position fix in a local planar frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fix {
    /// x position in the local planar frame, in millimetres.
    pub x_mm: i64,
    /// y position in the local planar frame, in millimetres.
    pub y_mm: i64,
    /// Timestamp, in milliseconds.
    pub t_ms: u64,
}

/// Largest speed a GNSS track may imply before the step is an anomaly: 25 m/s.
pub const MAX_PLAUSIBLE_SPEED_MM_S: i64 = 25_000;
/// Allowance added to the speed bound, so tiny fix noise is not an anomaly: 1 m.
pub const JUMP_TOLERANCE_MM: i64 = 1_000;
/// Consecutive consistent fixes required before GNSS is trusted again.
pub const RECOVERY_FIXES: u32 = 20;

/// Result of checking one new fix against the previous one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FixCheck {
    /// First fix, or consistent with the previous fix.
    Consistent,
    /// Implies a speed above the plausible bound.
    Anomalous,
    /// Timestamp did not advance. The fix is discarded.
    NonMonotonic,
}

/// Checks each fix for kinematic plausibility against the previous fix.
#[derive(Clone, Copy, Debug, Default)]
pub struct JumpDetector {
    last: Option<Fix>,
}

impl JumpDetector {
    #[must_use]
    /// A detector with no history.
    pub const fn new() -> Self {
        Self { last: None }
    }

    /// Checks `fix`. The previous fix is replaced only when the check is not
    /// `NonMonotonic`, so one bad fix cannot poison the reference.
    pub fn check(&mut self, fix: Fix) -> FixCheck {
        let Some(prev) = self.last else {
            self.last = Some(fix);
            return FixCheck::Consistent;
        };
        if fix.t_ms <= prev.t_ms {
            return FixCheck::NonMonotonic;
        }
        let dt_ms = (fix.t_ms - prev.t_ms) as i64;
        let dx = fix.x_mm - prev.x_mm;
        let dy = fix.y_mm - prev.y_mm;
        // Euclidean distance without a float: compare squared values.
        let dist_sq = i128::from(dx) * i128::from(dx) + i128::from(dy) * i128::from(dy);
        // Allowed distance over dt: speed * dt + tolerance, all in mm.
        let allowed_mm =
            MAX_PLAUSIBLE_SPEED_MM_S as i128 * dt_ms as i128 / 1_000 + JUMP_TOLERANCE_MM as i128;
        self.last = Some(fix);
        if dist_sq > allowed_mm * allowed_mm {
            FixCheck::Anomalous
        } else {
            FixCheck::Consistent
        }
    }
}

/// Which source the vehicle localizes with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// Localize with GNSS.
    Gnss,
    /// Localize with LiDAR.
    Lidar,
}

/// Chooses GNSS or LiDAR localization with hysteresis.
#[derive(Clone, Copy, Debug)]
pub struct SourceSelector {
    source: Source,
    detector: JumpDetector,
    consistent_run: u32,
}

impl Default for SourceSelector {
    fn default() -> Self {
        Self::new()
    }
}

impl SourceSelector {
    #[must_use]
    /// A selector that starts on GNSS, with no history.
    pub const fn new() -> Self {
        Self {
            source: Source::Gnss,
            detector: JumpDetector::new(),
            consistent_run: 0,
        }
    }

    #[must_use]
    /// The source chosen for this cycle.
    pub const fn source(&self) -> Source {
        self.source
    }

    /// Feeds one GNSS fix and returns the source to use for this cycle.
    ///
    /// An anomaly switches to LiDAR at once. Returning to GNSS needs
    /// [`RECOVERY_FIXES`] consistent fixes in a row. A non-monotonic fix is
    /// ignored and does not count toward recovery.
    pub fn on_gnss_fix(&mut self, fix: Fix) -> Source {
        match self.detector.check(fix) {
            FixCheck::Anomalous => {
                self.source = Source::Lidar;
                self.consistent_run = 0;
            }
            FixCheck::Consistent => {
                if self.source == Source::Lidar {
                    self.consistent_run += 1;
                    if self.consistent_run >= RECOVERY_FIXES {
                        self.source = Source::Gnss;
                        self.consistent_run = 0;
                    }
                }
            }
            FixCheck::NonMonotonic => {}
        }
        self.source
    }

    /// Reports that no GNSS fix arrived this cycle. Losing GNSS also falls back
    /// to LiDAR, and the recovery count restarts.
    pub fn on_gnss_lost(&mut self) -> Source {
        self.source = Source::Lidar;
        self.consistent_run = 0;
        self.source
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fix(x_mm: i64, t_ms: u64) -> Fix {
        Fix {
            x_mm,
            y_mm: 0,
            t_ms,
        }
    }

    #[test]
    fn smooth_motion_is_consistent() {
        let mut d = JumpDetector::new();
        // 10 m/s: 1 m every 100 ms.
        for i in 0..50 {
            assert_eq!(
                d.check(fix(i * 1_000, i as u64 * 100)),
                FixCheck::Consistent
            );
        }
    }

    #[test]
    fn five_metre_jump_in_one_fix_is_anomalous() {
        let mut d = JumpDetector::new();
        d.check(fix(0, 0));
        assert_eq!(d.check(fix(5_000, 100)), FixCheck::Anomalous);
    }

    #[test]
    fn non_monotonic_timestamp_is_discarded() {
        let mut d = JumpDetector::new();
        d.check(fix(0, 100));
        assert_eq!(d.check(fix(0, 100)), FixCheck::NonMonotonic);
        // The reference is still the fix at t=100, so a normal step is consistent.
        assert_eq!(d.check(fix(500, 200)), FixCheck::Consistent);
    }

    #[test]
    fn anomaly_switches_to_lidar_and_recovers_after_a_run() {
        let mut s = SourceSelector::new();
        s.on_gnss_fix(fix(0, 0));
        assert_eq!(s.on_gnss_fix(fix(5_000, 100)), Source::Lidar);

        // Consistent fixes at 10 m/s. The reference point is the anomalous fix.
        let mut x = 5_000;
        let mut t = 100;
        for _ in 0..RECOVERY_FIXES - 1 {
            x += 1_000;
            t += 100;
            assert_eq!(s.on_gnss_fix(fix(x, t)), Source::Lidar);
        }
        x += 1_000;
        t += 100;
        assert_eq!(s.on_gnss_fix(fix(x, t)), Source::Gnss);
    }

    #[test]
    fn losing_gnss_falls_back_to_lidar() {
        let mut s = SourceSelector::new();
        assert_eq!(s.on_gnss_lost(), Source::Lidar);
    }

    #[test]
    fn non_monotonic_fix_does_not_count_toward_recovery() {
        let mut s = SourceSelector::new();
        s.on_gnss_fix(fix(0, 100));
        s.on_gnss_fix(fix(5_000, 200)); // anomaly
        for _ in 0..RECOVERY_FIXES * 2 {
            s.on_gnss_fix(fix(5_000, 150)); // stale timestamp
        }
        assert_eq!(s.source(), Source::Lidar);
    }
}
