//! GNSS urban-canyon scenarios driven through the localization source selector.
//!
//! A vehicle drives along +x at 10 m/s, and its GNSS receiver follows a schedule
//! of multipath and denial windows. Each 100 ms the selector sees the fix (or its
//! absence) and picks a source. The position used is the GNSS fix when GNSS is
//! selected, and the true position when LiDAR is selected. LiDAR is modeled as
//! ground truth here. These tests check the switching, not sensor accuracy.

use tpt_t_urban_perception::gnss::{RECOVERY_FIXES, Source, SourceSelector};
use tpt_t_urban_sim::{Condition, Gnss, Window};

const SPEED_MM_S: i64 = 10_000;
const STEP_MS: u64 = 100;

/// One logged cycle.
struct Step {
    t_ms: u64,
    source: Source,
    /// Position error of the estimate used, mm.
    error_mm: i64,
}

/// Runs the vehicle for `duration_ms` through the receiver's schedule.
fn drive(windows: &[Window], duration_ms: u64) -> Vec<Step> {
    let mut gnss = Gnss::new(windows, 20, 0xC0FF_EE01);
    let mut selector = SourceSelector::new();
    let mut steps = Vec::new();
    for t_ms in (0..duration_ms).step_by(STEP_MS as usize) {
        let truth_x = SPEED_MM_S * t_ms as i64 / 1_000;
        let source = match gnss.fix(t_ms, truth_x, 0) {
            Some(fix) => selector.on_gnss_fix(fix),
            None => selector.on_gnss_lost(),
        };
        let used_x = match source {
            Source::Gnss => gnss.fix(t_ms, truth_x, 0).map_or(truth_x, |f| f.x_mm),
            Source::Lidar => truth_x,
        };
        steps.push(Step {
            t_ms,
            source,
            error_mm: (used_x - truth_x).abs(),
        });
    }
    steps
}

fn multipath_jump(start_ms: u64, end_ms: u64) -> Window {
    Window {
        start_ms,
        end_ms,
        condition: Condition::Multipath {
            dx_mm: 5_000,
            dy_mm: 0,
        },
    }
}

#[test]
fn a_clean_track_stays_on_gnss() {
    let steps = drive(&[], 10_000);
    assert!(steps.iter().all(|s| s.source == Source::Gnss));
}

#[test]
fn a_five_metre_multipath_jump_switches_to_lidar_and_is_never_used() {
    let steps = drive(&[multipath_jump(2_000, 4_000)], 8_000);

    // The jump at the window's start switches the source at once.
    let switch = steps.iter().find(|s| s.t_ms >= 2_000).unwrap();
    assert_eq!(switch.source, Source::Lidar);

    // Without the selector the estimate would be 5 m off. Here it never is.
    let worst = steps.iter().map(|s| s.error_mm).max().unwrap();
    assert!(worst < 500, "estimate error {worst} mm reached the planner");
}

#[test]
fn denial_falls_back_to_lidar_and_gnss_returns_after_a_consistent_run() {
    let denied_until = 3_000;
    let windows = [Window {
        start_ms: 1_000,
        end_ms: denied_until,
        condition: Condition::Denied,
    }];
    let steps = drive(&windows, 12_000);

    // Lost during the denial window.
    assert!(
        steps
            .iter()
            .filter(|s| (1_000..denied_until).contains(&s.t_ms))
            .all(|s| s.source == Source::Lidar)
    );

    // Back on GNSS exactly RECOVERY_FIXES fixes after the signal returns.
    let regained = steps
        .iter()
        .find(|s| s.t_ms >= denied_until && s.source == Source::Gnss)
        .expect("GNSS was never regained");
    let fixes_needed = (regained.t_ms - denied_until) / STEP_MS + 1;
    assert_eq!(fixes_needed, u64::from(RECOVERY_FIXES));
}

#[test]
fn a_jump_during_recovery_restarts_the_count() {
    // Denial ends at 3 s. A second multipath jump at 4 s interrupts the recovery.
    let windows = [
        Window {
            start_ms: 1_000,
            end_ms: 3_000,
            condition: Condition::Denied,
        },
        multipath_jump(4_000, 4_500),
    ];
    let steps = drive(&windows, 12_000);
    let regained = steps
        .iter()
        .find(|s| s.t_ms > 4_500 && s.source == Source::Gnss)
        .expect("GNSS was never regained");
    // The count restarted after the jump, so recovery needs a full run again.
    let after_jump_fixes = (regained.t_ms - 4_500) / STEP_MS;
    assert!(after_jump_fixes >= u64::from(RECOVERY_FIXES));
}
