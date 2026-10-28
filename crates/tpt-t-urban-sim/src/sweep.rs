//! Scenario injection: a seeded sweep over many pedestrian crossings.
//!
//! Hand-written scenarios check the cases someone thought of. The sweep samples
//! speed, crossing distance, walking speed, noise and planner braking at random,
//! from a fixed seed, so a run is reproducible. Each sample is run twice, with
//! the guardian on and off.
//!
//! A crossing is **avoidable** when an observer that sees the pedestrian from
//! `t = 0` could have stopped in time: it brakes at the guardian's full AEB
//! deceleration after the spec's reaction time, and stops with [`STOP_MARGIN_M`]
//! in hand. The reaction time is physical, so no controller can skip it. A
//! guarded collision on an avoidable crossing is a failure of the guardian or
//! the chain that feeds it. Unavoidable crossings are counted but not judged.

use tpt_t_urban_safety::{AEB_DECEL_MM_S2, REACTION_MS};

use crate::scenario::{FRONT_OFFSET_M, Outcome, Scenario, run_crossing};

/// Distance the vehicle must keep in hand beyond its stopping distance, m.
pub const STOP_MARGIN_M: f32 = 1.0;

/// Summary of a sweep.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SweepReport {
    /// Crossings sampled.
    pub samples: u32,
    /// Crossings where the vehicle could have stopped in time.
    pub avoidable: u32,
    /// Collisions with the guardian on.
    pub guarded_collisions: u32,
    /// Collisions with the guardian off, for comparison.
    pub unguarded_collisions: u32,
    /// Guarded collisions on avoidable crossings. Must be empty.
    pub failures: Vec<(Scenario, Outcome)>,
}

/// Whether AEB, applied from the start after the reaction time, stops the vehicle
/// short of the crossing point with [`STOP_MARGIN_M`] in hand.
#[must_use]
pub fn avoidable(s: &Scenario) -> bool {
    let aeb = AEB_DECEL_MM_S2 as f32 / 1_000.0;
    let reaction = REACTION_MS as f32 / 1_000.0;
    // Distance covered before the brake takes hold, then the braking distance.
    let stop_distance = s.speed_mps * reaction + s.speed_mps * s.speed_mps / (2.0 * aeb);
    let front_to_crossing = s.crossing_x_m - FRONT_OFFSET_M;
    front_to_crossing - stop_distance >= STOP_MARGIN_M
}

/// One xorshift32 step, mapped to `[0, 1)`.
fn unit(state: &mut u32) -> f32 {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    *state as f32 / (u32::MAX as f32 + 1.0)
}

/// A uniform sample in `[lo, hi)` from the xorshift stream.
fn range(state: &mut u32, lo: f32, hi: f32) -> f32 {
    lo + unit(state) * (hi - lo)
}

/// Draws one crossing scenario. Guardian is left on; the caller turns it off for
/// the comparison run.
fn sample(state: &mut u32) -> Scenario {
    Scenario {
        speed_mps: range(state, 2.0, 20.0),
        planner_decel_mps2: range(state, 0.0, 6.0),
        crossing_x_m: range(state, 15.0, 120.0),
        walk_speed_mps: range(state, 0.8, 2.0),
        noise_m: range(state, 0.0, 0.3),
        guardian_enabled: true,
        duration_s: 10.0,
        dt_s: 0.05,
    }
}

/// Runs `count` seeded crossings and reports the guardian's record on them.
///
/// `seed` must be non-zero, since an xorshift stream stays at zero once it gets
/// there. Zero is replaced with a fixed non-zero constant.
#[must_use]
pub fn sweep(seed: u32, count: u32) -> SweepReport {
    let mut state = if seed == 0 { 0x2545_F491 } else { seed };
    let mut report = SweepReport::default();
    for _ in 0..count {
        let guarded = sample(&mut state);
        let unguarded = Scenario {
            guardian_enabled: false,
            ..guarded
        };
        let outcome = run_crossing(&guarded);
        let baseline = run_crossing(&unguarded);

        report.samples += 1;
        if baseline.collided {
            report.unguarded_collisions += 1;
        }
        if outcome.collided {
            report.guarded_collisions += 1;
        }
        if avoidable(&guarded) {
            report.avoidable += 1;
            if outcome.collided {
                report.failures.push((guarded, outcome));
            }
        }
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_crossing_far_ahead_is_avoidable_and_one_on_top_of_the_car_is_not() {
        let far = Scenario {
            crossing_x_m: 100.0,
            ..Scenario::default()
        };
        assert!(avoidable(&far));
        let on_top = Scenario {
            crossing_x_m: 3.0,
            ..Scenario::default()
        };
        assert!(!avoidable(&on_top));
    }

    #[test]
    fn the_same_seed_gives_the_same_report() {
        assert_eq!(sweep(7, 50), sweep(7, 50));
    }

    #[test]
    fn the_guardian_never_hits_an_avoidable_crossing() {
        for seed in [0xC0FF_EE01, 0x0BAD_F00D] {
            let report = sweep(seed, 1_000);
            assert!(report.avoidable > 0, "sweep sampled no avoidable crossings");
            assert!(
                report.failures.is_empty(),
                "seed {seed:#x}: {} of {} avoidable crossings hit:\n{:#?}",
                report.failures.len(),
                report.avoidable,
                report.failures.first(),
            );
        }
    }

    #[test]
    fn the_guardian_reduces_collisions_over_no_guardian() {
        let report = sweep(0x0BAD_5EED, 400);
        assert!(
            report.unguarded_collisions > 0,
            "sweep never crashed unguarded"
        );
        assert!(report.guarded_collisions < report.unguarded_collisions);
    }
}
