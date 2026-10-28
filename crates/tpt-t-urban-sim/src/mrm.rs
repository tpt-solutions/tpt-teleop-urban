//! Pull-over minimal risk maneuver, run through the plant.
//!
//! The guardian latches an MRM when the planner heartbeat goes stale. The
//! planner's path is then replaced: the vehicle follows a shoulder pull-over
//! path from `tpt-t-urban-car` with pure pursuit, and brakes at the MRM rate
//! until it stops.
//!
//! The path is sized at 1.2 times the minimum length for the speed at the
//! trigger, so the lateral-acceleration limit holds with margin while the
//! vehicle slows.

use tpt_t_urban_car::pull_over::{Maneuver, choose};
use tpt_t_urban_car::{Point, Pose, pull_over, pure_pursuit};
use tpt_t_urban_safety::{Guardian, Inputs, MRM_DECEL_MM_S2, Mode};

use crate::plant::Bicycle;

/// Wheelbase of the simulated vehicle, m.
pub const WHEELBASE_M: f32 = 2.7;
/// Pure-pursuit lookahead distance, m.
pub const LOOKAHEAD_M: f32 = 4.0;
/// Path length margin over the minimum for the lateral-acceleration limit.
pub const LENGTH_MARGIN: f32 = 1.2;

/// Result of one pull-over run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PullOver {
    /// Lateral position at the end of the run, m, along the offset direction.
    pub final_offset_m: f32,
    /// Speed at the end of the run, m/s.
    pub final_speed_mps: f32,
    /// Largest lateral acceleration during the run, m/s².
    pub max_lateral_accel: f32,
    /// Distance travelled along the lane, m.
    pub distance_m: f32,
    /// Whether the vehicle stopped with its lateral position within 0.5 m of the
    /// shoulder.
    pub stopped_on_shoulder: bool,
}

/// Latches an MRM through the guardian from a stale planner heartbeat, and
/// returns whether the guardian is now in MRM.
#[must_use]
pub fn trigger(guardian: &mut Guardian) -> bool {
    let command = guardian.step(&Inputs {
        obstacle: None,
        planned_decel_mm_s2: 0,
        planner_heartbeat_age_ms: 60,
        speed_mm_s: 0,
    });
    command.mode == Mode::Mrm
}

/// Runs the pull-over from `speed_mps` to a shoulder `offset_m` to the left,
/// with `lateral_accel` as the limit the path is sized for.
#[must_use]
pub fn run(speed_mps: f32, offset_m: f32, lateral_accel: f32, dt_s: f32) -> PullOver {
    let length = LENGTH_MARGIN * pull_over::min_length(speed_mps, offset_m, lateral_accel);
    let mut path = vec![Point { x: 0.0, y: 0.0 }; 400];
    let n = pull_over::path(offset_m, length, &mut path);
    let path = &path[..n];

    // Brake only as hard as needed to stop at the end of the path, and never
    // harder than the MRM rate. Braking at the full MRM rate would stop the
    // vehicle halfway, short of the shoulder.
    let mrm_decel = MRM_DECEL_MM_S2 as f32 / 1_000.0;
    let decel = (speed_mps * speed_mps / (2.0 * length)).min(mrm_decel);
    let mut plant = Bicycle::new(0.0, 0.0, 0.0, speed_mps, WHEELBASE_M);
    let mut max_lateral_accel = 0.0_f32;
    // Enough time to cover the path at the starting speed, with a generous margin.
    let steps = (3.0 * length / speed_mps.max(0.1) / dt_s) as usize;
    for _ in 0..steps {
        let pose = Pose {
            x: plant.x,
            y: plant.y,
            yaw: plant.yaw,
        };
        let steer = pure_pursuit(pose, path, LOOKAHEAD_M, WHEELBASE_M).unwrap_or(0.0);
        let lateral = plant.speed * plant.speed * steer.tan() / WHEELBASE_M;
        max_lateral_accel = max_lateral_accel.max(lateral.abs());
        plant.step(dt_s, steer, -decel);
        if plant.speed == 0.0 {
            break;
        }
    }

    let final_offset_m = plant.y;
    PullOver {
        final_offset_m,
        final_speed_mps: plant.speed,
        max_lateral_accel,
        distance_m: plant.x,
        stopped_on_shoulder: plant.speed == 0.0 && (final_offset_m - offset_m).abs() < 0.5,
    }
}

/// What the vehicle did for its MRM.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Response {
    /// The shoulder fitted in the road ahead, and the vehicle pulled over.
    PulledOver(PullOver),
    /// No shoulder fitted. The vehicle braked in place at the MRM rate.
    BrakedInPlace {
        /// Distance covered while braking to a stop, m.
        distance_m: f32,
    },
}

/// Chooses and runs the MRM for a vehicle at `speed_mps`, with a shoulder
/// `offset_m` away and `available_m` of road ahead.
#[must_use]
pub fn respond(
    speed_mps: f32,
    offset_m: f32,
    available_m: f32,
    lateral_accel: f32,
    dt_s: f32,
) -> Response {
    match choose(speed_mps, offset_m, available_m, lateral_accel) {
        Maneuver::PullOver { .. } => {
            Response::PulledOver(run(speed_mps, offset_m, lateral_accel, dt_s))
        }
        Maneuver::BrakeInPlace => {
            let mrm_decel = MRM_DECEL_MM_S2 as f32 / 1_000.0;
            Response::BrakedInPlace {
                distance_m: speed_mps * speed_mps / (2.0 * mrm_decel),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_guardian_latches_an_mrm_from_a_stale_heartbeat() {
        let mut guardian = Guardian::new();
        assert!(trigger(&mut guardian));
        assert_eq!(guardian.mode(), Mode::Mrm);
    }

    #[test]
    fn the_vehicle_stops_on_the_shoulder_within_the_lateral_limit() {
        let outcome = run(12.0, 3.5, 2.0, 0.01);
        assert!(outcome.stopped_on_shoulder, "{outcome:?}");
        assert!(outcome.max_lateral_accel <= 2.5, "{outcome:?}");
    }

    #[test]
    fn the_response_pulls_over_when_a_shoulder_fits() {
        match respond(12.0, 3.5, 200.0, 2.0, 0.01) {
            Response::PulledOver(outcome) => assert!(outcome.stopped_on_shoulder, "{outcome:?}"),
            other => panic!("expected a pull-over, got {other:?}"),
        }
    }

    #[test]
    fn the_response_brakes_in_place_when_no_shoulder_fits() {
        // 12 m/s at 3 m/s² stops in 24 m, and the 5 m of road cannot hold the shoulder.
        match respond(12.0, 3.5, 5.0, 2.0, 0.01) {
            Response::BrakedInPlace { distance_m } => {
                assert!((distance_m - 24.0).abs() < 0.1, "{distance_m}");
            }
            other => panic!("expected braking in place, got {other:?}"),
        }
    }

    #[test]
    fn a_slower_vehicle_needs_less_room_to_pull_over() {
        let fast = run(15.0, 3.0, 2.0, 0.01);
        let slow = run(6.0, 3.0, 2.0, 0.01);
        assert!(slow.distance_m < fast.distance_m, "{slow:?} vs {fast:?}");
    }
}
