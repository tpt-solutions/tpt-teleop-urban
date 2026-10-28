//! Closed-loop route driving: the planner follows a straight route at the speed
//! profile, and the guardian watches its heartbeat.
//!
//! The planner runs each step. It steers with pure pursuit toward the route,
//! and accelerates toward the profile speed at the nearest route point. It
//! publishes a heartbeat every step. If the planner stalls, the heartbeat goes
//! stale. The guardian then latches an MRM, and the sim responds with
//! [`crate::mrm::respond`]: a pull-over to the shoulder if the road ahead allows
//! it, or a brake in place.
//!
//! The route is straight along +x, from the origin, and the planner's lane is
//! the x axis. The pull-over shoulder is `SHOULDER_M` to the left.

use tpt_t_urban_car::profile::{Limits, Point, speed_profile};
use tpt_t_urban_car::{Pose, pure_pursuit};
use tpt_t_urban_safety::{Guardian, Inputs, Mode};

use crate::mrm::{LOOKAHEAD_M, Response, WHEELBASE_M, respond};
use crate::plant::Bicycle;

/// Shoulder offset from the lane, to the left, in metres.
pub const SHOULDER_M: f32 = 3.0;
/// Route point spacing, in metres.
const SPACING_M: f32 = 0.5;
/// Planner heartbeat age while it runs, in milliseconds.
const HEALTHY_HEARTBEAT_MS: u32 = 10;

/// Result of one drive.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Drive {
    /// Whether the planner reached the end of the route.
    pub completed: bool,
    /// Highest speed reached while the planner was in control, in m/s.
    pub max_speed_mps: f32,
    /// Largest lateral offset from the lane while the planner was in control, in metres.
    pub max_lateral_m: f32,
    /// Distance travelled when the planner stalled, in metres, or the route
    /// length if it did not stall.
    pub distance_at_stop_m: f32,
    /// The guardian's response, if the planner stalled.
    pub response: Option<Response>,
}

/// Drives a straight route of `route_m` metres under a `speed_limit_mps` limit.
///
/// The planner stalls at `stall_at_s` seconds if one is given, and stops
/// publishing its heartbeat from then on.
#[must_use]
pub fn run_route(route_m: f32, speed_limit_mps: f32, stall_at_s: Option<f32>, dt_s: f32) -> Drive {
    let n = (route_m / SPACING_M).ceil() as usize + 1;
    let route: Vec<Point> = (0..n)
        .map(|i| Point {
            x: (i as f32 * SPACING_M).min(route_m),
            y: 0.0,
        })
        .collect();
    let limits = Limits {
        speed_limit: speed_limit_mps,
        lateral_accel: 2.0,
        accel: 1.5,
        brake: 3.0,
    };
    let mut profile = vec![0.0_f32; n];
    speed_profile(&route, &limits, &mut profile);

    let mut plant = Bicycle::new(0.0, 0.0, 0.0, 0.0, WHEELBASE_M);
    let mut guardian = Guardian::new();
    let mut drive = Drive {
        completed: false,
        max_speed_mps: 0.0,
        max_lateral_m: 0.0,
        distance_at_stop_m: route_m,
        response: None,
    };
    let steps = (4.0 * route_m / speed_limit_mps.max(0.5) / dt_s) as usize;
    for step in 0..steps {
        let t = step as f32 * dt_s;
        let stalled = stall_at_s.is_some_and(|s| t >= s);
        if stalled {
            // The guardian sees the stale heartbeat and latches the MRM.
            let heartbeat_age_ms =
                ((t - stall_at_s.unwrap_or(t)) * 1_000.0) as u32 + HEALTHY_HEARTBEAT_MS;
            let command = guardian.step(&Inputs {
                obstacle: None,
                planned_decel_mm_s2: 0,
                planner_heartbeat_age_ms: heartbeat_age_ms,
                speed_mm_s: (plant.speed * 1_000.0) as u32,
            });
            if command.mode == Mode::Mrm {
                drive.distance_at_stop_m = plant.x;
                let available = (route_m - plant.x).max(0.0);
                drive.response = Some(respond(plant.speed, SHOULDER_M, available, 2.0, dt_s));
                return drive;
            }
            continue;
        }

        // The planner: the nearest route point sets the target speed.
        let nearest = (plant.x / SPACING_M).round().clamp(0.0, (n - 1) as f32) as usize;
        let target = profile[nearest];
        let pose = Pose {
            x: plant.x,
            y: plant.y,
            yaw: plant.yaw,
        };
        let steer = pure_pursuit(pose, &route, LOOKAHEAD_M, WHEELBASE_M).unwrap_or(0.0);
        let accel = ((target - plant.speed) / 0.5).clamp(-limits.brake, limits.accel);
        plant.step(dt_s, steer, accel);

        drive.max_speed_mps = drive.max_speed_mps.max(plant.speed);
        drive.max_lateral_m = drive.max_lateral_m.max(plant.y.abs());
        drive.distance_at_stop_m = plant.x;
        if plant.x >= route_m - 0.5 {
            drive.completed = true;
            return drive;
        }
    }
    drive
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_planner_completes_a_route_within_the_limit_and_stays_in_lane() {
        let drive = run_route(120.0, 13.9, None, 0.01);
        assert!(drive.completed, "{drive:?}");
        assert!(drive.max_speed_mps <= 13.9 + 0.2, "{drive:?}");
        assert!(drive.max_lateral_m < 0.05, "{drive:?}");
        assert!(drive.response.is_none());
    }

    #[test]
    fn a_stalled_planner_hands_over_to_the_guardian_mrm() {
        let drive = run_route(200.0, 13.9, Some(6.0), 0.01);
        assert!(!drive.completed, "{drive:?}");
        assert!(
            drive.distance_at_stop_m > 10.0,
            "the planner drove first: {drive:?}"
        );
        match drive.response {
            Some(Response::PulledOver(outcome)) => {
                assert!(outcome.stopped_on_shoulder, "{outcome:?}");
            }
            other => panic!("expected a pull-over, got {other:?}"),
        }
    }
}
