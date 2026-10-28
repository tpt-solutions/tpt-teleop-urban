//! Pedestrian crossing scenario: the last-mile chain end to end.
//!
//! A vehicle drives along +x at a constant speed. A pedestrian crosses the path
//! at `x = crossing_x_m`, walking in +y. Each cycle:
//!
//! 1. the pedestrian's true position is measured with Gaussian-like noise,
//! 2. the IMM predictor (`tpt-t-urban-predict`) estimates their position,
//! 3. if the estimate is inside the lane, the guardian (`tpt-t-urban-safety`)
//!    gets an obstacle with the gap and closing speed,
//! 4. the planner's command, or the guardian's override, drives the plant.
//!
//! The scenario runs with the guardian on and off, so the difference between the
//! two is visible in the outcome.

use tpt_t_urban_predict::Imm;
use tpt_t_urban_safety::{Guardian, Inputs, Mode, Obstacle};

use crate::plant::Bicycle;

/// Lane half-width used to decide whether the pedestrian is in the vehicle's path, m.
pub const LANE_HALF_WIDTH_M: f32 = 2.0;
/// Distance from the vehicle reference point to its front bumper, m.
pub const FRONT_OFFSET_M: f32 = 2.0;
/// Pedestrian contact distance: closer than this in x and y counts as a hit, m.
pub const CONTACT_M: f32 = 1.0;
/// Whether the estimated pedestrian is in the lane or walking into it. `est` is
/// `[x, y, vx, vy]`. No time limit applies here: the guardian's time-to-collision
/// check decides when to act, so a pedestrian who is far from the lane but
/// heading for it is still a threat.
fn in_or_entering_lane(est: &[f32; 4]) -> bool {
    let (y, vy) = (est[1], est[3]);
    y.abs() < LANE_HALF_WIDTH_M
        || (y < -LANE_HALF_WIDTH_M && vy > 0.0)
        || (y > LANE_HALF_WIDTH_M && vy < 0.0)
}

/// Configuration of one run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Scenario {
    /// Initial vehicle speed, m/s.
    pub speed_mps: f32,
    /// Deceleration the planner applies every cycle, m/s² (0 means the planner
    /// does not brake for the pedestrian).
    pub planner_decel_mps2: f32,
    /// Longitudinal position where the pedestrian crosses, m.
    pub crossing_x_m: f32,
    /// Pedestrian walking speed across the lane, m/s.
    pub walk_speed_mps: f32,
    /// Measurement noise amplitude, m.
    pub noise_m: f32,
    /// Whether the guardian can override the planner.
    pub guardian_enabled: bool,
    /// Simulated seconds.
    pub duration_s: f32,
    /// Simulation time step, in seconds.
    pub dt_s: f32,
}

impl Default for Scenario {
    fn default() -> Self {
        Self {
            speed_mps: 10.0,
            planner_decel_mps2: 0.0,
            crossing_x_m: 40.0,
            walk_speed_mps: 1.4,
            noise_m: 0.1,
            guardian_enabled: true,
            duration_s: 8.0,
            dt_s: 0.05,
        }
    }
}

/// Result of one run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Outcome {
    /// Whether the vehicle hit the pedestrian.
    pub collided: bool,
    /// Whether the guardian entered AEB at any point.
    pub aeb_engaged: bool,
    /// Smallest front-bumper gap to the pedestrian while they were in the lane, m.
    /// Negative means the bumper passed the pedestrian's position.
    pub min_gap_m: f32,
    /// Vehicle speed at the end of the run, in metres per second.
    pub final_speed_mps: f32,
}

/// Deterministic pseudo-random noise in `[-1, 1]`, from an xorshift stream.
struct Noise(u32);

impl Noise {
    fn next(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        (self.0 as f32 / u32::MAX as f32) * 2.0 - 1.0
    }
}

/// One simulated step, for plotting.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TracePoint {
    /// Time since the start of the run, in seconds.
    pub t_s: f32,
    /// Vehicle position along x, in metres.
    pub vehicle_x: f32,
    /// Vehicle speed, in metres per second.
    pub speed_mps: f32,
    /// Pedestrian position along x, in metres.
    pub ped_x: f32,
    /// Pedestrian position along y, in metres.
    pub ped_y: f32,
    /// Whether the guardian was overriding the planner this step.
    pub override_active: bool,
}

/// Runs the crossing scenario and returns the outcome.
#[must_use]
pub fn run_crossing(s: &Scenario) -> Outcome {
    run_crossing_traced(s, |_| {})
}

/// Runs the crossing scenario and calls `on_step` once per simulated step.
#[must_use]
pub fn run_crossing_traced(s: &Scenario, mut on_step: impl FnMut(TracePoint)) -> Outcome {
    let mut vehicle = Bicycle::new(0.0, 0.0, 0.0, s.speed_mps, 2.7);
    // Pedestrian starts on the far side of the lane and walks across.
    let mut ped_y = -6.0f32;
    let ped_x = s.crossing_x_m;
    let mut imm = Imm::new([ped_x, ped_y, 0.0, s.walk_speed_mps], 0.25, 0.25);
    let mut guardian = Guardian::new();
    let mut noise = Noise(0x9E37_79B9);
    let mut out = Outcome {
        collided: false,
        aeb_engaged: false,
        min_gap_m: f32::INFINITY,
        final_speed_mps: 0.0,
    };

    let steps = (s.duration_s / s.dt_s) as usize;
    for step in 0..steps {
        // Pedestrian moves, then is measured with noise.
        ped_y += s.walk_speed_mps * s.dt_s;
        let z = [
            ped_x + noise.next() * s.noise_m,
            ped_y + noise.next() * s.noise_m,
        ];
        imm.step(z, s.dt_s);
        let est = imm.estimate().x;

        // Guardian input: a pedestrian in or entering the lane, and ahead of the
        // bumper. Waiting for lane entry would leave the guardian blind until the
        // vehicle had already stopped.
        let bumper_x = vehicle.x + FRONT_OFFSET_M;
        let obstacle = if in_or_entering_lane(&est) && est[0] > bumper_x {
            Some(Obstacle {
                range_mm: ((est[0] - bumper_x) * 1_000.0) as u32,
                closing_speed_mm_s: (vehicle.speed * 1_000.0) as i32,
            })
        } else {
            None
        };
        let command = if s.guardian_enabled {
            let cmd = guardian.step(&Inputs {
                obstacle,
                planned_decel_mm_s2: (s.planner_decel_mps2 * 1_000.0) as u32,
                planner_heartbeat_age_ms: 10,
                speed_mm_s: (vehicle.speed * 1_000.0) as u32,
            });
            if cmd.mode == Mode::Aeb {
                out.aeb_engaged = true;
            }
            cmd.decel_mm_s2 as f32 / 1_000.0
        } else {
            0.0
        };
        let decel = if command > 0.0 {
            command
        } else {
            s.planner_decel_mps2
        };
        vehicle.step(s.dt_s, 0.0, -decel);

        // Record the true geometry this step.
        on_step(TracePoint {
            t_s: (step + 1) as f32 * s.dt_s,
            vehicle_x: vehicle.x,
            speed_mps: vehicle.speed,
            ped_x,
            ped_y,
            override_active: command > 0.0,
        });
        let front = vehicle.x + FRONT_OFFSET_M;
        if ped_y.abs() < LANE_HALF_WIDTH_M {
            out.min_gap_m = out.min_gap_m.min(ped_x - front);
        }
        if (ped_x - front).abs() < CONTACT_M && ped_y.abs() < CONTACT_M {
            out.collided = true;
        }
    }
    out.final_speed_mps = vehicle.speed;
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn without_a_guardian_the_vehicle_hits_the_pedestrian() {
        let outcome = run_crossing(&Scenario {
            guardian_enabled: false,
            ..Scenario::default()
        });
        assert!(outcome.collided, "{outcome:?}");
        assert!(!outcome.aeb_engaged);
    }

    #[test]
    fn guardian_engages_aeb_and_stops_short_of_the_pedestrian() {
        let outcome = run_crossing(&Scenario::default());
        assert!(outcome.aeb_engaged, "{outcome:?}");
        assert!(!outcome.collided, "{outcome:?}");
        assert!(outcome.min_gap_m > 0.0, "{outcome:?}");
        assert_eq!(outcome.final_speed_mps, 0.0, "{outcome:?}");
    }

    #[test]
    fn a_planner_that_brakes_early_avoids_aeb() {
        let outcome = run_crossing(&Scenario {
            planner_decel_mps2: 6.0,
            ..Scenario::default()
        });
        assert!(!outcome.collided, "{outcome:?}");
    }

    #[test]
    fn no_pedestrian_no_intervention() {
        let outcome = run_crossing(&Scenario {
            crossing_x_m: 500.0,
            ..Scenario::default()
        });
        assert!(!outcome.aeb_engaged && !outcome.collided, "{outcome:?}");
    }
}
