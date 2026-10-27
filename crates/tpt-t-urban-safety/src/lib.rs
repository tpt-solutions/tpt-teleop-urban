//! Independent guardian state machine for AEB, MRM, and actuation failover.
//!
//! The guardian watches the primary planner's output and the nearest in-path
//! obstacle. It overrides the planner in two cases:
//!
//! - **AEB** (automatic emergency braking): time-to-collision is below the
//!   threshold and the planner is not braking hard enough to stop in time.
//!   AEB latches until the closing speed is gone.
//! - **MRM** (minimal risk maneuver): the planner heartbeat has gone stale.
//!   MRM latches until an explicit reset, once the vehicle is at rest.
//!
//! All quantities are integers (millimetres, milliseconds, mm/s, mm/s²) so the
//! decision is bit-exact and reproducible. The guardian is a pure function of
//! its inputs and holds no clock of its own; the caller supplies the heartbeat
//! age.
//!
//! The spec requires the guardian to run on a separate compute partition
//! (spec §5.3). That isolation is a deployment property, not something this
//! crate can provide in software. This crate is the logic that would run there.
//!
//! Implements Phase 7 of `todo.md`. The <50 ms reaction budget and FMEA
//! (Phase 17) need hardware measurements and are not covered here.

/// Time-to-collision below which AEB is considered.
pub const AEB_TTC_MS: u32 = 1_500;
/// Planner heartbeat age that counts as stale.
pub const PLANNER_STALE_MS: u32 = 50;
/// Full service-brake deceleration: 7 m/s² in mm/s².
pub const AEB_DECEL_MM_S2: u32 = 7_000;
/// Minimal-risk-maneuver deceleration: 3 m/s² in mm/s².
pub const MRM_DECEL_MM_S2: u32 = 3_000;

/// Nearest obstacle in the vehicle's path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Obstacle {
    /// Distance to the obstacle in millimetres.
    pub range_mm: u32,
    /// Rate at which the gap is closing, in mm/s. Zero or negative means not closing.
    pub closing_speed_mm_s: i32,
}

/// What the guardian sees each cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Inputs {
    /// Nearest in-path obstacle, if any.
    pub obstacle: Option<Obstacle>,
    /// Deceleration the planner is currently commanding, in mm/s².
    pub planned_decel_mm_s2: u32,
    /// Age of the planner's last heartbeat in milliseconds.
    pub planner_heartbeat_age_ms: u32,
    /// Current vehicle speed in mm/s.
    pub speed_mm_s: u32,
}

/// The guardian's current mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Planner in control. The guardian only watches.
    Nominal,
    /// Automatic emergency braking is engaged.
    Aeb,
    /// Minimal risk maneuver is engaged.
    Mrm,
}

/// Override the guardian applies to the planner this cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Command {
    pub mode: Mode,
    /// Deceleration to apply in mm/s². Zero in `Nominal`, where the planner's own
    /// command stands.
    pub decel_mm_s2: u32,
}

/// Guardian state carried between cycles.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Guardian {
    mode: Mode,
}

impl Default for Guardian {
    fn default() -> Self {
        Self::new()
    }
}

impl Guardian {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            mode: Mode::Nominal,
        }
    }

    #[must_use]
    pub const fn mode(&self) -> Mode {
        self.mode
    }

    /// Advances one cycle and returns the override to apply.
    pub fn step(&mut self, inputs: &Inputs) -> Command {
        self.mode = match self.mode {
            // MRM latches. Only `reset` leaves it.
            Mode::Mrm => Mode::Mrm,
            _ if inputs.planner_heartbeat_age_ms > PLANNER_STALE_MS => Mode::Mrm,
            Mode::Aeb if closing(inputs) => Mode::Aeb,
            Mode::Aeb => Mode::Nominal,
            Mode::Nominal if aeb_required(inputs) => Mode::Aeb,
            Mode::Nominal => Mode::Nominal,
        };
        let decel_mm_s2 = match self.mode {
            Mode::Nominal => 0,
            Mode::Aeb => AEB_DECEL_MM_S2,
            Mode::Mrm => MRM_DECEL_MM_S2,
        };
        Command {
            mode: self.mode,
            decel_mm_s2,
        }
    }

    /// Clears a latched MRM once the vehicle is at rest. Returns whether it cleared.
    pub fn reset(&mut self, speed_mm_s: u32) -> bool {
        if self.mode == Mode::Mrm && speed_mm_s == 0 {
            self.mode = Mode::Nominal;
            true
        } else {
            false
        }
    }
}

/// Whether the obstacle is closing on the vehicle.
fn closing(inputs: &Inputs) -> bool {
    inputs.obstacle.is_some_and(|o| o.closing_speed_mm_s > 0)
}

/// AEB condition: the obstacle is closing, the time to collision is below the
/// threshold, and the planner's braking is short of what is needed to stop at
/// the obstacle.
fn aeb_required(inputs: &Inputs) -> bool {
    let Some(obstacle) = inputs.obstacle else {
        return false;
    };
    if obstacle.closing_speed_mm_s <= 0 {
        return false;
    }
    if obstacle.range_mm == 0 {
        // Already in contact while closing: no time left to plan around.
        return true;
    }
    let closing_mm_s = obstacle.closing_speed_mm_s as u64;
    let range_mm = u64::from(obstacle.range_mm);
    let ttc_ms = range_mm * 1_000 / closing_mm_s;
    if ttc_ms >= u64::from(AEB_TTC_MS) {
        return false;
    }
    // Deceleration that stops the closing speed within the gap: v² / (2 d).
    let required_decel = closing_mm_s * closing_mm_s / (2 * range_mm);
    u64::from(inputs.planned_decel_mm_s2) < required_decel
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inputs(obstacle: Option<Obstacle>, planned: u32, heartbeat: u32) -> Inputs {
        Inputs {
            obstacle,
            planned_decel_mm_s2: planned,
            planner_heartbeat_age_ms: heartbeat,
            speed_mm_s: 10_000,
        }
    }

    /// Obstacle 20 m ahead, closing at 10 m/s: TTC 2.0 s, above the threshold.
    fn far() -> Obstacle {
        Obstacle {
            range_mm: 20_000,
            closing_speed_mm_s: 10_000,
        }
    }

    /// Obstacle 8 m ahead, closing at 10 m/s: TTC 0.8 s.
    fn near() -> Obstacle {
        Obstacle {
            range_mm: 8_000,
            closing_speed_mm_s: 10_000,
        }
    }

    #[test]
    fn nominal_with_no_obstacle() {
        let mut g = Guardian::new();
        let cmd = g.step(&inputs(None, 0, 10));
        assert_eq!(
            cmd,
            Command {
                mode: Mode::Nominal,
                decel_mm_s2: 0
            }
        );
    }

    #[test]
    fn distant_closing_obstacle_does_not_trigger_aeb() {
        let mut g = Guardian::new();
        assert_eq!(g.step(&inputs(Some(far()), 0, 10)).mode, Mode::Nominal);
    }

    #[test]
    fn short_ttc_with_weak_planner_braking_triggers_aeb() {
        let mut g = Guardian::new();
        // Needs 10_000² / (2 × 8_000) = 6_250 mm/s²; the planner gives 2_000.
        let cmd = g.step(&inputs(Some(near()), 2_000, 10));
        assert_eq!(
            cmd,
            Command {
                mode: Mode::Aeb,
                decel_mm_s2: AEB_DECEL_MM_S2
            }
        );
    }

    #[test]
    fn planner_braking_that_is_enough_suppresses_aeb() {
        let mut g = Guardian::new();
        let cmd = g.step(&inputs(Some(near()), 7_000, 10));
        assert_eq!(cmd.mode, Mode::Nominal);
    }

    #[test]
    fn aeb_latches_while_closing_and_releases_after() {
        let mut g = Guardian::new();
        g.step(&inputs(Some(near()), 0, 10));
        assert_eq!(g.mode(), Mode::Aeb);
        // Planner now brakes hard, but AEB holds while the gap is still closing.
        assert_eq!(g.step(&inputs(Some(near()), 9_000, 10)).mode, Mode::Aeb);
        let opened = Obstacle {
            range_mm: 8_000,
            closing_speed_mm_s: -500,
        };
        assert_eq!(g.step(&inputs(Some(opened), 0, 10)).mode, Mode::Nominal);
    }

    #[test]
    fn stale_planner_triggers_latched_mrm() {
        let mut g = Guardian::new();
        let cmd = g.step(&inputs(None, 0, PLANNER_STALE_MS + 1));
        assert_eq!(
            cmd,
            Command {
                mode: Mode::Mrm,
                decel_mm_s2: MRM_DECEL_MM_S2
            }
        );
        // A fresh heartbeat does not clear MRM.
        assert_eq!(g.step(&inputs(None, 0, 0)).mode, Mode::Mrm);
    }

    #[test]
    fn mrm_reset_requires_the_vehicle_to_be_at_rest() {
        let mut g = Guardian::new();
        g.step(&inputs(None, 0, PLANNER_STALE_MS + 1));
        assert!(!g.reset(500));
        assert_eq!(g.mode(), Mode::Mrm);
        assert!(g.reset(0));
        assert_eq!(g.mode(), Mode::Nominal);
    }

    #[test]
    fn obstacle_at_zero_range_while_closing_triggers_aeb() {
        let mut g = Guardian::new();
        let touching = Obstacle {
            range_mm: 0,
            closing_speed_mm_s: 1_000,
        };
        assert_eq!(g.step(&inputs(Some(touching), 0, 10)).mode, Mode::Aeb);
    }
}
