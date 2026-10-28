//! Curb mounting: approach, align, and verify the wheel is against the curb.
//!
//! Ultrasonic sensors at the wheel report the gap to the curb face. The bot
//! creeps in and steers until the gap is inside the mount band, then stops.
//! It is verified only when the gap is inside the band and the bot is at rest.
//! Overshoot or a timeout aborts the mount, and the bot must reverse out before
//! it can try again.

/// Gap at which creeping in and steering starts, mm.
pub const APPROACH_GAP_MM: u32 = 300;
/// Gap at which the wheel counts as against the curb, mm.
pub const MOUNTED_GAP_MM: u32 = 60;
/// Lateral alignment tolerance at the moment of verification, mm.
pub const ALIGN_TOLERANCE_MM: i32 = 30;
/// Cycles allowed in the mounting phase before the attempt is aborted.
pub const MOUNT_TIMEOUT_CYCLES: u32 = 400;

/// Mount state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MountState {
    /// Driving toward the curb, not yet inside the approach band.
    Approach,
    /// Inside the approach band: creeping and steering to align.
    Mounting,
    /// Wheel is against the curb, aligned, and the bot is at rest.
    Verified,
    /// Attempt failed. Reverse out before retrying.
    Aborted,
}

/// What the bot does this cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MountAction {
    /// Drive forward toward the curb.
    Drive,
    /// Creep forward at low speed while aligning.
    Creep,
    /// Hold still.
    Stop,
    /// Back out of the curb.
    Reverse,
}

/// One cycle of sensor readings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reading {
    /// Ultrasonic gap to the curb face at the wheel, mm.
    pub gap_mm: u32,
    /// Lateral alignment of the wheel to the curb edge, mm (signed).
    pub lateral_mm: i32,
    /// Speed, mm/s.
    pub speed_mm_s: u32,
}

/// Curb mount controller carried across cycles.
#[derive(Clone, Copy, Debug)]
pub struct Mount {
    state: MountState,
    cycles_in_mount: u32,
    last_gap_mm: Option<u32>,
}

impl Default for Mount {
    fn default() -> Self {
        Self::new()
    }
}

impl Mount {
    #[must_use]
    /// A mount controller in the approach phase.
    pub const fn new() -> Self {
        Self {
            state: MountState::Approach,
            cycles_in_mount: 0,
            last_gap_mm: None,
        }
    }

    #[must_use]
    /// Current mount state.
    pub const fn state(&self) -> MountState {
        self.state
    }

    /// Advances one cycle and returns the action.
    pub fn step(&mut self, r: Reading) -> MountAction {
        match self.state {
            MountState::Approach => {
                if r.gap_mm <= APPROACH_GAP_MM {
                    self.state = MountState::Mounting;
                    self.cycles_in_mount = 0;
                    self.last_gap_mm = Some(r.gap_mm);
                    return MountAction::Creep;
                }
                MountAction::Drive
            }
            MountState::Mounting => {
                self.cycles_in_mount += 1;
                // Gap grew since the last cycle: the bot is pulling away from the
                // curb, so the attempt is no longer valid.
                let receding = self.last_gap_mm.is_some_and(|g| r.gap_mm > g + 10);
                self.last_gap_mm = Some(r.gap_mm);
                if receding || self.cycles_in_mount > MOUNT_TIMEOUT_CYCLES {
                    self.state = MountState::Aborted;
                    return MountAction::Reverse;
                }
                if r.gap_mm <= MOUNTED_GAP_MM && r.speed_mm_s == 0 {
                    if r.lateral_mm.abs() <= ALIGN_TOLERANCE_MM {
                        self.state = MountState::Verified;
                        return MountAction::Stop;
                    }
                    // Against the curb but misaligned: stop, and the caller reverses and retries.
                    self.state = MountState::Aborted;
                    return MountAction::Reverse;
                }
                if r.gap_mm <= MOUNTED_GAP_MM {
                    return MountAction::Stop;
                }
                MountAction::Creep
            }
            MountState::Verified => MountAction::Stop,
            MountState::Aborted => MountAction::Reverse,
        }
    }

    /// Restarts the attempt after the bot has reversed clear of the curb.
    pub fn retry(&mut self, gap_mm: u32) -> bool {
        if self.state == MountState::Aborted && gap_mm > APPROACH_GAP_MM {
            *self = Self::new();
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(gap_mm: u32, lateral_mm: i32, speed_mm_s: u32) -> Reading {
        Reading {
            gap_mm,
            lateral_mm,
            speed_mm_s,
        }
    }

    #[test]
    fn successful_mount_is_verified() {
        let mut m = Mount::new();
        assert_eq!(m.step(r(1_000, 0, 800)), MountAction::Drive);
        assert_eq!(m.step(r(290, 10, 300)), MountAction::Creep);
        assert_eq!(m.step(r(120, 5, 200)), MountAction::Creep);
        assert_eq!(m.step(r(50, 5, 0)), MountAction::Stop);
        assert_eq!(m.state(), MountState::Verified);
    }

    #[test]
    fn misaligned_contact_aborts_and_reverses() {
        let mut m = Mount::new();
        m.step(r(290, 0, 300));
        assert_eq!(m.step(r(50, 80, 0)), MountAction::Reverse);
        assert_eq!(m.state(), MountState::Aborted);
    }

    #[test]
    fn receding_gap_aborts_the_attempt() {
        let mut m = Mount::new();
        m.step(r(200, 0, 300));
        assert_eq!(m.step(r(260, 0, 300)), MountAction::Reverse);
        assert_eq!(m.state(), MountState::Aborted);
    }

    #[test]
    fn timeout_aborts_the_attempt() {
        let mut m = Mount::new();
        m.step(r(200, 0, 300));
        for _ in 0..MOUNT_TIMEOUT_CYCLES {
            m.step(r(200, 0, 300));
        }
        assert_eq!(m.step(r(200, 0, 300)), MountAction::Reverse);
    }

    #[test]
    fn retry_needs_the_bot_to_be_clear_of_the_curb() {
        let mut m = Mount::new();
        m.step(r(200, 0, 300));
        m.step(r(260, 0, 300));
        assert!(!m.retry(100));
        assert!(m.retry(APPROACH_GAP_MM + 1));
        assert_eq!(m.state(), MountState::Approach);
    }
}
