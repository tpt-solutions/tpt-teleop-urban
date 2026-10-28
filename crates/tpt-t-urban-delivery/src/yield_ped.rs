//! Pedestrian right-of-way: yield, signal politely, and hold until clear.
//!
//! A pedestrian is a conflict when their predicted path crosses the bot's path
//! within the conflict horizon. On conflict the bot stops and gives an audible
//! and visual signal. It resumes only after the conflict has been clear for a
//! full hold period, so it does not start and stop repeatedly as someone walks
//! past.

/// A conflict is any predicted crossing within this time, ms.
pub const CONFLICT_HORIZON_MS: u32 = 3_000;
/// Lateral distance from the bot's path that still counts as a conflict, mm.
pub const CONFLICT_LATERAL_MM: u32 = 1_500;
/// Time the path must stay clear before the bot resumes, ms.
pub const HOLD_CLEAR_MS: u32 = 2_000;

/// Prediction for one pedestrian, from the trajectory cone.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pedestrian {
    /// Time until the predicted position crosses the bot's path, ms.
    pub time_to_path_ms: u32,
    /// Lateral distance of the predicted position from the path at that
    /// moment, mm.
    pub lateral_mm: u32,
}

/// Decision for this cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    /// Path is clear and the bot may move.
    Proceed,
    /// Stop and signal. The reason is recorded in the ledger.
    Yield,
    /// Clear for less than the hold period. Keep stopped.
    Hold,
}

/// What the bot tells the pedestrians this cycle.
///
/// The signal is polite: the bot chimes once when it starts to yield, and then
/// only shows the light while it yields. It never chimes again until it has
/// proceeded and yields again.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Signal {
    /// Visual indicator on: the bot is yielding or holding.
    pub visual: bool,
    /// Audible chime: the bot started to yield this cycle.
    pub audible: bool,
}

/// Yield controller with hold-off.
#[derive(Clone, Copy, Debug, Default)]
pub struct Yielder {
    yielding: bool,
    clear_ms: u32,
    onset: bool,
}

impl Yielder {
    #[must_use]
    /// A yielder that is not yielding.
    pub const fn new() -> Self {
        Self {
            yielding: false,
            clear_ms: 0,
            onset: false,
        }
    }

    /// The signal for the last cycle `step` ran.
    #[must_use]
    pub const fn signal(&self) -> Signal {
        Signal {
            visual: self.yielding,
            audible: self.onset,
        }
    }

    /// Whether the bot is currently yielding.
    #[must_use]
    pub const fn is_yielding(&self) -> bool {
        self.yielding
    }

    /// Advances by `dt_ms` with the current pedestrians, and returns the decision.
    pub fn step(&mut self, pedestrians: &[Pedestrian], dt_ms: u32) -> Decision {
        self.onset = false;
        let conflict = pedestrians.iter().any(|p| {
            p.time_to_path_ms <= CONFLICT_HORIZON_MS && p.lateral_mm <= CONFLICT_LATERAL_MM
        });
        if conflict {
            self.onset = !self.yielding;
            self.yielding = true;
            self.clear_ms = 0;
            return Decision::Yield;
        }
        if !self.yielding {
            return Decision::Proceed;
        }
        self.clear_ms = self.clear_ms.saturating_add(dt_ms);
        if self.clear_ms >= HOLD_CLEAR_MS {
            self.yielding = false;
            self.clear_ms = 0;
            Decision::Proceed
        } else {
            Decision::Hold
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WALKER: Pedestrian = Pedestrian {
        time_to_path_ms: 1_000,
        lateral_mm: 500,
    };
    const CLEAR: Pedestrian = Pedestrian {
        time_to_path_ms: 10_000,
        lateral_mm: 500,
    };

    #[test]
    fn proceeds_when_nobody_is_near_the_path() {
        let mut y = Yielder::new();
        assert_eq!(y.step(&[CLEAR], 100), Decision::Proceed);
        assert!(!y.is_yielding());
    }

    #[test]
    fn yields_to_a_pedestrian_crossing_the_path() {
        let mut y = Yielder::new();
        assert_eq!(y.step(&[WALKER], 100), Decision::Yield);
        assert!(y.is_yielding());
    }

    #[test]
    fn holds_until_the_path_has_been_clear_long_enough() {
        let mut y = Yielder::new();
        y.step(&[WALKER], 100);
        assert_eq!(y.step(&[], 1_000), Decision::Hold);
        assert_eq!(y.step(&[], 1_000), Decision::Proceed);
    }

    #[test]
    fn a_returning_pedestrian_resets_the_hold() {
        let mut y = Yielder::new();
        y.step(&[WALKER], 100);
        y.step(&[], 1_500);
        assert_eq!(y.step(&[WALKER], 100), Decision::Yield);
        assert_eq!(y.step(&[], 1_500), Decision::Hold);
    }

    #[test]
    fn distant_lateral_offset_is_not_a_conflict() {
        let far = Pedestrian {
            time_to_path_ms: 1_000,
            lateral_mm: CONFLICT_LATERAL_MM + 1,
        };
        let mut y = Yielder::new();
        assert_eq!(y.step(&[far], 100), Decision::Proceed);
    }

    #[test]
    fn chimes_once_at_onset_and_shows_the_light_while_yielding() {
        let mut y = Yielder::new();
        y.step(&[WALKER], 100);
        assert_eq!(
            y.signal(),
            Signal {
                visual: true,
                audible: true
            }
        );
        y.step(&[WALKER], 100);
        assert_eq!(
            y.signal(),
            Signal {
                visual: true,
                audible: false
            }
        );
    }

    #[test]
    fn the_light_goes_off_when_it_proceeds_and_a_new_conflict_chimes_again() {
        let mut y = Yielder::new();
        y.step(&[WALKER], 100);
        y.step(&[], 1_000);
        y.step(&[], 1_000);
        assert_eq!(
            y.signal(),
            Signal {
                visual: false,
                audible: false
            }
        );
        y.step(&[WALKER], 100);
        assert!(y.signal().audible);
    }
}
