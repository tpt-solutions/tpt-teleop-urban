//! Dual-channel steer-by-wire failover (spec §5.3).
//!
//! Two independent steering channels each report a heartbeat every cycle. The
//! primary is used while it is healthy. After `MISS_LIMIT` consecutive missed
//! heartbeats it is faulted, and control moves to the secondary. The switch is
//! latched: the primary must be re-qualified by a run of good heartbeats before
//! it is used again. This avoids flapping between channels.

/// Consecutive missed heartbeats that fault a channel.
pub const MISS_LIMIT: u32 = 3;
/// Consecutive good heartbeats needed to re-qualify a faulted primary.
pub const REQUALIFY_RUN: u32 = 20;

/// Which channel is driving the steering.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    Primary,
    Secondary,
}

/// Both channels are down: there is no valid steering source.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NoChannel;

/// Failover selector.
#[derive(Clone, Copy, Debug)]
pub struct Failover {
    active: Channel,
    primary_misses: u32,
    primary_good_run: u32,
    secondary_misses: u32,
    primary_faulted: bool,
}

impl Default for Failover {
    fn default() -> Self {
        Self::new()
    }
}

impl Failover {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            active: Channel::Primary,
            primary_misses: 0,
            primary_good_run: 0,
            secondary_misses: 0,
            primary_faulted: false,
        }
    }

    /// Records one cycle's heartbeats and returns the channel to use, or
    /// `NoChannel` if both are down.
    pub fn step(&mut self, primary_ok: bool, secondary_ok: bool) -> Result<Channel, NoChannel> {
        self.update_primary(primary_ok);
        self.secondary_misses = if secondary_ok {
            0
        } else {
            self.secondary_misses.saturating_add(1)
        };
        let secondary_healthy = self.secondary_misses < MISS_LIMIT;

        self.active = match (self.primary_faulted, secondary_healthy) {
            (false, _) => Channel::Primary,
            (true, true) => Channel::Secondary,
            (true, false) => return Err(NoChannel),
        };
        Ok(self.active)
    }

    fn update_primary(&mut self, ok: bool) {
        if ok {
            self.primary_misses = 0;
            if self.primary_faulted {
                self.primary_good_run += 1;
                if self.primary_good_run >= REQUALIFY_RUN {
                    self.primary_faulted = false;
                    self.primary_good_run = 0;
                }
            }
        } else {
            self.primary_good_run = 0;
            self.primary_misses = self.primary_misses.saturating_add(1);
            if self.primary_misses >= MISS_LIMIT {
                self.primary_faulted = true;
            }
        }
    }

    #[must_use]
    pub const fn active(&self) -> Channel {
        self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn primary_is_used_while_healthy() {
        let mut f = Failover::new();
        for _ in 0..100 {
            assert_eq!(f.step(true, true), Ok(Channel::Primary));
        }
    }

    #[test]
    fn primary_faults_after_consecutive_misses_and_switches() {
        let mut f = Failover::new();
        let _ = f.step(false, true);
        let _ = f.step(false, true);
        assert_eq!(f.step(false, true), Ok(Channel::Secondary));
    }

    #[test]
    fn a_single_miss_does_not_switch() {
        let mut f = Failover::new();
        let _ = f.step(false, true);
        assert_eq!(f.step(true, true), Ok(Channel::Primary));
    }

    #[test]
    fn primary_is_requalified_only_after_a_good_run() {
        let mut f = Failover::new();
        for _ in 0..MISS_LIMIT {
            let _ = f.step(false, true);
        }
        for _ in 0..REQUALIFY_RUN - 1 {
            assert_eq!(f.step(true, true), Ok(Channel::Secondary));
        }
        assert_eq!(f.step(true, true), Ok(Channel::Primary));
    }

    #[test]
    fn both_channels_down_is_reported() {
        let mut f = Failover::new();
        for _ in 0..MISS_LIMIT {
            let _ = f.step(false, false);
        }
        assert_eq!(f.step(false, false), Err(NoChannel));
    }

    #[test]
    fn secondary_takes_over_even_when_primary_is_down_and_secondary_recovers_later() {
        let mut f = Failover::new();
        for _ in 0..MISS_LIMIT {
            let _ = f.step(false, true);
        }
        // Secondary goes down too: no channel is usable.
        for _ in 0..MISS_LIMIT {
            let _ = f.step(false, false);
        }
        assert_eq!(f.step(false, false), Err(NoChannel));
        // Secondary comes back and is taken up.
        assert_eq!(f.step(false, true), Ok(Channel::Secondary));
    }
}
