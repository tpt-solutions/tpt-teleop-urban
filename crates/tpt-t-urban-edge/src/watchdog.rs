//! Watchdog: the control loop must feed it within a timeout, or the actuators go
//! to a safe state.
//!
//! The hardware watchdog timer resets the MCU if it is not fed. This type is the
//! software side: it tracks the time since the last feed, reports expiry, and
//! latches the fault until the loop has been healthy again. The caller decides
//! what the safe state is (brake and steer straight).

/// Result of a watchdog check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Fed within the timeout.
    Healthy,
    /// Timed out. The fault stays latched until `feed` is called.
    Expired,
}

/// Software watchdog with a latched expiry.
#[derive(Clone, Copy, Debug)]
pub struct Watchdog {
    timeout_ms: u32,
    since_feed_ms: u32,
    expired: bool,
}

impl Watchdog {
    /// A watchdog that expires `timeout_ms` after the last feed.
    #[must_use]
    pub const fn new(timeout_ms: u32) -> Self {
        Self {
            timeout_ms,
            since_feed_ms: 0,
            expired: false,
        }
    }

    /// Advances time by `dt_ms` and reports the state.
    pub fn tick(&mut self, dt_ms: u32) -> State {
        if !self.expired {
            self.since_feed_ms = self.since_feed_ms.saturating_add(dt_ms);
            if self.since_feed_ms > self.timeout_ms {
                self.expired = true;
            }
        }
        self.state()
    }

    /// Records a healthy loop iteration. Clears a latched fault.
    pub fn feed(&mut self) {
        self.since_feed_ms = 0;
        self.expired = false;
    }

    #[must_use]
    /// Current state. `Expired` stays latched until `feed` is called.
    pub const fn state(&self) -> State {
        if self.expired {
            State::Expired
        } else {
            State::Healthy
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_while_fed() {
        let mut w = Watchdog::new(50);
        for _ in 0..100 {
            assert_eq!(w.tick(10), State::Healthy);
            w.feed();
        }
    }

    #[test]
    fn expires_after_the_timeout_and_latches() {
        let mut w = Watchdog::new(50);
        for _ in 0..5 {
            w.tick(10);
        }
        assert_eq!(w.tick(10), State::Expired);
        // Still expired on later ticks, with no feed.
        assert_eq!(w.tick(1), State::Expired);
    }

    #[test]
    fn feed_clears_the_latch() {
        let mut w = Watchdog::new(10);
        w.tick(20);
        w.feed();
        assert_eq!(w.state(), State::Healthy);
    }
}
