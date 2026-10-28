//! Secure package compartment: lock state and code verification.
//!
//! The compartment unlocks only when all of these hold:
//!
//! 1. the bot is verified against the curb (see `mount`),
//! 2. the bot is at rest,
//! 3. the recipient's code matches the hash stored at dispatch.
//!
//! The code is compared as a SHA-256 hash in constant time, so the stored value
//! reveals nothing useful and the comparison leaks no timing. The compartment
//! relocks on its own after a timeout, or when the door closes.

use ring::digest::{SHA256, digest};

/// Seconds the compartment stays open before it relocks on its own.
pub const OPEN_TIMEOUT_S: u32 = 120;

/// SHA-256 of the recipient code, stored at dispatch.
pub type CodeHash = [u8; 32];

/// Hashes a recipient code for storage or comparison.
#[must_use]
pub fn hash_code(code: &[u8]) -> CodeHash {
    let mut out = [0u8; 32];
    out.copy_from_slice(digest(&SHA256, code).as_ref());
    out
}

/// Compares two equal-length byte strings without an early exit on the first
/// difference. Every byte is always visited, so the time taken does not depend
/// on where the strings differ.
#[must_use]
fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let diff = a.iter().zip(b).fold(0u8, |acc, (x, y)| acc | (x ^ y));
    core::hint::black_box(diff) == 0
}

/// Compartment state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lock {
    /// Closed and latched.
    Locked,
    /// Open, and re-locks once the countdown ends.
    Open {
        /// Seconds left before the compartment re-locks.
        seconds_left: u32,
    },
}

/// Why an unlock request was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// The bot has not verified its wheel against the curb.
    NotVerifiedAtCurb,
    /// The bot is still moving.
    NotAtRest,
    /// The code does not match the one the compartment was set for.
    WrongCode,
}

/// Compartment controller.
#[derive(Clone, Copy, Debug)]
pub struct Compartment {
    lock: Lock,
    expected: CodeHash,
}

impl Compartment {
    /// A locked compartment that opens only for `expected`.
    #[must_use]
    pub const fn new(expected: CodeHash) -> Self {
        Self {
            lock: Lock::Locked,
            expected,
        }
    }

    #[must_use]
    /// Current lock state.
    pub const fn lock(&self) -> Lock {
        self.lock
    }

    /// Tries to unlock. Every condition is checked before the code, so the code
    /// is compared only for a bot that is already at the curb and at rest.
    pub fn request_unlock(
        &mut self,
        verified_at_curb: bool,
        at_rest: bool,
        code: &[u8],
    ) -> Result<(), Refusal> {
        if !verified_at_curb {
            return Err(Refusal::NotVerifiedAtCurb);
        }
        if !at_rest {
            return Err(Refusal::NotAtRest);
        }
        let presented = hash_code(code);
        if !constant_time_eq(&presented, &self.expected) {
            return Err(Refusal::WrongCode);
        }
        self.lock = Lock::Open {
            seconds_left: OPEN_TIMEOUT_S,
        };
        Ok(())
    }

    /// Advances the open timer by `dt_s` seconds. The door closing relocks it.
    pub fn tick(&mut self, dt_s: u32, door_closed: bool) {
        self.lock = match self.lock {
            Lock::Open { seconds_left } if !door_closed => {
                let left = seconds_left.saturating_sub(dt_s);
                if left == 0 {
                    Lock::Locked
                } else {
                    Lock::Open { seconds_left: left }
                }
            }
            _ => Lock::Locked,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn comp() -> Compartment {
        Compartment::new(hash_code(b"4821"))
    }

    #[test]
    fn correct_code_opens_when_verified_and_at_rest() {
        let mut c = comp();
        assert_eq!(c.request_unlock(true, true, b"4821"), Ok(()));
        assert_eq!(
            c.lock(),
            Lock::Open {
                seconds_left: OPEN_TIMEOUT_S
            }
        );
    }

    #[test]
    fn wrong_code_is_refused() {
        let mut c = comp();
        assert_eq!(
            c.request_unlock(true, true, b"0000"),
            Err(Refusal::WrongCode)
        );
        assert_eq!(c.lock(), Lock::Locked);
    }

    #[test]
    fn unverified_or_moving_bot_is_refused_before_the_code_is_checked() {
        let mut c = comp();
        assert_eq!(
            c.request_unlock(false, true, b"4821"),
            Err(Refusal::NotVerifiedAtCurb)
        );
        assert_eq!(
            c.request_unlock(true, false, b"4821"),
            Err(Refusal::NotAtRest)
        );
        assert_eq!(c.lock(), Lock::Locked);
    }

    #[test]
    fn open_compartment_relocks_after_timeout() {
        let mut c = comp();
        c.request_unlock(true, true, b"4821").unwrap();
        c.tick(OPEN_TIMEOUT_S - 1, false);
        assert!(matches!(c.lock(), Lock::Open { .. }));
        c.tick(1, false);
        assert_eq!(c.lock(), Lock::Locked);
    }

    #[test]
    fn closing_the_door_relocks_at_once() {
        let mut c = comp();
        c.request_unlock(true, true, b"4821").unwrap();
        c.tick(1, true);
        assert_eq!(c.lock(), Lock::Locked);
    }
}
