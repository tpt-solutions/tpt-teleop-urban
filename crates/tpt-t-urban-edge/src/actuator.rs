//! Actuation command limiter: clamps and rate-limits what reaches the hardware.
//!
//! Whatever the planner or the teleop link asks for, the limiter ensures the
//! value sent to the steering, brake and throttle actuators is in range and
//! changes no faster than the actuator can safely follow. Units are integers:
//! steering in centi-degrees, pressure and throttle in permille (0 to 1000).

/// Largest steering angle magnitude, centi-degrees (±35.00°).
pub const MAX_STEER_CDEG: i32 = 3_500;
/// Largest steering change per control tick, centi-degrees (5.00° per tick).
pub const MAX_STEER_RATE_CDEG: i32 = 500;
/// Largest brake or throttle value, permille.
pub const MAX_PERMILLE: u16 = 1_000;
/// Largest brake or throttle change per control tick, permille.
pub const MAX_PERMILLE_RATE: u16 = 250;

/// Moves `current` toward `target` by at most `rate`, never past `target`.
#[must_use]
pub const fn slew_i32(current: i32, target: i32, rate: i32) -> i32 {
    let delta = target - current;
    if delta > rate {
        current + rate
    } else if delta < -rate {
        current - rate
    } else {
        target
    }
}

/// Moves `current` toward `target` by at most `rate`, never past `target`.
#[must_use]
pub const fn slew_u16(current: u16, target: u16, rate: u16) -> u16 {
    if target > current {
        let step = target - current;
        if step > rate { current + rate } else { target }
    } else {
        let step = current - target;
        if step > rate { current - rate } else { target }
    }
}

/// Final command for one control tick.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Actuation {
    /// Steering angle, signed, in centidegrees.
    pub steer_cdeg: i32,
    /// Brake command, in parts per thousand.
    pub brake_permille: u16,
    /// Throttle command, in parts per thousand.
    pub throttle_permille: u16,
}

/// Limiter state carried between ticks.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Limiter {
    last: Actuation,
}

impl Limiter {
    #[must_use]
    /// A limiter with no previous output.
    pub const fn new() -> Self {
        Self {
            last: Actuation {
                steer_cdeg: 0,
                brake_permille: 0,
                throttle_permille: 0,
            },
        }
    }

    #[must_use]
    /// The output of the last applied tick.
    pub const fn last(&self) -> Actuation {
        self.last
    }

    /// Limits a requested command. Braking always overrides throttle, so the
    /// throttle is forced to zero whenever any brake is requested.
    pub fn apply(&mut self, steer_cdeg: i32, brake: u16, throttle: u16) -> Actuation {
        let steer_target = steer_cdeg.clamp(-MAX_STEER_CDEG, MAX_STEER_CDEG);
        let brake_target = brake.min(MAX_PERMILLE);
        let throttle_target = if brake_target > 0 {
            0
        } else {
            throttle.min(MAX_PERMILLE)
        };
        self.last = Actuation {
            steer_cdeg: slew_i32(self.last.steer_cdeg, steer_target, MAX_STEER_RATE_CDEG),
            brake_permille: slew_u16(self.last.brake_permille, brake_target, MAX_PERMILLE_RATE),
            throttle_permille: slew_u16(
                self.last.throttle_permille,
                throttle_target,
                MAX_PERMILLE_RATE,
            ),
        };
        self.last
    }

    /// Full brake and zero throttle at once, bypassing the slew limit. Steering
    /// holds its last value so the stop does not also turn the wheel.
    ///
    /// For emergency stops only. Ordinary braking goes through [`Limiter::apply`]
    /// so it stays smooth.
    pub fn emergency_brake(&mut self) -> Actuation {
        self.last = Actuation {
            steer_cdeg: self.last.steer_cdeg,
            brake_permille: MAX_PERMILLE,
            throttle_permille: 0,
        };
        self.last
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steering_is_clamped_to_the_mechanical_limit() {
        let mut l = Limiter::new();
        for _ in 0..50 {
            l.apply(9_000, 0, 0);
        }
        assert_eq!(l.last().steer_cdeg, MAX_STEER_CDEG);
    }

    #[test]
    fn steering_rate_is_limited_per_tick() {
        let mut l = Limiter::new();
        let out = l.apply(3_000, 0, 0);
        assert_eq!(out.steer_cdeg, MAX_STEER_RATE_CDEG);
    }

    #[test]
    fn braking_forces_throttle_to_zero() {
        let mut l = Limiter::new();
        for _ in 0..10 {
            l.apply(0, 0, 800);
        }
        let out = l.apply(0, 400, 800);
        assert_eq!(out.throttle_permille, 800 - MAX_PERMILLE_RATE);
        for _ in 0..10 {
            l.apply(0, 400, 800);
        }
        assert_eq!(l.last().throttle_permille, 0);
    }

    #[test]
    fn brake_never_exceeds_full_scale() {
        let mut l = Limiter::new();
        for _ in 0..10 {
            l.apply(0, u16::MAX, 0);
        }
        assert_eq!(l.last().brake_permille, MAX_PERMILLE);
    }

    #[test]
    fn slew_helpers_do_not_overshoot() {
        assert_eq!(slew_i32(0, 10, 100), 10);
        assert_eq!(slew_i32(0, -10, 100), -10);
        assert_eq!(slew_u16(990, 1_000, 100), 1_000);
        assert_eq!(slew_u16(500, 0, 100), 400);
    }
}
