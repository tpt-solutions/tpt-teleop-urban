//! Kinematic bicycle model of the vehicle plant.
//!
//! State is position, heading and speed. Inputs are steering angle and
//! longitudinal acceleration. The model is the standard kinematic bicycle, which
//! is accurate at urban speeds where tyre slip is small.

/// Vehicle plant state in metres, radians and metres per second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bicycle {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub speed: f32,
    /// Distance between axles, metres.
    pub wheelbase: f32,
}

impl Bicycle {
    #[must_use]
    pub const fn new(x: f32, y: f32, yaw: f32, speed: f32, wheelbase: f32) -> Self {
        Self {
            x,
            y,
            yaw,
            speed,
            wheelbase,
        }
    }

    /// Advances by `dt` seconds with steering `steer` (radians) and acceleration
    /// `accel` (m/s²). Speed is clamped at zero, so the vehicle never reverses here.
    pub fn step(&mut self, dt: f32, steer: f32, accel: f32) {
        self.x += self.speed * self.yaw.cos() * dt;
        self.y += self.speed * self.yaw.sin() * dt;
        self.yaw += self.speed / self.wheelbase * steer.tan() * dt;
        self.speed = (self.speed + accel * dt).max(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_line_at_constant_speed() {
        let mut b = Bicycle::new(0.0, 0.0, 0.0, 10.0, 2.7);
        for _ in 0..100 {
            b.step(0.01, 0.0, 0.0);
        }
        assert!((b.x - 10.0).abs() < 1e-3);
        assert!(b.y.abs() < 1e-6);
    }

    #[test]
    fn braking_stops_at_zero_speed_and_does_not_reverse() {
        let mut b = Bicycle::new(0.0, 0.0, 0.0, 5.0, 2.7);
        for _ in 0..1_000 {
            b.step(0.01, 0.0, -7.0);
        }
        assert_eq!(b.speed, 0.0);
        // Stopping distance from 5 m/s at 7 m/s² is v²/2a ≈ 1.786 m. Forward Euler
        // at dt = 10 ms adds about 1.4 % here, so allow 0.05 m.
        assert!((b.x - 1.786).abs() < 0.05, "x {}", b.x);
    }

    #[test]
    fn positive_steer_turns_left() {
        let mut b = Bicycle::new(0.0, 0.0, 0.0, 5.0, 2.7);
        for _ in 0..100 {
            b.step(0.01, 0.2, 0.0);
        }
        assert!(b.yaw > 0.0 && b.y > 0.0);
    }
}
