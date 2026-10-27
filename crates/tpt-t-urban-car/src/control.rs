//! Path tracking with pure pursuit steering.
//!
//! The controller picks the first path point at least `lookahead` metres away
//! from the vehicle, converts it to the vehicle frame, and steers toward it.
//! The steering angle is `δ = atan(2 L y / Ld²)`, where `L` is the wheelbase,
//! `Ld` the lookahead distance and `y` the lateral offset of the target.

use crate::profile::Point;

/// Vehicle pose: position in metres and heading in radians (counter-clockwise
/// from +x).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
}

/// Steering angle in radians that tracks toward the path.
///
/// Returns `None` when no path point is at least `lookahead` away, which means
/// the path is too short to track and the caller should stop.
#[must_use]
pub fn pure_pursuit(pose: Pose, path: &[Point], lookahead: f32, wheelbase: f32) -> Option<f32> {
    let target = path.iter().find(|p| {
        let dx = p.x - pose.x;
        let dy = p.y - pose.y;
        (dx * dx + dy * dy).sqrt() >= lookahead
    })?;
    // Target offset in the vehicle frame: rotate by -yaw.
    let dx = target.x - pose.x;
    let dy = target.y - pose.y;
    let (sin, cos) = pose.yaw.sin_cos();
    let lateral = -sin * dx + cos * dy;
    Some((2.0 * wheelbase * lateral / (lookahead * lookahead)).atan())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line_ahead() -> Vec<Point> {
        (0..=50)
            .map(|i| Point {
                x: i as f32,
                y: 0.0,
            })
            .collect()
    }

    #[test]
    fn straight_ahead_needs_no_steering() {
        let pose = Pose {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        };
        let steer = pure_pursuit(pose, &line_ahead(), 5.0, 2.7).unwrap();
        assert!(steer.abs() < 1e-6);
    }

    #[test]
    fn path_to_the_left_steers_left() {
        // Vehicle heading +x, path offset 1 m to the left (+y).
        let path: Vec<Point> = (0..=50)
            .map(|i| Point {
                x: i as f32,
                y: 1.0,
            })
            .collect();
        let pose = Pose {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        };
        let steer = pure_pursuit(pose, &path, 5.0, 2.7).unwrap();
        assert!(steer > 0.0, "steer {steer}");
    }

    #[test]
    fn heading_is_accounted_for() {
        // Vehicle turned 90° left, so a point straight "ahead" on the old axis is
        // now to the vehicle's right.
        let pose = Pose {
            x: 0.0,
            y: 0.0,
            yaw: core::f32::consts::FRAC_PI_2,
        };
        let steer = pure_pursuit(pose, &line_ahead(), 5.0, 2.7).unwrap();
        assert!(steer < 0.0, "steer {steer}");
    }

    #[test]
    fn short_path_has_no_target() {
        let path = [Point { x: 0.0, y: 0.0 }, Point { x: 1.0, y: 0.0 }];
        let pose = Pose {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        };
        assert_eq!(pure_pursuit(pose, &path, 5.0, 2.7), None);
    }
}
