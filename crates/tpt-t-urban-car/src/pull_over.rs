//! Pull-over path to a shoulder, for a minimal risk maneuver (spec §5.3).
//!
//! The path runs along the lane from the vehicle's position (the origin, heading
//! `+x`) and moves laterally by `offset_m` over `length_m`. The lateral profile
//! is a smoothstep, `y = offset · (3t² − 2t³)` with `t = x / length`. Its
//! curvature is zero at both ends, so a vehicle joins and leaves the path without
//! a steering step.
//!
//! The peak curvature is at the ends: `κ = 6 · offset / length²`. Holding a
//! speed `v` under a lateral-acceleration limit `a` gives the closed-form minimum
//! length [`min_length`]. The true curvature is a little lower in the middle, so
//! the bound is conservative.
//!
//! Units are SI: metres, m/s, m/s².

use crate::profile::Point;

/// Shortest pull-over length for a vehicle at `speed_m_s` to shift `offset_m`
/// sideways without exceeding `lateral_accel` (m/s²).
///
/// Returns `f32::INFINITY` if `lateral_accel` is not positive, since no length
/// satisfies that limit.
#[must_use]
pub fn min_length(speed_m_s: f32, offset_m: f32, lateral_accel: f32) -> f32 {
    if lateral_accel <= 0.0 {
        return f32::INFINITY;
    }
    speed_m_s * (6.0 * offset_m.abs() / lateral_accel).sqrt()
}

/// Whether a pull-over of `offset_m` can be completed within `available_m` at
/// `speed_m_s` under `lateral_accel`.
#[must_use]
pub fn feasible(speed_m_s: f32, offset_m: f32, available_m: f32, lateral_accel: f32) -> bool {
    min_length(speed_m_s, offset_m, lateral_accel) <= available_m
}

/// Fills `out` with `n` points of the pull-over path, from the origin to
/// `(length_m, offset_m)`, and returns how many were written.
///
/// Writes nothing if `n < 2` or `length_m` is not positive. Shorter `out` slices
/// are filled as far as they reach. The function allocates nothing.
pub fn path(offset_m: f32, length_m: f32, out: &mut [Point]) -> usize {
    let n = out.len();
    if n < 2 || length_m <= 0.0 {
        return 0;
    }
    for (i, point) in out.iter_mut().enumerate() {
        let t = i as f32 / (n - 1) as f32;
        let s = t * t * (3.0 - 2.0 * t);
        *point = Point {
            x: length_m * t,
            y: offset_m * s,
        };
    }
    n
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::curvature;

    #[test]
    fn the_path_starts_at_the_origin_and_ends_on_the_shoulder() {
        let mut pts = [Point { x: 0.0, y: 0.0 }; 50];
        assert_eq!(path(3.0, 60.0, &mut pts), 50);
        assert_eq!(pts[0], Point { x: 0.0, y: 0.0 });
        let last = pts[49];
        assert!((last.x - 60.0).abs() < 1e-4 && (last.y - 3.0).abs() < 1e-4);
    }

    #[test]
    fn the_path_moves_forward_monotonically() {
        let mut pts = [Point { x: 0.0, y: 0.0 }; 40];
        path(-2.5, 30.0, &mut pts);
        assert!(pts.windows(2).all(|w| w[1].x > w[0].x));
    }

    #[test]
    fn a_path_of_the_minimum_length_stays_within_the_lateral_limit() {
        // 10 m/s, 3 m shift, 2 m/s² lateral limit.
        let (v, offset, lat) = (10.0, 3.0, 2.0);
        let length = min_length(v, offset, lat);
        let mut pts = [Point { x: 0.0, y: 0.0 }; 400];
        path(offset, length, &mut pts);
        let peak = pts
            .windows(3)
            .map(|w| curvature(w[0], w[1], w[2]))
            .fold(0.0_f32, f32::max);
        // Lateral acceleration v²κ must stay within the limit, with discretization slack.
        assert!(v * v * peak <= lat * 1.02, "lateral accel {} exceeds {lat}", v * v * peak);
    }

    #[test]
    fn a_shorter_path_breaks_the_limit() {
        let (v, offset, lat) = (10.0, 3.0, 2.0);
        let length = 0.5 * min_length(v, offset, lat);
        let mut pts = [Point { x: 0.0, y: 0.0 }; 400];
        path(offset, length, &mut pts);
        let peak = pts
            .windows(3)
            .map(|w| curvature(w[0], w[1], w[2]))
            .fold(0.0_f32, f32::max);
        assert!(v * v * peak > lat);
    }

    #[test]
    fn feasibility_follows_the_available_distance() {
        let need = min_length(12.0, 3.5, 2.5);
        assert!(feasible(12.0, 3.5, need + 1.0, 2.5));
        assert!(!feasible(12.0, 3.5, need - 1.0, 2.5));
        assert!(!feasible(12.0, 3.5, 1_000.0, 0.0));
    }

    #[test]
    fn degenerate_inputs_write_nothing() {
        let mut one = [Point { x: 0.0, y: 0.0 }; 1];
        assert_eq!(path(1.0, 10.0, &mut one), 0);
        let mut pts = [Point { x: 0.0, y: 0.0 }; 4];
        assert_eq!(path(1.0, 0.0, &mut pts), 0);
    }
}
