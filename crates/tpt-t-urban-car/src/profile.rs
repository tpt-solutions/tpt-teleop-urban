//! Path curvature and the speed profile along it.
//!
//! The speed at each point is the smallest of three limits:
//!
//! - the posted speed limit,
//! - the lateral-acceleration limit `sqrt(a_lat / κ)` for the local curvature κ,
//! - what the accel and brake limits allow, found with a forward and a backward
//!   pass over the path.
//!
//! Units are SI: metres, seconds, m/s, m/s².

/// Path point in a local planar frame, metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

/// Limits applied to every point of the profile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits {
    /// Posted speed limit, m/s.
    pub speed_limit: f32,
    /// Maximum lateral acceleration, m/s².
    pub lateral_accel: f32,
    /// Maximum acceleration, m/s².
    pub accel: f32,
    /// Maximum braking deceleration, m/s² (positive).
    pub brake: f32,
}

/// Curvature (1/m) of the circle through three points, from the Menger formula
/// κ = 2|cross| / (|ab| |bc| |ca|). Zero when the points are collinear or
/// coincident.
#[must_use]
pub fn curvature(a: Point, b: Point, c: Point) -> f32 {
    let (abx, aby) = (b.x - a.x, b.y - a.y);
    let (bcx, bcy) = (c.x - b.x, c.y - b.y);
    let (cax, cay) = (a.x - c.x, a.y - c.y);
    let cross = abx * (c.y - a.y) - aby * (c.x - a.x);
    let product = dist(abx, aby) * dist(bcx, bcy) * dist(cax, cay);
    if product <= f32::EPSILON {
        0.0
    } else {
        2.0 * cross.abs() / product
    }
}

fn dist(x: f32, y: f32) -> f32 {
    (x * x + y * y).sqrt()
}

/// Fills `out` with the speed at each path point and returns how many were
/// written: the smaller of `path.len()` and `out.len()`.
///
/// The path must have at least two points. Shorter paths write nothing. The
/// first and last points take the curvature of their nearest interior neighbour.
/// The function allocates nothing.
pub fn speed_profile(path: &[Point], limits: &Limits, out: &mut [f32]) -> usize {
    let n = path.len().min(out.len());
    if path.len() < 2 || n < 2 {
        return 0;
    }
    let len = path.len();
    // Curvature at every point.
    for i in 0..n {
        let curv = if i == 0 {
            curvature(path[0], path[1], path[2.min(len - 1)])
        } else if i == len - 1 {
            curvature(path[len - 3.min(len - 1)], path[len - 2], path[len - 1])
        } else {
            curvature(path[i - 1], path[i], path[i + 1])
        };
        let lateral = if curv > f32::EPSILON {
            (limits.lateral_accel / curv).sqrt()
        } else {
            f32::INFINITY
        };
        out[i] = limits.speed_limit.min(lateral);
    }
    // Forward pass: cannot accelerate faster than `accel` allows.
    for i in 1..n {
        let ds = dist(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
        out[i] = out[i].min((out[i - 1] * out[i - 1] + 2.0 * limits.accel * ds).sqrt());
    }
    // Backward pass: must brake early enough to meet each slower point.
    for i in (0..n - 1).rev() {
        let ds = dist(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
        out[i] = out[i].min((out[i + 1] * out[i + 1] + 2.0 * limits.brake * ds).sqrt());
    }
    n
}

#[cfg(test)]
mod tests {
    use super::*;

    const LIMITS: Limits = Limits {
        speed_limit: 13.9,
        lateral_accel: 2.5,
        accel: 1.5,
        brake: 3.0,
    };

    fn straight(n: usize, step: f32) -> Vec<Point> {
        (0..n)
            .map(|i| Point {
                x: i as f32 * step,
                y: 0.0,
            })
            .collect()
    }

    #[test]
    fn curvature_of_a_unit_circle_is_one() {
        // Three points on the unit circle.
        let a = Point { x: 1.0, y: 0.0 };
        let b = Point { x: 0.0, y: 1.0 };
        let c = Point { x: -1.0, y: 0.0 };
        assert!((curvature(a, b, c) - 1.0).abs() < 1e-5);
    }

    #[test]
    fn collinear_points_have_zero_curvature() {
        let p = straight(3, 1.0);
        assert_eq!(curvature(p[0], p[1], p[2]), 0.0);
    }

    #[test]
    fn straight_path_runs_at_the_speed_limit() {
        let path = straight(20, 1.0);
        let mut out = [0.0; 20];
        assert_eq!(speed_profile(&path, &LIMITS, &mut out), 20);
        assert!(out.iter().all(|&v| (v - LIMITS.speed_limit).abs() < 1e-4));
    }

    #[test]
    fn tight_corner_caps_speed_at_the_lateral_limit() {
        // A 4 m radius arc: κ = 0.25, so v <= sqrt(2.5 / 0.25) = 3.16 m/s.
        let path: Vec<Point> = (0..=30)
            .map(|i| {
                let t = i as f32 * 0.05;
                Point {
                    x: 4.0 * t.sin(),
                    y: 4.0 * (1.0 - t.cos()),
                }
            })
            .collect();
        let mut out = vec![0.0; path.len()];
        speed_profile(&path, &LIMITS, &mut out);
        let mid = out[15];
        assert!(mid <= 3.2, "mid-corner speed {mid}");
    }

    #[test]
    fn profile_respects_brake_limit_into_a_corner() {
        // Straight run, then a sharp corner at the end. The profile must ramp
        // down so that consecutive points never need more than `brake`.
        let mut path = straight(40, 1.0);
        path.push(Point { x: 39.0, y: 0.5 });
        path.push(Point { x: 38.0, y: 1.0 });
        let mut out = vec![0.0; path.len()];
        speed_profile(&path, &LIMITS, &mut out);
        for i in 0..path.len() - 1 {
            let ds = dist(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
            let decel = (out[i] * out[i] - out[i + 1] * out[i + 1]) / (2.0 * ds);
            assert!(decel <= LIMITS.brake + 1e-3, "step {i}: decel {decel}");
        }
    }

    #[test]
    fn short_path_writes_nothing() {
        let mut out = [0.0; 4];
        assert_eq!(speed_profile(&straight(1, 1.0), &LIMITS, &mut out), 0);
    }
}
