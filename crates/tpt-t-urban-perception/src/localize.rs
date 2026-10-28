//! LiDAR localization by scan matching against a known static map.
//!
//! [`refine`] runs point-to-point iterative closest point (ICP) in two dimensions.
//! The scan is in the vehicle frame and the map is in the world frame. Each
//! iteration pairs every scan point, moved by the current pose estimate, with its
//! nearest map point. It then solves for the rigid motion that best aligns the
//! pairs, in closed form, and applies it to the pose.
//!
//! Pairs farther apart than `max_corr_m` are dropped, so points that have no map
//! counterpart (a person, a parked car) do not drag the estimate. The map is
//! assumed static: a moving object in the scan is treated as an outlier only if it
//! is farther than `max_corr_m` from the map.
//!
//! Accuracy depends on the map and the initial guess. ICP converges to a local
//! minimum, so the initial pose must be within roughly the map's feature spacing.
//! The caller supplies it, normally from the last estimate plus odometry.

/// Vehicle pose in the world frame: position in metres, heading in radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose2 {
    /// Position x, in metres.
    pub x: f32,
    /// Position y, in metres.
    pub y: f32,
    /// Heading, counter-clockwise from +x, in radians.
    pub yaw: f32,
}

impl Pose2 {
    /// Maps a point in the vehicle frame into the world frame.
    #[must_use]
    pub fn transform(&self, p: [f32; 2]) -> [f32; 2] {
        let (s, c) = self.yaw.sin_cos();
        [c * p[0] - s * p[1] + self.x, s * p[0] + c * p[1] + self.y]
    }
}

/// Refines `initial` so that `scan` (vehicle frame) lines up with `map` (world
/// frame).
///
/// Returns the refined pose and the root-mean-square distance of the final matched
/// pairs, in metres. Returns the initial pose and `f32::INFINITY` if no scan point
/// has a map point within `max_corr_m`.
#[must_use]
pub fn refine(
    map: &[[f32; 2]],
    scan: &[[f32; 2]],
    initial: Pose2,
    iterations: u32,
    max_corr_m: f32,
) -> (Pose2, f32) {
    let mut pose = initial;
    for _ in 0..iterations {
        let Some(delta) = best_rigid_step(map, scan, pose, max_corr_m) else {
            return (initial, f32::INFINITY);
        };
        // The step is in the world frame, so it applies to the pose from outside:
        // rotate and translate the pose by it.
        let (s, c) = delta.yaw.sin_cos();
        pose = Pose2 {
            x: c * pose.x - s * pose.y + delta.x,
            y: s * pose.x + c * pose.y + delta.y,
            yaw: pose.yaw + delta.yaw,
        };
    }
    (
        pose,
        residual(map, scan, pose, max_corr_m).unwrap_or(f32::INFINITY),
    )
}

/// The rigid motion that best aligns the matched pairs under `pose`, or `None` when
/// there are no pairs.
fn best_rigid_step(
    map: &[[f32; 2]],
    scan: &[[f32; 2]],
    pose: Pose2,
    max_corr_m: f32,
) -> Option<Pose2> {
    let mut pairs = Vec::with_capacity(scan.len());
    for p in scan {
        let moved = pose.transform(*p);
        if let Some((m, d2)) = nearest(map, moved)
            && d2 <= max_corr_m * max_corr_m
        {
            pairs.push((moved, m));
        }
    }
    if pairs.is_empty() {
        return None;
    }
    let n = pairs.len() as f32;
    let (mut ax, mut ay, mut bx, mut by) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    for (a, b) in &pairs {
        ax += a[0];
        ay += a[1];
        bx += b[0];
        by += b[1];
    }
    let (ax, ay, bx, by) = (ax / n, ay / n, bx / n, by / n);

    // Cross-covariance of the centred pairs. Its angle is the best rotation.
    let (mut sxx, mut sxy, mut syx, mut syy) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    for (a, b) in &pairs {
        let (px, py) = (a[0] - ax, a[1] - ay);
        let (qx, qy) = (b[0] - bx, b[1] - by);
        sxx += px * qx;
        sxy += px * qy;
        syx += py * qx;
        syy += py * qy;
    }
    let yaw = (sxy - syx).atan2(sxx + syy);
    let (s, c) = yaw.sin_cos();
    Some(Pose2 {
        x: bx - (c * ax - s * ay),
        y: by - (s * ax + c * ay),
        yaw,
    })
}

/// The map point nearest to `p`, and its squared distance.
fn nearest(map: &[[f32; 2]], p: [f32; 2]) -> Option<([f32; 2], f32)> {
    map.iter()
        .map(|m| {
            let dx = m[0] - p[0];
            let dy = m[1] - p[1];
            (*m, dx * dx + dy * dy)
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

/// RMS distance of the pairs under `pose`, or `None` when there are no pairs.
fn residual(map: &[[f32; 2]], scan: &[[f32; 2]], pose: Pose2, max_corr_m: f32) -> Option<f32> {
    let limit = max_corr_m * max_corr_m;
    let (sum, count) = scan
        .iter()
        .filter_map(|p| nearest(map, pose.transform(*p)))
        .filter(|(_, d2)| *d2 <= limit)
        .fold((0.0f32, 0u32), |(s, n), (_, d2)| (s + d2, n + 1));
    (count > 0).then(|| (sum / count as f32).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Points along the outline of a square of side `side`, centred on `(cx, cy)`.
    fn square(cx: f32, cy: f32, side: f32) -> Vec<[f32; 2]> {
        let h = side / 2.0;
        let steps = (side / 0.05) as usize;
        let mut out = Vec::new();
        for i in 0..steps {
            let t = -h + i as f32 * 0.05;
            out.push([cx + t, cy - h]);
            out.push([cx + t, cy + h]);
            out.push([cx - h, cy + t]);
            out.push([cx + h, cy + t]);
        }
        out
    }

    #[test]
    fn recovers_a_known_offset_and_heading() {
        let map = [square(6.0, 0.0, 4.0), square(-3.0, 5.0, 2.0)].concat();
        let truth = Pose2 {
            x: 1.0,
            y: 0.5,
            yaw: 0.05,
        };
        // Scan the map from the true pose, in the vehicle frame.
        let inverse_yaw = -truth.yaw;
        let (s, c) = inverse_yaw.sin_cos();
        let scan: Vec<[f32; 2]> = map
            .iter()
            .map(|m| {
                let dx = m[0] - truth.x;
                let dy = m[1] - truth.y;
                [c * dx - s * dy, s * dx + c * dy]
            })
            .collect();

        let initial = Pose2 {
            x: truth.x + 0.3,
            y: truth.y - 0.2,
            yaw: truth.yaw + 0.03,
        };
        let (pose, rms) = refine(&map, &scan, initial, 30, 1.0);
        assert!(
            (pose.x - truth.x).hypot(pose.y - truth.y) < 0.01,
            "{pose:?}"
        );
        assert!((pose.yaw - truth.yaw).abs() < 0.001, "{pose:?}");
        assert!(rms < 0.01, "{rms}");
    }

    #[test]
    fn with_no_pair_in_range_the_initial_pose_is_returned() {
        let map = square(100.0, 100.0, 2.0);
        let scan = [[0.0, 0.0]];
        let initial = Pose2 {
            x: 1.0,
            y: 2.0,
            yaw: 0.1,
        };
        let (pose, rms) = refine(&map, &scan, initial, 5, 1.0);
        assert_eq!(pose, initial);
        assert!(rms.is_infinite());
    }
}
