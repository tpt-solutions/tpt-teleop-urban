//! 2D LiDAR by ray casting against axis-aligned boxes.
//!
//! The world is a set of boxes (buildings, parked cars, curbs seen from above).
//! A scan casts one ray per beam from the sensor. Each ray returns the nearest
//! box it crosses, within `max_range_m`, with uniform range noise. A beam that
//! hits nothing returns `None`.
//!
//! Ray and box intersection uses the slab method. Noise comes from a seeded
//! xorshift stream, so a scan is reproducible.

/// Axis-aligned box in the world frame, in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    /// Minimum corner `[x, y]`.
    pub min: [f32; 2],
    /// Maximum corner `[x, y]`.
    pub max: [f32; 2],
}

/// Distance along a ray from `origin` in unit direction `dir` to where it enters
/// `aabb`, or `None` if it misses or the box lies behind the origin.
///
/// A ray that starts inside the box returns `Some(0.0)`.
#[must_use]
pub fn ray_distance(origin: [f32; 2], dir: [f32; 2], aabb: &Aabb) -> Option<f32> {
    let mut t_near = f32::NEG_INFINITY;
    let mut t_far = f32::INFINITY;
    for axis in 0..2 {
        let (o, d) = (origin[axis], dir[axis]);
        let (lo, hi) = (aabb.min[axis], aabb.max[axis]);
        if d.abs() < f32::EPSILON {
            if o < lo || o > hi {
                return None;
            }
        } else {
            let t1 = (lo - o) / d;
            let t2 = (hi - o) / d;
            t_near = t_near.max(t1.min(t2));
            t_far = t_far.min(t1.max(t2));
        }
    }
    if t_near > t_far || t_far < 0.0 {
        None
    } else {
        Some(t_near.max(0.0))
    }
}

/// A 2D LiDAR with evenly spaced beams over the full circle.
#[derive(Clone, Debug)]
pub struct Lidar {
    /// Number of beams per scan.
    pub beams: usize,
    /// Maximum range, in metres. Beams with no return within it report `None`.
    pub max_range_m: f32,
    /// Half-width of the uniform range noise, in metres.
    pub noise_m: f32,
    state: u32,
}

impl Lidar {
    /// A sensor with `beams` beams, range limit, noise amplitude, and a non-zero
    /// seed. A zero seed is replaced with a fixed constant.
    #[must_use]
    pub fn new(beams: usize, max_range_m: f32, noise_m: f32, seed: u32) -> Self {
        Self {
            beams,
            max_range_m,
            noise_m,
            state: if seed == 0 { 0x6C8E_9CF5 } else { seed },
        }
    }

    fn noise(&mut self) -> f32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        let unit = self.state as f32 / u32::MAX as f32;
        (unit * 2.0 - 1.0) * self.noise_m
    }

    /// One scan from `origin` against `world`. Entry `i` is the range of beam
    /// `i`, beam 0 pointing along +x, or `None` for no return.
    pub fn scan(&mut self, origin: [f32; 2], world: &[Aabb]) -> Vec<Option<f32>> {
        (0..self.beams)
            .map(|i| {
                let angle = core::f32::consts::TAU * i as f32 / self.beams as f32;
                let dir = [angle.cos(), angle.sin()];
                let nearest = world
                    .iter()
                    .filter_map(|b| ray_distance(origin, dir, b))
                    .filter(|&t| t <= self.max_range_m)
                    .fold(None, |best: Option<f32>, t| {
                        Some(best.map_or(t, |b| b.min(t)))
                    })?;
                let noisy = nearest + self.noise();
                Some(noisy.clamp(0.0, self.max_range_m))
            })
            .collect()
    }

    /// The scan as points in the world frame, for beams with a return.
    pub fn points(&mut self, origin: [f32; 2], world: &[Aabb]) -> Vec<[f32; 2]> {
        let ranges = self.scan(origin, world);
        ranges
            .iter()
            .enumerate()
            .filter_map(|(i, r)| {
                let r = (*r)?;
                let angle = core::f32::consts::TAU * i as f32 / self.beams as f32;
                Some([origin[0] + r * angle.cos(), origin[1] + r * angle.sin()])
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wall_at_x10() -> Aabb {
        Aabb {
            min: [10.0, -5.0],
            max: [11.0, 5.0],
        }
    }

    #[test]
    fn a_ray_hits_the_near_face_of_a_box() {
        let t = ray_distance([0.0, 0.0], [1.0, 0.0], &wall_at_x10()).expect("hit");
        assert!((t - 10.0).abs() < 1e-5, "t {t}");
    }

    #[test]
    fn a_ray_away_from_the_box_misses_and_a_parallel_ray_misses() {
        assert_eq!(ray_distance([0.0, 0.0], [-1.0, 0.0], &wall_at_x10()), None);
        assert_eq!(ray_distance([0.0, 6.0], [1.0, 0.0], &wall_at_x10()), None);
    }

    #[test]
    fn a_ray_from_inside_returns_zero() {
        assert_eq!(
            ray_distance([10.5, 0.0], [1.0, 0.0], &wall_at_x10()),
            Some(0.0)
        );
    }

    #[test]
    fn the_nearest_of_two_boxes_on_a_beam_is_returned() {
        let near = Aabb {
            min: [4.0, -1.0],
            max: [5.0, 1.0],
        };
        let mut lidar = Lidar::new(4, 50.0, 0.0, 1);
        let ranges = lidar.scan([0.0, 0.0], &[wall_at_x10(), near]);
        assert!((ranges[0].expect("beam 0 hits") - 4.0).abs() < 1e-5);
    }

    #[test]
    fn beams_that_hit_nothing_or_exceed_the_range_return_none() {
        let mut lidar = Lidar::new(4, 8.0, 0.0, 1);
        let ranges = lidar.scan([0.0, 0.0], &[wall_at_x10()]);
        // The wall is 10 m away, beyond the 8 m range, so no beam returns.
        assert!(ranges.iter().all(Option::is_none));
    }

    #[test]
    fn noise_stays_within_its_bound() {
        let mut lidar = Lidar::new(360, 50.0, 0.05, 99);
        for r in lidar
            .scan([0.0, 0.0], &[wall_at_x10()])
            .into_iter()
            .flatten()
        {
            assert!((0.0..=50.0).contains(&r));
        }
        let mut lidar = Lidar::new(1, 50.0, 0.05, 5);
        let r = lidar.scan([0.0, 0.0], &[wall_at_x10()])[0].expect("hit");
        assert!((r - 10.0).abs() <= 0.05 + 1e-5, "range {r}");
    }

    #[test]
    fn the_point_cloud_lies_on_the_wall() {
        let mut lidar = Lidar::new(72, 50.0, 0.0, 3);
        let points = lidar.points([0.0, 0.0], &[wall_at_x10()]);
        assert!(!points.is_empty());
        for p in points {
            // Points on the near face are at x = 10 and within the wall's y span.
            let on_face = (p[0] - 10.0).abs() < 1e-3 && p[1].abs() <= 5.0 + 1e-3;
            let on_side = p[1] >= -5.0 - 1e-3 && p[1] <= 5.0 + 1e-3 && p[0] >= 10.0 - 1e-3;
            assert!(on_face || on_side, "{p:?} not on the box");
        }
    }
}
