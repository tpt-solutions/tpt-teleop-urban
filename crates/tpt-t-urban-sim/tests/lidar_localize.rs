//! Localization accuracy with the simulated LiDAR, against a map of the static
//! world.
//!
//! The vehicle starts from a guess 0.3 m and 0.2 m off, and 0.03 rad off in
//! heading. Each seed gives a different noisy scan. The refined pose is compared
//! with the truth, and the worst error over all seeds is the accuracy claim.
//!
//! The LiDAR has no heading of its own, so the true heading is zero. Its beams
//! are the ones `Lidar::points` casts, with the noise it adds to each range.

use tpt_t_urban_perception::localize::{Pose2, refine};
use tpt_t_urban_sim::lidar::{Aabb, Lidar};

/// Static boxes the map is built from. The vehicle sees all three.
const WORLD: [Aabb; 3] = [
    Aabb {
        min: [8.0, -1.0],
        max: [9.0, 1.0],
    },
    Aabb {
        min: [-3.0, 4.0],
        max: [-1.0, 6.0],
    },
    Aabb {
        min: [2.0, -6.0],
        max: [6.0, -5.0],
    },
];

/// Map points along the outline of every box, 5 cm apart.
fn map() -> Vec<[f32; 2]> {
    let mut out = Vec::new();
    for b in &WORLD {
        let step = 0.05;
        let nx = ((b.max[0] - b.min[0]) / step).round() as usize;
        let ny = ((b.max[1] - b.min[1]) / step).round() as usize;
        for i in 0..nx {
            let x = b.min[0] + i as f32 * step;
            out.push([x, b.min[1]]);
            out.push([x, b.max[1]]);
        }
        for j in 0..ny {
            let y = b.min[1] + j as f32 * step;
            out.push([b.min[0], y]);
            out.push([b.max[0], y]);
        }
    }
    out
}

#[test]
fn the_refined_pose_is_within_ten_centimetres_for_every_seed() {
    let truth = [1.0f32, 0.5];
    let initial = Pose2 {
        x: truth[0] + 0.3,
        y: truth[1] - 0.2,
        yaw: 0.03,
    };
    let map = map();

    let mut worst_position_m = 0.0f32;
    let mut worst_yaw_rad = 0.0f32;
    for seed in 1..=20u32 {
        let mut lidar = Lidar::new(720, 40.0, 0.02, seed.wrapping_mul(0x9E37_79B9));
        // The scan comes back in the world frame, so move it into the vehicle frame.
        let scan: Vec<[f32; 2]> = lidar
            .points(truth, &WORLD)
            .into_iter()
            .map(|p| [p[0] - truth[0], p[1] - truth[1]])
            .collect();
        assert!(scan.len() > 100, "seed {seed}: only {} returns", scan.len());

        let (pose, rms) = refine(&map, &scan, initial, 40, 1.0);
        let position_m = (pose.x - truth[0]).hypot(pose.y - truth[1]);
        worst_position_m = worst_position_m.max(position_m);
        worst_yaw_rad = worst_yaw_rad.max(pose.yaw.abs());
        assert!(rms < 0.05, "seed {seed}: residual {rms} m");
    }
    eprintln!(
        "scan-match accuracy over 20 seeds: worst position {:.1} mm, worst yaw {:.3} deg",
        worst_position_m * 1_000.0,
        worst_yaw_rad.to_degrees()
    );
    assert!(
        worst_position_m < 0.10,
        "worst position {worst_position_m} m"
    );
    assert!(worst_yaw_rad < 0.01, "worst yaw {worst_yaw_rad} rad");
}
