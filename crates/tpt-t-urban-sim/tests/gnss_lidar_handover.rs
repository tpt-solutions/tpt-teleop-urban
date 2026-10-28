//! GNSS fails, LiDAR scan matching takes over, and the pose stays accurate.
//!
//! The vehicle drives along +x at 1 m/s. Its GNSS receiver takes a 5 m multipath
//! jump for one second. The source selector switches to LiDAR at once, and the
//! pose is then refined from each scan, starting from the previous pose advanced by
//! the speed. The test checks that every pose used while on LiDAR is within 10 cm
//! of the truth, and that the GNSS error it avoided was really 5 m.

use tpt_t_urban_perception::gnss::{Source, SourceSelector};
use tpt_t_urban_perception::localize::{Pose2, refine};
use tpt_t_urban_sim::lidar::{Aabb, Lidar};
use tpt_t_urban_sim::{Condition, Gnss, Window};

const SPEED_M_S: f32 = 1.0;
const STEP_MS: u64 = 100;
const DURATION_MS: u64 = 4_000;

/// Static boxes the map is built from.
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
fn the_lidar_pose_stays_within_ten_centimetres_through_a_gnss_jump() {
    let windows = [Window {
        start_ms: 1_500,
        end_ms: 2_500,
        condition: Condition::Multipath {
            dx_mm: 5_000,
            dy_mm: 0,
        },
    }];
    let mut gnss = Gnss::new(&windows, 20, 0x5EED_0001);
    let mut selector = SourceSelector::new();
    let mut lidar = Lidar::new(720, 40.0, 0.02, 0x1D4A_0042);
    let map = map();

    let dt_s = STEP_MS as f32 / 1_000.0;
    let mut estimate = Pose2 {
        x: 0.5,
        y: 0.0,
        yaw: 0.0,
    };
    let mut lidar_steps = 0;
    let mut worst_lidar_error_m = 0.0f32;
    let mut worst_gnss_error_m = 0.0f32;

    for t_ms in (0..DURATION_MS).step_by(STEP_MS as usize) {
        let truth_x_m = 0.5 + SPEED_M_S * t_ms as f32 / 1_000.0;
        let truth_x_mm = (truth_x_m * 1_000.0).round() as i64;

        let fix = gnss.fix(t_ms, truth_x_mm, 0);
        if let Some(f) = fix {
            worst_gnss_error_m =
                worst_gnss_error_m.max((f.x_mm as f32 / 1_000.0 - truth_x_m).abs());
        }
        let source = match fix {
            Some(f) => selector.on_gnss_fix(f),
            None => selector.on_gnss_lost(),
        };

        estimate = match (source, fix) {
            (Source::Gnss, Some(f)) => Pose2 {
                x: f.x_mm as f32 / 1_000.0,
                y: f.y_mm as f32 / 1_000.0,
                yaw: 0.0,
            },
            // The GNSS fix is not used on LiDAR, so refine from the previous pose
            // advanced by the speed.
            _ => {
                lidar_steps += 1;
                let scan: Vec<[f32; 2]> = lidar
                    .points([truth_x_m, 0.0], &WORLD)
                    .into_iter()
                    .map(|p| [p[0] - truth_x_m, p[1]])
                    .collect();
                let initial = Pose2 {
                    x: estimate.x + SPEED_M_S * dt_s,
                    ..estimate
                };
                let (pose, _) = refine(&map, &scan, initial, 40, 1.0);
                let error = (pose.x - truth_x_m).hypot(pose.y);
                worst_lidar_error_m = worst_lidar_error_m.max(error);
                pose
            }
        };
    }

    eprintln!(
        "handover: {lidar_steps} LiDAR steps, worst LiDAR pose error {:.1} mm, worst GNSS error {:.2} m",
        worst_lidar_error_m * 1_000.0,
        worst_gnss_error_m
    );
    assert!(lidar_steps >= 10, "only {lidar_steps} LiDAR steps");
    assert!(
        worst_gnss_error_m > 4.5,
        "the jump should be 5 m off, got {worst_gnss_error_m} m"
    );
    assert!(
        worst_lidar_error_m < 0.10,
        "LiDAR pose error {worst_lidar_error_m} m"
    );
}
