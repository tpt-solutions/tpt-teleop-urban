//! LiDAR to tracks: the sim's ray-cast scan is clustered by the perception
//! crate and the clusters are tracked across frames.
//!
//! Two boxes are in view. One moves along +x at 2 m/s and one is static. After
//! three seconds of 10 Hz scans the tracker should hold two tracks, with the
//! moving one at about 2 m/s and the static one at rest.

use tpt_t_urban_perception::detect::cluster;
use tpt_t_urban_perception::track::{Config, Tracker};
use tpt_t_urban_sim::lidar::{Aabb, Lidar};

fn boxes(t: f32) -> [Aabb; 2] {
    let x = 10.0 + 2.0 * t;
    [
        Aabb {
            min: [x, 2.5],
            max: [x + 1.0, 3.5],
        },
        Aabb {
            min: [-7.0, -4.5],
            max: [-6.0, -3.5],
        },
    ]
}

#[test]
fn a_moving_and_a_static_object_are_tracked_with_their_velocities() {
    let mut lidar = Lidar::new(720, 40.0, 0.02, 0xBEEF);
    let mut tracker = Tracker::new(Config {
        gate_m: 1.5,
        alpha: 0.5,
        beta: 0.3,
        max_misses: 3,
    });
    let dt = 0.1;
    for step in 0..30 {
        let t = step as f32 * dt;
        let world = boxes(t);
        let points = lidar.points([0.0, 0.0], &world);
        let detections: Vec<[f32; 2]> = cluster(&points, 0.6)
            .iter()
            .filter(|c| c.points >= 3)
            .map(|c| c.centroid)
            .collect();
        tracker.update(&detections, dt);
    }

    let tracks = tracker.tracks();
    assert_eq!(tracks.len(), 2, "{tracks:?}");
    let moving = tracks
        .iter()
        .find(|t| t.position[0] > 0.0)
        .expect("the moving object is tracked");
    let stat = tracks
        .iter()
        .find(|t| t.position[0] < 0.0)
        .expect("the static object is tracked");
    assert!((moving.velocity[0] - 2.0).abs() < 0.5, "{moving:?}");
    assert!(
        stat.velocity[0].abs() < 0.2 && stat.velocity[1].abs() < 0.2,
        "{stat:?}"
    );
    assert!(moving.hits >= 20 && stat.hits >= 20, "{tracks:?}");
}
