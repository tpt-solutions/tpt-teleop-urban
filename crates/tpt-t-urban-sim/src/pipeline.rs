//! Three pinned threads passing messages over the core bus: the thread-per-core
//! layout of spec §4.5, run as one program.
//!
//! - Core 0, sensor: ray-casts a LiDAR scan of the scene each frame.
//! - Core 1, perception: clusters the scan and tracks the objects.
//! - Core 2, consumer: receives the tracks and measures how long each frame took
//!   from the sensor's send to the consumer's receive.
//!
//! The bus carries fixed-size `Copy` messages and never blocks. Each thread spins
//! with `try_send` or `try_recv` and yields between attempts. The scene and the
//! sensor are the ones from the perception tracking test, so the result can be
//! checked.
//!
//! Host timing only, and pinning is reported, not enforced by the test. The
//! target-hardware latency budget is a separate check.

use std::time::Instant;

use tpt_t_urban_core::affinity::{PinError, spawn_pinned};
use tpt_t_urban_core::bus::{Receiver, Sender, channel};
use tpt_t_urban_perception::detect::cluster;
use tpt_t_urban_perception::track::{Config, Tracker};

use crate::lidar::{Aabb, Lidar};

/// Beams in each scan. Also the most points a scan message carries.
pub const BEAMS: usize = 720;
/// Most tracks a track message carries.
pub const MAX_TRACKS: usize = 8;
/// Bus capacity in messages. A power of two, as the ring requires.
const CAPACITY: usize = 4;

#[derive(Clone, Copy)]
struct ScanMsg {
    seq: u32,
    sent_ns: u64,
    count: u16,
    points: [[f32; 2]; BEAMS],
}

#[derive(Clone, Copy)]
struct TrackMsg {
    seq: u32,
    sent_ns: u64,
    count: u8,
    /// Position and velocity per track: `[x, y, vx, vy]`.
    tracks: [[f32; 4]; MAX_TRACKS],
}

/// What a pipeline run measured.
#[derive(Clone, Debug, PartialEq)]
pub struct PipelineReport {
    /// Frames the consumer received.
    pub frames: u32,
    /// Mean sensor-to-consumer latency per frame, in microseconds.
    pub mean_latency_us: f64,
    /// Worst sensor-to-consumer latency, in microseconds.
    pub max_latency_us: f64,
    /// Frames that arrived out of sequence. Must be zero, since the bus keeps order.
    pub out_of_order: u32,
    /// Tracks held in the last frame the consumer received.
    pub final_tracks: usize,
    /// Pin result of the sensor, perception and consumer threads, in that order.
    pub pinned: [Result<u64, PinError>; 3],
}

/// The scene at time `t`: a box moving along +x at 2 m/s, and a static box.
fn scene(t: f32) -> [Aabb; 2] {
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

fn now_ns(start: Instant) -> u64 {
    start.elapsed().as_nanos() as u64
}

fn send_blocking<T: Copy>(tx: &mut Sender<T, CAPACITY>, mut value: T) {
    loop {
        match tx.try_send(value) {
            Ok(()) => return,
            Err(back) => {
                value = back;
                std::thread::yield_now();
            }
        }
    }
}

fn recv_blocking<T: Copy>(rx: &mut Receiver<T, CAPACITY>) -> T {
    loop {
        if let Some(value) = rx.try_recv() {
            return value;
        }
        std::thread::yield_now();
    }
}

/// Runs `frames` frames of `dt_s` seconds through the three pinned stages.
#[must_use]
pub fn run(frames: u32, dt_s: f32) -> PipelineReport {
    let (mut scan_tx, mut scan_rx) = channel::<ScanMsg, CAPACITY>();
    let (mut track_tx, mut track_rx) = channel::<TrackMsg, CAPACITY>();
    let start = Instant::now();

    let sensor = spawn_pinned(0, move || {
        let mut lidar = Lidar::new(BEAMS, 40.0, 0.02, 0xBEEF);
        for seq in 0..frames {
            let world = scene(seq as f32 * dt_s);
            let mut msg = ScanMsg {
                seq,
                sent_ns: 0,
                count: 0,
                points: [[0.0; 2]; BEAMS],
            };
            let points = lidar.points([0.0, 0.0], &world);
            msg.count = points.len() as u16;
            for (slot, p) in msg.points.iter_mut().zip(points) {
                *slot = p;
            }
            msg.sent_ns = now_ns(start);
            send_blocking(&mut scan_tx, msg);
        }
    });

    let perception = spawn_pinned(1, move || {
        let mut tracker = Tracker::new(Config {
            gate_m: 1.5,
            alpha: 0.5,
            beta: 0.3,
            max_misses: 3,
        });
        for _ in 0..frames {
            let scan = recv_blocking(&mut scan_rx);
            let points = &scan.points[..scan.count as usize];
            let detections: Vec<[f32; 2]> = cluster(points, 0.6)
                .iter()
                .filter(|c| c.points >= 3)
                .map(|c| c.centroid)
                .collect();
            tracker.update(&detections, dt_s);
            let mut out = TrackMsg {
                seq: scan.seq,
                sent_ns: scan.sent_ns,
                count: 0,
                tracks: [[0.0; 4]; MAX_TRACKS],
            };
            for (slot, t) in out
                .tracks
                .iter_mut()
                .zip(tracker.tracks().iter().take(MAX_TRACKS))
            {
                *slot = [t.position[0], t.position[1], t.velocity[0], t.velocity[1]];
                out.count += 1;
            }
            send_blocking(&mut track_tx, out);
        }
    });

    let consumer = spawn_pinned(2, move || {
        let mut latencies_ns: Vec<u64> = Vec::with_capacity(frames as usize);
        let mut last_count = 0;
        let mut out_of_order = 0;
        for expected in 0..frames {
            let msg = recv_blocking(&mut track_rx);
            if msg.seq != expected {
                out_of_order += 1;
            }
            latencies_ns.push(now_ns(start).saturating_sub(msg.sent_ns));
            last_count = msg.count as usize;
        }
        (latencies_ns, last_count, out_of_order)
    });

    let (pin_sensor, ()) = sensor.join().expect("sensor thread finished");
    let (pin_perception, ()) = perception.join().expect("perception thread finished");
    let (pin_consumer, (latencies_ns, final_tracks, out_of_order)) =
        consumer.join().expect("consumer thread finished");

    let total: u64 = latencies_ns.iter().sum();
    let mean_latency_us = total as f64 / frames.max(1) as f64 / 1_000.0;
    let max_latency_us = latencies_ns.iter().copied().max().unwrap_or(0) as f64 / 1_000.0;
    PipelineReport {
        frames: latencies_ns.len() as u32,
        mean_latency_us,
        max_latency_us,
        out_of_order,
        final_tracks,
        pinned: [pin_sensor, pin_perception, pin_consumer],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_frame_crosses_all_three_stages_and_the_scene_is_tracked() {
        let report = run(120, 0.1);
        assert_eq!(report.frames, 120, "{report:?}");
        assert_eq!(report.out_of_order, 0, "{report:?}");
        assert_eq!(report.final_tracks, 2, "{report:?}");
        assert!(report.max_latency_us >= report.mean_latency_us);
        // Pinning is supported on Windows and Linux, and must succeed there. Other
        // platforms report Unsupported, which is not a failure of the pipeline.
        for pin in report.pinned {
            match pin {
                Ok(_) => {}
                Err(PinError::Unsupported) if !cfg!(any(windows, target_os = "linux")) => {}
                Err(other) => panic!("pinning failed: {other:?}"),
            }
        }
    }
}
