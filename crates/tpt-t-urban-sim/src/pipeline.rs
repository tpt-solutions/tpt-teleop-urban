//! Four pinned threads passing messages over the core bus: the thread-per-core
//! layout of spec §4.5, run as one program.
//!
//! - Core 0, sensor: ray-casts a LiDAR scan of the scene each frame.
//! - Core 1, perception: clusters the scan, tracks the objects, and predicts each
//!   track's position one second ahead with an IMM filter.
//! - Core 2, planner: measures how long each frame took from the sensor's send,
//!   forwards the tracks to the edge, and runs the car planner and the compliance
//!   FSM. Each frame it sends the edge a heartbeat carrying the steering and the
//!   deceleration it plans.
//! - Core 3, edge: ticks at 100 Hz. Runs the safety guardian over the latest
//!   track and the heartbeat, applies the limiter and watchdog to the commands,
//!   and measures how long each track took from the scan to the edge.
//!
//! The bus carries fixed-size `Copy` messages and never blocks. Each sending
//! thread spins with `try_send` and yields between attempts. The edge keeps the
//! newest track and heartbeat it has received, so a burst within one tick is
//! reduced to the newest.
//!
//! The ego is not moved by this pipeline. It holds one pose and a constant speed,
//! so the compliance FSM sees one approach at each frame and does not run through
//! its stop sequence. The ledger is not written.
//!
//! Host timing only, and pinning is reported, not enforced by the test. The
//! target-hardware latency budget is a separate check.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use tpt_t_urban_car::{Limits, Point, Pose, pure_pursuit, speed_profile};
use tpt_t_urban_compliance::{ApproachState, Decision, Observation, Record, evaluate, reason_code};
use tpt_t_urban_core::affinity::{PinError, spawn_pinned};
use tpt_t_urban_core::bus::{Receiver, Sender, channel};
use tpt_t_urban_edge::{Limiter, Watchdog, WatchdogState};
use tpt_t_urban_perception::detect::cluster;
use tpt_t_urban_perception::track::{Config, Tracker};
use tpt_t_urban_predict::{ConePoint, Imm};
use tpt_t_urban_safety::{AEB_DECEL_MM_S2, Guardian, Inputs, Mode, Obstacle};

use crate::lidar::{Aabb, Lidar};

/// Beams in each scan. Also the most points a scan message carries.
pub const BEAMS: usize = 720;
/// Most tracks a track message carries.
pub const MAX_TRACKS: usize = 8;
/// Bus capacity in messages. A power of two, as the ring requires.
const CAPACITY: usize = 4;
/// How far ahead each track is predicted, in seconds.
const HORIZON_S: f32 = 1.0;
/// Edge actuation period: 100 Hz, as spec §4.5 puts core 3.
const EDGE_TICK: Duration = Duration::from_millis(10);
/// The edge's watchdog expires this long after the last heartbeat. A healthy
/// planner sends one per frame.
const WATCHDOG_MS: u32 = 150;
/// Ego speed, in mm/s (10 m/s). Held constant.
const EGO_SPEED_MM_S: u32 = 10_000;
/// Half-width of the ego lane in metres. A track inside it and ahead of the ego is
/// in path.
const LANE_HALF_M: f32 = 3.0;
/// Throttle while nothing is planned to brake, in parts per thousand.
const CRUISE_PERMILLE: u16 = 300;

/// Ego lateral offset from the lane centre, in metres. Pure pursuit steers it back.
const EGO_OFFSET_M: f32 = -0.5;
/// Distance from the ego to the stop line, in metres.
const STOP_LINE_M: f32 = 40.0;
/// Length of the lane path, in metres, and the spacing of its points.
const PATH_END_M: f32 = 60.0;
const PATH_STEP_M: f32 = 0.5;
/// Pure pursuit lookahead and wheelbase, in metres.
const LOOKAHEAD_M: f32 = 5.0;
const WHEELBASE_M: f32 = 2.7;
/// Speed limits for the profile, in m/s and m/s².
const SPEED_LIMIT_M_S: f32 = 10.0;
const LATERAL_ACCEL_M_S2: f32 = 2.0;
const ACCEL_M_S2: f32 = 2.0;
const BRAKE_M_S2: f32 = 3.0;

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
    /// Per track: `[x, y, vx, vy, x_pred, y_pred]`, where the prediction is the
    /// position `HORIZON_S` seconds ahead.
    tracks: [[f32; 6]; MAX_TRACKS],
}

/// The planner's output for one frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct PlanMsg {
    seq: u32,
    /// Deceleration the planner asks for, in mm/s². Zero when it proceeds.
    planned_decel_mm_s2: u32,
    /// Steering angle the planner asks for, in centidegrees, signed.
    steer_cdeg: i32,
}

/// What a pipeline run measured.
#[derive(Clone, Debug, PartialEq)]
pub struct PipelineReport {
    /// Frames the planner received.
    pub frames: u32,
    /// Mean sensor-to-planner latency per frame, in microseconds.
    pub mean_latency_us: f64,
    /// Worst sensor-to-planner latency, in microseconds.
    pub max_latency_us: f64,
    /// Frames that arrived out of sequence. Must be zero, since the bus keeps order.
    pub out_of_order: u32,
    /// Tracks held in the last frame the planner received.
    pub final_tracks: usize,
    /// Worst distance, in metres, from a track's one-second prediction in the last
    /// frame to the nearest object's true centre one second later. The track sits
    /// on the visible face, so about 0.5 m of this is the centroid offset.
    pub prediction_error_m: f32,
    /// Largest deceleration the planner asked for, in mm/s².
    pub max_planned_decel_mm_s2: u32,
    /// Edge ticks on which the guardian was in AEB.
    pub aeb_ticks: u32,
    /// Largest steering command the edge applied, in centidegrees.
    pub max_steer_cdeg: i32,
    /// Mean sensor-to-actuator latency per track, in microseconds: from the scan
    /// being sent to the edge applying the command that track produced.
    pub mean_actuation_us: f64,
    /// Worst sensor-to-actuator latency, in microseconds.
    pub max_actuation_us: f64,
    /// Highest brake command the edge applied, in parts per thousand.
    pub max_brake_permille: u16,
    /// Edge ticks run.
    pub edge_ticks: u32,
    /// Worst lateness of an edge tick against its 100 Hz release, in microseconds.
    pub max_tick_overrun_us: f64,
    /// Ledger records of the run, in time order: each change of the FSM decision and
    /// of the guardian's intervention.
    pub events: Vec<Record>,
    /// Pin result of the sensor, perception, planner and edge threads, in that order.
    pub pinned: [Result<u64, PinError>; 4],
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

/// The nearest track ahead of the ego and inside its lane, as the guardian sees it.
fn nearest_in_lane(msg: &TrackMsg) -> Option<Obstacle> {
    msg.tracks[..msg.count as usize]
        .iter()
        .filter(|t| t[0] > 0.0 && t[1].abs() < LANE_HALF_M)
        .map(|t| Obstacle {
            range_mm: (t[0] * 1_000.0) as u32,
            closing_speed_mm_s: (EGO_SPEED_MM_S as f32 - t[2] * 1_000.0) as i32,
        })
        .min_by_key(|o| o.range_mm)
}

/// The lane centre line, from the ego's start to the end of the path.
fn lane_path() -> Vec<Point> {
    let steps = (PATH_END_M / PATH_STEP_M).round() as usize;
    (0..=steps)
        .map(|i| Point {
            x: i as f32 * PATH_STEP_M,
            y: 0.0,
        })
        .collect()
}

/// Deceleration in mm/s² that stops a vehicle at `speed_mm_s` in `distance_mm`.
fn stopping_decel_mm_s2(speed_mm_s: u32, distance_mm: i32) -> u32 {
    let v = speed_mm_s as f32 / 1_000.0;
    let d = (distance_mm as f32 / 1_000.0).max(0.1);
    (v * v / (2.0 * d) * 1_000.0) as u32
}

fn mean_us(values_ns: &[u64]) -> f64 {
    let total: u64 = values_ns.iter().sum();
    total as f64 / values_ns.len().max(1) as f64 / 1_000.0
}

fn max_us(values_ns: &[u64]) -> f64 {
    values_ns.iter().copied().max().unwrap_or(0) as f64 / 1_000.0
}

/// Runs `frames` frames of `dt_s` seconds through the four pinned stages, as fast
/// as the stages allow.
#[must_use]
pub fn run(frames: u32, dt_s: f32) -> PipelineReport {
    run_with(frames, dt_s, None)
}

/// Runs the pipeline with the sensor released every `period`, as a real LiDAR
/// would. The report then shows whether the stages keep up with that rate.
#[must_use]
pub fn run_paced(frames: u32, dt_s: f32, period: Duration) -> PipelineReport {
    run_with(frames, dt_s, Some(period))
}

fn run_with(frames: u32, dt_s: f32, period: Option<Duration>) -> PipelineReport {
    let (mut scan_tx, mut scan_rx) = channel::<ScanMsg, CAPACITY>();
    let (mut track_tx, mut track_rx) = channel::<TrackMsg, CAPACITY>();
    let (mut edge_track_tx, mut edge_track_rx) = channel::<TrackMsg, CAPACITY>();
    let (mut plan_tx, mut plan_rx) = channel::<PlanMsg, CAPACITY>();
    // Set by the planner after its last frame. The edge runs until it sees it.
    let done = Arc::new(AtomicBool::new(false));
    let planner_done = Arc::clone(&done);
    let start = Instant::now();

    let sensor = spawn_pinned(0, move || {
        let mut lidar = Lidar::new(BEAMS, 40.0, 0.02, 0xBEEF);
        for seq in 0..frames {
            if let Some(period) = period {
                // Release frame `seq` at `seq * period` after the start, so a slow
                // stage cannot push the release times back.
                let release = period * seq;
                if let Some(wait) = release.checked_sub(start.elapsed()) {
                    std::thread::sleep(wait);
                }
            }
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
        // One IMM per live track, keyed by the tracker's stable id.
        let mut predictors: HashMap<u32, Imm> = HashMap::new();
        let horizon_steps = (HORIZON_S / dt_s).round() as usize;
        let mut cone = [ConePoint {
            t_s: 0.0,
            x: 0.0,
            y: 0.0,
            radius: 0.0,
        }; 64];
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
                tracks: [[0.0; 6]; MAX_TRACKS],
            };
            for (slot, t) in out
                .tracks
                .iter_mut()
                .zip(tracker.tracks().iter().take(MAX_TRACKS))
            {
                let imm = predictors.entry(t.id).or_insert_with(|| {
                    Imm::new([t.position[0], t.position[1], 0.0, 0.0], 1.0, 4.0)
                });
                imm.step(t.position, dt_s);
                let steps = imm.cone(horizon_steps, dt_s, 0.0, &mut cone);
                let ahead = cone[steps - 1];
                *slot = [
                    t.position[0],
                    t.position[1],
                    t.velocity[0],
                    t.velocity[1],
                    ahead.x,
                    ahead.y,
                ];
                out.count += 1;
            }
            predictors.retain(|id, _| tracker.tracks().iter().any(|t| t.id == *id));
            send_blocking(&mut track_tx, out);
        }
    });

    let planner = spawn_pinned(2, move || {
        let path = lane_path();
        let limits = Limits {
            speed_limit: SPEED_LIMIT_M_S,
            lateral_accel: LATERAL_ACCEL_M_S2,
            accel: ACCEL_M_S2,
            brake: BRAKE_M_S2,
        };
        let mut speeds = vec![0.0; path.len()];
        speed_profile(&path, &limits, &mut speeds);
        let ego = Pose {
            x: 0.0,
            y: EGO_OFFSET_M,
            yaw: 0.0,
        };
        // A fixed pose, so the steering and the limit are the same every frame.
        let steer_rad = pure_pursuit(ego, &path, LOOKAHEAD_M, WHEELBASE_M).unwrap_or(0.0);
        let steer_cdeg = (steer_rad.to_degrees() * 100.0).round() as i32;
        // The posted limit where the ego is, from the profile, in mm/s.
        let limit_mm_s = (speeds[0] * 1_000.0) as u32;

        let mut latencies_ns: Vec<u64> = Vec::with_capacity(frames as usize);
        let mut last = None;
        let mut out_of_order = 0;
        let mut compliance = ApproachState::Approaching;
        let mut max_planned_decel = 0;
        let mut events: Vec<Record> = Vec::new();
        let mut last_decision: Option<(Decision, &'static str)> = None;
        for expected in 0..frames {
            let msg = recv_blocking(&mut track_rx);
            if msg.seq != expected {
                out_of_order += 1;
            }
            latencies_ns.push(now_ns(start).saturating_sub(msg.sent_ns));
            send_blocking(&mut edge_track_tx, msg);

            let distance_mm = ((STOP_LINE_M - ego.x) * 1_000.0) as i32;
            let eval = evaluate(
                compliance,
                &Observation {
                    stop_sign: true,
                    distance_to_line_mm: distance_mm,
                    speed_mm_s: EGO_SPEED_MM_S,
                    speed_limit_mm_s: limit_mm_s,
                    conflict_on_right: false,
                },
            );
            compliance = eval.next;
            let t_ms = now_ns(start) / 1_000_000;
            if last_decision != Some((eval.decision, eval.reason)) {
                last_decision = Some((eval.decision, eval.reason));
                let reason = reason_code(eval.reason).expect("every FSM reason is tabled");
                events.push(Record::Decision {
                    t_ms,
                    decision: eval.decision,
                    reason,
                });
            }
            let planned_decel_mm_s2 = match eval.decision {
                Decision::Proceed => 0,
                Decision::Decelerate => stopping_decel_mm_s2(EGO_SPEED_MM_S, distance_mm),
                Decision::Yield | Decision::Stop => (BRAKE_M_S2 * 1_000.0) as u32,
            };
            max_planned_decel = max_planned_decel.max(planned_decel_mm_s2);
            send_blocking(
                &mut plan_tx,
                PlanMsg {
                    seq: msg.seq,
                    planned_decel_mm_s2,
                    steer_cdeg,
                },
            );
            last = Some(msg);
        }
        planner_done.store(true, Ordering::Release);
        (latencies_ns, last, out_of_order, max_planned_decel, events)
    });

    let edge = spawn_pinned(3, move || {
        let mut guardian = Guardian::new();
        let mut limiter = Limiter::new();
        let mut watchdog = Watchdog::new(WATCHDOG_MS);
        let mut latest: Option<TrackMsg> = None;
        let mut plan = PlanMsg::default();
        let mut last_heartbeat = Instant::now();
        let mut actuation_ns: Vec<u64> = Vec::new();
        let mut max_brake_permille = 0;
        let mut max_steer_cdeg = 0;
        let mut aeb_ticks = 0;
        let mut events: Vec<Record> = Vec::new();
        let mut intervening = false;
        let mut ticks = 0;
        let mut max_overrun_ns = 0;
        let mut release = Instant::now();
        loop {
            release += EDGE_TICK;
            std::thread::sleep(release.saturating_duration_since(Instant::now()));
            let overrun = Instant::now().saturating_duration_since(release);
            max_overrun_ns = max_overrun_ns.max(overrun.as_nanos() as u64);
            ticks += 1;

            // Load the flag before draining. Everything the planner sent before
            // setting it is then in the rings when we drain.
            let finished = done.load(Ordering::Acquire);
            let mut fresh = None;
            while let Some(msg) = edge_track_rx.try_recv() {
                latest = Some(msg);
                fresh = Some(msg);
            }
            while let Some(msg) = plan_rx.try_recv() {
                plan = msg;
                last_heartbeat = Instant::now();
                watchdog.feed();
            }
            if let Some(msg) = fresh {
                actuation_ns.push(now_ns(start).saturating_sub(msg.sent_ns));
            }

            let command = guardian.step(&Inputs {
                obstacle: latest.as_ref().and_then(nearest_in_lane),
                planned_decel_mm_s2: plan.planned_decel_mm_s2,
                planner_heartbeat_age_ms: last_heartbeat.elapsed().as_millis() as u32,
                speed_mm_s: EGO_SPEED_MM_S,
            });
            if command.mode == Mode::Aeb {
                aeb_ticks += 1;
            }
            if (command.mode != Mode::Nominal) != intervening {
                intervening = command.mode != Mode::Nominal;
                events.push(Record::Guardian {
                    t_ms: now_ns(start) / 1_000_000,
                    intervening,
                });
            }
            let actuation = match watchdog.tick(EDGE_TICK.as_millis() as u32) {
                WatchdogState::Expired => limiter.emergency_brake(),
                WatchdogState::Healthy => {
                    // The stronger of the guardian's and the planner's deceleration.
                    let decel = command.decel_mm_s2.max(plan.planned_decel_mm_s2);
                    let brake = (decel * 1_000 / AEB_DECEL_MM_S2).min(1_000) as u16;
                    let throttle = if brake > 0 { 0 } else { CRUISE_PERMILLE };
                    limiter.apply(plan.steer_cdeg, brake, throttle)
                }
            };
            max_brake_permille = max_brake_permille.max(actuation.brake_permille);
            max_steer_cdeg = max_steer_cdeg.max(actuation.steer_cdeg);
            if finished {
                break;
            }
        }
        (
            actuation_ns,
            max_brake_permille,
            max_steer_cdeg,
            aeb_ticks,
            ticks,
            max_overrun_ns,
            events,
        )
    });

    let (pin_sensor, ()) = sensor.join().expect("sensor thread finished");
    let (pin_perception, ()) = perception.join().expect("perception thread finished");
    let (pin_planner, (latencies_ns, last, out_of_order, max_planned_decel_mm_s2, mut events)) =
        planner.join().expect("planner thread finished");
    let (
        pin_edge,
        (
            actuation_ns,
            max_brake_permille,
            max_steer_cdeg,
            aeb_ticks,
            edge_ticks,
            max_overrun_ns,
            edge_events,
        ),
    ) = edge.join().expect("edge thread finished");

    events.extend(edge_events);
    events.sort_by_key(record_time);

    let (final_tracks, prediction_error_m) = match last {
        Some(msg) => {
            let truth_t = (frames - 1) as f32 * dt_s + HORIZON_S;
            (msg.count as usize, prediction_error(&msg, &scene(truth_t)))
        }
        None => (0, 0.0),
    };
    PipelineReport {
        frames: latencies_ns.len() as u32,
        mean_latency_us: mean_us(&latencies_ns),
        max_latency_us: max_us(&latencies_ns),
        out_of_order,
        final_tracks,
        prediction_error_m,
        max_planned_decel_mm_s2,
        aeb_ticks,
        max_steer_cdeg,
        mean_actuation_us: mean_us(&actuation_ns),
        max_actuation_us: max_us(&actuation_ns),
        max_brake_permille,
        edge_ticks,
        max_tick_overrun_us: max_overrun_ns as f64 / 1_000.0,
        events,
        pinned: [pin_sensor, pin_perception, pin_planner, pin_edge],
    }
}

/// Time of a record in milliseconds, for ordering a run's events.
fn record_time(record: &Record) -> u64 {
    match *record {
        Record::Guardian { t_ms, .. } | Record::Decision { t_ms, .. } => t_ms,
        Record::Outcome { .. } => 0,
    }
}

/// Worst distance from any track's predicted position to the nearest object's
/// centre in `truth`.
fn prediction_error(msg: &TrackMsg, truth: &[Aabb; 2]) -> f32 {
    let centres = truth.map(|b| [(b.min[0] + b.max[0]) / 2.0, (b.min[1] + b.max[1]) / 2.0]);
    msg.tracks[..msg.count as usize]
        .iter()
        .map(|t| {
            centres
                .iter()
                .map(|c| ((t[4] - c[0]).powi(2) + (t[5] - c[1]).powi(2)).sqrt())
                .fold(f32::INFINITY, f32::min)
        })
        .fold(0.0, f32::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_frame_crosses_all_four_stages_and_the_scene_is_tracked() {
        let report = run(120, 0.1);
        assert_eq!(report.frames, 120, "{report:?}");
        assert_eq!(report.out_of_order, 0, "{report:?}");
        assert_eq!(report.final_tracks, 2, "{report:?}");
        assert!(report.max_latency_us >= report.mean_latency_us);
        assert!(report.max_actuation_us >= report.mean_actuation_us);
        // The moving box is 10 m ahead in the lane and closing at 8 m/s, so the
        // guardian must reach AEB and the edge must apply the brake.
        assert!(report.aeb_ticks > 0, "{report:?}");
        assert_eq!(report.max_brake_permille, 1_000, "{report:?}");
        // The ego sits 0.5 m right of the lane centre, so pure pursuit steers left,
        // which is positive. The edge must apply that steering.
        assert!(report.max_steer_cdeg > 0, "{report:?}");
        // The stop line is 40 m ahead at 10 m/s, so the FSM asks to stop at
        // v²/2d = 1.25 m/s².
        assert_eq!(report.max_planned_decel_mm_s2, 1_250, "{report:?}");
        // The tracked position is the centroid of the visible LiDAR points, which lie on
        // the sensor-facing face of each 1 m box, about 0.5 m from its centre. That is
        // a floor the prediction cannot beat, so the bound allows for it.
        assert!(report.prediction_error_m < 1.0, "{report:?}");
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

    #[test]
    fn keeps_up_with_a_30_hz_sensor() {
        // At 30 Hz a frame is 33.3 ms old by the next release. If a stage fell
        // behind, the latencies would keep growing and the mean would pass a period.
        let period = Duration::from_micros(33_333);
        let report = run_paced(60, 1.0 / 30.0, period);
        assert_eq!(report.frames, 60, "{report:?}");
        assert_eq!(report.out_of_order, 0, "{report:?}");
        // A single scheduling spike on this host reaches tens of milliseconds
        // under load, so the worst case is reported, not asserted. Falling behind
        // would raise the mean, since every later frame would queue behind the
        // slow one, so the mean must stay under one period.
        assert!(
            report.mean_latency_us < period.as_micros() as f64,
            "{report:?}"
        );
        assert!(
            report.mean_actuation_us < period.as_micros() as f64,
            "{report:?}"
        );
    }
}
