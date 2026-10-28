//! Measures the spec's latency budgets on the host machine.
//!
//! Run in release mode for meaningful numbers:
//!
//! ```text
//! cargo run --release -p tpt-t-urban-sim --example budgets
//! ```
//!
//! These are host-machine figures, not target-hardware measurements. The spec's
//! budgets are for the embedded target (todo.md Phase 18), so a pass here is
//! necessary but not sufficient.

use std::hint::black_box;
use std::time::Instant;

use tpt_t_urban_car::{Limits, Point, speed_profile};
use tpt_t_urban_compliance::{ApproachState, Observation, evaluate};
use tpt_t_urban_predict::Imm;
use tpt_t_urban_predict::imm_batch::ImmBatch;
use tpt_t_urban_privacy::{Frame, Rect, Redactor};
use tpt_t_urban_safety::{Guardian, Inputs, Obstacle};
use tpt_t_urban_v2x::bsm::BsmCoreData;
use tpt_t_urban_v2x::uper::BitReader;

/// Runs `f` for `iters` iterations and returns the mean time per call in nanoseconds.
fn mean_ns(iters: u32, mut f: impl FnMut()) -> f64 {
    // Warm up so the first calls do not dominate.
    for _ in 0..iters / 10 + 1 {
        f();
    }
    let start = Instant::now();
    for _ in 0..iters {
        f();
    }
    start.elapsed().as_nanos() as f64 / f64::from(iters)
}

fn report(name: &str, measured_ns: f64, budget: &str, budget_ns: f64) {
    let verdict = if measured_ns <= budget_ns {
        "PASS"
    } else {
        "FAIL"
    };
    println!(
        "{verdict}  {name:<46} {:>12.3} µs   budget {budget}",
        measured_ns / 1_000.0
    );
}

fn main() {
    println!("Host-machine latency check (not target hardware)\n");

    // V2X: decode one BSM core-data message. An all-zero buffer decodes to the
    // lower bound of every field, which is a valid message.
    let zeros = [0u8; 64];
    let bsm = mean_ns(200_000, || {
        let d = BsmCoreData::decode(&mut BitReader::new(black_box(&zeros)));
        black_box(d.is_ok());
    });
    report("v2x: BSM core-data decode", bsm, "< 10 µs", 10_000.0);

    // Compliance: one stop-sign evaluation. The spec budget covers whole
    // intersection evaluation, so this is a lower bound on the real cost.
    let obs = Observation {
        stop_sign: true,
        distance_to_line_mm: 5_000,
        speed_mm_s: 8_000,
        speed_limit_mm_s: 13_900,
        conflict_on_right: false,
    };
    let comp = mean_ns(2_000_000, || {
        black_box(evaluate(
            black_box(ApproachState::Approaching),
            black_box(&obs),
        ));
    });
    report(
        "compliance: stop-sign evaluation",
        comp,
        "< 50 µs",
        50_000.0,
    );

    // Predict: one IMM step per agent, with 50 agents, against the 2 ms frame budget.
    let mut agents: Vec<Imm> = (0..50)
        .map(|i| Imm::new([i as f32, 0.0, 5.0, 0.0], 0.5, 1.0))
        .collect();
    let frame_ns = mean_ns(2_000, || {
        for (i, imm) in agents.iter_mut().enumerate() {
            imm.step(black_box([i as f32 + 0.5, 0.0]), 0.033);
        }
    });
    report(
        "predict: IMM frame, 50 agents",
        frame_ns,
        "< 2 ms",
        2_000_000.0,
    );

    // The same 50 agents in one structure-of-arrays batch. The batch uses the
    // widest lane the build supports: eight agents per step under `simd`.
    let mut batch = ImmBatch::new();
    for i in 0..50 {
        batch.push([i as f32, 0.0, 5.0, 0.0], 0.5, 1.0);
    }
    let zx: Vec<f32> = (0..50).map(|i| i as f32 + 0.5).collect();
    let zy = vec![0.0_f32; 50];
    let batch_ns = mean_ns(2_000, || {
        batch.step(black_box(&zx), black_box(&zy), 0.033);
    });
    report(
        "predict: batched IMM frame, 50 agents",
        batch_ns,
        "< 2 ms",
        2_000_000.0,
    );

    // 128 agents, the spec's 100+ agent case, in the same batch.
    let mut big = ImmBatch::new();
    for i in 0..128 {
        big.push([i as f32, 0.0, 5.0, 0.0], 0.5, 1.0);
    }
    let zx: Vec<f32> = (0..128).map(|i| i as f32 + 0.5).collect();
    let zy = vec![0.0_f32; 128];
    let big_ns = mean_ns(2_000, || {
        big.step(black_box(&zx), black_box(&zy), 0.033);
    });
    report(
        "predict: batched IMM frame, 128 agents",
        big_ns,
        "< 2 ms",
        2_000_000.0,
    );

    // Safety: one guardian cycle with an obstacle present.
    let mut guardian = Guardian::new();
    let inputs = Inputs {
        obstacle: Some(Obstacle {
            range_mm: 20_000,
            closing_speed_mm_s: 10_000,
        }),
        planned_decel_mm_s2: 0,
        planner_heartbeat_age_ms: 10,
        speed_mm_s: 10_000,
    };
    let guard = mean_ns(5_000_000, || {
        black_box(guardian.step(black_box(&inputs)));
    });
    report(
        "safety: guardian cycle",
        guard,
        "< 50 ms reaction",
        50_000_000.0,
    );

    // Privacy: blur one 200×200 detected region in a 1280×720 frame, the
    // size of a typical face detection, against the 3 ms added-latency budget.
    let (w, h) = (1280, 720);
    let mut pixels = vec![0u8; w * h];
    let region = Rect {
        x: 500,
        y: 250,
        w: 200,
        h: 200,
    };
    let redactor = Redactor::new(4);
    let mut scratch = vec![0u8; Redactor::scratch_len(&[region], w, h)];
    let blur = mean_ns(200, || {
        let frame = Frame::new(&mut pixels, w, h).unwrap();
        let _ = black_box(redactor.redact(frame, &[region], &mut scratch));
    });
    report(
        "privacy: blur 200x200 region, 1280x720",
        blur,
        "< 3 ms",
        3_000_000.0,
    );

    // Car: speed profile over a 200-point path.
    let path: Vec<Point> = (0..200)
        .map(|i| {
            let t = i as f32 * 0.05;
            Point {
                x: 4.0 * t.sin() + t,
                y: 4.0 * (1.0 - t.cos()),
            }
        })
        .collect();
    let limits = Limits {
        speed_limit: 13.9,
        lateral_accel: 2.5,
        accel: 1.5,
        brake: 3.0,
    };
    let mut out = vec![0.0f32; path.len()];
    let profile = mean_ns(20_000, || {
        black_box(speed_profile(black_box(&path), &limits, &mut out));
    });
    report(
        "car: speed profile, 200 points",
        profile,
        "(no spec budget)",
        f64::INFINITY,
    );
}
