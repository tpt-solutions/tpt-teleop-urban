//! Last-mile delivery driven by sensor models, end to end (todo.md Phase 16).
//!
//! The bot drives along a lane at 5 m/s. Along the way:
//!
//! 1. Its GNSS receiver takes a 5 m multipath jump for one second. The selector
//!    switches to LiDAR, and the pose is refined from each scan against a known
//!    map of the static scene (`perception::localize`).
//! 2. A pedestrian walks across the lane 25 m ahead. The pedestrian is observed
//!    with noise, the IMM predicts a cone, and the reaction turns the cone into a
//!    yield and a smooth deceleration. The guardian watches the pedestrian as an
//!    obstacle, and its AEB must never engage.
//! 3. The bot resumes when the lane has been clear for the hold period.
//!
//! Every source switch and yield decision is written to the signed ledger, the
//! chain is verified, and a tampered copy must fail.
//!
//! Then the delivery itself:
//!
//! 4. The camera frame taken as the pedestrian crosses is blurred over the
//!    detected face before it is recorded, and only the detected region changes.
//! 5. The bot creeps to the curb. Its ultrasonic gap is a kinematic model, and the
//!    mount controller must verify the wheel against the curb.
//! 6. The compartment opens for the recipient only after the mount is verified.
//!
//! The sensors are models: LiDAR scans are ray-cast, GNSS is the receiver model,
//! the camera frame is synthetic, the curb gap comes from the bot's own motion,
//! and the pedestrian is a noisy observation of a walker. The approach to the
//! curb is not a real sensor run.

use ring::rand::SystemRandom;
use ring::signature::Ed25519KeyPair;
use tpt_t_urban_compliance::{Ledger, verify_chain};
use tpt_t_urban_delivery::Compartment;
use tpt_t_urban_delivery::compartment::{Lock, hash_code};
use tpt_t_urban_delivery::mount::{Mount, MountAction, MountState, Reading};
use tpt_t_urban_delivery::reaction::{conflict_from_cone, reaction_decel_mm_s2};
use tpt_t_urban_delivery::yield_ped::{Decision, Pedestrian, Yielder};
use tpt_t_urban_perception::gnss::{Source, SourceSelector};
use tpt_t_urban_perception::localize::{Pose2, refine};
use tpt_t_urban_predict::{ConePoint, Imm};
use tpt_t_urban_privacy::{Frame, Rect, Redactor, changed_outside_detections};
use tpt_t_urban_safety::{Guardian, Inputs, Mode, Obstacle};
use tpt_t_urban_sim::lidar::{Aabb, Lidar};
use tpt_t_urban_sim::{Condition, Gnss, Window};

const DT_S: f32 = 0.1;
const DT_MS: u64 = 100;
const CRUISE_M_S: f32 = 5.0;
const ACCEL_M_S2: f32 = 1.0;
const CROSS_X_M: f32 = 25.0;
const LANE_HALF_M: f32 = 0.5;
const CONE_STEPS: usize = 30;
const K_SIGMA: f32 = 2.0;
const STEPS: usize = 120;

/// Static scene: buildings and kerb features off the lane, at varied distances so
/// that scan matching is constrained in every direction.
const STATIC: [Aabb; 5] = [
    Aabb {
        min: [3.0, -6.0],
        max: [4.0, -5.0],
    },
    Aabb {
        min: [10.0, 4.0],
        max: [12.0, 6.0],
    },
    Aabb {
        min: [20.0, -5.0],
        max: [21.0, -4.0],
    },
    Aabb {
        min: [30.0, 5.0],
        max: [33.0, 7.0],
    },
    Aabb {
        min: [38.0, -4.0],
        max: [39.0, -3.0],
    },
];

/// Map points along each static box's outline, 10 cm apart.
fn map() -> Vec<[f32; 2]> {
    let mut out = Vec::new();
    for b in &STATIC {
        let step = 0.1;
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

/// Deterministic noise in ±`amplitude` metres, from a linear congruential generator.
struct Noise(u32);

impl Noise {
    fn next(&mut self, amplitude: f32) -> f32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let unit = (self.0 >> 8) as f32 / (1u32 << 24) as f32;
        (unit - 0.5) * 2.0 * amplitude
    }
}

fn ledger() -> Ledger {
    let pkcs8 = Ed25519KeyPair::generate_pkcs8(&SystemRandom::new()).expect("keygen");
    Ledger::new(Ed25519KeyPair::from_pkcs8(pkcs8.as_ref()).expect("pkcs8"))
}

#[test]
fn a_sensor_driven_delivery_survives_a_gnss_jump_and_a_jaywalker() {
    let windows = [Window {
        start_ms: 1_500,
        end_ms: 2_500,
        condition: Condition::Multipath {
            dx_mm: 5_000,
            dy_mm: 0,
        },
    }];
    let mut gnss = Gnss::new(&windows, 20, 0x5EED_0016);
    let mut selector = SourceSelector::new();
    let mut lidar = Lidar::new(720, 40.0, 0.02, 0x16_16_16_16);
    let map = map();
    let mut noise = Noise(0x2468_ACE0);
    let mut log = ledger();

    let mut imm: Option<Imm> = None;
    let mut cone = [ConePoint {
        t_s: 0.0,
        x: 0.0,
        y: 0.0,
        radius: 0.0,
    }; CONE_STEPS];
    let mut yielder = Yielder::new();
    let mut guardian = Guardian::new();

    let mut bot_x = 0.0f32;
    let mut bot_v = CRUISE_M_S;
    let mut estimate = Pose2 {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    };
    let mut last_source = None;
    let mut last_decision = Decision::Proceed;
    let mut lidar_steps = 0;
    let mut worst_lidar_error_m = 0.0f32;
    let mut max_bot_x_in_lane = 0.0f32;
    let mut aeb_steps = 0;
    let mut saw_yield = false;
    let mut resumed = false;

    for k in 0..STEPS {
        let t_ms = k as u64 * DT_MS;
        let t_s = k as f32 * DT_S;
        let truth_x = bot_x;

        // Localization: GNSS while it is trusted, LiDAR scan matching otherwise.
        let fix = gnss.fix(t_ms, (truth_x * 1_000.0).round() as i64, 0);
        let source = match fix {
            Some(f) => selector.on_gnss_fix(f),
            None => selector.on_gnss_lost(),
        };
        if last_source != Some(source) {
            log.append(format!("t={t_s:.1}s localization {source:?}").as_bytes());
            last_source = Some(source);
        }
        estimate = match (source, fix) {
            (Source::Gnss, Some(f)) => Pose2 {
                x: f.x_mm as f32 / 1_000.0,
                y: f.y_mm as f32 / 1_000.0,
                yaw: 0.0,
            },
            _ => {
                lidar_steps += 1;
                let mut world: Vec<Aabb> = STATIC.to_vec();
                let ped_y = -6.0 + 1.2 * t_s;
                world.push(Aabb {
                    min: [CROSS_X_M - 0.25, ped_y - 0.25],
                    max: [CROSS_X_M + 0.25, ped_y + 0.25],
                });
                let scan: Vec<[f32; 2]> = lidar
                    .points([truth_x, 0.0], &world)
                    .into_iter()
                    .map(|p| [p[0] - truth_x, p[1]])
                    .collect();
                let initial = Pose2 {
                    x: estimate.x + bot_v * DT_S,
                    ..estimate
                };
                let (pose, _) = refine(&map, &scan, initial, 25, 1.0);
                worst_lidar_error_m = worst_lidar_error_m.max((pose.x - truth_x).abs());
                pose
            }
        };

        // Pedestrian: a noisy observation, predicted by the IMM.
        let truth_ped = [CROSS_X_M, -6.0 + 1.2 * t_s];
        let seen = [
            truth_ped[0] + noise.next(0.05),
            truth_ped[1] + noise.next(0.05),
        ];
        let filter = imm.get_or_insert_with(|| Imm::new([seen[0], seen[1], 0.0, 0.0], 0.25, 1.0));
        filter.step(seen, DT_S);
        let steps = filter.cone(CONE_STEPS, DT_S, K_SIGMA, &mut cone);

        // Planning uses the estimated position, not the truth.
        let conflict = conflict_from_cone(&cone[..steps], estimate.x, bot_v, LANE_HALF_M);
        let peds: Vec<Pedestrian> = conflict.iter().map(|c| c.pedestrian).collect();
        let decision = yielder.step(&peds, DT_MS as u32);
        if decision != last_decision {
            log.append(format!("t={t_s:.1}s yield decision {decision:?}").as_bytes());
            if decision == Decision::Yield {
                saw_yield = true;
            }
            if decision == Decision::Proceed && last_decision == Decision::Hold {
                resumed = true;
            }
            last_decision = decision;
        }
        let speed_mm_s = (bot_v * 1_000.0) as u32;
        let decel_mm_s2 = reaction_decel_mm_s2(decision, conflict, speed_mm_s);

        // The guardian sees the pedestrian as an obstacle when the seen track is in
        // the lane and ahead of the estimated position.
        let obstacle = (seen[1].abs() <= 1.0 && CROSS_X_M > estimate.x).then_some(Obstacle {
            range_mm: ((CROSS_X_M - estimate.x) * 1_000.0) as u32,
            closing_speed_mm_s: speed_mm_s as i32,
        });
        let command = guardian.step(&Inputs {
            obstacle,
            planned_decel_mm_s2: decel_mm_s2,
            planner_heartbeat_age_ms: 0,
            speed_mm_s,
        });
        if command.mode == Mode::Aeb {
            aeb_steps += 1;
        }

        // Motion: brake at the commanded rate, or accelerate to cruise.
        if decel_mm_s2 > 0 {
            bot_v = (bot_v - decel_mm_s2 as f32 / 1_000.0 * DT_S).max(0.0);
        } else {
            bot_v = (bot_v + ACCEL_M_S2 * DT_S).min(CRUISE_M_S);
        }
        bot_x += bot_v * DT_S;
        if truth_ped[1].abs() <= 1.0 {
            max_bot_x_in_lane = max_bot_x_in_lane.max(bot_x);
        }
    }

    eprintln!(
        "last-mile: {lidar_steps} LiDAR steps (worst pose error {:.1} mm), furthest in lane {max_bot_x_in_lane:.2} m, AEB steps {aeb_steps}",
        worst_lidar_error_m * 1_000.0
    );

    // Localization: the receiver was denied, LiDAR took over, and GNSS came back.
    assert!(lidar_steps >= 10, "only {lidar_steps} LiDAR steps");
    assert_eq!(last_source, Some(Source::Gnss), "GNSS was not regained");
    assert!(
        worst_lidar_error_m < 0.10,
        "LiDAR pose error {worst_lidar_error_m} m"
    );

    // Jaywalker: the bot yields, stops before the crossing, and resumes.
    assert!(saw_yield, "the bot never yielded");
    assert!(resumed, "the bot did not resume after the lane cleared");
    assert!(
        max_bot_x_in_lane < CROSS_X_M - 1.5,
        "the bot reached {max_bot_x_in_lane} m while the pedestrian was in the lane"
    );
    assert_eq!(aeb_steps, 0, "AEB engaged {aeb_steps} times");

    // Camera: the frame taken as the pedestrian crosses is redacted over the face.
    const W: usize = 64;
    const H: usize = 48;
    let original: Vec<u8> = (0..W * H).map(|i| ((i * 7) % 251) as u8).collect();
    let mut pixels = original.clone();
    let face = Rect {
        x: 28,
        y: 16,
        w: 8,
        h: 16,
    };
    let redactor = Redactor::new(3);
    let mut scratch = vec![0u8; Redactor::scratch_len(&[face], W, H)];
    let frame = Frame::new(&mut pixels, W, H).expect("frame matches its size");
    let redacted = redactor
        .redact(frame, &[face], &mut scratch)
        .expect("face is inside the frame");
    assert!(
        !changed_outside_detections(&original, &redacted, &[face]),
        "redaction touched pixels outside the face"
    );
    assert!(
        (face.y..face.y + face.h)
            .flat_map(|y| (face.x..face.x + face.w).map(move |x| y * W + x))
            .any(|i| redacted.pixels()[i] != original[i]),
        "the face was not blurred"
    );
    log.append(b"camera: face redacted before recording");

    // Curb: the bot creeps in. Its gap and lateral error come from its own motion.
    let mut mount = Mount::new();
    let mut gap_mm = 1_000.0f32;
    let mut lateral_mm = 40.0f32;
    for _ in 0..400 {
        let speed_mm_s = if gap_mm > 300.0 {
            800
        } else if gap_mm > 120.0 {
            300
        } else if gap_mm > 60.0 {
            150
        } else {
            0
        };
        let action = mount.step(Reading {
            gap_mm: gap_mm as u32,
            lateral_mm: lateral_mm as i32,
            speed_mm_s,
        });
        if mount.state() == MountState::Verified || action == MountAction::Stop {
            break;
        }
        gap_mm -= speed_mm_s as f32 * DT_S;
        lateral_mm *= 0.9;
    }
    assert_eq!(mount.state(), MountState::Verified, "gap {gap_mm} mm");
    log.append(b"mount: verified at curb");

    // Compartment: refused before the mount is verified, then opened for the recipient.
    let code = b"4821";
    let mut compartment = Compartment::new(hash_code(code));
    let verified = mount.state() == MountState::Verified;
    assert!(compartment.request_unlock(false, true, code).is_err());
    compartment
        .request_unlock(verified, true, code)
        .expect("verified, at rest, right code");
    assert!(matches!(compartment.lock(), Lock::Open { .. }));
    log.append(b"compartment: unlocked for recipient");

    // Audit: every decision is in the chain, and tampering breaks it.
    let entries = log.entries();
    assert!(entries.len() >= 8, "entries: {}", entries.len());
    let key = log.public_key();
    assert_eq!(verify_chain(&key, entries), Ok(()));
    let mut tampered = entries.to_vec();
    tampered[1].body[0] ^= 1;
    assert!(verify_chain(&key, &tampered).is_err());
}
