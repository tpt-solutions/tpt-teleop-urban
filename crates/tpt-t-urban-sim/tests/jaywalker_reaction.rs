//! A jaywalker, predicted by the IMM filter, stops the bot before the AEB backstop
//! is needed.
//!
//! The bot cruises at 5 m/s along a lane centred on `y = 0`. A pedestrian walks
//! across the lane at 1.2 m/s, crossing it 25 m ahead. The pedestrian is seen with
//! 5 cm noise each 100 ms. The IMM predicts a 3 s cone, the yield controller and
//! the reaction turn that into a deceleration, and the guardian watches the same
//! pedestrian as an obstacle. Its AEB must never engage.
//!
//! The bot's motion is a simple model: it brakes at the commanded rate, and it
//! accelerates to cruise at 1 m/s² when it proceeds.

use tpt_t_urban_delivery::reaction::{STOP_SHORT_M, conflict_from_cone, reaction_decel_mm_s2};
use tpt_t_urban_delivery::yield_ped::{Decision, Pedestrian, Yielder};
use tpt_t_urban_predict::{ConePoint, Imm};
use tpt_t_urban_safety::{Guardian, Inputs, Mode, Obstacle};

const DT_S: f32 = 0.1;
const CRUISE_M_S: f32 = 5.0;
const ACCEL_M_S2: f32 = 1.0;
const CROSS_X_M: f32 = 25.0;
const LANE_HALF_M: f32 = 0.5;
const CONE_STEPS: usize = 30;
const K_SIGMA: f32 = 2.0;

/// Deterministic noise in ±`amplitude` metres, from a linear congruential generator.
struct Noise(u32);

impl Noise {
    fn next(&mut self, amplitude: f32) -> f32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let unit = (self.0 >> 8) as f32 / (1u32 << 24) as f32;
        (unit - 0.5) * 2.0 * amplitude
    }
}

#[test]
fn the_bot_yields_smoothly_and_the_guardian_never_needs_aeb() {
    let mut noise = Noise(0x1234_5678);
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
    let mut max_bot_x_in_lane = 0.0f32;
    let mut saw_yield = false;
    let mut aeb_steps = 0;
    let mut min_decel_while_yielding = u32::MAX;
    let mut max_decel = 0u32;

    for k in 0..140 {
        let t = k as f32 * DT_S;
        let truth = [CROSS_X_M, -6.0 + 1.2 * t];
        let seen = [truth[0] + noise.next(0.05), truth[1] + noise.next(0.05)];

        let filter = imm.get_or_insert_with(|| Imm::new([seen[0], seen[1], 0.0, 0.0], 0.25, 1.0));
        filter.step(seen, DT_S);
        let steps = filter.cone(CONE_STEPS, DT_S, K_SIGMA, &mut cone);

        let conflict = conflict_from_cone(&cone[..steps], bot_x, bot_v, LANE_HALF_M);
        let peds: Vec<Pedestrian> = conflict.iter().map(|c| c.pedestrian).collect();
        let decision = yielder.step(&peds, (DT_S * 1_000.0) as u32);
        saw_yield |= decision == Decision::Yield;

        let speed_mm_s = (bot_v * 1_000.0) as u32;
        let decel_mm_s2 = reaction_decel_mm_s2(decision, conflict, speed_mm_s);
        max_decel = max_decel.max(decel_mm_s2);
        if decision == Decision::Yield {
            min_decel_while_yielding = min_decel_while_yielding.min(decel_mm_s2);
        }

        // The guardian sees the pedestrian as an obstacle when the seen track is in
        // the lane, ahead of the bot.
        let obstacle = (seen[1].abs() <= 1.0 && CROSS_X_M > bot_x).then_some(Obstacle {
            range_mm: ((CROSS_X_M - bot_x) * 1_000.0) as u32,
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

        // The bot's motion: brake at the commanded rate, or accelerate to cruise.
        if decel_mm_s2 > 0 {
            bot_v = (bot_v - decel_mm_s2 as f32 / 1_000.0 * DT_S).max(0.0);
        } else {
            bot_v = (bot_v + ACCEL_M_S2 * DT_S).min(CRUISE_M_S);
        }
        bot_x += bot_v * DT_S;
        if truth[1].abs() <= 1.0 {
            max_bot_x_in_lane = max_bot_x_in_lane.max(bot_x);
        }
    }

    assert!(saw_yield, "the bot never yielded");
    // While the pedestrian is in the lane, the bot must stop before the crossing point.
    assert!(
        max_bot_x_in_lane < CROSS_X_M - STOP_SHORT_M + 0.5,
        "the bot reached {max_bot_x_in_lane} m while the pedestrian was in the lane"
    );
    assert_eq!(aeb_steps, 0, "AEB engaged {aeb_steps} times");
    assert!(
        max_decel <= 3_000,
        "the reaction asked for {max_decel} mm/s², above its cap"
    );
    assert!(
        min_decel_while_yielding > 0,
        "a yield was issued with no deceleration"
    );
    eprintln!(
        "jaywalker: furthest bot position in the lane {max_bot_x_in_lane:.2} m, crossing at {CROSS_X_M} m, peak request {max_decel} mm/s²"
    );
}
