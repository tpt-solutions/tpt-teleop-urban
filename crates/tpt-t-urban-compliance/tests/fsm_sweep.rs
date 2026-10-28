//! Seeded sweep of the stop-sign FSM over random observation sequences.
//!
//! Each sequence runs the FSM from `Approaching` with random observations, and
//! every step is checked against the rules the FSM must keep. The sequences are
//! random, so some of them are physically impossible, such as stopped with
//! speed above the limit. The invariants are stated so that holds for any input.

use tpt_t_urban_compliance::{ApproachState, Decision, Observation, evaluate};

const LIMIT_MM_S: u32 = 13_900;

/// xorshift32 stream, mapped to `[0, 1)`.
struct Rng(u32);

impl Rng {
    fn unit(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0 as f32 / (u32::MAX as f32 + 1.0)
    }

    fn observation(&mut self) -> Observation {
        Observation {
            stop_sign: self.unit() < 0.9,
            distance_to_line_mm: (self.unit() * 40_000.0) as i32 - 5_000,
            // A fifth of the draws are standstill, so the FSM reaches `Halted`.
            speed_mm_s: if self.unit() < 0.2 { 0 } else { (self.unit() * 25_000.0) as u32 },
            speed_limit_mm_s: LIMIT_MM_S,
            conflict_on_right: self.unit() < 0.3,
        }
    }
}

/// Checks one step. Returns a description of the first broken rule, if any.
fn violation(state: ApproachState, obs: &Observation, decision: Decision) -> Option<&'static str> {
    if obs.stop_sign && state == ApproachState::Approaching {
        if obs.distance_to_line_mm > 0 && decision != Decision::Decelerate {
            return Some("approaching a stop line without slowing down");
        }
        if obs.distance_to_line_mm <= 0 && obs.speed_mm_s > 0 && decision != Decision::Stop {
            return Some("reached the line still moving and did not stop");
        }
    }
    if obs.stop_sign && state == ApproachState::Halted && obs.conflict_on_right {
        if decision != Decision::Yield {
            return Some("halted with a vehicle on the right and did not yield");
        }
    }
    if decision == Decision::Proceed && obs.speed_mm_s > obs.speed_limit_mm_s {
        return Some("proceeded above the speed limit");
    }
    None
}

#[test]
fn the_fsm_keeps_its_rules_over_random_sequences() {
    let mut rng = Rng(0x5EED_CAFE);
    let mut steps = 0u64;
    let mut failures: Vec<(u32, &'static str)> = Vec::new();
    let mut visited = [false; 3];
    let mut decided = [false; 4];
    for run in 0..5_000 {
        let mut state = ApproachState::Approaching;
        for _ in 0..40 {
            let obs = rng.observation();
            let eval = evaluate(state, &obs);
            visited[state as usize] = true;
            decided[eval.decision as usize] = true;
            steps += 1;
            if let Some(rule) = violation(state, &obs, eval.decision) {
                failures.push((run, rule));
            }
            state = eval.next;
        }
    }
    // Coverage: every approach state and every decision must actually occur, or
    // the sweep is not testing the rules it claims to.
    assert!(visited.iter().all(|&v| v), "not every state was reached: {visited:?}");
    assert!(decided.iter().all(|&v| v), "not every decision was reached: {decided:?}");
    assert!(
        failures.is_empty(),
        "{} of {steps} steps broke a rule, first: {:?}",
        failures.len(),
        failures.first(),
    );
}
