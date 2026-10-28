//! Time for one evaluation of the stop-sign FSM, against the 50 µs budget.
//!
//! The budget applies to an optimised build, so the timing assertion runs only in
//! release. Debug builds still run the loop, so the code path is covered either way.

use std::time::Instant;

use tpt_t_urban_compliance::{ApproachState, Observation, evaluate};

#[test]
fn one_evaluation_is_inside_the_fifty_microsecond_budget() {
    const ROUNDS: u32 = 100_000;
    let states = [
        ApproachState::Approaching,
        ApproachState::Halted,
        ApproachState::Clear,
    ];
    let mut proceeds = 0u32;
    let start = Instant::now();
    for i in 0..ROUNDS {
        let obs = Observation {
            stop_sign: i % 3 != 0,
            distance_to_line_mm: (i % 40_000) as i32 - 5_000,
            speed_mm_s: i % 15_000,
            speed_limit_mm_s: 13_900,
            conflict_on_right: i % 5 == 0,
        };
        let eval = evaluate(states[(i % 3) as usize], &obs);
        // Use the result, so the call cannot be optimised away.
        proceeds += u32::from(eval.decision == tpt_t_urban_compliance::Decision::Proceed);
    }
    let elapsed = start.elapsed();
    assert!(proceeds > 0 && proceeds < ROUNDS);

    let mean_us = elapsed.as_secs_f64() * 1e6 / f64::from(ROUNDS);
    if !cfg!(debug_assertions) {
        assert!(mean_us < 50.0, "mean evaluation {mean_us:.3} µs");
    }
    eprintln!("FSM evaluation: mean {mean_us:.4} µs over {ROUNDS} rounds");
}
