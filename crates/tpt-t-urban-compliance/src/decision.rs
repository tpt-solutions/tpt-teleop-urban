//! Stop-sign and speed-limit rules as a deterministic finite state machine.
//!
//! Every decision comes with a fixed, human-readable reason. The reason is
//! what gets recorded in the audit ledger, so an auditor can read why the
//! vehicle yielded or stopped without decoding a model.
//!
//! The rules are a deliberately small subset of traffic law: stop signs,
//! right-of-way at the stop line, and a speed limit. Other rules are not
//! encoded here.

/// What the vehicle must do this step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Decision {
    /// The vehicle may move on.
    Proceed = 0,
    /// Slow down, for an approaching stop line or a speed above the limit.
    Decelerate = 1,
    /// Brake and yield to a vehicle with right-of-way.
    Yield = 2,
    /// Stop at the line.
    Stop = 3,
}

/// Where the vehicle is in the stop-sign sequence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApproachState {
    /// Approaching the stop line and not yet at rest there.
    Approaching,
    /// At rest at the stop line, waiting for right-of-way.
    Halted,
    /// Past the stop sign, or no stop sign applies.
    Clear,
}

/// Observations for one evaluation step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Observation {
    /// Whether a stop sign governs this approach.
    pub stop_sign: bool,
    /// Distance to the stop line in millimetres. Zero or negative means the
    /// vehicle is at or past the line.
    pub distance_to_line_mm: i32,
    /// Current speed in millimetres per second.
    pub speed_mm_s: u32,
    /// Posted limit in millimetres per second.
    pub speed_limit_mm_s: u32,
    /// A vehicle with right-of-way is arriving from the right.
    pub conflict_on_right: bool,
}

/// Result of one evaluation step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Evaluation {
    /// What the vehicle must do this step.
    pub decision: Decision,
    /// Stop-sign state for the next step.
    pub next: ApproachState,
    /// Fixed, human-readable reason, recorded in the ledger.
    pub reason: &'static str,
}

/// Every reason string [`evaluate`] can return, in a fixed order. A ledger record
/// stores the index of its reason here, so the text is recovered from the code.
pub const REASONS: [&str; 9] = [
    "approaching stop line",
    "stopped at stop line",
    "arrived at stop line above zero speed, must stop",
    "yielding to vehicle on right",
    "above the limit at the stop line, not proceeding",
    "right-of-way clear, proceeding",
    "speed above posted limit",
    "no stop control on this approach",
    "stop sign cleared",
];

/// The index of `reason` in [`REASONS`], or `None` if it is not one of them.
#[must_use]
pub fn reason_code(reason: &str) -> Option<u8> {
    REASONS
        .iter()
        .position(|r| *r == reason)
        .and_then(|i| u8::try_from(i).ok())
}

/// The text for a reason code, or `None` if the code is unknown.
#[must_use]
pub fn reason_text(code: u8) -> Option<&'static str> {
    REASONS.get(usize::from(code)).copied()
}

/// Evaluates one step of the stop-sign FSM.
#[must_use]
pub fn evaluate(state: ApproachState, obs: &Observation) -> Evaluation {
    if !obs.stop_sign {
        return clear_or_limit(
            ApproachState::Clear,
            obs,
            "no stop control on this approach",
        );
    }
    match state {
        ApproachState::Approaching if obs.distance_to_line_mm > 0 => Evaluation {
            decision: Decision::Decelerate,
            next: ApproachState::Approaching,
            reason: "approaching stop line",
        },
        ApproachState::Approaching if obs.speed_mm_s == 0 => Evaluation {
            decision: Decision::Stop,
            next: ApproachState::Halted,
            reason: "stopped at stop line",
        },
        ApproachState::Approaching => Evaluation {
            decision: Decision::Stop,
            next: ApproachState::Approaching,
            reason: "arrived at stop line above zero speed, must stop",
        },
        ApproachState::Halted if obs.conflict_on_right => Evaluation {
            decision: Decision::Yield,
            next: ApproachState::Halted,
            reason: "yielding to vehicle on right",
        },
        // Halted but over the limit is an inconsistent reading. Never proceed on it.
        ApproachState::Halted if obs.speed_mm_s > obs.speed_limit_mm_s => Evaluation {
            decision: Decision::Decelerate,
            next: ApproachState::Halted,
            reason: "above the limit at the stop line, not proceeding",
        },
        ApproachState::Halted => Evaluation {
            decision: Decision::Proceed,
            next: ApproachState::Clear,
            reason: "right-of-way clear, proceeding",
        },
        ApproachState::Clear => clear_or_limit(ApproachState::Clear, obs, "stop sign cleared"),
    }
}

/// Speed-limit check applied once no stop-line action is needed.
fn clear_or_limit(
    next: ApproachState,
    obs: &Observation,
    proceed_reason: &'static str,
) -> Evaluation {
    if obs.speed_mm_s > obs.speed_limit_mm_s {
        Evaluation {
            decision: Decision::Decelerate,
            next,
            reason: "speed above posted limit",
        }
    } else {
        Evaluation {
            decision: Decision::Proceed,
            next,
            reason: proceed_reason,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_reason_evaluate_returns_is_in_the_table() {
        // Sweep the inputs that reach each branch, in every state.
        let states = [
            ApproachState::Approaching,
            ApproachState::Halted,
            ApproachState::Clear,
        ];
        let distances = [-500, 0, 1, 20_000];
        let speeds = [0, 5_000, 13_900, 20_000];
        let limits = [13_900, 5_000];
        for state in states {
            for distance in distances {
                for speed in speeds {
                    for limit in limits {
                        for stop_sign in [false, true] {
                            for conflict in [false, true] {
                                let o = Observation {
                                    stop_sign,
                                    distance_to_line_mm: distance,
                                    speed_mm_s: speed,
                                    speed_limit_mm_s: limit,
                                    conflict_on_right: conflict,
                                };
                                let reason = evaluate(state, &o).reason;
                                let code = reason_code(reason)
                                    .unwrap_or_else(|| panic!("untabled reason {reason:?}"));
                                assert_eq!(reason_text(code), Some(reason));
                            }
                        }
                    }
                }
            }
        }
        assert_eq!(reason_text(REASONS.len() as u8), None);
    }

    const LIMIT: u32 = 13_900; // 50 km/h in mm/s

    fn obs(distance_mm: i32, speed: u32, conflict: bool) -> Observation {
        Observation {
            stop_sign: true,
            distance_to_line_mm: distance_mm,
            speed_mm_s: speed,
            speed_limit_mm_s: LIMIT,
            conflict_on_right: conflict,
        }
    }

    #[test]
    fn halted_over_the_limit_does_not_proceed() {
        let eval = evaluate(ApproachState::Halted, &obs(0, LIMIT + 5_000, false));
        assert_eq!(eval.decision, Decision::Decelerate);
        assert_eq!(eval.next, ApproachState::Halted);
    }

    #[test]
    fn full_stop_sign_sequence() {
        let mut state = ApproachState::Approaching;
        let steps = [
            (obs(20_000, 8_000, false), Decision::Decelerate),
            (obs(0, 0, false), Decision::Stop),
            (obs(0, 0, true), Decision::Yield),
            (obs(0, 0, false), Decision::Proceed),
            (obs(-500, 5_000, false), Decision::Proceed),
        ];
        for (input, expected) in steps {
            let eval = evaluate(state, &input);
            assert_eq!(eval.decision, expected, "state {state:?}, obs {input:?}");
            state = eval.next;
        }
        assert_eq!(state, ApproachState::Clear);
    }

    #[test]
    fn arriving_at_the_line_while_moving_forces_a_stop() {
        let eval = evaluate(ApproachState::Approaching, &obs(0, 4_000, false));
        assert_eq!(eval.decision, Decision::Stop);
        assert_eq!(eval.next, ApproachState::Approaching);
    }

    #[test]
    fn speed_above_limit_decelerates_without_a_stop_sign() {
        let input = Observation {
            stop_sign: false,
            speed_mm_s: LIMIT + 1,
            ..obs(-1, 0, false)
        };
        let eval = evaluate(ApproachState::Clear, &input);
        assert_eq!(eval.decision, Decision::Decelerate);
        assert_eq!(eval.reason, "speed above posted limit");
    }

    #[test]
    fn every_decision_carries_a_reason() {
        for state in [
            ApproachState::Approaching,
            ApproachState::Halted,
            ApproachState::Clear,
        ] {
            for distance in [-1, 0, 10_000] {
                for conflict in [false, true] {
                    let eval = evaluate(state, &obs(distance, 0, conflict));
                    assert!(!eval.reason.is_empty());
                }
            }
        }
    }
}
