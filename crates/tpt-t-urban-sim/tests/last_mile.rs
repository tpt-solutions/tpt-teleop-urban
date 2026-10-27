//! Last-mile delivery data flow across the workspace crates (todo.md Phase 16).
//!
//! The timeline follows spec §6, with the sensing replaced by scripted inputs:
//!
//! 1. Approach a signal: GLOSA advises a speed that reaches the line on green.
//! 2. Reach a stop sign: the compliance FSM stops the bot, then clears it.
//! 3. A pedestrian crosses the path: the bot yields, then proceeds.
//! 4. Park at the curb: the mount sequence verifies the wheel against the curb.
//! 5. Unlock the compartment with the recipient's code.
//!
//! Every decision is written to the signed audit ledger. The test ends by
//! verifying the chain, and checks that a tampered copy of it fails.
//!
//! Sensor-driven versions of these steps are not covered here. This test covers
//! how the crates hand off to each other.

use ring::rand::SystemRandom;
use ring::signature::Ed25519KeyPair;
use tpt_t_urban_car::{Advice, Context, Signal, advise};
use tpt_t_urban_compliance::{
    ApproachState, Decision, Ledger, Observation, evaluate, verify_chain,
};
use tpt_t_urban_delivery::{
    Compartment, Lock, MountAction, MountState, Pedestrian, Yielder, hash_code, mount::Mount,
    mount::Reading,
};

fn ledger() -> Ledger {
    let pkcs8 = Ed25519KeyPair::generate_pkcs8(&SystemRandom::new()).expect("keygen");
    Ledger::new(Ed25519KeyPair::from_pkcs8(pkcs8.as_ref()).expect("pkcs8"))
}

#[test]
fn last_mile_delivery_end_to_end() {
    let mut log = ledger();

    // 1. GLOSA: 100 m from the line at 10 m/s, red in 8 s. At 10 m/s the line is
    //    reached after 10 s, which is after red, so the advisory must speed up
    //    to 12.5 m/s to reach it at red onset.
    let ctx = Context {
        distance_m: 100.0,
        speed_m_s: 10.0,
        speed_limit_m_s: 13.9,
        min_speed_m_s: 1.0,
    };
    let advice = advise(Signal::Green { time_to_red_s: 8.0 }, &ctx);
    let Advice::Speed(v) = advice else {
        panic!("GLOSA should advise a speed, got {advice:?}");
    };
    assert!((v - 12.5).abs() < 1e-3, "advised {v}");
    log.append(format!("glosa: advised {v:.2} m/s").as_bytes());

    // 2. Stop sign: the bot arrives, halts, and waits for an empty right.
    let stop_sign = [
        Observation {
            stop_sign: true,
            distance_to_line_mm: 6_000,
            speed_mm_s: 6_000,
            speed_limit_mm_s: 13_900,
            conflict_on_right: false,
        },
        Observation {
            stop_sign: true,
            distance_to_line_mm: 0,
            speed_mm_s: 0,
            speed_limit_mm_s: 13_900,
            conflict_on_right: false,
        },
        Observation {
            stop_sign: true,
            distance_to_line_mm: 0,
            speed_mm_s: 0,
            speed_limit_mm_s: 13_900,
            conflict_on_right: false,
        },
    ];
    let mut state = ApproachState::Approaching;
    let mut decisions = Vec::new();
    for obs in &stop_sign {
        let eval = evaluate(state, obs);
        log.append(format!("stop-sign: {:?} ({})", eval.decision, eval.reason).as_bytes());
        decisions.push(eval.decision);
        state = eval.next;
    }
    assert_eq!(decisions.last(), Some(&Decision::Proceed));
    assert!(decisions.contains(&Decision::Stop));

    // 3. Pedestrian crosses the path, then clears it.
    let mut yielder = Yielder::new();
    let walker = Pedestrian {
        time_to_path_ms: 1_000,
        lateral_mm: 800,
    };
    let clear = Pedestrian {
        time_to_path_ms: 9_000,
        lateral_mm: 800,
    };
    let first = yielder.step(&[walker], 100);
    log.append(format!("pedestrian: {first:?}").as_bytes());
    assert_eq!(first, tpt_t_urban_delivery::Decision::Yield);
    let mut resumed = false;
    for _ in 0..30 {
        if yielder.step(&[clear], 100) == tpt_t_urban_delivery::Decision::Proceed {
            resumed = true;
            break;
        }
    }
    assert!(resumed, "bot should resume after the path is clear");
    log.append(b"pedestrian: proceed");

    // 4. Park at the curb. The wheel closes on the curb, aligned, at rest.
    let mut mount = Mount::new();
    let approach = [
        Reading {
            gap_mm: 1_000,
            lateral_mm: 0,
            speed_mm_s: 800,
        },
        Reading {
            gap_mm: 280,
            lateral_mm: 5,
            speed_mm_s: 300,
        },
        Reading {
            gap_mm: 120,
            lateral_mm: 3,
            speed_mm_s: 200,
        },
        Reading {
            gap_mm: 50,
            lateral_mm: 2,
            speed_mm_s: 0,
        },
    ];
    let mut actions = Vec::new();
    for r in approach {
        actions.push(mount.step(r));
    }
    assert_eq!(actions.last(), Some(&MountAction::Stop));
    assert_eq!(mount.state(), MountState::Verified);
    log.append(b"mount: verified at curb");

    // 5. Unlock. Before the mount is verified the code must not be checked, so
    //    test the refusal first, then the real unlock.
    let code = b"4821";
    let mut compartment = Compartment::new(hash_code(code));
    let verified = mount.state() == MountState::Verified;
    assert!(compartment.request_unlock(false, true, code).is_err());
    compartment
        .request_unlock(verified, true, code)
        .expect("verified, at rest, right code");
    assert!(matches!(compartment.lock(), Lock::Open { .. }));
    log.append(b"compartment: unlocked for recipient");

    // Audit: the whole sequence is in one verifiable chain.
    let key = log.public_key();
    assert_eq!(verify_chain(&key, log.entries()), Ok(()));
    assert!(log.entries().len() >= 8, "entries: {}", log.entries().len());

    // A tampered copy of the record must fail verification.
    let mut tampered = log.entries().to_vec();
    tampered[3].body = b"stop-sign: Proceed (edited)".to_vec();
    assert!(verify_chain(&key, &tampered).is_err());
}
