//! Journaled crossing runs: guardian interventions and the outcome, written to
//! the durable black-box ledger.
//!
//! Each change of guardian intervention becomes a record, followed by one
//! outcome record at the end. The records are appended in order and synced by
//! [`DurableLedger::append`], so the run can be audited after a restart.

use tpt_t_urban_compliance::{DurableLedger, Record, StoreError};

use crate::scenario::{Outcome, Scenario, run_crossing_traced};

/// Runs the crossing and journals it. Returns the outcome, which the caller can
/// compare with the ledger after a reopen.
///
/// # Errors
///
/// Returns the store error if any record cannot be written and synced.
pub fn run_journaled(s: &Scenario, ledger: &mut DurableLedger) -> Result<Outcome, StoreError> {
    let mut records = Vec::new();
    let mut intervening = false;
    let outcome = run_crossing_traced(s, |p| {
        if p.override_active != intervening {
            intervening = p.override_active;
            records.push(Record::Guardian {
                t_ms: (p.t_s * 1_000.0).round() as u64,
                intervening,
            });
        }
    });
    for record in &records {
        ledger.append(&record.encode())?;
    }
    let outcome_record = Record::Outcome {
        collided: outcome.collided,
        aeb_engaged: outcome.aeb_engaged,
        final_speed_mm_s: (outcome.final_speed_mps.max(0.0) * 1_000.0).round() as u32,
    };
    ledger.append(&outcome_record.encode())?;
    Ok(outcome)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ring::rand::SystemRandom;
    use ring::signature::Ed25519KeyPair;
    use tpt_t_urban_compliance::verify_chain;

    #[test]
    fn a_journaled_run_survives_a_reopen_and_verifies() {
        let path = std::env::temp_dir().join(format!("tpt-sim-journal-{}.log", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let pkcs8 = Ed25519KeyPair::generate_pkcs8(&SystemRandom::new()).expect("keygen");
        let signer = || Ed25519KeyPair::from_pkcs8(pkcs8.as_ref()).expect("pkcs8");

        let outcome = {
            let (mut ledger, _) = DurableLedger::open(&path, signer()).unwrap();
            run_journaled(&Scenario::default(), &mut ledger).unwrap()
        };

        let (ledger, recovery) = DurableLedger::open(&path, signer()).unwrap();
        assert_eq!(recovery.discarded_bytes, 0);
        let entries = ledger.entries();
        assert_eq!(verify_chain(&ledger.public_key(), entries), Ok(()));

        let bodies: Vec<Record> = entries
            .iter()
            .map(|e| Record::decode(&e.body).expect("every body decodes"))
            .collect();
        // The default crossing engages AEB, so the intervention starts and ends.
        assert!(matches!(
            bodies.first(),
            Some(Record::Guardian {
                intervening: true,
                ..
            })
        ));
        let Some(Record::Outcome {
            collided,
            aeb_engaged,
            ..
        }) = bodies.last().copied()
        else {
            panic!("the last record is the outcome");
        };
        assert_eq!(
            (collided, aeb_engaged),
            (outcome.collided, outcome.aeb_engaged)
        );

        let _ = std::fs::remove_file(&path);
    }
}
