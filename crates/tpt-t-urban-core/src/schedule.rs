//! Schedulability check for the thread-per-core layout (spec §4.5, todo Phase 13).
//!
//! Each stage runs on one core at a fixed period, with a worst-case execution
//! time. The check answers two questions for a proposed layout:
//!
//! 1. **Per core**: the sum of `worst_case / period` over the stages pinned to
//!    a core is at most 1. Under earliest-deadline-first scheduling with
//!    implicit deadlines, that is sufficient for every deadline to be met.
//! 2. **Per chain**: the worst-case latency through an ordered chain of stages,
//!    counting one period of waiting per stage, is within the chain's deadline.
//!
//! This is an analytic model. It does not measure a running system. Its numbers
//! must come from measured worst cases on the target, and the spec's pinned
//! layout must be checked against the real ones before sign-off.

/// One periodic stage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stage {
    /// Stage name, for reports.
    pub name: &'static str,
    /// Core the stage is pinned to.
    pub core: u8,
    /// Period in microseconds (e.g. 33_333 for 30 Hz).
    pub period_us: u32,
    /// Measured or budgeted worst-case execution time in microseconds.
    pub worst_case_us: u32,
}

/// Why a layout was rejected.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Violation {
    /// A core's utilization exceeds 1.0. Utilization is in parts per million.
    Overloaded {
        /// The overloaded core.
        core: u8,
        /// Its utilization, in parts per million.
        utilization_ppm: u64,
    },
    /// A chain's worst-case latency exceeds its deadline.
    ChainTooSlow {
        /// Name of the chain that missed its deadline.
        chain: &'static str,
        /// Worst-case end-to-end latency of the chain, in microseconds.
        latency_us: u64,
        /// Deadline of the chain, in microseconds.
        deadline_us: u64,
    },
    /// A stage has a zero period, so its utilization is undefined.
    ZeroPeriod {
        /// The stage whose period is zero.
        stage: &'static str,
    },
}

/// Utilization of `core` in parts per million, from the stages pinned to it.
#[must_use]
pub fn utilization_ppm(stages: &[Stage], core: u8) -> u64 {
    stages
        .iter()
        .filter(|s| s.core == core && s.period_us > 0)
        .map(|s| u64::from(s.worst_case_us) * 1_000_000 / u64::from(s.period_us))
        .sum()
}

/// Checks per-core utilization. Returns every violation found.
#[must_use]
pub fn check_cores(stages: &[Stage]) -> Vec<Violation> {
    let mut out = Vec::new();
    for s in stages {
        if s.period_us == 0 {
            out.push(Violation::ZeroPeriod { stage: s.name });
        }
    }
    let mut cores: Vec<u8> = stages.iter().map(|s| s.core).collect();
    cores.sort_unstable();
    cores.dedup();
    for core in cores {
        let u = utilization_ppm(stages, core);
        if u > 1_000_000 {
            out.push(Violation::Overloaded {
                core,
                utilization_ppm: u,
            });
        }
    }
    out
}

/// Worst-case latency of a chain: each stage waits up to one period for its
/// input, then runs for its worst case. Returns `None` if any stage in the chain
/// is missing from `stages`.
#[must_use]
pub fn chain_latency_us(stages: &[Stage], chain: &[&str]) -> Option<u64> {
    chain.iter().try_fold(0u64, |acc, name| {
        let s = stages.iter().find(|s| s.name == *name)?;
        Some(acc + u64::from(s.period_us) + u64::from(s.worst_case_us))
    })
}

/// Checks a chain against its deadline.
#[must_use]
pub fn check_chain(
    stages: &[Stage],
    chain: &'static str,
    names: &[&str],
    deadline_us: u64,
) -> Option<Violation> {
    let latency_us = chain_latency_us(stages, names)?;
    (latency_us > deadline_us).then_some(Violation::ChainTooSlow {
        chain,
        latency_us,
        deadline_us,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const PERCEPTION_PERIOD: u32 = 33_333; // 30 Hz
    const ACTUATION_PERIOD: u32 = 10_000; // 100 Hz

    fn layout(worst_perception_us: u32) -> Vec<Stage> {
        vec![
            Stage {
                name: "perception",
                core: 1,
                period_us: PERCEPTION_PERIOD,
                worst_case_us: worst_perception_us,
            },
            Stage {
                name: "predict",
                core: 1,
                period_us: PERCEPTION_PERIOD,
                worst_case_us: 2_000,
            },
            Stage {
                name: "plan",
                core: 2,
                period_us: PERCEPTION_PERIOD,
                worst_case_us: 3_000,
            },
            Stage {
                name: "compliance",
                core: 2,
                period_us: PERCEPTION_PERIOD,
                worst_case_us: 1,
            },
            Stage {
                name: "guardian",
                core: 3,
                period_us: ACTUATION_PERIOD,
                worst_case_us: 1,
            },
            Stage {
                name: "actuate",
                core: 3,
                period_us: ACTUATION_PERIOD,
                worst_case_us: 200,
            },
        ]
    }

    #[test]
    fn spec_layout_with_budgeted_times_is_schedulable() {
        assert!(check_cores(&layout(5_000)).is_empty());
    }

    #[test]
    fn an_overloaded_core_is_reported() {
        let v = check_cores(&layout(40_000));
        assert!(
            v.iter()
                .any(|x| matches!(x, Violation::Overloaded { core: 1, .. })),
            "{v:?}"
        );
    }

    #[test]
    fn zero_period_is_reported_not_divided_by() {
        let stages = [Stage {
            name: "broken",
            core: 0,
            period_us: 0,
            worst_case_us: 1,
        }];
        assert!(check_cores(&stages).contains(&Violation::ZeroPeriod { stage: "broken" }));
    }

    #[test]
    fn chain_latency_counts_one_period_per_stage() {
        let stages = layout(5_000);
        let latency = chain_latency_us(&stages, &["perception", "predict"]).unwrap();
        assert_eq!(latency, u64::from(PERCEPTION_PERIOD) * 2 + 5_000 + 2_000);
    }

    #[test]
    fn chain_over_its_deadline_is_reported() {
        let stages = layout(5_000);
        let v = check_chain(
            &stages,
            "perception-to-plan",
            &["perception", "predict", "plan"],
            50_000,
        );
        assert!(matches!(v, Some(Violation::ChainTooSlow { .. })), "{v:?}");
    }

    #[test]
    fn missing_stage_has_no_latency() {
        assert_eq!(chain_latency_us(&layout(1), &["nope"]), None);
    }
}
