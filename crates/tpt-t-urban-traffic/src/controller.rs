//! Signal phase controller with emergency-vehicle preemption.
//!
//! Each phase runs green, then yellow, then all-red before the next phase. The
//! all-red interval lets the intersection clear before the conflicting phase
//! starts. Because the controller is a single state, only one phase is ever
//! green, and no phase change skips yellow or all-red.
//!
//! Preemption (SRM from an emergency vehicle, spec §5.2) takes the same
//! yellow and all-red path to the requested phase. It holds that phase green
//! while the request stays active, then resumes the normal cycle.
//!
//! Times are milliseconds. The caller advances the controller with `step`.

/// Largest number of phases the controller supports.
pub const MAX_PHASES: usize = 8;
/// Yellow interval, ms.
pub const YELLOW_MS: u32 = 3_000;
/// All-red clearance interval, ms.
pub const ALL_RED_MS: u32 = 1_000;

/// What a phase shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Interval {
    Green,
    Yellow,
    AllRed,
}

/// Output for this cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Output {
    /// Phase index that owns the current interval.
    pub phase: usize,
    pub interval: Interval,
    /// Whether the controller is serving a preemption request.
    pub preempted: bool,
}

/// Signal controller over `N` phases with fixed green times.
#[derive(Clone, Copy, Debug)]
pub struct Controller<const N: usize> {
    green_ms: [u32; N],
    current: usize,
    interval: Interval,
    elapsed_ms: u32,
    /// Phase requested by an active preemption, if any.
    preempt_target: Option<usize>,
    /// Set when the controller is in a yellow/all-red transition toward a
    /// preempted phase rather than the next phase in the cycle.
    transitioning_to_preempt: bool,
    /// Green times queued by [`Controller::set_greens`], applied at the next phase start.
    pending_greens: Option<[u32; N]>,
}

impl<const N: usize> Controller<N> {
    /// Creates a controller starting phase 0 green. `N` must be 1 to `MAX_PHASES`.
    ///
    /// # Panics
    ///
    /// Panics if `N` is zero or larger than [`MAX_PHASES`].
    #[must_use]
    pub fn new(green_ms: [u32; N]) -> Self {
        assert!((1..=MAX_PHASES).contains(&N), "phase count out of range");
        Self {
            green_ms,
            current: 0,
            interval: Interval::Green,
            elapsed_ms: 0,
            preempt_target: None,
            transitioning_to_preempt: false,
            pending_greens: None,
        }
    }

    /// Queues new green times, typically from
    /// [`crate::optimize::allocate_greens`]. They apply when the next phase
    /// starts, so the phase running now keeps the green it started with.
    pub fn set_greens(&mut self, green_ms: [u32; N]) {
        self.pending_greens = Some(green_ms);
    }

    /// Advances by `dt_ms`. `preempt` is the phase an emergency vehicle requests,
    /// or `None` when no request is active.
    pub fn step(&mut self, dt_ms: u32, preempt: Option<usize>) -> Output {
        self.preempt_target = preempt.filter(|&p| p < N);
        self.elapsed_ms = self.elapsed_ms.saturating_add(dt_ms);
        loop {
            let duration = match self.interval {
                Interval::Green => {
                    let target_wanted = self.preempt_target.is_some_and(|t| t != self.current);
                    if target_wanted {
                        // Leave early: a preempted phase needs the conflicting one stopped first.
                        Some(0)
                    } else {
                        Some(self.green_ms[self.current])
                    }
                }
                Interval::Yellow => Some(YELLOW_MS),
                Interval::AllRed => Some(ALL_RED_MS),
            };
            let Some(limit) = duration else { break };
            if self.elapsed_ms < limit {
                break;
            }
            self.elapsed_ms -= limit;
            self.advance();
        }
        Output {
            phase: self.current,
            interval: self.interval,
            preempted: self.transitioning_to_preempt || self.preempt_target == Some(self.current),
        }
    }

    /// Moves to the next interval in the sequence.
    fn advance(&mut self) {
        self.interval = match self.interval {
            Interval::Green => Interval::Yellow,
            Interval::Yellow => Interval::AllRed,
            Interval::AllRed => {
                self.current = match self.preempt_target {
                    Some(t) => {
                        self.transitioning_to_preempt = true;
                        t
                    }
                    None => {
                        self.transitioning_to_preempt = false;
                        (self.current + 1) % N
                    }
                };
                Interval::Green
            }
        };
        if self.interval == Interval::Green {
            // Queued greens take effect at a phase start, never mid-green.
            if let Some(green_ms) = self.pending_greens.take() {
                self.green_ms = green_ms;
            }
            // Once the preempted phase is green, its request keeps it there.
            if self.preempt_target == Some(self.current) {
                self.transitioning_to_preempt = false;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run<const N: usize>(
        c: &mut Controller<N>,
        steps: u32,
        dt: u32,
        preempt: impl Fn(u32) -> Option<usize>,
    ) -> Vec<Output> {
        (0..steps).map(|i| c.step(dt, preempt(i))).collect()
    }

    #[test]
    fn normal_cycle_visits_each_phase_in_order_with_clearance() {
        let mut c = Controller::new([5_000, 4_000, 6_000]);
        let outs = run(&mut c, 10_000, 100, |_| None);
        // Phases seen in order, with yellow and all-red between each.
        let mut seen = Vec::new();
        for o in &outs {
            if seen.last() != Some(&(o.phase, o.interval)) {
                seen.push((o.phase, o.interval));
            }
        }
        // The controller keeps cycling, so check the first full sequence only.
        let expected = [
            (0, Interval::Green),
            (0, Interval::Yellow),
            (0, Interval::AllRed),
            (1, Interval::Green),
            (1, Interval::Yellow),
            (1, Interval::AllRed),
            (2, Interval::Green),
        ];
        assert_eq!(&seen[..expected.len()], &expected[..]);
    }

    #[test]
    fn green_is_never_adjacent_to_another_phases_green() {
        let mut c = Controller::new([2_000, 2_000, 2_000]);
        let mut last_green: Option<usize> = None;
        let mut saw_clearance = false;
        for o in run(
            &mut c,
            5_000,
            50,
            |i| if i > 2_000 { Some(2) } else { None },
        ) {
            match o.interval {
                Interval::Green => {
                    if last_green.is_some_and(|prev| prev != o.phase) {
                        assert!(saw_clearance, "phase change without all-red");
                    }
                    last_green = Some(o.phase);
                    saw_clearance = false;
                }
                Interval::AllRed => saw_clearance = true,
                Interval::Yellow => {}
            }
        }
    }

    #[test]
    fn preemption_reaches_the_requested_phase_within_clearance_time() {
        let mut c = Controller::new([10_000, 10_000, 10_000]);
        // Request phase 2 at t = 1 s, while phase 0 is green.
        let outs = run(
            &mut c,
            20_000,
            100,
            |i| if i >= 10 { Some(2) } else { None },
        );
        let first_green_2 = outs
            .iter()
            .position(|o| o.phase == 2 && o.interval == Interval::Green)
            .expect("phase 2 must turn green");
        let requested_at = 10;
        // Yellow + all-red = 4000 ms = 40 steps, with some slack for the first step.
        assert!(
            first_green_2 - requested_at <= 42,
            "took {} steps",
            first_green_2 - requested_at
        );
        assert!(outs[first_green_2].preempted);
    }

    #[test]
    fn releasing_the_request_returns_to_the_normal_cycle() {
        let mut c = Controller::new([10_000, 10_000, 10_000]);
        run(&mut c, 3_000, 100, |_| Some(2));
        let outs = run(&mut c, 2_000, 100, |_| None);
        assert!(outs.iter().all(|o| !o.preempted || o.phase != 2));
        assert!(outs.iter().any(|o| o.phase == 0 || o.phase == 1));
    }

    #[test]
    fn out_of_range_request_is_ignored() {
        let mut c = Controller::new([1_000, 1_000]);
        let outs = run(&mut c, 500, 100, |_| Some(7));
        // The request is discarded, so no preemption is ever reported.
        assert!(outs.iter().all(|o| !o.preempted));
    }

    /// Count of 100 ms outputs showing `phase` in green.
    fn green_steps(outs: &[Output], phase: usize) -> usize {
        outs.iter()
            .filter(|o| o.phase == phase && o.interval == Interval::Green)
            .count()
    }

    #[test]
    fn queued_greens_apply_at_the_next_phase_start() {
        let mut c = Controller::new([5_000, 5_000]);
        // Phase 0 is running. Queue a short phase 1 green while it runs.
        c.set_greens([5_000, 2_000]);
        let outs = run(&mut c, 120, 100, |_| None);
        // Phase 0 keeps the 5 s green it started with.
        assert!(green_steps(&outs, 0) >= 45, "running green was cut short");
        // Phase 1 takes the queued 2 s green, not the original 5 s.
        let p1 = green_steps(&outs, 1);
        assert!((19..=21).contains(&p1), "phase 1 green was {p1} steps");
    }
}
