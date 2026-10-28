//! Deterministic single-threaded event loop driving the vehicle state machine.
//!
//! Events arrive on two bus lanes. The emergency lane is always drained first,
//! so an `EmergencyStop` never waits behind routine traffic. Processing order is
//! fixed by the lanes and the sequence counter, so a recorded input stream
//! replays to the same transition log.

use crate::bus::Receiver;
use crate::fsm::next_state;
use crate::wire::{Event, Transition, VehicleState};

/// Result of one `EventLoop::step`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// The event was valid and changed the state.
    Applied(Transition),
    /// The event is not valid in the current state. The state is unchanged,
    /// but the event still consumed `seq`, so the log has no gaps.
    Rejected {
        /// Sequence number the rejected event consumed.
        seq: u64,
        /// State the event was rejected in.
        state: VehicleState,
        /// The rejected event.
        event: Event,
    },
    /// Both lanes were empty.
    Idle,
}

/// Owns the vehicle state and consumes events from the bus.
pub struct EventLoop<const N: usize> {
    state: VehicleState,
    seq: u64,
    emergency: Receiver<Event, N>,
    routine: Receiver<Event, N>,
}

impl<const N: usize> EventLoop<N> {
    /// Creates a loop starting in `initial`, reading from the two lanes.
    #[must_use]
    pub fn new(
        initial: VehicleState,
        emergency: Receiver<Event, N>,
        routine: Receiver<Event, N>,
    ) -> Self {
        Self {
            state: initial,
            seq: 0,
            emergency,
            routine,
        }
    }

    /// Current vehicle state.
    #[must_use]
    pub fn state(&self) -> VehicleState {
        self.state
    }

    /// Processes at most one event: the oldest emergency event if any, else the
    /// oldest routine event.
    pub fn step(&mut self) -> Outcome {
        let Some(event) = self
            .emergency
            .try_recv()
            .or_else(|| self.routine.try_recv())
        else {
            return Outcome::Idle;
        };
        let seq = self.seq;
        self.seq += 1;
        match next_state(self.state, event) {
            Some(to) => {
                let from = self.state;
                self.state = to;
                Outcome::Applied(Transition {
                    seq,
                    from,
                    to,
                    event,
                })
            }
            None => Outcome::Rejected {
                seq,
                state: self.state,
                event,
            },
        }
    }

    /// Steps until both lanes are empty, passing every non-idle outcome to
    /// `on_outcome`.
    pub fn run_until_idle(&mut self, mut on_outcome: impl FnMut(Outcome)) {
        loop {
            match self.step() {
                Outcome::Idle => break,
                outcome => on_outcome(outcome),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::{Sender, channel};

    fn lanes() -> (Sender<Event, 8>, Sender<Event, 8>, EventLoop<8>) {
        let (emergency_tx, emergency_rx) = channel::<Event, 8>();
        let (routine_tx, routine_rx) = channel::<Event, 8>();
        let event_loop = EventLoop::new(VehicleState::Parked, emergency_rx, routine_rx);
        (emergency_tx, routine_tx, event_loop)
    }

    #[test]
    fn routine_events_drive_the_state_in_order() {
        let (_emergency, mut routine, mut event_loop) = lanes();
        for event in [Event::Depart, Event::RouteReady, Event::Halt] {
            routine.try_send(event).unwrap();
        }
        let mut log = Vec::new();
        event_loop.run_until_idle(|o| log.push(o));

        assert_eq!(event_loop.state(), VehicleState::Stopped);
        let seqs: Vec<u64> = log
            .iter()
            .map(|o| match o {
                Outcome::Applied(t) => t.seq,
                other => panic!("expected Applied, got {other:?}"),
            })
            .collect();
        assert_eq!(seqs, vec![0, 1, 2]);
    }

    #[test]
    fn emergency_lane_preempts_queued_routine_events() {
        let (mut emergency, mut routine, mut event_loop) = lanes();
        routine.try_send(Event::Depart).unwrap();
        emergency.try_send(Event::EmergencyStop).unwrap();

        // The emergency event is processed first, so the routine Depart arrives
        // while the vehicle is already in Emergency and is rejected.
        assert!(matches!(
            event_loop.step(),
            Outcome::Applied(Transition {
                to: VehicleState::Emergency,
                ..
            })
        ));
        assert!(matches!(
            event_loop.step(),
            Outcome::Rejected {
                state: VehicleState::Emergency,
                event: Event::Depart,
                ..
            }
        ));
        assert_eq!(event_loop.state(), VehicleState::Emergency);
    }

    #[test]
    fn invalid_event_is_rejected_without_changing_state() {
        let (_emergency, mut routine, mut event_loop) = lanes();
        routine.try_send(Event::Halt).unwrap();
        assert_eq!(
            event_loop.step(),
            Outcome::Rejected {
                seq: 0,
                state: VehicleState::Parked,
                event: Event::Halt,
            }
        );
        assert_eq!(event_loop.state(), VehicleState::Parked);
    }

    #[test]
    fn empty_lanes_report_idle() {
        let (_emergency, _routine, mut event_loop) = lanes();
        assert_eq!(event_loop.step(), Outcome::Idle);
    }

    #[test]
    fn identical_input_produces_identical_log() {
        let run = || {
            let (mut emergency, mut routine, mut event_loop) = lanes();
            for event in [Event::Depart, Event::RouteReady, Event::Halt, Event::Go] {
                routine.try_send(event).unwrap();
            }
            emergency.try_send(Event::EmergencyStop).unwrap();
            let mut log = Vec::new();
            event_loop.run_until_idle(|o| log.push(o));
            log
        };
        assert_eq!(run(), run());
    }
}
