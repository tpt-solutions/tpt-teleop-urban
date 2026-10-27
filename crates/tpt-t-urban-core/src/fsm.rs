//! Vehicle state machine: Parked → Routing → Driving → Stopped → Emergency.
//!
//! The transition table is a single pure function, so every legal path can be
//! checked exhaustively and the function never allocates or blocks.

use crate::wire::{Event, VehicleState};

/// State reached from `state` on `event`, or `None` if the event is invalid
/// in that state.
///
/// `EmergencyStop` is accepted from every non-emergency state. Once in
/// `Emergency`, the only exit is `Clear`, and it lands in `Stopped` rather than
/// `Driving`, so the vehicle must be re-engaged deliberately.
#[must_use]
pub const fn next_state(state: VehicleState, event: Event) -> Option<VehicleState> {
    use Event as E;
    use VehicleState as S;
    match (state, event) {
        (S::Emergency, E::Clear) => Some(S::Stopped),
        (S::Emergency, _) => None,
        (_, E::EmergencyStop) => Some(S::Emergency),
        (S::Parked, E::Depart) => Some(S::Routing),
        (S::Routing, E::RouteReady) => Some(S::Driving),
        (S::Routing, E::RouteFailed) => Some(S::Parked),
        (S::Driving, E::Halt) => Some(S::Stopped),
        (S::Stopped, E::Go) => Some(S::Driving),
        (S::Stopped, E::Park) => Some(S::Parked),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STATES: [VehicleState; 5] = [
        VehicleState::Parked,
        VehicleState::Routing,
        VehicleState::Driving,
        VehicleState::Stopped,
        VehicleState::Emergency,
    ];

    const EVENTS: [Event; 8] = [
        Event::Depart,
        Event::RouteReady,
        Event::RouteFailed,
        Event::Halt,
        Event::Go,
        Event::Park,
        Event::EmergencyStop,
        Event::Clear,
    ];

    #[test]
    fn documented_happy_path_is_legal() {
        let path = [
            (VehicleState::Parked, Event::Depart, VehicleState::Routing),
            (
                VehicleState::Routing,
                Event::RouteReady,
                VehicleState::Driving,
            ),
            (VehicleState::Driving, Event::Halt, VehicleState::Stopped),
            (VehicleState::Stopped, Event::Go, VehicleState::Driving),
            (VehicleState::Driving, Event::Halt, VehicleState::Stopped),
            (VehicleState::Stopped, Event::Park, VehicleState::Parked),
        ];
        for (from, event, to) in path {
            assert_eq!(next_state(from, event), Some(to), "{from:?} --{event:?}-->");
        }
    }

    #[test]
    fn emergency_stop_is_accepted_from_every_non_emergency_state() {
        for state in STATES.into_iter().filter(|s| *s != VehicleState::Emergency) {
            assert_eq!(
                next_state(state, Event::EmergencyStop),
                Some(VehicleState::Emergency),
                "from {state:?}"
            );
        }
    }

    #[test]
    fn emergency_only_exits_through_clear_into_stopped() {
        for event in EVENTS {
            let next = next_state(VehicleState::Emergency, event);
            match event {
                Event::Clear => assert_eq!(next, Some(VehicleState::Stopped)),
                _ => assert_eq!(next, None, "{event:?} must not leave Emergency"),
            }
        }
    }

    #[test]
    fn every_pair_not_listed_in_the_table_is_rejected() {
        let legal = [
            (VehicleState::Parked, Event::Depart),
            (VehicleState::Routing, Event::RouteReady),
            (VehicleState::Routing, Event::RouteFailed),
            (VehicleState::Driving, Event::Halt),
            (VehicleState::Stopped, Event::Go),
            (VehicleState::Stopped, Event::Park),
            (VehicleState::Emergency, Event::Clear),
        ];
        for state in STATES {
            for event in EVENTS {
                let expected_legal = legal.contains(&(state, event))
                    || (event == Event::EmergencyStop && state != VehicleState::Emergency);
                assert_eq!(
                    next_state(state, event).is_some(),
                    expected_legal,
                    "{state:?} x {event:?}"
                );
            }
        }
    }
}
