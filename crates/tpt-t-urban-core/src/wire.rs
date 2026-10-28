//! Wire types shared by urban crates (the rkyv prelude, Phase 2).
//!
//! Every type here is `Copy` and archived with rkyv, so it can cross the bus
//! and land in the audit ledger without a separate serialization step.

use rkyv::{Archive, Deserialize, Serialize};

/// Operating state of the vehicle.
#[derive(Archive, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum VehicleState {
    /// Parked, not driving.
    Parked,
    /// Computing a route.
    Routing,
    /// Following a route.
    Driving,
    /// Halted, with the route still held.
    Stopped,
    /// Emergency stop latched. Only `Clear` leaves it.
    Emergency,
}

/// Input to the vehicle state machine.
#[derive(Archive, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Event {
    /// Start a trip from `Parked`.
    Depart,
    /// The route is computed: `Routing` to `Driving`.
    RouteReady,
    /// No route could be found: `Routing` to `Parked`.
    RouteFailed,
    /// Stop on the road: `Driving` to `Stopped`.
    Halt,
    /// Resume after a halt: `Stopped` to `Driving`.
    Go,
    /// Park after a halt: `Stopped` to `Parked`.
    Park,
    /// Emergency stop, accepted from every state except `Emergency`.
    EmergencyStop,
    /// Leave the emergency: `Emergency` to `Stopped`.
    Clear,
}

/// One accepted state change, recorded for the audit trail.
#[derive(Archive, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transition {
    /// Logical sequence number, assigned by the event loop in processing order.
    pub seq: u64,
    /// State before the transition.
    pub from: VehicleState,
    /// State after the transition.
    pub to: VehicleState,
    /// The event that caused the transition.
    pub event: Event,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transition_round_trips_through_rkyv() {
        let original = Transition {
            seq: 42,
            from: VehicleState::Driving,
            to: VehicleState::Stopped,
            event: Event::Halt,
        };
        let bytes = rkyv::to_bytes::<rkyv::rancor::Error>(&original).expect("serialize");
        let restored: Transition =
            rkyv::from_bytes::<Transition, rkyv::rancor::Error>(&bytes).expect("deserialize");
        assert_eq!(restored, original);
    }
}
