//! Wire types shared by urban crates (the rkyv prelude, Phase 2).
//!
//! Every type here is `Copy` and archived with rkyv, so it can cross the bus
//! and land in the audit ledger without a separate serialization step.

use rkyv::{Archive, Deserialize, Serialize};

/// Operating state of the vehicle.
#[derive(Archive, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum VehicleState {
    Parked,
    Routing,
    Driving,
    Stopped,
    Emergency,
}

/// Input to the vehicle state machine.
#[derive(Archive, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Event {
    Depart,
    RouteReady,
    RouteFailed,
    Halt,
    Go,
    Park,
    EmergencyStop,
    Clear,
}

/// One accepted state change, recorded for the audit trail.
#[derive(Archive, Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transition {
    /// Logical sequence number, assigned by the event loop in processing order.
    pub seq: u64,
    pub from: VehicleState,
    pub to: VehicleState,
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
