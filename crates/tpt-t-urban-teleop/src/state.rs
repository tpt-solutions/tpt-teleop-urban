//! Universal teleop safety state machine (bridge spec §4.3).
//!
//! ```text
//! AUTONOMOUS → REQUESTING_TELEOP → TELEOP_ACTIVE → RETURNING_TO_AUTONOMY → AUTONOMOUS
//!                    ↓                                       ↓
//!              EMERGENCY_STOP ←───────────── any state ──────┘
//! ```
//!
//! The transition function is pure. An event that is not valid in the current
//! state is an error, and the state does not change.

/// Mode of the vehicle with respect to teleoperation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Autonomous,
    RequestingTeleop,
    TeleopActive,
    ReturningToAutonomy,
    EmergencyStop,
}

/// Inputs that move the state machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// The domain detected a fault and requests human help.
    Request,
    /// The request was withdrawn before an operator engaged.
    Cancel,
    /// An operator took control.
    Engage,
    /// The operator released control.
    Release,
    /// Handover back to autonomy is finished.
    ReturnComplete,
    /// Operator or domain triggered an immediate stop.
    EStop,
    /// The vehicle is at rest and the stop has been cleared.
    Reset,
}

/// An event that is not valid in the current mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidEvent {
    pub mode: Mode,
    pub event: Event,
}

/// Applies `event` to `mode`.
pub fn transition(mode: Mode, event: Event) -> Result<Mode, InvalidEvent> {
    use Event as E;
    use Mode as M;
    let next = match (mode, event) {
        (_, E::EStop) => M::EmergencyStop,
        (M::Autonomous, E::Request) => M::RequestingTeleop,
        (M::RequestingTeleop, E::Cancel) => M::Autonomous,
        (M::RequestingTeleop, E::Engage) => M::TeleopActive,
        (M::TeleopActive, E::Release) => M::ReturningToAutonomy,
        (M::ReturningToAutonomy, E::ReturnComplete) => M::Autonomous,
        (M::EmergencyStop, E::Reset) => M::Autonomous,
        _ => return Err(InvalidEvent { mode, event }),
    };
    Ok(next)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_handover_round_trip() {
        let mut m = Mode::Autonomous;
        for e in [
            Event::Request,
            Event::Engage,
            Event::Release,
            Event::ReturnComplete,
        ] {
            m = transition(m, e).unwrap();
        }
        assert_eq!(m, Mode::Autonomous);
    }

    #[test]
    fn estop_is_accepted_from_every_mode() {
        for m in [
            Mode::Autonomous,
            Mode::RequestingTeleop,
            Mode::TeleopActive,
            Mode::ReturningToAutonomy,
            Mode::EmergencyStop,
        ] {
            assert_eq!(transition(m, Event::EStop), Ok(Mode::EmergencyStop));
        }
    }

    #[test]
    fn engage_without_a_request_is_rejected_and_mode_is_kept() {
        let err = transition(Mode::Autonomous, Event::Engage).unwrap_err();
        assert_eq!(err.mode, Mode::Autonomous);
    }

    #[test]
    fn emergency_stop_only_clears_through_reset() {
        for e in [
            Event::Request,
            Event::Engage,
            Event::Release,
            Event::ReturnComplete,
        ] {
            assert!(transition(Mode::EmergencyStop, e).is_err());
        }
        assert_eq!(
            transition(Mode::EmergencyStop, Event::Reset),
            Ok(Mode::Autonomous)
        );
    }
}
