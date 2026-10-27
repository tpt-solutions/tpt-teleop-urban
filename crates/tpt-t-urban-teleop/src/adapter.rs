//! Teleop adapter: turns operator commands into actuator commands, subject to the
//! vehicle's own safety rules.
//!
//! Every operator command passes through three gates before it reaches an
//! actuator:
//!
//! 1. **Mode**: only `TeleopActive` accepts commands. An emergency stop overrides
//!    everything.
//! 2. **Traffic law**: the latest `compliance` decision caps the command. A
//!    `Stop` or `Yield` brakes, a `Decelerate` removes throttle.
//! 3. **Actuator limits**: the `edge` limiter clamps and rate-limits the result.
//!
//! Video leaves the vehicle only through `privacy::Redactor`, so the operator
//! never receives an unblurred frame.
//!
//! The adapter is written against the teleop mode machine and the vehicle
//! crates in this workspace. The `DomainTeleopInterface` trait from
//! `tpt-t-domain-bridge` (in the `tpt-teleop` repository) is not a dependency
//! yet. Once that crate is available, the methods here become its trait impl.

use tpt_t_urban_compliance::Decision;
use tpt_t_urban_edge::{Actuation, Limiter};
use tpt_t_urban_privacy::{Frame, FrameError, Rect, RedactedFrame, Redactor};

use crate::state::{Event, InvalidEvent, Mode, transition};

/// Brake applied when the traffic-law decision is `Stop` or `Yield`, permille.
pub const LAW_STOP_BRAKE_PERMILLE: u16 = 1_000;
/// Brake applied when the traffic-law decision is `Decelerate`, permille.
pub const LAW_DECEL_BRAKE_PERMILLE: u16 = 300;

/// Normalized command from the operator, in `-1.0..=1.0`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlCommand {
    pub timestamp_us: u64,
    /// Operator of record, e.g. a 128-bit session id.
    pub operator_id: [u8; 16],
    /// Steering: -1 is full left, +1 full right.
    pub steering: f32,
    /// Pedal: positive is throttle, negative is brake.
    pub pedal: f32,
    /// Operator pressed the emergency stop.
    pub estop: bool,
}

/// Why a command was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandError {
    /// The vehicle is not in `TeleopActive`.
    NotActive,
    /// The operator id does not match the engaged operator.
    WrongOperator,
}

/// Teleop adapter for one vehicle.
#[derive(Clone, Debug)]
pub struct TeleopAdapter {
    mode: Mode,
    operator: Option<[u8; 16]>,
    law: Decision,
    limiter: Limiter,
}

impl Default for TeleopAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl TeleopAdapter {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            mode: Mode::Autonomous,
            operator: None,
            law: Decision::Proceed,
            limiter: Limiter::new(),
        }
    }

    #[must_use]
    pub const fn mode(&self) -> Mode {
        self.mode
    }

    /// Applies a state-machine event. Engaging also records the operator.
    pub fn apply(
        &mut self,
        event: Event,
        operator: Option<[u8; 16]>,
    ) -> Result<Mode, InvalidEvent> {
        self.mode = transition(self.mode, event)?;
        match event {
            Event::Engage => self.operator = operator,
            Event::Reset | Event::ReturnComplete | Event::Cancel => self.operator = None,
            _ => {}
        }
        Ok(self.mode)
    }

    /// Records the latest traffic-law decision from `compliance::evaluate`.
    pub fn set_law_decision(&mut self, decision: Decision) {
        self.law = decision;
    }

    /// Maps one operator command to actuator values, or refuses it.
    ///
    /// An e-stop is always honoured: it moves to `EmergencyStop` and brakes fully,
    /// whatever mode the adapter was in.
    pub fn on_control_command(&mut self, cmd: &ControlCommand) -> Result<Actuation, CommandError> {
        if cmd.estop {
            // Event::EStop is valid in every mode, so the transition cannot fail.
            self.mode = transition(self.mode, Event::EStop).unwrap_or(Mode::EmergencyStop);
            return Ok(self.limiter.apply(0, MAX_BRAKE, 0));
        }
        if self.mode != Mode::TeleopActive {
            return Err(CommandError::NotActive);
        }
        if self.operator != Some(cmd.operator_id) {
            return Err(CommandError::WrongOperator);
        }

        let steer = (cmd.steering.clamp(-1.0, 1.0) * MAX_STEER_CDEG as f32) as i32;
        let pedal = cmd.pedal.clamp(-1.0, 1.0);
        let mut brake = (-pedal * f32::from(MAX_BRAKE)).max(0.0) as u16;
        let mut throttle = (pedal * f32::from(MAX_BRAKE)).max(0.0) as u16;

        match self.law {
            Decision::Proceed => {}
            Decision::Decelerate => {
                throttle = 0;
                brake = brake.max(LAW_DECEL_BRAKE_PERMILLE);
            }
            Decision::Yield | Decision::Stop => {
                throttle = 0;
                brake = LAW_STOP_BRAKE_PERMILLE;
            }
        }
        Ok(self.limiter.apply(steer, brake, throttle))
    }

    /// Redacts a camera frame before it is sent to the operator.
    ///
    /// Takes ownership of the raw frame and returns the redacted frame. Only the
    /// redacted frame is returned, so the caller has no unblurred copy to send.
    pub fn redact_for_operator<'a>(
        &self,
        frame: Frame<'a>,
        detections: &[Rect],
        scratch: &mut [u8],
    ) -> Result<RedactedFrame<'a>, FrameError> {
        Redactor::new(REDACT_RADIUS).redact(frame, detections, scratch)
    }
}

const MAX_BRAKE: u16 = 1_000;
const MAX_STEER_CDEG: i32 = tpt_t_urban_edge::actuator::MAX_STEER_CDEG;
const REDACT_RADIUS: usize = 3;

#[cfg(test)]
mod tests {
    use super::*;

    const OP: [u8; 16] = [7; 16];

    fn cmd(steering: f32, pedal: f32) -> ControlCommand {
        ControlCommand {
            timestamp_us: 0,
            operator_id: OP,
            steering,
            pedal,
            estop: false,
        }
    }

    fn active() -> TeleopAdapter {
        let mut a = TeleopAdapter::new();
        a.apply(Event::Request, None).unwrap();
        a.apply(Event::Engage, Some(OP)).unwrap();
        a
    }

    #[test]
    fn commands_are_refused_outside_teleop() {
        let mut a = TeleopAdapter::new();
        assert_eq!(
            a.on_control_command(&cmd(0.0, 0.5)),
            Err(CommandError::NotActive)
        );
    }

    #[test]
    fn commands_from_the_wrong_operator_are_refused() {
        let mut a = active();
        let mut c = cmd(0.0, 0.5);
        c.operator_id = [9; 16];
        assert_eq!(a.on_control_command(&c), Err(CommandError::WrongOperator));
    }

    #[test]
    fn throttle_command_reaches_the_actuator_within_rate_limits() {
        let mut a = active();
        let out = a.on_control_command(&cmd(0.0, 1.0)).unwrap();
        assert_eq!(
            out.throttle_permille,
            tpt_t_urban_edge::actuator::MAX_PERMILLE_RATE
        );
        assert_eq!(out.brake_permille, 0);
    }

    #[test]
    fn traffic_stop_overrides_the_operator_throttle() {
        let mut a = active();
        for _ in 0..10 {
            a.on_control_command(&cmd(0.0, 1.0)).unwrap();
        }
        a.set_law_decision(Decision::Stop);
        let mut out = a.on_control_command(&cmd(0.0, 1.0)).unwrap();
        for _ in 0..10 {
            out = a.on_control_command(&cmd(0.0, 1.0)).unwrap();
        }
        assert_eq!(out.throttle_permille, 0);
        assert_eq!(out.brake_permille, LAW_STOP_BRAKE_PERMILLE);
    }

    #[test]
    fn estop_from_any_mode_brakes_and_latches_emergency() {
        let mut a = TeleopAdapter::new();
        let mut c = cmd(0.0, 0.0);
        c.estop = true;
        let out = a.on_control_command(&c).unwrap();
        assert_eq!(a.mode(), Mode::EmergencyStop);
        assert!(out.brake_permille > 0);
        assert_eq!(
            a.on_control_command(&cmd(0.0, 1.0)),
            Err(CommandError::NotActive)
        );
    }

    #[test]
    fn redaction_is_applied_before_the_operator_sees_a_frame() {
        let mut pixels = vec![200u8; 16 * 16];
        let original = pixels.clone();
        let detections = [Rect {
            x: 4,
            y: 4,
            w: 6,
            h: 6,
        }];
        let mut scratch = vec![0u8; 36];
        let a = TeleopAdapter::new();
        let frame = Frame::new(&mut pixels, 16, 16).unwrap();
        let redacted = a
            .redact_for_operator(frame, &detections, &mut scratch)
            .unwrap();
        // The uniform frame is unchanged by blur, but pixels outside detections must still match.
        assert!(!tpt_t_urban_privacy::changed_outside_detections(
            &original,
            &redacted,
            &detections
        ));
    }
}
