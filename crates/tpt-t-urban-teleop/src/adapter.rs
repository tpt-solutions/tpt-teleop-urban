//! Teleop adapter: the urban vehicle's side of tpt-teleop's control protocol.
//!
//! The adapter takes the upstream [`ControlCommand`] (tpt-teleop-core,
//! `ser::cmd`) and the upstream [`StateMachine`] (`machine`), and turns them into
//! actuator commands. Each command passes these gates, in order:
//!
//! 1. **Wire check**: magic, version, and a strictly increasing `seq`. A replayed
//!    or malformed command is refused.
//! 2. **Mode**: the command's mode must match the machine's current mode. The
//!    adapter acts on the machine, not on what the operator's side believes.
//!    `Auto` refuses operator input. `Assist` passes steering only, because the
//!    upstream blending model (spec §5.4) is not implemented yet. `FullTeleop`
//!    drives directly. `EmergencyStop` brakes fully and latches.
//! 3. **Traffic law**: the latest `compliance` decision caps the result. `Stop`
//!    and `Yield` brake, and `Decelerate` removes throttle.
//! 4. **Actuator limits**: the `edge` limiter clamps and rate-limits the result.
//!
//! Video leaves the vehicle only through `privacy::Redactor`.
//!
//! The DomainTeleopInterface trait from the bridge spec is not in upstream
//! tpt-teleop yet. Its `on_control_command` maps to
//! [`TeleopAdapter::on_control_command`] here.

use tpt_t_core::ser::cmd::{ControlCommand, MAGIC_COMMAND};
use tpt_t_core::{Mode, ModeError, StateMachine};
use tpt_t_urban_compliance::Decision;
use tpt_t_urban_edge::actuator::{MAX_PERMILLE, MAX_STEER_CDEG};
use tpt_t_urban_edge::{Actuation, Limiter};
use tpt_t_urban_privacy::{Frame, FrameError, Rect, RedactedFrame, Redactor};

/// Brake applied when the traffic-law decision is `Stop` or `Yield`, permille.
pub const LAW_STOP_BRAKE_PERMILLE: u16 = 1_000;
/// Brake applied when the traffic-law decision is `Decelerate`, permille.
pub const LAW_DECEL_BRAKE_PERMILLE: u16 = 300;
/// Blur radius for operator video, pixels.
pub const REDACT_RADIUS: usize = 3;

/// Index of the roll axis in `ControlCommand::axes`, used as steering.
pub const AXIS_STEERING: usize = 0;
/// Index of the throttle axis, positive throttle and negative brake.
pub const AXIS_THROTTLE: usize = 3;

/// Why a command was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandError {
    /// Wrong magic or version, or an unknown mode value.
    Malformed,
    /// `seq` did not increase, so the command is a replay or out of order.
    Replay,
    /// The command was authored for a mode that is no longer current.
    ModeMismatch { current: Mode },
    /// The machine is in `Auto`, where operator input is not accepted.
    NotActive,
}

/// Teleop adapter for one vehicle.
#[derive(Debug)]
pub struct TeleopAdapter {
    machine: StateMachine,
    law: Decision,
    limiter: Limiter,
    last_seq: Option<u64>,
}

impl Default for TeleopAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl TeleopAdapter {
    /// A new adapter in `Mode::Auto`, with the spec-default transition table.
    #[must_use]
    pub fn new() -> Self {
        Self {
            machine: StateMachine::new(),
            law: Decision::Proceed,
            limiter: Limiter::new(),
            last_seq: None,
        }
    }

    /// Current mode.
    #[must_use]
    pub fn mode(&self) -> Mode {
        self.machine.current()
    }

    /// Requests a mode change through the transition table. Returns the
    /// previous mode on success.
    pub fn request_mode(&self, to: Mode) -> Result<Mode, ModeError> {
        self.machine.try_transition(to)
    }

    /// Emergency stop. Bypasses the table and is idempotent.
    pub fn emergency_stop(&self) -> Mode {
        self.machine.force_emergency_stop()
    }

    /// Records the latest traffic-law decision from `compliance::evaluate`.
    pub fn set_law_decision(&mut self, decision: Decision) {
        self.law = decision;
    }

    /// Maps one command to actuator values, or refuses it. A refused command
    /// leaves the replay counter unchanged.
    pub fn on_control_command(&mut self, cmd: &ControlCommand) -> Result<Actuation, CommandError> {
        if cmd.magic != MAGIC_COMMAND || cmd.version != 1 {
            return Err(CommandError::Malformed);
        }
        if self.last_seq.is_some_and(|prev| cmd.seq <= prev) {
            return Err(CommandError::Replay);
        }
        let commanded = cmd.mode().ok_or(CommandError::Malformed)?;

        // An operator E-stop is honoured in any mode, before the mode check.
        if commanded == Mode::EmergencyStop {
            self.emergency_stop();
        }
        self.last_seq = Some(cmd.seq);

        let current = self.machine.current();
        if current == Mode::EmergencyStop {
            return Ok(self.limiter.apply(0, MAX_PERMILLE, 0));
        }
        if commanded != current {
            return Err(CommandError::ModeMismatch { current });
        }

        let steer_axis = cmd.axes[AXIS_STEERING].clamp(-1.0, 1.0);
        let pedal = cmd.axes[AXIS_THROTTLE].clamp(-1.0, 1.0);
        let steer = (steer_axis * MAX_STEER_CDEG as f32) as i32;
        let mut brake = (-pedal * f32::from(MAX_PERMILLE)).max(0.0) as u16;
        let mut throttle = (pedal * f32::from(MAX_PERMILLE)).max(0.0) as u16;

        match current {
            Mode::Auto => return Err(CommandError::NotActive),
            Mode::Assist => {
                // Blending is not implemented. Steering passes, throttle is held.
                throttle = 0;
            }
            Mode::FullTeleop => {}
            Mode::EmergencyStop => unreachable!("handled above"),
        }

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

    /// Redacts a camera frame before it is sent to the operator. Takes ownership
    /// of the raw frame and returns only the redacted one.
    pub fn redact_for_operator<'a>(
        &self,
        frame: Frame<'a>,
        detections: &[Rect],
        scratch: &mut [u8],
    ) -> Result<RedactedFrame<'a>, FrameError> {
        Redactor::new(REDACT_RADIUS).redact(frame, detections, scratch)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(seq: u64, mode: Mode, steering: f32, pedal: f32) -> ControlCommand {
        let mut c = ControlCommand::zeroed(mode);
        c.seq = seq;
        c.axes[AXIS_STEERING] = steering;
        c.axes[AXIS_THROTTLE] = pedal;
        c
    }

    /// Adapter driven to `FullTeleop` through the legal Auto → Assist → FullTeleop path.
    fn teleop() -> TeleopAdapter {
        let a = TeleopAdapter::new();
        a.request_mode(Mode::Assist).unwrap();
        a.request_mode(Mode::FullTeleop).unwrap();
        a
    }

    #[test]
    fn auto_refuses_operator_input() {
        let mut a = TeleopAdapter::new();
        assert_eq!(
            a.on_control_command(&cmd(1, Mode::Auto, 0.0, 0.5)),
            Err(CommandError::NotActive)
        );
    }

    #[test]
    fn direct_auto_to_teleop_is_refused_by_the_table() {
        let a = TeleopAdapter::new();
        assert!(a.request_mode(Mode::FullTeleop).is_err());
        assert_eq!(a.mode(), Mode::Auto);
    }

    #[test]
    fn full_teleop_throttle_is_rate_limited() {
        let mut a = teleop();
        let out = a
            .on_control_command(&cmd(1, Mode::FullTeleop, 0.0, 1.0))
            .unwrap();
        assert_eq!(
            out.throttle_permille,
            tpt_t_urban_edge::actuator::MAX_PERMILLE_RATE
        );
        assert_eq!(out.brake_permille, 0);
    }

    #[test]
    fn negative_pedal_brakes_and_steering_follows_roll_axis() {
        let mut a = teleop();
        let out = a
            .on_control_command(&cmd(1, Mode::FullTeleop, 1.0, -1.0))
            .unwrap();
        assert!(out.brake_permille > 0);
        assert!(out.steer_cdeg > 0);
    }

    #[test]
    fn assist_passes_steering_but_holds_throttle() {
        let a = TeleopAdapter::new();
        a.request_mode(Mode::Assist).unwrap();
        let mut a = a;
        let out = a
            .on_control_command(&cmd(1, Mode::Assist, 0.5, 1.0))
            .unwrap();
        assert_eq!(out.throttle_permille, 0);
        assert!(out.steer_cdeg > 0);
    }

    #[test]
    fn replayed_or_out_of_order_commands_are_refused() {
        let mut a = teleop();
        a.on_control_command(&cmd(5, Mode::FullTeleop, 0.0, 0.0))
            .unwrap();
        assert_eq!(
            a.on_control_command(&cmd(5, Mode::FullTeleop, 0.0, 0.0)),
            Err(CommandError::Replay)
        );
        assert_eq!(
            a.on_control_command(&cmd(4, Mode::FullTeleop, 0.0, 0.0)),
            Err(CommandError::Replay)
        );
    }

    #[test]
    fn command_for_a_stale_mode_is_refused() {
        let mut a = teleop();
        assert_eq!(
            a.on_control_command(&cmd(1, Mode::Assist, 0.0, 0.0)),
            Err(CommandError::ModeMismatch {
                current: Mode::FullTeleop
            })
        );
    }

    #[test]
    fn malformed_command_is_refused_without_consuming_its_seq() {
        let mut a = teleop();
        let mut bad = cmd(3, Mode::FullTeleop, 0.0, 0.0);
        bad.magic = 0;
        assert_eq!(a.on_control_command(&bad), Err(CommandError::Malformed));
        assert!(
            a.on_control_command(&cmd(3, Mode::FullTeleop, 0.0, 0.0))
                .is_ok()
        );
    }

    #[test]
    fn operator_estop_latches_and_brakes_in_any_mode() {
        let mut a = teleop();
        let out = a
            .on_control_command(&cmd(1, Mode::EmergencyStop, 0.0, 1.0))
            .unwrap();
        assert_eq!(a.mode(), Mode::EmergencyStop);
        assert!(out.brake_permille > 0);
        assert_eq!(out.throttle_permille, 0);
        // Further drive commands are still braked, never throttled.
        let later = a
            .on_control_command(&cmd(2, Mode::FullTeleop, 0.0, 1.0))
            .unwrap();
        assert_eq!(later.throttle_permille, 0);
    }

    #[test]
    fn traffic_stop_overrides_the_operator_throttle() {
        let mut a = teleop();
        a.set_law_decision(Decision::Stop);
        let mut out = Actuation::default();
        for seq in 1..=12 {
            out = a
                .on_control_command(&cmd(seq, Mode::FullTeleop, 0.0, 1.0))
                .unwrap();
        }
        assert_eq!(out.throttle_permille, 0);
        assert_eq!(out.brake_permille, LAW_STOP_BRAKE_PERMILLE);
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
        assert!(!tpt_t_urban_privacy::changed_outside_detections(
            &original,
            &redacted,
            &detections
        ));
    }
}
