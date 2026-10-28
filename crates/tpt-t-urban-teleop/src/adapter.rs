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
//!    drives directly. `EmergencyStop` latches, and applies full brake and zero
//!    throttle at once. It bypasses the slew limit, because a stop is the one
//!    command where delay costs more than a jerk.
//! 3. **Traffic law**: the latest `compliance` decision caps the result. `Stop`
//!    and `Yield` brake, and `Decelerate` removes throttle.
//! 4. **Guardian**: while the `safety` guardian holds AEB or MRM, throttle is
//!    removed and the brake is applied in full, whatever the operator asks.
//! 5. **Actuator limits**: the `edge` limiter clamps and rate-limits the result.
//!
//! Video leaves the vehicle only through `privacy::Redactor`.
//!
//! [`TeleopAdapter`] implements the upstream [`DomainTeleopInterface`] from
//! `tpt-t-domain-bridge`. The trait's `on_control_command` runs the gates above
//! through [`TeleopAdapter::apply_command`]. Lifecycle calls (engage, disengage,
//! emergency stop) arrive through a `DomainSession`, which enforces the order.

use tpt_t_core::ser::cmd::{ControlCommand, MAGIC_COMMAND};
use tpt_t_core::{Mode, ModeError, StateMachine};
use tpt_t_domain_bridge::wire::{FAULT_SENSOR, HapticFeedback, PointCloud, VideoFrame, VideoKind};
use tpt_t_domain_bridge::{
    DomainError, DomainState, DomainTeleopInterface, OperatorId, Pose6DOF, SensorFeed,
};
use tpt_t_urban_compliance::{ApproachState, Decision, Evaluation, Observation, evaluate};
use tpt_t_urban_edge::actuator::{MAX_PERMILLE, MAX_STEER_CDEG};
use tpt_t_urban_edge::{Actuation, Limiter};
use tpt_t_urban_perception::Estimate;
use tpt_t_urban_privacy::{Frame, FrameError, Rect, RedactedFrame, Redactor};
use tpt_t_urban_safety::{Command as GuardianCommand, Mode as GuardianMode};

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
    ModeMismatch {
        /// The mode the vehicle is in now.
        current: Mode,
    },
    /// The machine is in `Auto`, where operator input is not accepted.
    NotActive,
}

/// Teleop adapter for one vehicle. Implements [`DomainTeleopInterface`].
#[derive(Debug)]
pub struct TeleopAdapter {
    machine: StateMachine,
    law: Decision,
    limiter: Limiter,
    last_seq: Option<u64>,
    /// Operator holding the vehicle, set by `on_teleop_engage`.
    operator: Option<OperatorId>,
    /// Output of the last accepted command, handed to the actuators.
    actuation: Option<Actuation>,
    /// The guardian's latest mode. AEB or MRM overrides the operator.
    guardian: GuardianMode,
    /// Domain state reported upward. Starts as a sensor fault until
    /// [`TeleopAdapter::update_localization`] supplies a real estimate.
    state: DomainState,
    /// Stop-sign approach state, advanced by [`TeleopAdapter::update_compliance`].
    approach: ApproachState,
    /// Latest redacted video frame, taken by the next sensor feed.
    video: Option<VideoFrame>,
    /// Latest point cloud, taken by the next sensor feed.
    point_cloud: Option<PointCloud>,
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
            operator: None,
            actuation: None,
            guardian: GuardianMode::Nominal,
            state: DomainState {
                timestamp_us: 0,
                pose: Pose6DOF::IDENTITY,
                battery_pct: 0.0,
                fault_flags: FAULT_SENSOR,
            },
            approach: ApproachState::Clear,
            video: None,
            point_cloud: None,
        }
    }

    /// Replaces the whole reported domain state, including battery and faults.
    pub fn update_state(&mut self, state: DomainState) {
        self.state = state;
    }

    /// Feeds the fused position from `perception` into the reported pose and
    /// clears the sensor fault. The height is not estimated, so `z` is zero.
    pub fn update_localization(&mut self, estimate: Estimate, timestamp_us: u64) {
        self.state.timestamp_us = timestamp_us;
        self.state.pose.position = [estimate.x, estimate.y, 0.0];
        self.state.fault_flags &= !FAULT_SENSOR;
    }

    /// Raises or clears one `FAULT_*` bit in the reported state.
    pub fn set_fault(&mut self, flag: u32, raised: bool) {
        if raised {
            self.state.fault_flags |= flag;
        } else {
            self.state.fault_flags &= !flag;
        }
    }

    /// Marks the vehicle as approaching a stop line. The compliance FSM ignores
    /// stop signs until this is called, so call it when the route enters one.
    pub fn begin_stop_approach(&mut self) {
        self.approach = ApproachState::Approaching;
    }

    /// Runs one compliance step, caps the actuators with its decision, and
    /// returns the evaluation. The caller records its `reason` in the ledger.
    pub fn update_compliance(&mut self, obs: &Observation) -> Evaluation {
        let evaluation = evaluate(self.approach, obs);
        self.approach = evaluation.next;
        self.law = evaluation.decision;
        evaluation
    }

    /// Queues a point cloud for the next sensor feed. A newer cloud replaces an
    /// older one that was not yet taken.
    pub fn push_point_cloud(&mut self, cloud: PointCloud) {
        self.point_cloud = Some(cloud);
    }

    /// Blurs the detected regions of one RGB frame and queues the redacted copy
    /// for the next sensor feed. The raw pixels are consumed here and never kept.
    pub fn push_video(
        &mut self,
        mut pixels: Vec<u8>,
        width: u32,
        height: u32,
        timestamp_us: u64,
        detections: &[Rect],
    ) -> Result<(), FrameError> {
        let (w, h) = (width as usize, height as usize);
        let mut scratch = vec![0u8; Redactor::scratch_len(detections, w, h)];
        let frame = Frame::new(&mut pixels, w, h)?;
        let redacted = self.redact_for_operator(frame, detections, &mut scratch)?;
        self.video = Some(VideoFrame {
            timestamp_us,
            kind: VideoKind::Rgb,
            width,
            height,
            data: redacted.pixels().to_vec(),
        });
        Ok(())
    }

    /// The actuator values from the last accepted command, if any.
    #[must_use]
    pub fn last_actuation(&self) -> Option<Actuation> {
        self.actuation
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
    pub fn apply_command(&mut self, cmd: &ControlCommand) -> Result<Actuation, CommandError> {
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
            return Ok(self.limiter.emergency_brake());
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
        // The guardian overrides the operator while it holds AEB or MRM. Throttle
        // is removed and the brake is applied in full, whatever the operator asks.
        if self.guardian != GuardianMode::Nominal {
            throttle = 0;
            brake = LAW_STOP_BRAKE_PERMILLE;
        }
        Ok(self.limiter.apply(steer, brake, throttle))
    }

    /// Records the guardian's latest override. Call once per control cycle.
    pub fn set_guardian(&mut self, command: &GuardianCommand) {
        self.guardian = command.mode;
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

/// The bridge error for a refused command. The message is static, so a refusal
/// does not allocate.
fn refusal(error: CommandError) -> DomainError {
    DomainError::adapter(match error {
        CommandError::Malformed => "command malformed or from an unknown protocol version",
        CommandError::Replay => "command sequence did not increase",
        CommandError::ModeMismatch { .. } => "command was made for a mode the vehicle is not in",
        CommandError::NotActive => "vehicle is in autonomy and takes no operator input",
    })
}

impl DomainTeleopInterface for TeleopAdapter {
    fn on_teleop_engage(&mut self, operator: OperatorId) -> tpt_t_domain_bridge::Result<()> {
        if self.operator.is_some() {
            return Err(DomainError::adapter(
                "an operator already holds the vehicle",
            ));
        }
        self.operator = Some(operator);
        Ok(())
    }

    fn on_teleop_disengage(&mut self) -> tpt_t_domain_bridge::Result<()> {
        self.operator
            .take()
            .map(|_| ())
            .ok_or_else(|| DomainError::adapter("no operator holds the vehicle"))
    }

    fn on_control_command(&mut self, cmd: ControlCommand) -> tpt_t_domain_bridge::Result<()> {
        if self.operator.is_none() {
            return Err(DomainError::adapter(
                "command received with no operator engaged",
            ));
        }
        let actuation = self.apply_command(&cmd).map_err(refusal)?;
        self.actuation = Some(actuation);
        Ok(())
    }

    fn get_domain_state(&self) -> DomainState {
        self.state
    }

    /// Drains the queued video and point cloud. Each one is sent once, so a
    /// second call in the same tick returns none. Haptics and audio have no
    /// source in this crate yet, so they are always empty.
    fn get_sensor_feed(&mut self) -> SensorFeed {
        SensorFeed {
            timestamp_us: self.state.timestamp_us,
            video_frames: self.video.take().into_iter().collect(),
            point_clouds: self.point_cloud.take().into_iter().collect(),
            telemetry: self.state,
            haptic_data: HapticFeedback {
                force_torque: [0.0; 6],
            },
            audio: None,
        }
    }

    fn on_emergency_stop(&mut self) {
        self.emergency_stop();
        self.actuation = Some(self.limiter.emergency_brake());
        self.operator = None;
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
            a.apply_command(&cmd(1, Mode::Auto, 0.0, 0.5)),
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
            .apply_command(&cmd(1, Mode::FullTeleop, 0.0, 1.0))
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
            .apply_command(&cmd(1, Mode::FullTeleop, 1.0, -1.0))
            .unwrap();
        assert!(out.brake_permille > 0);
        assert!(out.steer_cdeg > 0);
    }

    #[test]
    fn assist_passes_steering_but_holds_throttle() {
        let a = TeleopAdapter::new();
        a.request_mode(Mode::Assist).unwrap();
        let mut a = a;
        let out = a.apply_command(&cmd(1, Mode::Assist, 0.5, 1.0)).unwrap();
        assert_eq!(out.throttle_permille, 0);
        assert!(out.steer_cdeg > 0);
    }

    #[test]
    fn replayed_or_out_of_order_commands_are_refused() {
        let mut a = teleop();
        a.apply_command(&cmd(5, Mode::FullTeleop, 0.0, 0.0))
            .unwrap();
        assert_eq!(
            a.apply_command(&cmd(5, Mode::FullTeleop, 0.0, 0.0)),
            Err(CommandError::Replay)
        );
        assert_eq!(
            a.apply_command(&cmd(4, Mode::FullTeleop, 0.0, 0.0)),
            Err(CommandError::Replay)
        );
    }

    #[test]
    fn command_for_a_stale_mode_is_refused() {
        let mut a = teleop();
        assert_eq!(
            a.apply_command(&cmd(1, Mode::Assist, 0.0, 0.0)),
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
        assert_eq!(a.apply_command(&bad), Err(CommandError::Malformed));
        assert!(a.apply_command(&cmd(3, Mode::FullTeleop, 0.0, 0.0)).is_ok());
    }

    #[test]
    fn operator_estop_latches_and_brakes_in_any_mode() {
        let mut a = teleop();
        let out = a
            .apply_command(&cmd(1, Mode::EmergencyStop, 0.0, 1.0))
            .unwrap();
        assert_eq!(a.mode(), Mode::EmergencyStop);
        assert!(out.brake_permille > 0);
        assert_eq!(out.throttle_permille, 0);
        // Further drive commands are still braked, never throttled.
        let later = a
            .apply_command(&cmd(2, Mode::FullTeleop, 0.0, 1.0))
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
                .apply_command(&cmd(seq, Mode::FullTeleop, 0.0, 1.0))
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

    use tpt_t_domain_bridge::wire::FAULT_OVER_TEMP;

    fn estimate(x: f32, y: f32) -> Estimate {
        Estimate {
            x,
            y,
            variance: 1.0,
        }
    }

    fn stop_obs(distance_mm: i32, speed: u32) -> Observation {
        Observation {
            stop_sign: true,
            distance_to_line_mm: distance_mm,
            speed_mm_s: speed,
            speed_limit_mm_s: 13_900,
            conflict_on_right: false,
        }
    }

    #[test]
    fn localization_clears_the_sensor_fault_and_sets_the_pose() {
        let mut a = TeleopAdapter::new();
        assert_ne!(a.get_domain_state().fault_flags & FAULT_SENSOR, 0);
        a.update_localization(estimate(12.5, -3.0), 42);
        let state = a.get_domain_state();
        assert_eq!(state.fault_flags & FAULT_SENSOR, 0);
        assert_eq!(state.pose.position, [12.5, -3.0, 0.0]);
        assert_eq!(state.timestamp_us, 42);
    }

    #[test]
    fn faults_can_be_raised_and_cleared() {
        let mut a = TeleopAdapter::new();
        a.set_fault(FAULT_OVER_TEMP, true);
        a.set_fault(FAULT_OVER_TEMP, true);
        assert_ne!(a.get_domain_state().fault_flags & FAULT_OVER_TEMP, 0);
        a.set_fault(FAULT_OVER_TEMP, false);
        assert_eq!(a.get_domain_state().fault_flags & FAULT_OVER_TEMP, 0);
    }

    #[test]
    fn compliance_stop_caps_the_operator_throttle() {
        let mut a = teleop();
        a.begin_stop_approach();
        let eval = a.update_compliance(&stop_obs(0, 0));
        assert_eq!(eval.decision, Decision::Stop);
        // The brake slew-rates up, so run enough ticks to reach the full value.
        let mut out = Actuation::default();
        for seq in 1..=12 {
            out = a
                .apply_command(&cmd(seq, Mode::FullTeleop, 0.0, 1.0))
                .unwrap();
        }
        assert_eq!(out.throttle_permille, 0);
        assert_eq!(out.brake_permille, LAW_STOP_BRAKE_PERMILLE);
    }

    #[test]
    fn stop_sign_is_ignored_until_the_approach_begins() {
        let mut a = teleop();
        let eval = a.update_compliance(&stop_obs(0, 0));
        assert_eq!(eval.decision, Decision::Proceed);
    }

    #[test]
    fn video_is_redacted_before_it_is_queued() {
        let (w, h) = (16u32, 16u32);
        // A varying pattern, since a blur leaves a flat region unchanged.
        let original: Vec<u8> = (0..w * h).map(|i| ((i * 7) % 251) as u8).collect();
        let detections = [Rect {
            x: 4,
            y: 4,
            w: 6,
            h: 6,
        }];
        let mut a = TeleopAdapter::new();
        a.push_video(original.clone(), w, h, 7, &detections)
            .unwrap();
        let feed = a.get_sensor_feed();
        assert_eq!(feed.video_frames.len(), 1);
        let frame = &feed.video_frames[0];
        assert_eq!(
            (frame.width, frame.height, frame.kind),
            (w, h, VideoKind::Rgb)
        );
        assert_eq!(frame.timestamp_us, 7);
        // Outside the detection the pixels are untouched; inside they are blurred.
        let inside = |x: u32, y: u32| {
            detections.iter().any(|r| {
                (r.x as u32..(r.x + r.w) as u32).contains(&x)
                    && (r.y as u32..(r.y + r.h) as u32).contains(&y)
            })
        };
        for (i, (raw, out)) in original.iter().zip(&frame.data).enumerate() {
            let (x, y) = (i as u32 % w, i as u32 / w);
            if !inside(x, y) {
                assert_eq!(raw, out, "pixel ({x}, {y}) outside the detection changed");
            }
        }
        assert_ne!(frame.data, original);
    }

    #[test]
    fn sensor_feed_drains_queued_media_once() {
        let mut a = TeleopAdapter::new();
        a.push_video(vec![0u8; 4], 2, 2, 1, &[]).unwrap();
        a.push_point_cloud(PointCloud {
            timestamp_us: 1,
            points: vec![[1.0, 2.0, 3.0]],
        });
        let first = a.get_sensor_feed();
        assert_eq!(first.video_frames.len(), 1);
        assert_eq!(first.point_clouds.len(), 1);
        let second = a.get_sensor_feed();
        assert!(second.video_frames.is_empty());
        assert!(second.point_clouds.is_empty());
    }

    #[test]
    fn video_with_the_wrong_size_is_refused() {
        let mut a = TeleopAdapter::new();
        assert_eq!(
            a.push_video(vec![0u8; 3], 2, 2, 1, &[]),
            Err(FrameError::SizeMismatch)
        );
        assert!(a.get_sensor_feed().video_frames.is_empty());
    }
}
