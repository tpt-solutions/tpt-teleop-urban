//! The urban adapter driven through tpt-teleop's `DomainSession`, the way the
//! core calls it: handover, operator commands, and emergency stop.

use tpt_t_core::Mode;
use tpt_t_core::ser::ControlCommand;
use tpt_t_domain_bridge::wire::FAULT_SENSOR;
use tpt_t_domain_bridge::{
    AssistanceReason, DomainError, DomainSession, DomainState, DomainTeleopInterface, OperatorId,
    Pose6DOF, TeleopLink,
};
use tpt_t_urban_edge::actuator::MAX_PERMILLE;
use tpt_t_urban_teleop::{AXIS_STEERING, AXIS_THROTTLE, TeleopAdapter};

/// A teleop link that accepts every request.
struct Link;

impl TeleopLink for Link {
    fn request_teleop_assistance(
        &mut self,
        _reason: AssistanceReason,
    ) -> tpt_t_domain_bridge::Result<()> {
        Ok(())
    }
}

fn operator() -> OperatorId {
    OperatorId::new([7; 16])
}

fn command(seq: u64, mode: Mode, steering: f32, pedal: f32) -> ControlCommand {
    let mut c = ControlCommand::zeroed(mode);
    c.seq = seq;
    c.axes[AXIS_STEERING] = steering;
    c.axes[AXIS_THROTTLE] = pedal;
    c
}

/// Session with the adapter staged through the legal Auto → Assist → FullTeleop
/// path, as the core does before it engages an operator.
fn staged_session() -> DomainSession<TeleopAdapter> {
    let session = DomainSession::new(TeleopAdapter::new());
    session.adapter().request_mode(Mode::Assist).unwrap();
    session.adapter().request_mode(Mode::FullTeleop).unwrap();
    session
}

#[test]
fn engage_command_disengage_cycle() {
    let mut session = staged_session();
    session
        .request_assistance(&mut Link, AssistanceReason::Degraded)
        .unwrap();
    session.engage(operator()).unwrap();

    session
        .command(command(1, Mode::FullTeleop, 0.0, 1.0))
        .unwrap();
    let out = session.adapter().last_actuation().expect("command applied");
    assert!(out.throttle_permille > 0);

    session.disengage().unwrap();
    assert_eq!(session.operator(), None);
    assert!(matches!(
        session.command(command(2, Mode::FullTeleop, 0.0, 1.0)),
        Err(DomainError::NotEngaged { .. })
    ));
}

#[test]
fn command_before_engage_never_reaches_the_adapter() {
    let mut session = staged_session();
    assert!(
        session
            .command(command(1, Mode::FullTeleop, 0.0, 1.0))
            .is_err()
    );
    assert!(session.adapter().last_actuation().is_none());
}

#[test]
fn emergency_stop_brakes_and_latches() {
    let mut session = staged_session();
    session
        .request_assistance(&mut Link, AssistanceReason::Degraded)
        .unwrap();
    session.engage(operator()).unwrap();
    session
        .command(command(1, Mode::FullTeleop, 0.0, 1.0))
        .unwrap();

    session.emergency_stop();
    // The stop bypasses the slew limit: full brake on the first tick.
    let out = session.adapter().last_actuation().expect("stop applied");
    assert_eq!(out.brake_permille, MAX_PERMILLE);
    assert_eq!(out.throttle_permille, 0);
    assert_eq!(session.adapter().mode(), Mode::EmergencyStop);

    // Once stopped, a throttle command cannot move the unit.
    assert!(
        session
            .command(command(2, Mode::FullTeleop, 0.0, 1.0))
            .is_err()
    );
}

#[test]
fn adapter_refuses_a_command_the_machine_cannot_take() {
    // Engaged directly on the adapter while the machine is still in Auto, which
    // the session would not allow. The adapter's own gate must still refuse it.
    let mut adapter = TeleopAdapter::new();
    adapter.on_teleop_engage(operator()).unwrap();
    let err = adapter
        .on_control_command(command(1, Mode::Auto, 0.0, 0.5))
        .unwrap_err();
    assert!(matches!(err, DomainError::Adapter(msg) if msg.contains("autonomy")));
}

#[test]
fn reports_sensor_fault_until_state_is_supplied() {
    let mut adapter = TeleopAdapter::new();
    assert_eq!(adapter.get_domain_state().fault_flags, FAULT_SENSOR);
    assert!(!adapter.get_domain_state().is_healthy());

    adapter.update_state(DomainState {
        timestamp_us: 42,
        pose: Pose6DOF::IDENTITY,
        battery_pct: 80.0,
        fault_flags: 0,
    });
    let mut session = DomainSession::new(adapter);
    assert!(session.domain_state().is_healthy());
    assert_eq!(session.sensor_feed().timestamp_us, 42);
}
