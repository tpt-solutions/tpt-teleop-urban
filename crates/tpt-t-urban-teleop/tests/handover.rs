//! The handover trigger, end to end: the urban guardian's output decides whether
//! the teleop session asks for an operator.

use tpt_t_core::Mode as CoreMode;
use tpt_t_core::ser::ControlCommand;
use tpt_t_domain_bridge::{
    AssistanceReason, DomainError, DomainSession, HandoverState, OperatorId, TeleopLink,
};
use tpt_t_urban_safety::{Guardian, Inputs, Mode as GuardianMode, Obstacle};
use tpt_t_urban_teleop::{
    AXIS_THROTTLE, TeleopAdapter, assistance_reason, clear_mrm, ready_to_return,
};

/// Records the reasons it was asked for, or fails every request.
#[derive(Default)]
struct Link {
    requests: Vec<AssistanceReason>,
    down: bool,
}

impl TeleopLink for Link {
    fn request_teleop_assistance(
        &mut self,
        reason: AssistanceReason,
    ) -> tpt_t_domain_bridge::Result<()> {
        if self.down {
            return Err(DomainError::adapter("link down"));
        }
        self.requests.push(reason);
        Ok(())
    }
}

fn quiet_inputs() -> Inputs {
    Inputs {
        obstacle: None,
        planned_decel_mm_s2: 0,
        planner_heartbeat_age_ms: 0,
        speed_mm_s: 8_000,
    }
}

#[test]
fn stale_planner_latches_mrm_and_requests_assistance() {
    let mut guardian = Guardian::new();
    let stale = Inputs {
        planner_heartbeat_age_ms: 200,
        ..quiet_inputs()
    };
    let command = guardian.step(&stale);
    assert_eq!(command.mode, GuardianMode::Mrm);

    let reason = assistance_reason(&command).expect("MRM asks for a human");
    let mut session = DomainSession::new(TeleopAdapter::new());
    let mut link = Link::default();
    session.request_assistance(&mut link, reason).unwrap();

    assert_eq!(session.state(), HandoverState::RequestingTeleop);
    assert_eq!(link.requests, vec![AssistanceReason::Degraded]);
}

#[test]
fn aeb_recovers_on_its_own_and_asks_for_nothing() {
    let mut guardian = Guardian::new();
    let closing = Inputs {
        obstacle: Some(Obstacle {
            range_mm: 2_000,
            closing_speed_mm_s: 2_000,
        }),
        ..quiet_inputs()
    };
    let command = guardian.step(&closing);
    assert_eq!(command.mode, GuardianMode::Aeb);
    assert_eq!(assistance_reason(&command), None);
}

#[test]
fn nominal_driving_asks_for_nothing() {
    let mut guardian = Guardian::new();
    let command = guardian.step(&quiet_inputs());
    assert_eq!(command.mode, GuardianMode::Nominal);
    assert_eq!(assistance_reason(&command), None);
}

#[test]
fn return_waits_until_the_mrm_is_cleared_at_rest() {
    let mut guardian = Guardian::new();
    guardian.step(&Inputs {
        planner_heartbeat_age_ms: 200,
        ..quiet_inputs()
    });
    assert!(!ready_to_return(&guardian));

    // Still moving: the latch holds.
    assert!(!clear_mrm(&mut guardian, 2_000));
    assert!(!ready_to_return(&guardian));

    // At rest the latch clears, and the vehicle may hand back.
    assert!(clear_mrm(&mut guardian, 0));
    assert!(ready_to_return(&guardian));
}

#[test]
fn full_handover_round_trip_returns_to_autonomy() {
    let mut guardian = Guardian::new();
    let mut session = DomainSession::new(TeleopAdapter::new());
    session.adapter().request_mode(CoreMode::Assist).unwrap();
    session
        .adapter()
        .request_mode(CoreMode::FullTeleop)
        .unwrap();

    // The planner stalls, the guardian latches, and an operator is asked for.
    let command = guardian.step(&Inputs {
        planner_heartbeat_age_ms: 200,
        ..quiet_inputs()
    });
    let mut link = Link::default();
    session
        .request_assistance(&mut link, assistance_reason(&command).unwrap())
        .unwrap();
    session.engage(OperatorId::new([9; 16])).unwrap();
    session.disengage().unwrap();
    assert_eq!(session.state(), HandoverState::ReturningToAutonomy);

    // The core must not complete the handover while the MRM is still latched.
    assert!(!ready_to_return(&guardian));

    clear_mrm(&mut guardian, 0);
    assert!(ready_to_return(&guardian));
    session.complete_handover().unwrap();
    assert_eq!(session.state(), HandoverState::Autonomous);
}

#[test]
fn guardian_mrm_overrides_operator_throttle() {
    let mut guardian = Guardian::new();
    let command = guardian.step(&Inputs {
        planner_heartbeat_age_ms: 200,
        ..quiet_inputs()
    });

    let mut adapter = TeleopAdapter::new();
    adapter.set_guardian(&command);
    adapter.request_mode(CoreMode::Assist).unwrap();
    adapter.request_mode(CoreMode::FullTeleop).unwrap();
    let mut session = DomainSession::new(adapter);
    session
        .request_assistance(&mut Link::default(), AssistanceReason::Degraded)
        .unwrap();

    // The operator is allowed to engage while the MRM is latched.
    session.engage(OperatorId::new([3; 16])).unwrap();

    let mut full_throttle = ControlCommand::zeroed(CoreMode::FullTeleop);
    full_throttle.seq = 1;
    full_throttle.axes[AXIS_THROTTLE] = 1.0;
    session.command(full_throttle).unwrap();

    let out = session.adapter().last_actuation().unwrap();
    assert_eq!(out.throttle_permille, 0);
    assert!(out.brake_permille > 0);
}

#[test]
fn failed_request_leaves_the_vehicle_autonomous() {
    let mut session = DomainSession::new(TeleopAdapter::new());
    let mut link = Link {
        down: true,
        ..Link::default()
    };
    assert!(
        session
            .request_assistance(&mut link, AssistanceReason::Degraded)
            .is_err()
    );
    assert_eq!(session.state(), HandoverState::Autonomous);
    assert_eq!(session.mode(), CoreMode::Auto);
}
