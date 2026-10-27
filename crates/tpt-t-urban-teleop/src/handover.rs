//! Handover trigger: when autonomy can no longer keep the vehicle safe, ask for
//! a teleop operator.
//!
//! The bridge spec has the domain repo watch its own health and call
//! `request_teleop_assistance()` when autonomy cannot continue safely. Here the
//! signal is the guardian's output:
//!
//! - **MRM** latches when the planner heartbeat goes stale. The planner is not
//!   coming back on its own, so a human is asked for. This is the only case
//!   that requests assistance.
//! - **AEB** is automatic and clears once the closing speed is gone. The
//!   vehicle is recovering by itself, so no request is made.

use tpt_t_domain_bridge::AssistanceReason;
use tpt_t_urban_safety::{Command, Guardian, Mode};

/// The assistance request for one guardian output, or `None` if autonomy can
/// carry on.
#[must_use]
pub fn assistance_reason(command: &Command) -> Option<AssistanceReason> {
    match command.mode {
        Mode::Mrm => Some(AssistanceReason::Degraded),
        Mode::Nominal | Mode::Aeb => None,
    }
}

/// Whether control can return to autonomy: the guardian is not holding an MRM.
///
/// The session's `complete_handover` does not check this, so the caller must.
/// A latched MRM means the planner has not recovered, and the vehicle must not
/// hand back to it. Clear the latch first with [`clear_mrm`].
#[must_use]
pub fn ready_to_return(guardian: &Guardian) -> bool {
    guardian.mode() == Mode::Nominal
}

/// Clears a latched MRM once the vehicle is at rest. Returns whether it cleared.
pub fn clear_mrm(guardian: &mut Guardian, speed_mm_s: u32) -> bool {
    guardian.reset(speed_mm_s)
}
