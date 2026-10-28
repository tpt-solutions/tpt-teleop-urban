//! Emergency-vehicle preemption: the car's response to an emergency vehicle that
//! is closing in.
//!
//! The input is an [`Alert`], not a wire message. SRM decoding is not implemented
//! (Phase 3), so the alert can come from any source that can fill it in, and the
//! response does not depend on how it arrived.
//!
//! The response is the minimal-risk maneuver from [`crate::pull_over`]: pull to
//! the shoulder if it fits in the road ahead, otherwise stop in place. An alert
//! that is not closing, or that is still further away than [`PREEMPT_WINDOW_S`],
//! changes nothing.

use crate::pull_over::{Maneuver, choose};

/// An emergency vehicle's alert, as the car understands it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Alert {
    /// Estimated time until the emergency vehicle reaches the car, in seconds.
    pub time_to_arrival_s: f32,
    /// Whether the emergency vehicle is closing on the car.
    pub closing: bool,
}

/// An alert further away than this does not yet change the car's behaviour, in
/// seconds.
pub const PREEMPT_WINDOW_S: f32 = 30.0;

/// What the car does about the alert this cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Response {
    /// No emergency vehicle is near enough to act on.
    Continue,
    /// Pull over to the shoulder along a path of `length_m`.
    PullOver {
        /// Length of the pull-over path, in metres.
        length_m: f32,
    },
    /// No shoulder fits in the road ahead, so stop in place.
    StopInPlace,
}

/// The car's response to `alert`, given its speed, the shoulder's offset, the
/// road available ahead, and the lateral acceleration it may use.
///
/// All inputs use SI units: m/s, m, m and m/s².
#[must_use]
pub fn respond(
    alert: Option<Alert>,
    speed_m_s: f32,
    offset_m: f32,
    available_m: f32,
    lateral_accel: f32,
) -> Response {
    let Some(alert) = alert else {
        return Response::Continue;
    };
    if !alert.closing || alert.time_to_arrival_s > PREEMPT_WINDOW_S {
        return Response::Continue;
    }
    match choose(speed_m_s, offset_m, available_m, lateral_accel) {
        Maneuver::PullOver { length_m } => Response::PullOver { length_m },
        Maneuver::BrakeInPlace => Response::StopInPlace,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CLOSE: Alert = Alert {
        time_to_arrival_s: 10.0,
        closing: true,
    };

    #[test]
    fn no_alert_changes_nothing() {
        assert_eq!(respond(None, 10.0, 3.0, 200.0, 2.0), Response::Continue);
    }

    #[test]
    fn an_alert_that_is_not_closing_changes_nothing() {
        let receding = Alert {
            closing: false,
            ..CLOSE
        };
        assert_eq!(
            respond(Some(receding), 10.0, 3.0, 200.0, 2.0),
            Response::Continue
        );
    }

    #[test]
    fn a_distant_alert_changes_nothing() {
        let far = Alert {
            time_to_arrival_s: PREEMPT_WINDOW_S + 1.0,
            ..CLOSE
        };
        assert_eq!(
            respond(Some(far), 10.0, 3.0, 200.0, 2.0),
            Response::Continue
        );
    }

    #[test]
    fn a_closing_alert_pulls_over_when_the_shoulder_fits() {
        let Response::PullOver { length_m } = respond(Some(CLOSE), 10.0, 3.0, 200.0, 2.0) else {
            panic!("a 200 m road fits a pull-over at 10 m/s");
        };
        assert!(length_m > 0.0 && length_m <= 200.0, "{length_m}");
    }

    #[test]
    fn a_closing_alert_stops_in_place_when_no_shoulder_fits() {
        assert_eq!(
            respond(Some(CLOSE), 30.0, 3.0, 5.0, 2.0),
            Response::StopInPlace
        );
    }
}
