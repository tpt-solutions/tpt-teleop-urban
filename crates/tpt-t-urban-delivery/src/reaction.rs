//! Jaywalker reaction: the predictor's trajectory cone becomes a yield decision,
//! and a deceleration that is smooth and bounded, so the guardian's AEB is only a
//! backstop.
//!
//! The bot moves along a lane centred on `y = 0` at the speed it reports. A
//! pedestrian's cone is a sequence of predicted discs. The first disc that reaches
//! the lane is the conflict. Its distance ahead of the bot is where the bot has to
//! stop, and its time is when the pedestrian arrives there.

use tpt_t_urban_predict::ConePoint;

use crate::yield_ped::{Decision, Pedestrian};

/// The largest deceleration the reaction asks for, in mm/s² (3 m/s²). It is well
/// inside the guardian's AEB deceleration, so the reaction stops the bot before the
/// backstop has to.
pub const MAX_REACTION_DECEL_MM_S2: u32 = 3_000;

/// How far short of the conflict point the bot stops, in metres. It is more than
/// the guardian's own stop margin (1 m) plus the distance covered in its reaction
/// time, so braking that meets this target also satisfies the guardian's check.
pub const STOP_SHORT_M: f32 = 2.0;

/// Time, in seconds, in which a bot holding with no conflict comes to rest.
pub const HOLD_STOP_S: f32 = 1.0;

/// A pedestrian's first predicted conflict with the bot's lane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Conflict {
    /// The conflict as the yield controller sees it.
    pub pedestrian: Pedestrian,
    /// Distance from the bot to the conflict point along the lane, in metres.
    pub distance_m: f32,
}

/// The pedestrian's first predicted conflict with the bot's lane, or `None`.
///
/// A disc conflicts when it reaches the lane, that is when `|y| - radius` is within
/// `lane_half_m`. A disc the bot has already passed, at the bot's predicted position
/// for its time, is ignored.
#[must_use]
pub fn conflict_from_cone(
    cone: &[ConePoint],
    bot_x_m: f32,
    bot_speed_m_s: f32,
    lane_half_m: f32,
) -> Option<Conflict> {
    cone.iter().find_map(|p| {
        let bot_at_t = bot_x_m + bot_speed_m_s * p.t_s;
        if p.x < bot_at_t {
            return None;
        }
        (p.y.abs() - p.radius <= lane_half_m).then(|| Conflict {
            pedestrian: Pedestrian {
                time_to_path_ms: (p.t_s * 1_000.0).round() as u32,
                lateral_mm: (p.y.abs() * 1_000.0).round() as u32,
            },
            distance_m: p.x - bot_x_m,
        })
    })
}

/// The deceleration the bot should apply this cycle, in mm/s².
///
/// Proceeding asks for none. Yielding asks for the deceleration that stops the bot
/// [`STOP_SHORT_M`] short of the conflict point, `v² / 2d`. It grows as the bot
/// nears the point, and it is capped at [`MAX_REACTION_DECEL_MM_S2`]. Holding with no
/// conflict in view brakes gently, to rest in one second. A bot that is already
/// stopped asks for none.
#[must_use]
pub fn reaction_decel_mm_s2(
    decision: Decision,
    conflict: Option<Conflict>,
    speed_mm_s: u32,
) -> u32 {
    let v = speed_mm_s as f32 / 1_000.0;
    match (decision, conflict) {
        (Decision::Proceed, _) => 0,
        (_, _) if speed_mm_s == 0 => 0,
        // The Yielder only yields to a conflict, so this is a defensive cap.
        (Decision::Yield, None) => MAX_REACTION_DECEL_MM_S2,
        (Decision::Hold, None) => ((v / HOLD_STOP_S) * 1_000.0) as u32,
        (_, Some(c)) => {
            let d = (c.distance_m - STOP_SHORT_M).max(0.05);
            let decel = (v * v / (2.0 * d) * 1_000.0) as u32;
            decel.min(MAX_REACTION_DECEL_MM_S2)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn disc(t_s: f32, x: f32, y: f32, radius: f32) -> ConePoint {
        ConePoint { t_s, x, y, radius }
    }

    fn conflict(distance_m: f32) -> Conflict {
        Conflict {
            pedestrian: Pedestrian {
                time_to_path_ms: 1_000,
                lateral_mm: 0,
            },
            distance_m,
        }
    }

    #[test]
    fn a_pedestrian_far_from_the_lane_is_no_conflict() {
        let cone = [disc(0.0, 12.0, -6.0, 0.3), disc(1.0, 12.0, -5.0, 0.3)];
        assert_eq!(conflict_from_cone(&cone, 0.0, 5.0, 0.5), None);
    }

    #[test]
    fn the_first_disc_that_reaches_the_lane_gives_the_conflict_distance() {
        let cone = [
            disc(0.0, 12.0, -3.0, 0.3),
            disc(1.0, 12.0, -1.0, 0.6),
            disc(2.0, 12.0, 0.0, 0.6),
        ];
        assert_eq!(
            conflict_from_cone(&cone, 2.0, 5.0, 0.5),
            Some(Conflict {
                pedestrian: Pedestrian {
                    time_to_path_ms: 1_000,
                    lateral_mm: 1_000,
                },
                distance_m: 10.0,
            })
        );
    }

    #[test]
    fn a_pedestrian_the_bot_has_already_passed_is_ignored() {
        // At 5 m/s from x = 0 the bot is at x = 5 m at t = 1 s, past a disc at 2 m.
        let cone = [disc(1.0, 2.0, 0.0, 0.5)];
        assert_eq!(conflict_from_cone(&cone, 0.0, 5.0, 0.5), None);
    }

    #[test]
    fn proceeding_asks_for_no_deceleration() {
        assert_eq!(reaction_decel_mm_s2(Decision::Proceed, None, 5_000), 0);
    }

    #[test]
    fn yielding_stops_short_of_the_conflict_at_the_needed_rate() {
        // v² / 2d with v = 5 m/s and d = 15 - 2 = 13 m, which is 0.96 m/s².
        let decel = reaction_decel_mm_s2(Decision::Yield, Some(conflict(15.0)), 5_000);
        assert!(decel.abs_diff(961) <= 1, "{decel}");
    }

    #[test]
    fn the_request_grows_as_the_conflict_nears_and_stops_at_the_cap() {
        let far = reaction_decel_mm_s2(Decision::Yield, Some(conflict(15.0)), 5_000);
        let near = reaction_decel_mm_s2(Decision::Yield, Some(conflict(4.0)), 5_000);
        let at = reaction_decel_mm_s2(Decision::Yield, Some(conflict(1.0)), 5_000);
        assert!(far < near && near <= at);
        assert_eq!(at, MAX_REACTION_DECEL_MM_S2);
    }

    #[test]
    fn holding_with_no_conflict_comes_to_rest_in_a_second() {
        // 2 m/s over one second is 2 m/s².
        assert_eq!(reaction_decel_mm_s2(Decision::Hold, None, 2_000), 2_000);
    }

    #[test]
    fn a_stopped_bot_asks_for_nothing_more() {
        assert_eq!(reaction_decel_mm_s2(Decision::Hold, None, 0), 0);
    }
}
