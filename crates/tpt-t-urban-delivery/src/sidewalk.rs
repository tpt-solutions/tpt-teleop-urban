//! Sidewalk obstacle avoidance: a smooth detour around static obstacles.
//!
//! The bot follows a straight line along the sidewalk, `x` ahead and `y` across.
//! When an obstacle sits in the way, the planner builds a detour: the lateral
//! offset rises from zero to `offset_m` along a smoothstep, holds, then falls back
//! to zero. Each ramp is at least the minimum length that keeps the lateral
//! acceleration within the limit, from `tpt_t_urban_car::pull_over`.
//!
//! A detour is accepted only if the bot clears every obstacle, with its half
//! width and a margin, and stays inside the sidewalk. The planner tries the
//! smallest offsets first, and the earliest start on the road. If no detour
//! fits, it returns [`Plan::Stop`], which leaves braking to the caller.
//!
//! Units are SI: metres, m/s, m/s².

use tpt_t_urban_car::pull_over;

/// A static obstacle, modeled as a disc.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obstacle {
    /// Position along the sidewalk, in metres.
    pub x_m: f32,
    /// Lateral position across the sidewalk, in metres.
    pub y_m: f32,
    /// Radius, in metres.
    pub radius_m: f32,
}

/// Sidewalk geometry and the bot's clearance requirements.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sidewalk {
    /// Half the walkable width, centred on the nominal line, in metres.
    pub half_width_m: f32,
    /// Half the bot's footprint width, in metres.
    pub bot_half_width_m: f32,
    /// Extra clearance kept beyond the footprint, in metres.
    pub margin_m: f32,
}

/// A detour around the obstacles.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Detour {
    /// Peak lateral offset from the nominal line, in metres, signed.
    pub offset_m: f32,
    /// Where the rise begins, in metres along the sidewalk.
    pub start_m: f32,
    /// Length of each ramp, in metres.
    pub ramp_m: f32,
    /// Where the fall ends and the bot is back on the nominal line, in metres.
    pub end_m: f32,
}

impl Detour {
    /// Lateral offset at `x_m`, in metres.
    #[must_use]
    pub fn lateral_at(&self, x_m: f32) -> f32 {
        let s = |t: f32| t * t * (3.0 - 2.0 * t);
        if x_m <= self.start_m || x_m >= self.end_m {
            0.0
        } else if x_m < self.start_m + self.ramp_m {
            self.offset_m * s((x_m - self.start_m) / self.ramp_m)
        } else if x_m <= self.end_m - self.ramp_m {
            self.offset_m
        } else {
            self.offset_m * s((self.end_m - x_m) / self.ramp_m)
        }
    }
}

/// What the bot should do about the obstacles.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Plan {
    /// The straight line clears every obstacle.
    Straight,
    /// Follow this detour.
    Detour(Detour),
    /// No detour fits. The caller should stop.
    Stop,
}

/// Offset candidates are spaced this far apart, in metres.
const OFFSET_STEP_M: f32 = 0.1;
/// Start positions are spaced this far apart, in metres.
const START_STEP_M: f32 = 0.25;
/// Samples taken along the bot's length around each obstacle, in metres.
const EXTENT_M: f32 = 1.0;
const EXTENT_STEP_M: f32 = 0.25;

/// Whether the profile keeps the bot clear of `obstacle` with its footprint.
fn clears(obstacle: &Obstacle, detour: Option<&Detour>, sidewalk: &Sidewalk) -> bool {
    let needed = obstacle.radius_m + sidewalk.bot_half_width_m + sidewalk.margin_m;
    let mut dx = -EXTENT_M;
    while dx <= EXTENT_M + f32::EPSILON {
        let x = obstacle.x_m + dx;
        let y = detour.map_or(0.0, |d| d.lateral_at(x));
        if (y - obstacle.y_m).abs() < needed {
            return false;
        }
        dx += EXTENT_STEP_M;
    }
    true
}

/// Plans around `obstacles` on a road `road_m` long, at `speed_m_s`, under a
/// lateral-acceleration limit of `lateral_accel`.
#[must_use]
pub fn plan(
    obstacles: &[Obstacle],
    road_m: f32,
    speed_m_s: f32,
    lateral_accel: f32,
    sidewalk: &Sidewalk,
) -> Plan {
    if obstacles.iter().all(|o| clears(o, None, sidewalk)) {
        return Plan::Straight;
    }
    let last_x = obstacles
        .iter()
        .map(|o| o.x_m)
        .fold(f32::NEG_INFINITY, f32::max);
    let room = sidewalk.half_width_m - sidewalk.bot_half_width_m;

    // Offsets from the smallest outward, both sides, so the bot moves as little
    // as it can.
    let mut magnitudes = Vec::new();
    let mut m = OFFSET_STEP_M;
    while m <= room + f32::EPSILON {
        magnitudes.push(m);
        m += OFFSET_STEP_M;
    }
    for magnitude in magnitudes {
        for offset in [magnitude, -magnitude] {
            let ramp = pull_over::min_length(speed_m_s, offset, lateral_accel);
            if !ramp.is_finite() || ramp <= 0.0 {
                continue;
            }
            // The fall ends after the last obstacle, with the bot's length to spare.
            let end_floor = last_x + EXTENT_M + ramp;
            let mut start = 0.0;
            while start + 2.0 * ramp <= road_m {
                let end = (start + 2.0 * ramp).max(end_floor);
                if end > road_m {
                    break;
                }
                let detour = Detour {
                    offset_m: offset,
                    start_m: start,
                    ramp_m: ramp,
                    end_m: end,
                };
                if obstacles.iter().all(|o| clears(o, Some(&detour), sidewalk)) {
                    return Plan::Detour(detour);
                }
                start += START_STEP_M;
            }
        }
    }
    Plan::Stop
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sidewalk() -> Sidewalk {
        Sidewalk {
            half_width_m: 1.5,
            bot_half_width_m: 0.4,
            margin_m: 0.2,
        }
    }

    #[test]
    fn a_clear_sidewalk_needs_no_detour() {
        let obstacles = [Obstacle {
            x_m: 20.0,
            y_m: 1.2,
            radius_m: 0.3,
        }];
        assert_eq!(
            plan(&obstacles, 60.0, 1.5, 0.5, &sidewalk()),
            Plan::Straight
        );
    }

    #[test]
    fn a_centred_obstacle_is_passed_with_clearance() {
        let obstacle = Obstacle {
            x_m: 20.0,
            y_m: 0.0,
            radius_m: 0.3,
        };
        let Plan::Detour(detour) = plan(&[obstacle], 60.0, 1.5, 0.5, &sidewalk()) else {
            panic!("expected a detour");
        };
        // Clearance needed: 0.3 + 0.4 + 0.2 = 0.9 m beside the obstacle.
        assert!(detour.offset_m.abs() >= 0.9, "{detour:?}");
        assert!(
            detour.offset_m.abs() <= 1.1,
            "stays on the sidewalk: {detour:?}"
        );
        let mut x: f32 = 0.0;
        while x <= 60.0 {
            if (x - 20.0).abs() <= 1.0 {
                let y = detour.lateral_at(x);
                assert!((y - obstacle.y_m).abs() >= 0.9 - 1e-3, "x {x}: y {y}");
            }
            x += 0.05;
        }
    }

    #[test]
    fn a_wall_across_the_sidewalk_is_a_stop() {
        let wall: Vec<Obstacle> = [-1.2, 0.0, 1.2]
            .iter()
            .map(|&y_m| Obstacle {
                x_m: 20.0,
                y_m,
                radius_m: 0.5,
            })
            .collect();
        assert_eq!(plan(&wall, 60.0, 1.5, 0.5, &sidewalk()), Plan::Stop);
    }

    #[test]
    fn the_detour_is_the_smallest_that_clears() {
        let obstacle = Obstacle {
            x_m: 20.0,
            y_m: 0.0,
            radius_m: 0.3,
        };
        let Plan::Detour(detour) = plan(&[obstacle], 60.0, 1.5, 0.5, &sidewalk()) else {
            panic!("expected a detour");
        };
        assert!(detour.offset_m.abs() < 1.0, "{detour:?}");
    }
}
