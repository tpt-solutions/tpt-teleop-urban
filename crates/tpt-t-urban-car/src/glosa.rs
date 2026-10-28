//! Green Light Optimal Speed Advisory (GLOSA).
//!
//! Given the distance to a stop line and the signal timing from SPAT, the
//! advisory is the speed that reaches the line at a time the signal allows:
//!
//! - **Green**: if the current speed reaches the line before red, keep it. If
//!   not, slow so the line is reached as the light turns red, so the vehicle
//!   never has to stop harshly.
//! - **Red**: arrive no earlier than the start of green, and advise a speed
//!   that does that without exceeding the limit.
//!
//! When the required speed falls below the minimum, the advisory is a stop
//! and `Advice::Stop` is returned.

/// Signal state and timing from SPAT, in seconds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Signal {
    /// Green now; `time_to_red_s` until it turns red.
    Green {
        /// Seconds until the light turns red.
        time_to_red_s: f32,
    },
    /// Red now; `time_to_green_s` until it turns green.
    Red {
        /// Seconds until the light turns green.
        time_to_green_s: f32,
    },
}

/// Speed bounds and distance for one advisory.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Context {
    /// Distance to the stop line, metres.
    pub distance_m: f32,
    /// Current speed, m/s.
    pub speed_m_s: f32,
    /// Posted limit, m/s.
    pub speed_limit_m_s: f32,
    /// Lowest speed the vehicle should crawl at, m/s.
    pub min_speed_m_s: f32,
}

/// Advisory for the approaching vehicle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Advice {
    /// Hold or move to this speed, m/s.
    Speed(f32),
    /// The signal cannot be met without stopping. Stop at the line.
    Stop,
}

/// Computes the advisory speed for `ctx` given the signal.
#[must_use]
pub fn advise(signal: Signal, ctx: &Context) -> Advice {
    if ctx.distance_m <= 0.0 {
        return Advice::Speed(ctx.speed_m_s.min(ctx.speed_limit_m_s));
    }
    match signal {
        Signal::Green { time_to_red_s } => {
            // Time to reach the line at the current speed.
            let eta = if ctx.speed_m_s > 0.0 {
                ctx.distance_m / ctx.speed_m_s
            } else {
                f32::INFINITY
            };
            if eta < time_to_red_s {
                return Advice::Speed(ctx.speed_m_s.min(ctx.speed_limit_m_s));
            }
            // Too slow to make this green: speed up to reach the line as it turns red.
            if time_to_red_s <= 0.0 {
                return Advice::Stop;
            }
            clamp_or_stop(ctx.distance_m / time_to_red_s, ctx)
        }
        Signal::Red { time_to_green_s } => {
            if time_to_green_s <= 0.0 {
                return Advice::Speed(ctx.speed_m_s.min(ctx.speed_limit_m_s));
            }
            // Arrive no earlier than green: speed at most distance / time_to_green.
            let needed = ctx.distance_m / time_to_green_s;
            if needed < ctx.min_speed_m_s {
                return Advice::Stop;
            }
            Advice::Speed(needed.min(ctx.speed_limit_m_s))
        }
    }
}

/// The green-light speed `needed` is only usable if it is within the limit.
/// Above the limit the line cannot be reached before red, so the vehicle must
/// stop and wait for the next green.
fn clamp_or_stop(needed: f32, ctx: &Context) -> Advice {
    if needed > ctx.speed_limit_m_s {
        Advice::Stop
    } else {
        Advice::Speed(needed.max(ctx.min_speed_m_s))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx(distance_m: f32, speed_m_s: f32) -> Context {
        Context {
            distance_m,
            speed_m_s,
            speed_limit_m_s: 13.9,
            min_speed_m_s: 1.0,
        }
    }

    #[test]
    fn keeps_speed_when_the_line_is_reached_before_red() {
        // 100 m at 10 m/s: 10 s. Red in 20 s.
        let a = advise(
            Signal::Green {
                time_to_red_s: 20.0,
            },
            &ctx(100.0, 10.0),
        );
        assert_eq!(a, Advice::Speed(10.0));
    }

    #[test]
    fn speeds_up_to_reach_the_line_as_red_starts() {
        // 100 m, red in 20 s, at 3 m/s the line would be reached after red (33 s).
        // Needs 5 m/s, which is under the limit.
        let a = advise(
            Signal::Green {
                time_to_red_s: 20.0,
            },
            &ctx(100.0, 3.0),
        );
        match a {
            Advice::Speed(v) => assert!((v - 5.0).abs() < 1e-4, "speed {v}"),
            Advice::Stop => panic!("expected a speed"),
        }
    }

    #[test]
    fn stops_when_the_line_cannot_be_reached_before_red() {
        // 100 m, red in 0.5 s: needs 200 m/s, far above the limit. Wait for green.
        let a = advise(Signal::Green { time_to_red_s: 0.5 }, &ctx(100.0, 13.0));
        assert_eq!(a, Advice::Stop);
    }

    #[test]
    fn red_light_advises_arrival_at_green() {
        // 60 m, green in 12 s: arrive at 5 m/s.
        let a = advise(
            Signal::Red {
                time_to_green_s: 12.0,
            },
            &ctx(60.0, 3.0),
        );
        assert_eq!(a, Advice::Speed(5.0));
    }

    #[test]
    fn red_light_too_close_to_wait_out_advises_stop() {
        // 60 m, green in 100 s: needs 0.6 m/s, below the minimum.
        let a = advise(
            Signal::Red {
                time_to_green_s: 100.0,
            },
            &ctx(60.0, 3.0),
        );
        assert_eq!(a, Advice::Stop);
    }

    #[test]
    fn advised_speed_never_exceeds_the_limit() {
        let a = advise(
            Signal::Red {
                time_to_green_s: 1.0,
            },
            &ctx(100.0, 3.0),
        );
        assert_eq!(a, Advice::Speed(13.9));
    }
}
