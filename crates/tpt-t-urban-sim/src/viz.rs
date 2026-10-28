//! Drawing model for the visualizer: a crossing run as a list of frames.
//!
//! The model holds no GUI types, so it is tested without a window. The
//! `visualizer` binary writes these frames into an HTML page.

use crate::scenario::{
    CONTACT_M, FRONT_OFFSET_M, LANE_HALF_WIDTH_M, Scenario, TracePoint, run_crossing_traced,
};

/// One drawn step of the run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    /// Time since the start, in seconds.
    pub t_s: f32,
    /// Vehicle reference point, along the lane, in metres.
    pub vehicle_x: f32,
    /// Vehicle speed, in metres per second.
    pub speed_mps: f32,
    /// Pedestrian position `[x, y]`, in metres.
    pub pedestrian: [f32; 2],
    /// Whether the guardian is overriding the planner this step.
    pub intervening: bool,
    /// Gap from the vehicle's front bumper to the pedestrian along the lane, in metres.
    pub gap_m: f32,
    /// Whether the pedestrian is within contact distance of the vehicle.
    pub in_contact: bool,
}

/// The lane and the scene bounds a view needs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    /// Smallest and largest `x` over the run, in metres.
    pub x: [f32; 2],
    /// Smallest and largest `y` over the run, in metres.
    pub y: [f32; 2],
}

/// Converts one trace step into a frame.
#[must_use]
pub fn frame_of(p: &TracePoint) -> Frame {
    let gap_m = p.ped_x - (p.vehicle_x + FRONT_OFFSET_M);
    Frame {
        t_s: p.t_s,
        vehicle_x: p.vehicle_x,
        speed_mps: p.speed_mps,
        pedestrian: [p.ped_x, p.ped_y],
        intervening: p.override_active,
        gap_m,
        in_contact: gap_m.abs() < CONTACT_M && p.ped_y.abs() < CONTACT_M,
    }
}

/// Runs the scenario and returns one frame per simulated step.
#[must_use]
pub fn frames(s: &Scenario) -> Vec<Frame> {
    let mut out = Vec::new();
    let _outcome = run_crossing_traced(s, |p| out.push(frame_of(&p)));
    out
}

/// Bounds of the frames, with the lane included so an empty run still has a
/// sensible view. Returns the lane bounds when `frames` is empty.
#[must_use]
pub fn bounds(frames: &[Frame]) -> Bounds {
    let mut x = [0.0_f32, 0.0_f32];
    let mut y = [-LANE_HALF_WIDTH_M, LANE_HALF_WIDTH_M];
    for f in frames {
        x[0] = x[0].min(f.vehicle_x).min(f.pedestrian[0]);
        x[1] = x[1].max(f.vehicle_x + FRONT_OFFSET_M).max(f.pedestrian[0]);
        y[0] = y[0].min(f.pedestrian[1]);
        y[1] = y[1].max(f.pedestrian[1]);
    }
    Bounds { x, y }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn there_is_one_frame_per_step_with_increasing_time() {
        let s = Scenario::default();
        let frames = frames(&s);
        assert_eq!(frames.len(), (s.duration_s / s.dt_s) as usize);
        assert!(frames.windows(2).all(|w| w[1].t_s > w[0].t_s));
    }

    #[test]
    fn the_default_crossing_shows_the_guardian_intervening_and_a_closing_gap() {
        let frames = frames(&Scenario::default());
        assert!(frames.iter().any(|f| f.intervening));
        let first = frames.first().expect("frames");
        let last = frames.last().expect("frames");
        assert!(
            last.gap_m < first.gap_m,
            "the vehicle closes on the pedestrian"
        );
    }

    #[test]
    fn an_unguarded_run_reaches_contact() {
        let frames = frames(&Scenario {
            guardian_enabled: false,
            ..Scenario::default()
        });
        assert!(frames.iter().any(|f| f.in_contact));
        assert!(frames.iter().all(|f| !f.intervening));
    }

    #[test]
    fn bounds_cover_the_vehicle_and_the_pedestrian_path() {
        let frames = frames(&Scenario::default());
        let b = bounds(&frames);
        assert!(b.x[0] <= 0.0 && b.x[1] >= 40.0, "{b:?}");
        assert!(
            b.y[0] < -LANE_HALF_WIDTH_M,
            "the pedestrian starts beyond the lane: {b:?}"
        );
    }

    #[test]
    fn empty_frames_give_the_lane_bounds() {
        let b = bounds(&[]);
        assert_eq!(b.x, [0.0, 0.0]);
        assert_eq!(b.y, [-LANE_HALF_WIDTH_M, LANE_HALF_WIDTH_M]);
    }
}
