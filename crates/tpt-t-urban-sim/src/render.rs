//! Headless renderer for crossing-scenario traces: an SVG top-down view.
//!
//! This is the no-display stand-in for the egui visualizer (todo.md Phase 15).
//! It draws the lane, the vehicle path, the pedestrian path, and the points
//! where the guardian was overriding the planner. It only formats text, so it
//! runs anywhere, including CI.

use std::fmt::Write;

use crate::scenario::TracePoint;

/// Drawing area in pixels.
const WIDTH_PX: f32 = 900.0;
const HEIGHT_PX: f32 = 360.0;
const MARGIN_PX: f32 = 30.0;

/// Renders `trace` as an SVG document. `x_span_m` is the metre range shown along
/// the horizontal axis, and `y_span_m` the range shown vertically.
#[must_use]
pub fn crossing_svg(trace: &[TracePoint], x_span_m: (f32, f32), y_span_m: (f32, f32)) -> String {
    let map = |x: f32, y: f32| -> (f32, f32) {
        let px =
            MARGIN_PX + (x - x_span_m.0) / (x_span_m.1 - x_span_m.0) * (WIDTH_PX - 2.0 * MARGIN_PX);
        let py = HEIGHT_PX
            - MARGIN_PX
            - (y - y_span_m.0) / (y_span_m.1 - y_span_m.0) * (HEIGHT_PX - 2.0 * MARGIN_PX);
        (px, py)
    };

    let mut svg = String::new();
    let _ = writeln!(
        svg,
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH_PX} {HEIGHT_PX}" width="{WIDTH_PX}" height="{HEIGHT_PX}">"#
    );
    svg.push_str(r##"<rect width="100%" height="100%" fill="#fbfbf8"/>"##);

    // Lane: a band around y = 0 (±2 m, the guardian's lane half-width).
    let (_, lane_top) = map(x_span_m.0, 2.0);
    let (_, lane_bottom) = map(x_span_m.0, -2.0);
    let _ = writeln!(
        svg,
        r##"<rect x="{MARGIN_PX}" y="{lane_top:.1}" width="{:.1}" height="{:.1}" fill="#e8ecf2"/>"##,
        WIDTH_PX - 2.0 * MARGIN_PX,
        lane_bottom - lane_top
    );

    // Vehicle path: polyline of bumper-less reference positions at y = 0.
    let mut vehicle_pts = String::new();
    let mut ped_pts = String::new();
    for p in trace {
        let (vx, vy) = map(p.vehicle_x, 0.0);
        let _ = write!(vehicle_pts, "{vx:.1},{vy:.1} ");
        let (px, py) = map(p.ped_x, p.ped_y);
        let _ = write!(ped_pts, "{px:.1},{py:.1} ");
    }
    let _ = writeln!(
        svg,
        r##"<polyline points="{vehicle_pts}" fill="none" stroke="#2b6cb0" stroke-width="2"/>"##
    );
    let _ = writeln!(
        svg,
        r##"<polyline points="{ped_pts}" fill="none" stroke="#805ad5" stroke-width="2" stroke-dasharray="6 4"/>"##
    );

    // Guardian override: a red marker at each step where it was active.
    for p in trace.iter().filter(|p| p.override_active) {
        let (vx, vy) = map(p.vehicle_x, 0.0);
        let _ = writeln!(
            svg,
            r##"<circle cx="{vx:.1}" cy="{vy:.1}" r="2.5" fill="#e53e3e"/>"##
        );
    }

    svg.push_str("</svg>\n");
    svg
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::{Scenario, run_crossing_traced};

    #[test]
    fn svg_contains_both_paths_and_override_markers_for_a_guarded_crossing() {
        let mut trace = Vec::new();
        let outcome = run_crossing_traced(&Scenario::default(), |p| trace.push(p));
        assert!(outcome.aeb_engaged);
        let svg = crossing_svg(&trace, (0.0, 50.0), (-8.0, 8.0));
        assert!(svg.starts_with("<svg"));
        assert!(svg.trim_end().ends_with("</svg>"));
        assert_eq!(svg.matches("<polyline").count(), 2);
        assert!(svg.contains("#e53e3e"), "expected override markers");
    }

    #[test]
    fn trace_covers_every_step() {
        let s = Scenario::default();
        let mut n = 0;
        let _ = run_crossing_traced(&s, |_| n += 1);
        assert_eq!(n, (s.duration_s / s.dt_s) as usize);
    }
}
