//! From vehicles on each approach to the green times the signal actually shows.
//!
//! Four approaches (N, E, S, W). The queue is counted by `queued_per_approach`,
//! split by `allocate_greens`, and queued into the controller. The test runs the
//! controller and measures the green durations it shows.

use tpt_t_urban_traffic::controller::Interval;
use tpt_t_urban_traffic::demand::{QueueZone, Vehicle, queued_per_approach};
use tpt_t_urban_traffic::{Controller, allocate_greens};

const APPROACHES: [[f32; 2]; 4] = [[0.0, 1.0], [1.0, 0.0], [0.0, -1.0], [-1.0, 0.0]];
const ZONE: QueueZone = QueueZone {
    queue_length_m: 30.0,
    lane_half_m: 2.0,
    stopped_speed_m_s: 0.5,
};
const CYCLE_MS: u32 = 90_000;
const MIN_GREEN_MS: u32 = 5_000;
const MAX_GREEN_MS: u32 = 40_000;
const TICK_MS: u32 = 100;

/// Stopped vehicles at `distance` metres along each ray, `count` of them.
fn queue(direction: [f32; 2], count: u32) -> Vec<Vehicle> {
    (0..count)
        .map(|i| Vehicle {
            pos: [
                direction[0] * (5.0 + 4.0 * i as f32),
                direction[1] * (5.0 + 4.0 * i as f32),
            ],
            vel: [0.0, 0.0],
        })
        .collect()
}

/// Runs the controller for `ticks` steps and returns `(phase, green_ms)` for every
/// green interval. Times are measured to the tick at which the interval changes.
fn green_intervals(controller: &mut Controller<4>, ticks: u32) -> Vec<(usize, u32)> {
    let mut out = Vec::new();
    let mut elapsed = 0u32;
    let mut last = controller.step(TICK_MS, None);
    elapsed += TICK_MS;
    let mut green_start = 0u32;
    for _ in 0..ticks {
        let next = controller.step(TICK_MS, None);
        let before = elapsed;
        elapsed += TICK_MS;
        if next.phase != last.phase || next.interval != last.interval {
            if last.interval == Interval::Green {
                out.push((last.phase, before - green_start));
            }
            if next.interval == Interval::Green {
                green_start = before;
            }
            last = next;
        }
    }
    out
}

#[test]
fn the_busiest_approach_gets_the_most_green_and_the_signal_shows_it() {
    // North has six queued vehicles, east two, south and west none.
    let mut vehicles = queue(APPROACHES[0], 6);
    vehicles.extend(queue(APPROACHES[1], 2));
    let demand = queued_per_approach(&APPROACHES, &vehicles, ZONE);
    assert_eq!(demand, [6, 2, 0, 0]);

    let greens = allocate_greens(demand, CYCLE_MS, MIN_GREEN_MS, MAX_GREEN_MS)
        .expect("the cycle fits four minimum greens and their clearance");
    assert!(
        greens[0] > greens[1] && greens[1] > MIN_GREEN_MS,
        "{greens:?}"
    );
    assert_eq!(greens[2], MIN_GREEN_MS);
    assert_eq!(greens[3], MIN_GREEN_MS);

    let mut controller = Controller::new([MIN_GREEN_MS; 4]);
    controller.set_greens(greens);
    // Three cycles: the first applies the queued greens at its phase start, and the
    // last one is measured in full.
    let cycle_ticks = (CYCLE_MS / TICK_MS) * 3;
    let shown = green_intervals(&mut controller, cycle_ticks);
    let last_cycle = &shown[shown.len() - 4..];

    for (phase, green_ms) in last_cycle {
        let expected = greens[*phase];
        assert!(
            green_ms.abs_diff(expected) <= TICK_MS,
            "phase {phase}: shown {green_ms} ms, allocated {expected} ms"
        );
    }
}
