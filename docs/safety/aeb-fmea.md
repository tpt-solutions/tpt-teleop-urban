# FMEA: AEB and MRM trigger path

Scope: the guardian's AEB and MRM decision, from sensed inputs to the override
command, as implemented in `crates/tpt-t-urban-safety`. This is an engineering
working document for todo.md Phase 17. It is not a certified FMEA. The
certification review (ISO 26262, ISO 21448, UL 4600) must confirm it against
the real hardware, sensors and timing.

Scoring: **S** severity, **O** occurrence, **D** detection, each 1 (low) to 10
(high). RPN = S × O × D. "Status" says whether the mitigation exists in code.

| ID | Failure mode | Effect | S | O | D | Mitigation | Status |
|----|--------------|--------|---|---|---|------------|--------|
| F1 | Pedestrian estimate jitters across the lane edge | Spurious AEB, sudden stop | 4 | 5 | 3 | AEB needs TTC below 1.5 s *and* planner braking short of the required deceleration; latched only while the gap closes | Implemented (`aeb_required`, latch in `Guardian::step`) |
| F2 | Obstacle missed: lane gate never admits the pedestrian | No AEB; planner alone decides | 9 | 3 | 6 | Lane gate uses the IMM estimate, so a missed detection stays missed until the estimate enters the lane. A pedestrian walking into the lane is tracked before the TTC threshold | Partial. Detection itself is not implemented (Phase 4 open) |
| F3 | Planner stops sending heartbeats | Guardian keeps trusting a dead planner | 8 | 2 | 1 | Heartbeat age above 50 ms forces MRM, latched until the vehicle is at rest | Implemented (`PLANNER_STALE_MS`, `Mode::Mrm`) |
| F4 | Guardian task hangs or stops running | No override at all | 10 | 2 | 2 | Hardware watchdog on the edge layer, and a software watchdog in `tpt-t-urban-edge`. The spec requires the guardian on a separate partition | Partial. The separate partition and the hardware watchdog wiring are deployment work |
| F5 | Actuator ignores or lags the AEB command | Less braking than commanded | 9 | 3 | 5 | Limiter enforces rate and clamps; no feedback loop yet | Open. Needs brake-pressure feedback from hardware |
| F6 | MRM or AEB latches forever | Vehicle stuck in override | 5 | 3 | 2 | MRM clears only at rest (`Guardian::reset`); AEB clears when the gap stops closing | Implemented. Operator-visible reset path open |
| F7 | Obstacle range computed in wrong units or frame | Wrong TTC | 9 | 2 | 6 | Integer millimetre units throughout; sim scenario checks the end-to-end geometry | Partial. Frame transforms from sensor to vehicle are not implemented |
| F8 | Required-deceleration overflow in the integer formula | Wrong AEB decision | 7 | 1 | 4 | `u64` arithmetic. Speed is bounded to 50 m/s, so v² fits | Implemented. Bounds should be asserted at the input boundary |
| F9 | Latency from sensor to override exceeds 50 ms | AEB arrives late | 9 | 3 | 5 | None in code. The spec budget covers the whole path, and the guardian's own cycle is measured at well under 1 µs (`budgets` example) | Open. Needs measurement on target |

## Reading this table

- The guardian logic is the easy part to get right. Most open items are about
  sensing (F2, F7), actuation (F5) and deployment (F4, F9). Those cannot be
  closed in software alone.
- F2 has the highest unmitigated RPN. Closing it needs the Phase 4 detector and
  a measured false-negative rate on real pedestrian data.
- The sim scenario (`crates/tpt-t-urban-sim/src/scenario.rs`) is the only
  end-to-end evidence here, and it uses simulated sensors with noise and one
  pedestrian.

## Revision

- Initial analytic version, from the code state at the Phase 7 commit and
  later. Re-score after each change to the guardian or to its inputs.
