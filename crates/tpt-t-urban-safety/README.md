# tpt-t-urban-safety

Independent guardian state machine for AEB, MRM, and actuation failover.

The guardian watches the primary planner's output and the nearest in-path
obstacle. It overrides the planner in two cases:

- **AEB** (automatic emergency braking): time-to-collision is below the
  threshold and the planner is not braking hard enough to stop in time. AEB
  latches until the closing speed is gone.
- **MRM** (minimal risk maneuver): the planner heartbeat has gone stale. MRM
  latches until an explicit reset, once the vehicle is at rest.

All quantities are integers (millimetres, milliseconds, mm/s, mm/s²), so the
decision is bit-exact and reproducible. The guardian is a pure function of its
inputs and holds no clock; the caller supplies the heartbeat age.

The spec requires the guardian to run on a separate compute partition (spec
§5.3). That isolation is a deployment property that this crate cannot provide
in software. This crate is the logic that would run there.

Implements Phase 7 of [todo.md](../../todo.md). The <50 ms reaction budget and
FMEA (Phase 17) need hardware measurements and are not covered here.

## Testing

```sh
cargo test -p tpt-t-urban-safety
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.
