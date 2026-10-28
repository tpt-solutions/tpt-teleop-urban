# tpt-t-urban-delivery

Last-mile sidewalk bot control: curb mounting, pedestrian yielding, and secure compartments.

## Modules

- `mount`: curb approach, alignment, and ultrasonic verification.
- `yield_ped`: pedestrian right-of-way with hold-off.
- `compartment`: package lock, gated by curb verification and rest, with a
  constant-time code check.

The jaywalker-reaction hookup (consuming `tpt-t-urban-predict` cones) is part
of Phase 16 and is not done here.

## Testing

```sh
cargo test -p tpt-t-urban-delivery
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.
