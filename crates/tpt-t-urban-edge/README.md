# tpt-t-urban-edge

Bare-metal/RTOS actuation layer: steer-by-wire, brake-by-wire, throttle, and watchdog.

This crate is `no_std`. It builds for embedded targets such as
`thumbv7em-none-eabihf`.

## Modules

- `actuator`: clamps and rate-limits steering, brake and throttle commands.
- `watchdog`: software watchdog with a latched expiry.
- `failover`: dual-channel steering failover with requalification.

Hardware bring-up (Phase 18) and the real-time task layout (Phase 13) are not
part of this crate.

## Testing

```sh
cargo test -p tpt-t-urban-edge
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.
