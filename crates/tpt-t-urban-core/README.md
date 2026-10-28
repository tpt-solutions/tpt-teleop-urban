# tpt-t-urban-core

Central deterministic event loop, vehicle state machine, and lock-free message bus.

## Modules

- `bus`: fixed-capacity SPSC ring used for all inter-crate messages.
- `fsm`: the vehicle state machine transition table.
- `event_loop`: single-threaded loop that consumes the bus and drives the FSM.
- `schedule`: schedulability check for the thread-per-core layout.
- `wire`: rkyv-archived types shared by urban crates.

Implements Phase 2 of [todo.md](../../todo.md), plus the analytic part of
Phase 13.

## Testing

```sh
cargo test -p tpt-t-urban-core
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.
