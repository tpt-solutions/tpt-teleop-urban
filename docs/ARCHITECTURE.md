# Architecture

Authoritative design: [spec.txt](../spec.txt). The teleop contract is in
[bridge spec.txt](../bridge%20spec.txt). Open work is tracked in
[todo.md](../todo.md).

## Workspace map

| Layer | Crates |
| --- | --- |
| Core & vehicle control | `tpt-t-urban-core`, `tpt-t-urban-car`, `tpt-t-urban-delivery`, `tpt-t-urban-edge` |
| Perception & prediction | `tpt-t-urban-perception`, `tpt-t-urban-predict` |
| Connectivity & infrastructure | `tpt-t-urban-v2x`, `tpt-t-urban-traffic` |
| Safety, compliance & privacy | `tpt-t-urban-safety`, `tpt-t-urban-compliance`, `tpt-t-urban-privacy` |
| Teleoperation & simulation | `tpt-t-urban-teleop`, `tpt-t-urban-sim` |

Each crate's `lib.rs` doc comment states its responsibility and lists what is
still open.

## Teleoperation (bridge contract)

The teleop path has three parts:

- **Upstream contract.** `tpt-t-core` (`ControlCommand` as a flat 56-byte POD,
  and the `StateMachine` with modes Auto, Assist, FullTeleop and EmergencyStop)
  and `tpt-t-domain-bridge` (`DomainTeleopInterface`, `DomainSession`, and the
  `HandoverState` lifecycle). Both are pinned git dependencies at the same rev
  in the root `Cargo.toml`.
- **Adapter.** `TeleopAdapter` in `tpt-t-urban-teleop` implements the trait.
  Each command passes these gates in order: wire check (magic, version, strictly
  increasing `seq`), mode match, guardian override, traffic law, then the
  actuator limiter. An emergency stop bypasses the limiter's slew rate.
- **Handover trigger.** `tpt-t-urban-safety`'s guardian latches MRM when the
  planner heartbeat goes stale. That latch is mapped to
  `AssistanceReason::Degraded` by `assistance_reason`. Return to autonomy waits
  until the MRM is cleared at rest (`ready_to_return`).

The core owns the lifecycle. `DomainSession` rejects out-of-order calls before
they reach the adapter. The adapter does not check lifecycle order itself, but
it does refuse operator commands with no operator engaged.

## Thread-per-core layout (spec §4.5)

| Core | Work | Rate |
| --- | --- | --- |
| 0 | io_uring ingest of LiDAR/Camera/Radar into slab ring buffers | sensor-driven |
| 1 | `tpt-t-urban-perception` + `tpt-t-urban-predict` | 30 Hz |
| 2 | `tpt-t-urban-car` planning + `tpt-t-urban-compliance` FSM | 30 Hz |
| 3 | `tpt-t-urban-edge` actuation + `tpt-t-urban-safety` guardian | 100 Hz |

Pinning to cores is not implemented yet (todo Phase 13). The schedulability check
in `tpt-t-urban-core::schedule` is analytic only.

## Hot-path rules

`tools/lock-audit.sh` fails if a hot-path crate (core, safety, compliance, edge,
teleop) uses a mutex, a blocking channel, or an async runtime. Zero-allocation
is not checked by any tool yet. The teleop hooks allocate only on their error
paths, because the upstream `DomainError::Adapter` holds a `String`.

## Toolchain

The workspace is on stable (`rust-toolchain.toml`).
`tpt-t-urban-predict` has a `simd` feature that needs nightly for
`portable_simd`. `tpt-t-urban-privacy` has no SIMD kernel yet. Its crate-local
nightly toolchain file is still open (todo Phases 5 and 8), so no crate-local
`rust-toolchain.toml` exists in the repository.
