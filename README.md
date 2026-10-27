# tpt-teleop-urban

Smart city and autonomous vehicle operations middleware. A Rust workspace for
robotaxis, last-mile sidewalk delivery bots, autonomous transit shuttles, and
smart-city traffic management.

Design doc: [spec.txt](spec.txt). Teleop integration contract:
[bridge spec.txt](bridge%20spec.txt). Work plan: [todo.md](todo.md).

## Status

Pre-release (v1.0.0 target). Phase 0 (workspace setup) is in place: every
crate exists as a stub with no behavior yet. Implementation proceeds phase by
phase as tracked in [todo.md](todo.md).

## Workspace Crates

| Crate | Responsibility |
| --- | --- |
| `tpt-t-urban-core` | Deterministic event loop, vehicle state machine, message bus |
| `tpt-t-urban-v2x` | Zero-copy C-V2X / DSRC parsing (BSM, MAP, SPAT, SRM) |
| `tpt-t-urban-perception` | Multi-sensor fusion, 3D detection, tracking |
| `tpt-t-urban-predict` | SIMD IMM Kalman trajectory prediction (nightly) |
| `tpt-t-urban-compliance` | Deterministic traffic-law FSM and audit ledger |
| `tpt-t-urban-safety` | Independent guardian: AEB and minimal risk maneuvers |
| `tpt-t-urban-privacy` | In-place face and license-plate blur (nightly) |
| `tpt-t-urban-car` | 4-wheel vehicle trajectory planning |
| `tpt-t-urban-delivery` | Sidewalk bot control and secure compartments |
| `tpt-t-urban-traffic` | Infrastructure-side intersection management |
| `tpt-t-urban-edge` | Bare-metal/RTOS actuation layer (`no_std` target) |
| `tpt-t-urban-teleop` | Adapter to `tpt-teleop`'s `DomainTeleopInterface` |
| `tpt-t-urban-sim` | Headless city simulator and egui visualizer |

## Zero-Allocation Data Path

Hot-path crates avoid heap allocation and locks. V2X messages are parsed
directly into pre-allocated rkyv structs, camera frames are blurred in place in
the ring buffer, and the traffic-law FSM is a fixed-size state machine. Threads
are pinned per core (see `docs/ARCHITECTURE.md`), so there is no async runtime
and no lock contention on the control path.

## Developer Experience

Prerequisites: a Rust toolchain via `rustup`. `rust-toolchain.toml` selects
stable automatically.

```sh
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo deny check
```

The nightly-only SIMD crates build with `cargo +nightly test -p tpt-t-urban-predict -p tpt-t-urban-privacy`.

See [docs/quickstart.md](docs/quickstart.md) and [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Dependency Policy

Only the MIT/BSD/ISC chain is allowed. `cargo deny` rejects Apache-only
dependencies, and the banned-crate list encodes the dependency matrix in
spec §7 (no serde, tokio, OpenSSL, heavy ASN.1 compilers, and so on). The
single permitted Apache-only exception is `ring`, see [deny.toml](deny.toml).

## Licensing

Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at
your option. Copyright TPT Solutions.
