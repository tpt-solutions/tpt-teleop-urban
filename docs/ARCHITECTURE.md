# Architecture

Skeleton. Filled in as each phase lands. Authoritative design: [spec.txt](../spec.txt).

## Workspace map

| Layer | Crates |
| --- | --- |
| Core & vehicle control | `tpt-t-urban-core`, `tpt-t-urban-car`, `tpt-t-urban-delivery`, `tpt-t-urban-edge` |
| Perception & prediction | `tpt-t-urban-perception`, `tpt-t-urban-predict` |
| Connectivity & infrastructure | `tpt-t-urban-v2x`, `tpt-t-urban-traffic` |
| Safety, compliance & privacy | `tpt-t-urban-safety`, `tpt-t-urban-compliance`, `tpt-t-urban-privacy` |
| Teleoperation & simulation | `tpt-t-urban-teleop`, `tpt-t-urban-sim` |

## Crate responsibilities

Each crate's `lib.rs` doc comment is its one-line responsibility. The phase that
implements it is listed in [todo.md](../todo.md).

## Thread-per-core layout (spec §4.5)

| Core | Work | Rate |
| --- | --- | --- |
| 0 | io_uring ingest of LiDAR/Camera/Radar into slab ring buffers | sensor-driven |
| 1 | `tpt-t-urban-perception` + `tpt-t-urban-predict` | 30 Hz |
| 2 | `tpt-t-urban-car` planning + `tpt-t-urban-compliance` FSM | 30 Hz |
| 3 | `tpt-t-urban-edge` actuation + `tpt-t-urban-safety` guardian | 100 Hz |

## Toolchain

Stable is the default. `tpt-t-urban-predict` and `tpt-t-urban-privacy` need
nightly for `#![feature(portable_simd)]`. Each pins its own nightly in a
crate-local `rust-toolchain.toml`, added in Phases 5 and 8.
