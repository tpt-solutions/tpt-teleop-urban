# Quickstart

Run everything from the repository root. The toolchain is pinned in
`rust-toolchain.toml` (stable).

## Build and check

```sh
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo deny check
tools/lock-audit.sh
```

`cargo deny check` also reports duplicate-version warnings (`windows-sys` 0.52
and 0.60). They do not fail the check.

## Run the examples

```sh
# Host timing against the spec budgets (release build, so the numbers mean something).
cargo run --release -p tpt-t-urban-sim --example budgets

# Headless SVG of the crossing-pedestrian scenario. Stands in for the visualizer.
cargo run -p tpt-t-urban-sim --example render_crossing
```

## Teleop handover

The urban adapter, `tpt-t-urban-teleop`, implements the upstream
`DomainTeleopInterface` from `tpt-t-domain-bridge`. The core drives it through a
`DomainSession`. The tests in `crates/tpt-t-urban-teleop/tests/` show the full
flow: handover request, engage, commands, disengage, and emergency stop.

```sh
cargo test -p tpt-t-urban-teleop
```

## Nightly SIMD

Only `tpt-t-urban-predict` has a SIMD kernel today, behind its `simd` feature.
That feature needs nightly:

```sh
cargo +nightly test -p tpt-t-urban-predict --features simd
```

`tpt-t-urban-privacy` has no SIMD kernel yet (todo Phase 8). Its tests run on
stable.
