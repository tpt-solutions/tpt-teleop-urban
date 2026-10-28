# tpt-t-urban-predict

SIMD-accelerated IMM Kalman prediction of pedestrian, cyclist, and vehicle trajectories.

## Modules

- `imm`: two-model IMM filter and trajectory cone. Scalar reference, builds on
  stable.
- `batch`: structure-of-arrays constant-velocity propagation over many agents.
  Scalar on stable, eight-wide `portable_simd` kernel behind the `simd` feature.
- `linalg`: fixed-size matrix helpers.

## Testing

```sh
cargo test -p tpt-t-urban-predict
```

The `simd` feature needs a nightly toolchain and is not enabled on stable:

```sh
cargo +nightly test -p tpt-t-urban-predict --features simd
```

CI runs the nightly job.

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.
