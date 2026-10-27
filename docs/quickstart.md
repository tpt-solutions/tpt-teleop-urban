# Quickstart

Stub. Expanded once the first runnable crate lands.

## Build and check

```sh
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo deny check
```

## Nightly SIMD crates

```sh
cargo +nightly test -p tpt-t-urban-predict -p tpt-t-urban-privacy
```
