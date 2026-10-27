# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Cargo workspace (resolver 3, edition 2024) with 13 stub crates matching the
  architecture in `spec.txt` §3.
- `rust-toolchain.toml` (stable default) and crate-level nightly guidance for
  the SIMD crates.
- `deny.toml` encoding the MIT/BSD/ISC license chain and the spec §7 banned
  crate list.
- CI workflows: build/test on Linux, macOS, and Windows; aarch64
  cross-compile check; nightly job for `tpt-t-urban-predict` and
  `tpt-t-urban-privacy`; fmt, clippy, and cargo-deny lint checks.
- Dual `MIT OR Apache-2.0` licensing, `rustfmt.toml`, `clippy.toml`.
- Root README, `SECURITY.md`, and `docs/` skeleton.
