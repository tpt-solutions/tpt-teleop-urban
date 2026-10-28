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
- Teleop adapter (`tpt-t-urban-teleop`): position from perception, fault flags,
  traffic-law gating from `compliance::evaluate`, and redacted video and point
  clouds in the sensor feed.
- Durable audit ledger (`tpt-t-urban-compliance::store`): each append is synced
  before it returns, and a torn or unverifiable final record is cut off on open.
  A bad record with valid records after it is refused, not repaired.
- Signal optimization (`tpt-t-urban-traffic::optimize`): demand-proportional
  green split with min and max bounds, and corridor offsets for a green wave.
  `Controller::set_greens` applies new greens at the next phase start.
- Coverage-guided fuzz targets for the BSM decoder and the UPER primitives
  (`tpt-t-urban-v2x/fuzz`).
- Simulator: seeded crossing sweep (`sweep` module and example), and a GNSS model
  with multipath and denial windows, tested against the source selector.

### Changed
- Guardian (`tpt-t-urban-safety`): the planner's braking must stop the vehicle
  `STOP_MARGIN_MM` (1 m) short of an obstacle, and must allow for the 50 ms
  `REACTION_MS` before the brake acts. The AEB time-to-collision threshold grows
  with the time a full stop takes. A planner that brakes just enough no longer
  suppresses AEB. A distant obstacle with no planner braking now engages AEB
  early enough to stop.

### Fixed
- The guardian let a stop end inside the pedestrian contact zone. The seeded
  crossing sweep found this; see the Changed entry above.
