# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- `TeleopAdapter` implementing the upstream `DomainTeleopInterface`, gated by
  the mode machine, traffic law, and actuator limits (`adapter`).
- Assistance reason, MRM clearance, and return-to-control checks (`handover`).
- Re-exports of `ControlCommand`, `Mode` and `StateMachine` from `tpt-t-core`.
