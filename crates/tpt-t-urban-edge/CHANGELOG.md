# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- `no_std` crate root with `deny(unsafe_op_in_unsafe_fn)`.
- Steering, brake and throttle clamping and rate limiting (`actuator`).
- Software watchdog with a latched expiry (`watchdog`).
- Dual-channel steering failover with requalification (`failover`).
