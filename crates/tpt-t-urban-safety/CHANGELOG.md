# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Guardian state machine with latched AEB and MRM overrides.
- Integer-only, bit-exact decision logic with no internal clock; the caller
  supplies the heartbeat age.
- `AEB_TTC_MS` threshold constant (1500 ms).
