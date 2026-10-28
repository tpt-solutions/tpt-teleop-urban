# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Curb mount sequence with alignment and ultrasonic verification (`mount`).
- Pedestrian yielding with hold-off (`yield_ped`).
- Package compartment lock gated by curb verification, with a constant-time
  code check (`compartment`).
