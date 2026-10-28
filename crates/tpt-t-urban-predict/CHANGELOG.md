# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Two-model IMM filter and trajectory cone, scalar reference (`imm`).
- Structure-of-arrays constant-velocity propagation, scalar on stable
  (`batch`).
- Eight-wide `portable_simd` propagation kernel behind the `simd` feature
  (nightly only).
- Fixed-size matrix helpers (`linalg`).
