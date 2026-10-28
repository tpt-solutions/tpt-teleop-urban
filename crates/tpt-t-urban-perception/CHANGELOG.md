# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- GNSS jump detection with LiDAR fallback and hysteresis, and a source
  selector (`gnss`).
- Curb edge detection from a lateral height profile (`curb`).
- Inverse-variance fusion of two position estimates (`fusion`).
