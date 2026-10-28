# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Stop-sign and speed-limit decision FSM with a reason on every decision
  (`decision`).
- Append-only audit ledger with hash-chained, Ed25519-signed entries and
  chain verification (`ledger`).
