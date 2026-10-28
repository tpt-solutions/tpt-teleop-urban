# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Fixed-capacity SPSC message bus (`bus`).
- Vehicle state machine transition table (`fsm`).
- Single-threaded deterministic event loop (`event_loop`).
- Schedulability check for the thread-per-core layout (`schedule`).
- rkyv-archived wire types shared across urban crates (`wire`).
