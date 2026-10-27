//! Central deterministic event loop, vehicle state machine, and lock-free message bus.
//!
//! - [`bus`]: fixed-capacity SPSC ring used for all inter-crate messages.
//! - [`fsm`]: the vehicle state machine transition table.
//! - [`event_loop`]: single-threaded loop that consumes the bus and drives the FSM.
//! - [`schedule`]: schedulability check for the thread-per-core layout.
//! - [`wire`]: rkyv-archived types shared by urban crates.
//!
//! Implements Phase 2 of `todo.md`, plus the analytic part of Phase 13.

#![deny(unsafe_op_in_unsafe_fn)]

pub mod bus;
pub mod event_loop;
pub mod fsm;
pub mod schedule;
pub mod wire;

pub use event_loop::{EventLoop, Outcome};
pub use wire::{Event, Transition, VehicleState};
