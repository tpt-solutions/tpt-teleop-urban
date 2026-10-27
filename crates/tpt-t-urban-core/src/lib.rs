//! Central deterministic event loop, vehicle state machine, and lock-free message bus.
//!
//! - [`bus`]: fixed-capacity SPSC ring used for all inter-crate messages.
//! - [`fsm`]: the vehicle state machine transition table.
//! - [`event_loop`]: single-threaded loop that consumes the bus and drives the FSM.
//! - [`wire`]: rkyv-archived types shared by urban crates.
//!
//! Implements Phase 2 of `todo.md`.

#![deny(unsafe_op_in_unsafe_fn)]

pub mod bus;
pub mod event_loop;
pub mod fsm;
pub mod wire;

pub use event_loop::{EventLoop, Outcome};
pub use wire::{Event, Transition, VehicleState};
