//! The inter-crate bus carries every hot-path message, so a send or receive must
//! not touch the heap. This test binary counts allocations per thread and checks
//! the bus.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use tpt_t_urban_core::bus;
use tpt_t_urban_core::{Event, EventLoop, VehicleState};

thread_local! {
    static ALLOCATIONS: Cell<usize> = const { Cell::new(0) };
}

struct Counting;

fn bump() {
    ALLOCATIONS.with(|c| c.set(c.get() + 1));
}

// SAFETY: forwards every call to the system allocator unchanged, and only
// increments a thread-local counter on the side.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        bump();
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        bump();
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// Runs `f` and returns its result with the number of allocations it made on
/// this thread.
fn allocations_during<R>(f: impl FnOnce() -> R) -> (R, usize) {
    let before = ALLOCATIONS.with(Cell::get);
    let result = f();
    (result, ALLOCATIONS.with(Cell::get) - before)
}

#[test]
fn counter_sees_a_real_allocation() {
    let (v, n) = allocations_during(|| vec![1_u8, 2, 3]);
    assert!(n >= 1);
    assert_eq!(v.len(), 3);
}

#[test]
fn bus_send_and_receive_make_no_heap_allocations() {
    let (mut tx, mut rx) = bus::channel::<u64, 8>();

    let (sent, n) = allocations_during(|| tx.try_send(42));
    assert!(sent.is_ok());
    assert_eq!(n, 0, "try_send allocated");

    let (got, n) = allocations_during(|| rx.try_recv());
    assert_eq!(got, Some(42));
    assert_eq!(n, 0, "try_recv allocated");
}

#[test]
fn event_loop_step_makes_no_heap_allocations() {
    let (_emergency_tx, emergency_rx) = bus::channel::<Event, 4>();
    let (mut routine_tx, routine_rx) = bus::channel::<Event, 4>();
    let mut event_loop = EventLoop::new(VehicleState::Parked, emergency_rx, routine_rx);
    routine_tx.try_send(Event::Depart).unwrap();

    let (_, n) = allocations_during(|| event_loop.step());
    assert_eq!(n, 0, "EventLoop::step allocated on a transition");

    let (_, n) = allocations_during(|| event_loop.step());
    assert_eq!(n, 0, "EventLoop::step allocated on an idle tick");
}
