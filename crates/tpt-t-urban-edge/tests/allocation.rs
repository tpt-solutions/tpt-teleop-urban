//! The actuation path runs in the 100 Hz loop, so it must not touch the heap.
//! This test binary counts allocations per thread and checks each step.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use tpt_t_urban_edge::{Failover, Limiter, Watchdog};

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
fn actuation_path_makes_no_heap_allocations() {
    let mut limiter = Limiter::new();
    let (_, n) = allocations_during(|| limiter.apply(1_000, 0, 500));
    assert_eq!(n, 0, "Limiter::apply allocated");

    let (_, n) = allocations_during(|| limiter.emergency_brake());
    assert_eq!(n, 0, "Limiter::emergency_brake allocated");

    let mut watchdog = Watchdog::new(100);
    let (_, n) = allocations_during(|| watchdog.tick(10));
    assert_eq!(n, 0, "Watchdog::tick allocated");

    let (_, n) = allocations_during(|| watchdog.feed());
    assert_eq!(n, 0, "Watchdog::feed allocated");

    let mut failover = Failover::new();
    let (_, n) = allocations_during(|| failover.step(false, true));
    assert_eq!(n, 0, "Failover::step allocated");
}
