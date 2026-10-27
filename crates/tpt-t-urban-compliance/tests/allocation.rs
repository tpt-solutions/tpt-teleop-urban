//! The traffic-law decision runs in the 30 Hz loop, so it must not touch the
//! heap. This test binary counts allocations per thread and checks `evaluate`.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use tpt_t_urban_compliance::{ApproachState, Observation, evaluate};

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
fn evaluate_makes_no_heap_allocations() {
    let obs = Observation {
        stop_sign: true,
        distance_to_line_mm: 12_000,
        speed_mm_s: 8_000,
        speed_limit_mm_s: 13_900,
        conflict_on_right: false,
    };
    let (_, n) = allocations_during(|| evaluate(ApproachState::Approaching, &obs));
    assert_eq!(n, 0, "evaluate allocated");
}
