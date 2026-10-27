//! The guardian runs in the 100 Hz loop, so its cycle must not touch the heap.
//! This test binary counts allocations per thread and checks the step.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use tpt_t_urban_safety::{Guardian, Inputs, Obstacle};

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
fn guardian_cycle_makes_no_heap_allocations() {
    let mut guardian = Guardian::new();
    let closing = Inputs {
        obstacle: Some(Obstacle {
            range_mm: 2_000,
            closing_speed_mm_s: 2_000,
        }),
        planned_decel_mm_s2: 0,
        planner_heartbeat_age_ms: 0,
        speed_mm_s: 8_000,
    };
    let (_, n) = allocations_during(|| guardian.step(&closing));
    assert_eq!(n, 0, "Guardian::step allocated on an AEB cycle");

    let stale = Inputs {
        planner_heartbeat_age_ms: 500,
        ..closing
    };
    let (_, n) = allocations_during(|| guardian.step(&stale));
    assert_eq!(n, 0, "Guardian::step allocated on an MRM cycle");
}
