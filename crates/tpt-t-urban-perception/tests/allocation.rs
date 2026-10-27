//! Perception runs in the 30 Hz loop, so fix checking, source selection, fusion
//! and curb detection must not touch the heap. This test binary counts
//! allocations per thread and checks each step.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use tpt_t_urban_perception::curb::Sample;
use tpt_t_urban_perception::gnss::JumpDetector;
use tpt_t_urban_perception::{Estimate, Fix, SourceSelector, detect_curb, fuse};

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
fn perception_steps_make_no_heap_allocations() {
    let mut detector = JumpDetector::new();
    let _ = allocations_during(|| {
        detector.check(Fix {
            x_mm: 0,
            y_mm: 0,
            t_ms: 0,
        })
    });
    let (_, n) = allocations_during(|| {
        detector.check(Fix {
            x_mm: 100,
            y_mm: 0,
            t_ms: 100,
        })
    });
    assert_eq!(n, 0, "JumpDetector::check allocated");

    let mut selector = SourceSelector::new();
    let (_, n) = allocations_during(|| {
        selector.on_gnss_fix(Fix {
            x_mm: 0,
            y_mm: 0,
            t_ms: 0,
        })
    });
    assert_eq!(n, 0, "SourceSelector::on_gnss_fix allocated");
    let (_, n) = allocations_during(|| selector.on_gnss_lost());
    assert_eq!(n, 0, "SourceSelector::on_gnss_lost allocated");

    let a = Estimate {
        x: 1.0,
        y: 2.0,
        variance: 0.5,
    };
    let b = Estimate {
        x: 1.2,
        y: 1.9,
        variance: 0.25,
    };
    let (_, n) = allocations_during(|| fuse(a, b));
    assert_eq!(n, 0, "fuse allocated");

    // The profile is built before measuring, because building it allocates.
    let profile: Vec<Sample> = (0..60)
        .map(|i| Sample {
            lateral_mm: i * 50,
            height_mm: if i < 20 { 0 } else { 120 },
        })
        .collect();
    let (_, n) = allocations_during(|| detect_curb(&profile));
    assert_eq!(n, 0, "detect_curb allocated");
}
