//! Prediction runs in the 30 Hz perception loop, so an IMM frame must not touch
//! the heap. This test binary counts allocations per thread and checks the
//! filter step, the estimate, and the scalar constant-velocity propagation.
//! The SIMD kernel needs nightly and is covered by its own feature build.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use tpt_t_urban_predict::{AgentsSoa, Imm, propagate_cv_scalar};

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
fn imm_frame_makes_no_heap_allocations() {
    let mut imm = Imm::new([0.0, 0.0, 5.0, 0.0], 1.0, 1.0);
    let (_, n) = allocations_during(|| imm.step([0.5, 0.1], 0.1));
    assert_eq!(n, 0, "Imm::step allocated");

    let (_, n) = allocations_during(|| imm.estimate());
    assert_eq!(n, 0, "Imm::estimate allocated");
}

#[test]
fn scalar_propagation_makes_no_heap_allocations() {
    let mut agents = AgentsSoa::default();
    for i in 0..50 {
        agents.push(i as f32, 0.0, 5.0, 0.0);
    }
    let (_, n) = allocations_during(|| propagate_cv_scalar(&mut agents, 0.1));
    assert_eq!(n, 0, "propagate_cv_scalar allocated");
}

/// The SIMD kernel must also be allocation-free. It is nightly-only, so the test
/// exists only with the `simd` feature.
#[cfg(feature = "simd")]
#[test]
fn simd_propagation_makes_no_heap_allocations() {
    use tpt_t_urban_predict::batch::propagate_cv_simd;

    let mut agents = AgentsSoa::default();
    for i in 0..50 {
        agents.push(i as f32, 0.0, 5.0, 0.0);
    }
    let (_, n) = allocations_during(|| propagate_cv_simd(&mut agents, 0.1));
    assert_eq!(n, 0, "propagate_cv_simd allocated");
}
