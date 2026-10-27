//! The teleop hooks run in the realtime loop, so their success paths must not
//! touch the heap. This test binary counts allocations per thread and checks
//! each hook.
//!
//! Error paths are not covered. `DomainError::Adapter` holds a `String`, so an
//! error allocates. That is an upstream type, and the bridge's own docs allow it
//! on error paths.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use tpt_t_core::Mode;
use tpt_t_core::ser::ControlCommand;
use tpt_t_domain_bridge::{DomainTeleopInterface, OperatorId};
use tpt_t_urban_teleop::{AXIS_THROTTLE, TeleopAdapter};

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
    // Control: the harness must notice an allocation, or the zero counts below
    // would prove nothing.
    let (v, n) = allocations_during(|| vec![1_u8, 2, 3]);
    assert!(n >= 1);
    assert_eq!(v.len(), 3);
}

#[test]
fn success_paths_make_no_heap_allocations() {
    let mut adapter = TeleopAdapter::new();
    adapter.request_mode(Mode::Assist).unwrap();
    adapter.request_mode(Mode::FullTeleop).unwrap();
    let operator = OperatorId::new([1; 16]);

    let (result, n) = allocations_during(|| adapter.on_teleop_engage(operator));
    result.unwrap();
    assert_eq!(n, 0, "on_teleop_engage allocated");

    let mut command = ControlCommand::zeroed(Mode::FullTeleop);
    command.seq = 1;
    command.axes[AXIS_THROTTLE] = 0.5;
    let (result, n) = allocations_during(|| adapter.on_control_command(command));
    result.unwrap();
    assert_eq!(n, 0, "on_control_command allocated");

    let (_state, n) = allocations_during(|| adapter.get_domain_state());
    assert_eq!(n, 0, "get_domain_state allocated");

    let (feed, n) = allocations_during(|| adapter.get_sensor_feed());
    assert!(feed.video_frames.is_empty());
    assert_eq!(n, 0, "get_sensor_feed allocated");

    let (result, n) = allocations_during(|| adapter.on_teleop_disengage());
    result.unwrap();
    assert_eq!(n, 0, "on_teleop_disengage allocated");

    let (_, n) = allocations_during(|| adapter.on_emergency_stop());
    assert_eq!(n, 0, "on_emergency_stop allocated");
}

#[test]
fn refused_command_makes_no_heap_allocations() {
    // Auto refuses operator input. The refusal is an error, so this covers the
    // error path, which now uses a static message.
    let mut adapter = TeleopAdapter::new();
    adapter.on_teleop_engage(OperatorId::new([2; 16])).unwrap();
    let mut command = ControlCommand::zeroed(Mode::Auto);
    command.seq = 1;

    let (result, n) = allocations_during(|| adapter.on_control_command(command));
    assert!(result.is_err());
    assert_eq!(n, 0, "a refused command allocated");
}
