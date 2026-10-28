//! Checks that BSM core-data decoding performs no heap allocation, and that its
//! mean time is inside the 10 µs budget on an optimised build.
//!
//! This file is its own test binary, so its counting allocator sees no other
//! test's allocations. The count is per thread, so the harness's own threads
//! cannot disturb it either.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::time::Instant;

use tpt_t_urban_v2x::bsm::{
    AccelerationSet4Way, BrakeSystemStatus, BsmCoreData, PositionalAccuracy, VehicleSize,
};
use tpt_t_urban_v2x::uper::{BitReader, BitWriter};

struct Counting;

thread_local! {
    static ALLOCS: Cell<u64> = const { Cell::new(0) };
}

fn allocations() -> u64 {
    ALLOCS.with(Cell::get)
}

// SAFETY: every call forwards to `System` unchanged. The only addition is a
// thread-local counter that uses a const initialiser, so it never allocates.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn sample() -> BsmCoreData {
    BsmCoreData {
        msg_cnt: 42,
        temporary_id: [0xDE, 0xAD, 0xBE, 0xEF],
        sec_mark: 31_337,
        lat: 277_000_000,
        lon: -825_000_000,
        elev: 1_200,
        accuracy: PositionalAccuracy {
            semi_major: 12,
            semi_minor: 7,
            orientation: 40_000,
        },
        transmission: 2,
        speed: 1_250,
        heading: 14_400,
        steer_angle: -5,
        accel: AccelerationSet4Way {
            long: -150,
            lat: 20,
            vert: 0,
            yaw: -300,
        },
        brakes: BrakeSystemStatus {
            wheel_brakes: 0b10100,
            traction: 2,
            abs: 1,
            scs: 2,
            brake_boost: 1,
            aux_brakes: 0,
        },
        size: VehicleSize {
            length: 480,
            width: 180,
        },
    }
}

#[test]
fn decoding_a_bsm_allocates_nothing_and_meets_the_time_budget() {
    // The counter must see an allocation, or a zero count below means nothing.
    let probe = allocations();
    std::hint::black_box(Vec::<u8>::with_capacity(16));
    assert!(allocations() > probe, "allocation counter is not counting");

    let mut w = BitWriter::new();
    sample().encode(&mut w).expect("sample is in range");
    let bytes = w.into_bytes();

    // Warm up, so one-time initialisation is not counted.
    let first = BsmCoreData::decode(&mut BitReader::new(&bytes)).expect("decodes");
    assert_eq!(first, sample());

    const ROUNDS: u32 = 10_000;
    let allocs_before = allocations();
    let start = Instant::now();
    let mut speed_sum = 0u64;
    for _ in 0..ROUNDS {
        let decoded = BsmCoreData::decode(&mut BitReader::new(&bytes)).expect("decodes");
        // Use the result, so the decode cannot be optimised away.
        speed_sum += u64::from(decoded.speed);
    }
    let elapsed = start.elapsed();
    let allocs = allocations() - allocs_before;

    assert_eq!(speed_sum, u64::from(sample().speed) * u64::from(ROUNDS));
    assert_eq!(allocs, 0, "decoding allocated {allocs} times");

    // The 10 µs budget is for an optimised build. Debug timings say nothing about it.
    let mean_us = elapsed.as_secs_f64() * 1e6 / f64::from(ROUNDS);
    if !cfg!(debug_assertions) {
        assert!(mean_us < 10.0, "mean decode {mean_us:.2} µs");
    }
    eprintln!("BSM decode: mean {mean_us:.3} µs over {ROUNDS} rounds, 0 allocations");
}
