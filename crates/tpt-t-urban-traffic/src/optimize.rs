//! Congestion optimization: how a corridor of signals shares its time.
//!
//! Two levers, both pure functions over integers:
//!
//! - [`allocate_greens`] splits one intersection's green time across its phases
//!   in proportion to queued demand. The cycle length stays fixed, so the
//!   signals still meet the timing the rest of the corridor depends on.
//! - [`corridor_offsets`] staggers the cycles along a corridor. Each signal starts
//!   its cycle after the travel time from the one before, so a platoon that
//!   clears one green reaches the next one as it turns green (a "green wave").
//!
//! Every green stays inside `[min_green_ms, max_green_ms]`, so an optimized
//! signal never drops a phase below the minimum that pedestrians and clearing
//! traffic need.

use crate::controller::{ALL_RED_MS, YELLOW_MS};

/// Splits the green time of one cycle across `N` phases.
///
/// `demand` is the queued vehicle count per phase. Green time is shared in
/// proportion to demand, after every phase gets `min_green_ms`. A phase never
/// gets more than `max_green_ms`, and time a capped phase cannot take goes to the
/// others. With no demand at all, the green time is split evenly.
///
/// The green times, plus the yellow and all-red clearance of each phase, add up
/// to `cycle_ms`, except when every phase reaches `max_green_ms` first. Then the
/// result is shorter than the cycle, and the caller must accept that.
///
/// Returns `None` when the cycle cannot fit every phase's minimum green and
/// clearance, or when `min_green_ms > max_green_ms`.
#[must_use]
pub fn allocate_greens<const N: usize>(
    demand: [u32; N],
    cycle_ms: u32,
    min_green_ms: u32,
    max_green_ms: u32,
) -> Option<[u32; N]> {
    if min_green_ms > max_green_ms {
        return None;
    }
    let clearance = N as u32 * (YELLOW_MS + ALL_RED_MS);
    let available = cycle_ms.checked_sub(clearance)?;
    let minimum_total = (N as u32).checked_mul(min_green_ms)?;
    if minimum_total > available {
        return None;
    }

    let mut greens = [min_green_ms; N];
    let mut spare = u64::from(available - minimum_total);

    // Share the spare time out in rounds. A phase that reaches its maximum drops
    // out, and the next round shares what it could not take. Each round either
    // caps a phase or hands out time, so N rounds are enough.
    for _ in 0..N {
        if spare == 0 {
            break;
        }
        let open_weight: u64 = (0..N)
            .filter(|&i| greens[i] < max_green_ms)
            .map(|i| u64::from(demand[i]))
            .sum();
        // Open phases with no demand share the time evenly, so no time is lost.
        let even = open_weight == 0;
        let total = if even {
            (0..N).filter(|&i| greens[i] < max_green_ms).count() as u64
        } else {
            open_weight
        };
        if total == 0 {
            break;
        }
        let pool = spare;
        let mut given = 0;
        for i in 0..N {
            if greens[i] >= max_green_ms {
                continue;
            }
            let w = if even { 1 } else { u64::from(demand[i]) };
            let share = pool * w / total;
            let room = u64::from(max_green_ms - greens[i]);
            let add = share.min(room);
            greens[i] += add as u32;
            given += add;
        }
        spare -= given;
        if given == 0 {
            break;
        }
    }

    // Integer division leaves a few milliseconds. Hand them out one at a time to
    // the phases that still have room, in index order.
    let mut i = 0;
    while spare > 0 {
        if greens.iter().all(|&g| g >= max_green_ms) {
            break;
        }
        if greens[i % N] < max_green_ms {
            greens[i % N] += 1;
            spare -= 1;
        }
        i += 1;
    }

    Some(greens)
}

/// Start offset of each signal in a corridor, in milliseconds into the cycle.
///
/// `travel_ms[i]` is the travel time from signal `i` to signal `i + 1`. Signal
/// zero is the reference, with offset zero. Each later offset adds the travel
/// time before it, modulo `cycle_ms`. A vehicle that clears signal `i` at its
/// green's end then meets signal `i + 1` at the start of its green.
///
/// `travel_ms` has `N - 1` entries. Extra entries are ignored. A `cycle_ms` of
/// zero yields all zeros, since there is no cycle to offset within.
#[must_use]
pub fn corridor_offsets<const N: usize>(travel_ms: &[u32], cycle_ms: u32) -> [u32; N] {
    let mut offsets = [0u32; N];
    if cycle_ms == 0 {
        return offsets;
    }
    for i in 1..N {
        let leg = travel_ms.get(i - 1).copied().unwrap_or(0) % cycle_ms;
        offsets[i] = (offsets[i - 1] + leg) % cycle_ms;
    }
    offsets
}

#[cfg(test)]
mod tests {
    use super::*;

    const CLEAR: u32 = YELLOW_MS + ALL_RED_MS;

    #[test]
    fn greens_follow_demand_and_fill_the_cycle() {
        // Two phases, 100 s cycle, 4 s clearance each.
        let cycle = 100_000;
        let greens = allocate_greens([300, 100], cycle, 5_000, 80_000).unwrap();
        assert!(greens[0] > greens[1], "heavier phase gets more green");
        assert_eq!(greens[0] + greens[1] + 2 * CLEAR, cycle);
    }

    #[test]
    fn equal_demand_splits_evenly() {
        let greens = allocate_greens([50, 50, 50], 100_000, 5_000, 80_000).unwrap();
        let available = 100_000 - 3 * CLEAR;
        assert_eq!(greens.iter().sum::<u32>(), available);
        let (lo, hi) = (greens.iter().min().unwrap(), greens.iter().max().unwrap());
        assert!(hi - lo <= 1, "even split within one millisecond");
    }

    #[test]
    fn no_demand_is_an_even_split() {
        let greens = allocate_greens([0, 0], 60_000, 5_000, 50_000).unwrap();
        assert_eq!(greens[0], greens[1]);
    }

    #[test]
    fn every_green_respects_the_bounds() {
        let greens = allocate_greens([1_000, 1, 1], 120_000, 8_000, 40_000).unwrap();
        for g in greens {
            assert!((8_000..=40_000).contains(&g), "green {g} out of bounds");
        }
    }

    #[test]
    fn time_a_capped_phase_cannot_take_goes_to_the_others() {
        // The heavy phase caps at 30 s; the rest of the spare time must not vanish.
        let cycle = 90_000;
        let greens = allocate_greens([10_000, 10, 10], cycle, 5_000, 30_000).unwrap();
        assert_eq!(greens[0], 30_000);
        assert_eq!(greens.iter().sum::<u32>(), cycle - 3 * CLEAR);
    }

    #[test]
    fn an_infeasible_cycle_is_refused() {
        // Three phases need 3 * (5 s + 4 s) of clearance and minimum green.
        assert_eq!(allocate_greens([1, 1, 1], 20_000, 5_000, 9_000), None);
        assert_eq!(allocate_greens([1, 1], 60_000, 9_000, 5_000), None);
    }

    #[test]
    fn corridor_offsets_accumulate_travel_time_modulo_the_cycle() {
        let offsets: [u32; 4] = corridor_offsets(&[20_000, 35_000, 50_000], 60_000);
        assert_eq!(offsets, [0, 20_000, 55_000, 45_000]);
    }

    #[test]
    fn corridor_offsets_handle_a_zero_cycle() {
        let offsets: [u32; 3] = corridor_offsets(&[10, 20], 0);
        assert_eq!(offsets, [0, 0, 0]);
    }
}
