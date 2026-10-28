#![no_main]

use libfuzzer_sys::fuzz_target;
use tpt_t_urban_v2x::uper::BitReader;

// Drives every UPER primitive from the same input, with bounds taken from the
// input itself, so extreme and inverted ranges are reached.
fuzz_target!(|data: &[u8]| {
    if data.len() < 17 {
        return;
    }
    let lb = i64::from_le_bytes(data[0..8].try_into().unwrap());
    let ub = i64::from_le_bytes(data[8..16].try_into().unwrap());
    let bits = u32::from(data[16]);
    let body = &data[17..];

    let mut reader = BitReader::new(body);
    let _ = reader.read_bits(bits);
    let _ = reader.read_bool();
    let _ = reader.constrained_whole(lb, ub);
    let _ = reader.enumerated(lb.unsigned_abs(), bits & 1 == 1);
    let _ = reader.normally_small();
    let _ = reader.length(if bits & 2 == 0 {
        None
    } else {
        Some(lb.unsigned_abs())
    });
    let _ = reader.fixed_octets::<4>();
});
