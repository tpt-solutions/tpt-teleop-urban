#![no_main]

use libfuzzer_sys::fuzz_target;
use tpt_t_urban_v2x::bsm::BsmCoreData;
use tpt_t_urban_v2x::uper::BitReader;

// Any byte string must decode to `Ok` or `Err`, never panic or read past the
// input. A malformed BSM is refused, so the decoder never sees a bad frame as a
// valid one.
fuzz_target!(|data: &[u8]| {
    let mut reader = BitReader::new(data);
    let _ = BsmCoreData::decode(&mut reader);
});
