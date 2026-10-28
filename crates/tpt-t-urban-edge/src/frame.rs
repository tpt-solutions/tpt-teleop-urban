//! Actuation frame codec: the 8-byte payload that carries one steering, brake and
//! throttle command on a CAN-style bus.
//!
//! This is this repository's own layout, not a vehicle's. A real vehicle defines
//! its bus in a signal database that this repository does not have, so the layout
//! here is what the bring-up test rig and the simulator agree on. Check it against
//! the vehicle's database before it drives anything.
//!
//! Layout, big-endian:
//!
//! | Bytes | Field                 | Range                       |
//! |-------|-----------------------|-----------------------------|
//! | 0-1   | steer, centidegrees   | `-MAX_STEER_CDEG..=MAX_STEER_CDEG` |
//! | 2-3   | brake, permille       | `0..=1000`                  |
//! | 4-5   | throttle, permille    | `0..=1000`                  |
//! | 6     | sequence counter      | any; wraps at 255           |
//! | 7     | CRC-8 (poly 0x07) of bytes 0-6 | checked on decode  |
//!
//! Encoding and decoding allocate nothing, so the codec is safe on `no_std`.

use crate::actuator::{MAX_PERMILLE, MAX_STEER_CDEG};

/// CAN identifier the actuation frame is sent on. Chosen here, not from a database.
pub const ACTUATION_ID: u16 = 0x100;

/// Length of the payload, in bytes.
pub const LEN: usize = 8;

/// A decoded actuation command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ActuationFrame {
    /// Steering angle, signed, in centidegrees.
    pub steer_cdeg: i32,
    /// Brake command, in parts per thousand.
    pub brake_permille: u16,
    /// Throttle command, in parts per thousand.
    pub throttle_permille: u16,
    /// Sequence counter, so the receiver can see repeats and gaps.
    pub seq: u8,
}

/// Why a payload was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// The CRC byte does not match the payload.
    Crc,
    /// A field is outside its range, even though the CRC matched.
    Range,
}

/// CRC-8 with polynomial 0x07 and initial value 0, over `bytes`.
#[must_use]
pub const fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    let mut i = 0;
    while i < bytes.len() {
        crc ^= bytes[i];
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
            bit += 1;
        }
        i += 1;
    }
    crc
}

impl ActuationFrame {
    /// The 8-byte payload for this command.
    #[must_use]
    pub const fn encode(&self) -> [u8; LEN] {
        let steer = (self.steer_cdeg as i16).to_be_bytes();
        let brake = self.brake_permille.to_be_bytes();
        let throttle = self.throttle_permille.to_be_bytes();
        let body = [
            steer[0],
            steer[1],
            brake[0],
            brake[1],
            throttle[0],
            throttle[1],
            self.seq,
        ];
        let crc = crc8(&body);
        [
            body[0], body[1], body[2], body[3], body[4], body[5], body[6], crc,
        ]
    }

    /// Parses a payload. Checks the CRC first, then the ranges.
    ///
    /// # Errors
    ///
    /// [`FrameError::Crc`] when the payload is corrupt, and [`FrameError::Range`]
    /// when a field is out of range.
    pub fn decode(payload: &[u8; LEN]) -> Result<Self, FrameError> {
        let (body, crc) = (&payload[..LEN - 1], payload[LEN - 1]);
        if crc8(body) != crc {
            return Err(FrameError::Crc);
        }
        let steer = i16::from_be_bytes([payload[0], payload[1]]);
        let brake = u16::from_be_bytes([payload[2], payload[3]]);
        let throttle = u16::from_be_bytes([payload[4], payload[5]]);
        if i32::from(steer).abs() > MAX_STEER_CDEG
            || brake > MAX_PERMILLE
            || throttle > MAX_PERMILLE
        {
            return Err(FrameError::Range);
        }
        Ok(Self {
            steer_cdeg: i32::from(steer),
            brake_permille: brake,
            throttle_permille: throttle,
            seq: payload[6],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: ActuationFrame = ActuationFrame {
        steer_cdeg: -616,
        brake_permille: 178,
        throttle_permille: 300,
        seq: 41,
    };

    #[test]
    fn every_field_round_trips() {
        assert_eq!(ActuationFrame::decode(&SAMPLE.encode()), Ok(SAMPLE));
    }

    #[test]
    fn the_extremes_round_trip() {
        for steer in [-MAX_STEER_CDEG, 0, MAX_STEER_CDEG] {
            for brake in [0, MAX_PERMILLE] {
                let f = ActuationFrame {
                    steer_cdeg: steer,
                    brake_permille: brake,
                    throttle_permille: MAX_PERMILLE - brake,
                    seq: u8::MAX,
                };
                assert_eq!(ActuationFrame::decode(&f.encode()), Ok(f));
            }
        }
    }

    #[test]
    fn every_single_bit_flip_is_caught() {
        let good = SAMPLE.encode();
        for byte in 0..LEN {
            for bit in 0..8 {
                let mut bad = good;
                bad[byte] ^= 1 << bit;
                assert_eq!(
                    ActuationFrame::decode(&bad),
                    Err(FrameError::Crc),
                    "flip byte {byte} bit {bit} was not caught"
                );
            }
        }
    }

    #[test]
    fn a_valid_crc_on_an_out_of_range_field_is_rejected() {
        // Steering past the limit, with a CRC that matches, must still be refused.
        let steer = (MAX_STEER_CDEG + 1) as i16;
        let mut body = [0u8; LEN];
        body[..2].copy_from_slice(&steer.to_be_bytes());
        body[6] = 0;
        body[LEN - 1] = crc8(&body[..LEN - 1]);
        assert_eq!(ActuationFrame::decode(&body), Err(FrameError::Range));
    }

    #[test]
    fn crc8_matches_the_standard_check_value() {
        // CRC-8 (poly 0x07, init 0) of "123456789" is 0xF4.
        assert_eq!(crc8(b"123456789"), 0xF4);
    }
}
