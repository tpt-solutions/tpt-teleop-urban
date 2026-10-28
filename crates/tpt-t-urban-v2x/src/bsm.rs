//! BSM Part I core data (SAE J2735 `BSMcoreData`), decoded from UPER.
//!
//! # Verification status: NOT interoperability-checked
//!
//! The field order, bit widths and value ranges below were written from memory
//! of SAE J2735. The authoritative ASN.1 module is sold by SAE and was not
//! available to check against, so the constants in this module are unverified.
//! Each range is a named constant so it can be corrected in one place once the
//! module is checked. Do not use this decoder against live radio traffic until
//! that check is done.
//!
//! The UPER primitives in [`crate::uper`] are verified against X.691 (02/2021).
//! Only the mapping of J2735 fields onto them is unverified.
//!
//! Parsing performs no heap allocation. The caller owns the input bytes.

use crate::uper::{BitReader, BitWriter, Result};

// Unverified J2735 ranges (see module docs). Bit widths follow from them.
/// Valid range of `msgCnt`, in raw units. Unverified against J2735.
pub const MSG_CNT_RANGE: (i64, i64) = (0, 127);
/// Valid range of `secMark`, in raw units. Unverified against J2735.
pub const SEC_MARK_RANGE: (i64, i64) = (0, 65_535);
/// Valid range of latitude, in tenths of a microdegree. Unverified against J2735.
pub const LAT_RANGE: (i64, i64) = (-900_000_000, 900_000_001);
/// Valid range of longitude, in tenths of a microdegree. Unverified against J2735.
pub const LON_RANGE: (i64, i64) = (-1_799_999_999, 1_800_000_001);
/// Valid range of elevation, in tenths of a metre. Unverified against J2735.
pub const ELEV_RANGE: (i64, i64) = (-4_096, 61_439);
/// Valid range of an error-ellipse semi-axis, in raw units. Unverified against J2735.
pub const SEMI_AXIS_RANGE: (i64, i64) = (0, 255);
/// Valid range of the error-ellipse orientation, in raw units. Unverified against J2735.
pub const ORIENTATION_RANGE: (i64, i64) = (0, 65_535);
/// Eight enumerators: neutral, park, forward, reverse, three reserved, unavailable.
pub const TRANSMISSION_RANGE: (i64, i64) = (0, 7);
/// Valid range of speed, in centimetres per second. Unverified against J2735.
pub const SPEED_RANGE: (i64, i64) = (0, 8_191);
/// Valid range of heading, in 0.0125 degree units. Unverified against J2735.
pub const HEADING_RANGE: (i64, i64) = (0, 28_800);
/// Valid range of steering-wheel angle, in raw units. Unverified against J2735.
pub const STEER_ANGLE_RANGE: (i64, i64) = (-126, 127);
/// Valid range of longitudinal acceleration, in raw units. Unverified against J2735.
pub const ACCEL_LONG_RANGE: (i64, i64) = (-2_000, 2_001);
/// Valid range of lateral acceleration, in raw units. Unverified against J2735.
pub const ACCEL_LAT_RANGE: (i64, i64) = (-2_000, 2_001);
/// Valid range of vertical acceleration, in raw units. Unverified against J2735.
pub const ACCEL_VERT_RANGE: (i64, i64) = (-127, 127);
/// Valid range of yaw rate, in raw units. Unverified against J2735.
pub const YAW_RANGE: (i64, i64) = (-32_767, 32_767);
/// Valid range of the traction control status index.
pub const TRACTION_RANGE: (i64, i64) = (0, 3);
/// Valid range of the anti-lock brake status index.
pub const ABS_RANGE: (i64, i64) = (0, 3);
/// Valid range of the stability control status index.
pub const SCS_RANGE: (i64, i64) = (0, 3);
/// Valid range of the brake boost status index.
pub const BRAKE_BOOST_RANGE: (i64, i64) = (0, 2);
/// Valid range of the auxiliary brake status index.
pub const AUX_BRAKES_RANGE: (i64, i64) = (0, 3);
/// Valid range of vehicle length, in centimetres. Unverified against J2735.
pub const VEHICLE_LENGTH_RANGE: (i64, i64) = (0, 1_023);
/// Valid range of vehicle width, in centimetres. Unverified against J2735.
pub const VEHICLE_WIDTH_RANGE: (i64, i64) = (0, 1_023);

/// Five-bit wheel-brake bit string (`BIT STRING (SIZE(5))`).
const WHEEL_BRAKES_BITS: u32 = 5;

/// Position error ellipse.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PositionalAccuracy {
    /// Semi-major axis of the position error ellipse, raw units.
    pub semi_major: u8,
    /// Semi-minor axis of the position error ellipse, raw units.
    pub semi_minor: u8,
    /// Orientation of the major axis, raw units.
    pub orientation: u16,
}

/// Longitudinal, lateral, vertical and yaw acceleration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AccelerationSet4Way {
    /// Longitudinal acceleration, raw units.
    pub long: i16,
    /// Lateral acceleration, raw units.
    pub lat: i16,
    /// Vertical acceleration, raw units.
    pub vert: i8,
    /// Yaw rate, raw units.
    pub yaw: i16,
}

/// Raw brake-system status values. Each enumeration is kept as its index so the
/// decoder claims nothing beyond the bit layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BrakeSystemStatus {
    /// Raw wheel-brake status, a five-bit set.
    pub wheel_brakes: u8,
    /// Raw traction-control status index.
    pub traction: u8,
    /// Raw anti-lock brake status index.
    pub abs: u8,
    /// Raw stability control status index.
    pub scs: u8,
    /// Raw brake boost status index.
    pub brake_boost: u8,
    /// Raw auxiliary brake status index.
    pub aux_brakes: u8,
}

/// Vehicle footprint in centimetres.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VehicleSize {
    /// Vehicle length, in centimetres.
    pub length: u16,
    /// Vehicle width, in centimetres.
    pub width: u16,
}

/// BSM Part I core data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BsmCoreData {
    /// Message count, wrapping at 127.
    pub msg_cnt: u8,
    /// Temporary vehicle identifier, four octets.
    pub temporary_id: [u8; 4],
    /// Millisecond within the minute.
    pub sec_mark: u16,
    /// Tenth of a microdegree.
    pub lat: i32,
    /// Tenth of a microdegree.
    pub lon: i32,
    /// Tenth of a metre.
    pub elev: i32,
    /// Position error ellipse.
    pub accuracy: PositionalAccuracy,
    /// Raw transmission state, within `TRANSMISSION_RANGE`.
    pub transmission: u8,
    /// Centimetres per second.
    pub speed: u16,
    /// 0.0125 degree units.
    pub heading: u16,
    /// Steering-wheel angle, raw units.
    pub steer_angle: i8,
    /// Accelerations in four axes.
    pub accel: AccelerationSet4Way,
    /// Brake system status.
    pub brakes: BrakeSystemStatus,
    /// Vehicle footprint.
    pub size: VehicleSize,
}

impl BsmCoreData {
    /// Decodes the core data from the reader's current position.
    pub fn decode(reader: &mut BitReader<'_>) -> Result<Self> {
        let msg_cnt = ranged(reader, MSG_CNT_RANGE)? as u8;
        let temporary_id = reader.fixed_octets::<4>()?;
        let sec_mark = ranged(reader, SEC_MARK_RANGE)? as u16;
        let lat = ranged(reader, LAT_RANGE)? as i32;
        let lon = ranged(reader, LON_RANGE)? as i32;
        let elev = ranged(reader, ELEV_RANGE)? as i32;
        let accuracy = PositionalAccuracy {
            semi_major: ranged(reader, SEMI_AXIS_RANGE)? as u8,
            semi_minor: ranged(reader, SEMI_AXIS_RANGE)? as u8,
            orientation: ranged(reader, ORIENTATION_RANGE)? as u16,
        };
        let transmission = ranged(reader, TRANSMISSION_RANGE)? as u8;
        let speed = ranged(reader, SPEED_RANGE)? as u16;
        let heading = ranged(reader, HEADING_RANGE)? as u16;
        let steer_angle = ranged(reader, STEER_ANGLE_RANGE)? as i8;
        let accel = AccelerationSet4Way {
            long: ranged(reader, ACCEL_LONG_RANGE)? as i16,
            lat: ranged(reader, ACCEL_LAT_RANGE)? as i16,
            vert: ranged(reader, ACCEL_VERT_RANGE)? as i8,
            yaw: ranged(reader, YAW_RANGE)? as i16,
        };
        let brakes = BrakeSystemStatus {
            wheel_brakes: reader.read_bits(WHEEL_BRAKES_BITS)? as u8,
            traction: ranged(reader, TRACTION_RANGE)? as u8,
            abs: ranged(reader, ABS_RANGE)? as u8,
            scs: ranged(reader, SCS_RANGE)? as u8,
            brake_boost: ranged(reader, BRAKE_BOOST_RANGE)? as u8,
            aux_brakes: ranged(reader, AUX_BRAKES_RANGE)? as u8,
        };
        let size = VehicleSize {
            length: ranged(reader, VEHICLE_LENGTH_RANGE)? as u16,
            width: ranged(reader, VEHICLE_WIDTH_RANGE)? as u16,
        };
        Ok(Self {
            msg_cnt,
            temporary_id,
            sec_mark,
            lat,
            lon,
            elev,
            accuracy,
            transmission,
            speed,
            heading,
            steer_angle,
            accel,
            brakes,
            size,
        })
    }
}

fn ranged(reader: &mut BitReader<'_>, (lb, ub): (i64, i64)) -> Result<i64> {
    reader.constrained_whole(lb, ub)
}

impl BsmCoreData {
    /// Encodes the core data, field by field in the order [`BsmCoreData::decode`]
    /// reads them. A field outside its range is refused, so the output always
    /// decodes.
    ///
    /// # Errors
    ///
    /// Returns [`UperError::OutOfRange`] for a field outside its range.
    pub fn encode(&self, w: &mut BitWriter) -> Result<()> {
        let s =
            |w: &mut BitWriter, v: i64, (lb, ub): (i64, i64)| w.write_constrained_whole(v, lb, ub);
        s(w, self.msg_cnt.into(), MSG_CNT_RANGE)?;
        w.write_octets(&self.temporary_id)?;
        s(w, self.sec_mark.into(), SEC_MARK_RANGE)?;
        s(w, self.lat.into(), LAT_RANGE)?;
        s(w, self.lon.into(), LON_RANGE)?;
        s(w, self.elev.into(), ELEV_RANGE)?;
        s(w, self.accuracy.semi_major.into(), SEMI_AXIS_RANGE)?;
        s(w, self.accuracy.semi_minor.into(), SEMI_AXIS_RANGE)?;
        s(w, self.accuracy.orientation.into(), ORIENTATION_RANGE)?;
        s(w, self.transmission.into(), TRANSMISSION_RANGE)?;
        s(w, self.speed.into(), SPEED_RANGE)?;
        s(w, self.heading.into(), HEADING_RANGE)?;
        s(w, self.steer_angle.into(), STEER_ANGLE_RANGE)?;
        s(w, self.accel.long.into(), ACCEL_LONG_RANGE)?;
        s(w, self.accel.lat.into(), ACCEL_LAT_RANGE)?;
        s(w, self.accel.vert.into(), ACCEL_VERT_RANGE)?;
        s(w, self.accel.yaw.into(), YAW_RANGE)?;
        w.write_bits(self.brakes.wheel_brakes.into(), WHEEL_BRAKES_BITS)?;
        s(w, self.brakes.traction.into(), TRACTION_RANGE)?;
        s(w, self.brakes.abs.into(), ABS_RANGE)?;
        s(w, self.brakes.scs.into(), SCS_RANGE)?;
        s(w, self.brakes.brake_boost.into(), BRAKE_BOOST_RANGE)?;
        s(w, self.brakes.aux_brakes.into(), AUX_BRAKES_RANGE)?;
        s(w, self.size.length.into(), VEHICLE_LENGTH_RANGE)?;
        s(w, self.size.width.into(), VEHICLE_WIDTH_RANGE)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::uper::{BitWriter, UperError};

    fn encode(d: &BsmCoreData) -> Vec<u8> {
        let mut w = BitWriter::new();
        d.encode(&mut w).expect("sample is in range");
        w.into_bytes()
    }

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
    fn core_data_round_trips() {
        let original = sample();
        let bytes = encode(&original);
        let decoded = BsmCoreData::decode(&mut BitReader::new(&bytes)).unwrap();
        assert_eq!(decoded, original);
    }

    #[test]
    fn range_extremes_round_trip() {
        let low = BsmCoreData {
            msg_cnt: 0,
            lat: -900_000_000,
            lon: -1_799_999_999,
            elev: -4_096,
            steer_angle: -126,
            accel: AccelerationSet4Way {
                long: -2_000,
                lat: -2_000,
                vert: -127,
                yaw: -32_767,
            },
            ..sample()
        };
        let high = BsmCoreData {
            msg_cnt: 127,
            sec_mark: 65_535,
            lat: 900_000_001,
            lon: 1_800_000_001,
            elev: 61_439,
            speed: 8_191,
            heading: 28_800,
            steer_angle: 127,
            accel: AccelerationSet4Way {
                long: 2_001,
                lat: 2_001,
                vert: 127,
                yaw: 32_767,
            },
            size: VehicleSize {
                length: 1_023,
                width: 1_023,
            },
            ..sample()
        };
        for original in [low, high] {
            let bytes = encode(&original);
            let decoded = BsmCoreData::decode(&mut BitReader::new(&bytes)).unwrap();
            assert_eq!(decoded, original);
        }
    }

    #[test]
    fn truncated_input_is_reported() {
        let bytes = encode(&sample());
        let short = &bytes[..bytes.len() - 2];
        let err = BsmCoreData::decode(&mut BitReader::new(short)).unwrap_err();
        assert_eq!(err, UperError::EndOfInput);
    }

    #[test]
    fn out_of_range_acceleration_is_rejected() {
        // The long-acceleration range has 4002 values, so 12 bits can carry 4095,
        // one past the end of the range. The decoder must reject it.
        let mut w = BitWriter::new();
        w.write_bits(4095, 12).unwrap();
        let raw = w.into_bytes();
        let err = BitReader::new(&raw)
            .constrained_whole(ACCEL_LONG_RANGE.0, ACCEL_LONG_RANGE.1)
            .unwrap_err();
        assert_eq!(err, UperError::OutOfRange);
    }

    /// Feeds pseudo-random bytes to the decoder. Every input must produce `Ok`
    /// or an error, never a panic or an out-of-range value.
    #[test]
    fn random_bytes_never_panic_and_decoded_values_are_in_range() {
        let mut state = 0x2545_F491_u32;
        let mut buf = [0u8; 40];
        for _ in 0..20_000 {
            for b in &mut buf {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                *b = state as u8;
            }
            let Ok(d) = BsmCoreData::decode(&mut BitReader::new(&buf)) else {
                continue;
            };
            assert!((MSG_CNT_RANGE.0..=MSG_CNT_RANGE.1).contains(&i64::from(d.msg_cnt)));
            assert!((LAT_RANGE.0..=LAT_RANGE.1).contains(&i64::from(d.lat)));
            assert!((LON_RANGE.0..=LON_RANGE.1).contains(&i64::from(d.lon)));
            assert!((SPEED_RANGE.0..=SPEED_RANGE.1).contains(&i64::from(d.speed)));
            assert!(
                (VEHICLE_WIDTH_RANGE.0..=VEHICLE_WIDTH_RANGE.1).contains(&i64::from(d.size.width))
            );
        }
    }
}

#[cfg(test)]
mod encode_tests {
    use super::*;
    use crate::uper::UperError;

    #[test]
    fn a_field_outside_its_range_is_refused() {
        let mut bsm = BsmCoreData {
            msg_cnt: 0,
            temporary_id: [0; 4],
            sec_mark: 0,
            lat: 0,
            lon: 0,
            elev: 0,
            accuracy: PositionalAccuracy {
                semi_major: 0,
                semi_minor: 0,
                orientation: 0,
            },
            transmission: 0,
            speed: 0,
            heading: 0,
            steer_angle: 0,
            accel: AccelerationSet4Way {
                long: 0,
                lat: 0,
                vert: 0,
                yaw: 0,
            },
            brakes: BrakeSystemStatus {
                wheel_brakes: 0,
                traction: 0,
                abs: 0,
                scs: 0,
                brake_boost: 0,
                aux_brakes: 0,
            },
            size: VehicleSize {
                length: 0,
                width: 0,
            },
        };
        let mut ok = BitWriter::new();
        assert_eq!(bsm.encode(&mut ok), Ok(()));
        bsm.speed = 9_000;
        let mut w = BitWriter::new();
        assert_eq!(bsm.encode(&mut w), Err(UperError::OutOfRange));
    }
}
