//! Record encoding for the black-box ledger (Phase 6 logging).
//!
//! Each record is a tag byte followed by fixed-width big-endian fields. A
//! [`crate::DurableLedger`] stores the encoded bytes as an entry body, so the
//! chain, signature and recovery rules apply to every record.
//!
//! Only producers that exist in this workspace are represented. Sensor data,
//! V2X messages and planner outputs have no producer yet, so they have no
//! variant here.

const TAG_GUARDIAN: u8 = 1;
const TAG_OUTCOME: u8 = 2;

/// One thing worth keeping in the black box.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Record {
    /// The guardian started or stopped overriding the planner.
    Guardian {
        /// Simulated time of the change, in milliseconds.
        t_ms: u64,
        /// `true` when the override starts, `false` when it ends.
        intervening: bool,
    },
    /// The end of a run, after the last step.
    Outcome {
        /// Whether the vehicle hit the pedestrian.
        collided: bool,
        /// Whether automatic emergency braking engaged at any point.
        aeb_engaged: bool,
        /// Vehicle speed at the end, in millimetres per second.
        final_speed_mm_s: u32,
    },
}

impl Record {
    /// The bytes stored as the ledger entry body.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        match *self {
            Self::Guardian { t_ms, intervening } => {
                let mut out = vec![TAG_GUARDIAN];
                out.extend_from_slice(&t_ms.to_be_bytes());
                out.push(u8::from(intervening));
                out
            }
            Self::Outcome {
                collided,
                aeb_engaged,
                final_speed_mm_s,
            } => {
                let mut out = vec![TAG_OUTCOME, u8::from(collided), u8::from(aeb_engaged)];
                out.extend_from_slice(&final_speed_mm_s.to_be_bytes());
                out
            }
        }
    }

    /// Parses a stored body. Returns `None` for an unknown tag, a wrong length,
    /// or a flag byte other than 0 or 1.
    #[must_use]
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let (&tag, rest) = bytes.split_first()?;
        match tag {
            TAG_GUARDIAN if rest.len() == 9 => Some(Self::Guardian {
                t_ms: u64::from_be_bytes(rest[0..8].try_into().ok()?),
                intervening: flag(rest[8])?,
            }),
            TAG_OUTCOME if rest.len() == 6 => Some(Self::Outcome {
                collided: flag(rest[0])?,
                aeb_engaged: flag(rest[1])?,
                final_speed_mm_s: u32::from_be_bytes(rest[2..6].try_into().ok()?),
            }),
            _ => None,
        }
    }
}

fn flag(byte: u8) -> Option<bool> {
    match byte {
        0 => Some(false),
        1 => Some(true),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_record_round_trips() {
        let records = [
            Record::Guardian {
                t_ms: 1_234_567,
                intervening: true,
            },
            Record::Guardian {
                t_ms: 0,
                intervening: false,
            },
            Record::Outcome {
                collided: false,
                aeb_engaged: true,
                final_speed_mm_s: 0,
            },
        ];
        for record in records {
            assert_eq!(Record::decode(&record.encode()), Some(record));
        }
    }

    #[test]
    fn malformed_bodies_are_refused() {
        assert_eq!(Record::decode(&[]), None);
        assert_eq!(Record::decode(&[9, 0, 0]), None, "unknown tag");
        assert_eq!(Record::decode(&[TAG_GUARDIAN, 0, 0]), None, "short");
        let mut bad_flag = Record::Guardian {
            t_ms: 5,
            intervening: true,
        }
        .encode();
        *bad_flag.last_mut().unwrap() = 2;
        assert_eq!(Record::decode(&bad_flag), None);
    }
}
