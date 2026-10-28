//! V2X traffic injection: neighbouring vehicles broadcast BSMs over a channel
//! with a fixed latency and random loss.
//!
//! Each neighbour is a position and speed in a local frame, in metres. It is
//! encoded as a BSM core-data message with the UPER encoder from
//! `tpt-t-urban-v2x`, sent through a [`Channel`], and decoded at the receiver.
//! Decoded positions are mapped back to the local frame, so the report can state
//! the position error the message format adds.
//!
//! The local frame is east-north, anchored at a latitude and longitude. Positions
//! are converted with a flat-earth approximation, which is accurate to well under
//! a metre across an urban area.

use std::collections::VecDeque;

use tpt_t_urban_v2x::bsm::{
    AccelerationSet4Way, BrakeSystemStatus, BsmCoreData, PositionalAccuracy, VehicleSize,
};
use tpt_t_urban_v2x::uper::{BitReader, BitWriter};

/// Metres per degree of latitude, approximately.
const METRES_PER_DEG_LAT: f64 = 111_320.0;
/// BSM latitude and longitude are in tenths of a microdegree.
const UNITS_PER_DEG: f64 = 10_000_000.0;

/// The local frame's anchor, in degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Anchor {
    /// Latitude of the local origin, in degrees.
    pub lat_deg: f64,
    /// Longitude of the local origin, in degrees.
    pub lon_deg: f64,
}

/// A neighbouring vehicle's state in the local frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Neighbour {
    /// East position, in metres.
    pub x_m: f32,
    /// North position, in metres.
    pub y_m: f32,
    /// Speed, in metres per second.
    pub speed_mps: f32,
}

/// Builds the BSM core data for a neighbour at `anchor`.
#[must_use]
pub fn bsm_for(n: &Neighbour, anchor: Anchor, msg_cnt: u8) -> BsmCoreData {
    let lat = anchor.lat_deg + f64::from(n.y_m) / METRES_PER_DEG_LAT;
    let lon = anchor.lon_deg
        + f64::from(n.x_m) / (METRES_PER_DEG_LAT * anchor.lat_deg.to_radians().cos());
    BsmCoreData {
        msg_cnt,
        temporary_id: [0x12, 0x34, 0x56, 0x78],
        sec_mark: 0,
        lat: (lat * UNITS_PER_DEG).round() as i32,
        lon: (lon * UNITS_PER_DEG).round() as i32,
        elev: 0,
        accuracy: PositionalAccuracy {
            semi_major: 5,
            semi_minor: 5,
            orientation: 0,
        },
        transmission: 0,
        // Saturates to the field type, so a speed the encoder cannot take is refused there.
        speed: (n.speed_mps * 100.0)
            .round()
            .clamp(0.0, f32::from(u16::MAX)) as u16,
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
            length: 480,
            width: 180,
        },
    }
}

/// The local-frame position `(x_m, y_m)` of a decoded BSM.
#[must_use]
pub fn position_of(bsm: &BsmCoreData, anchor: Anchor) -> (f32, f32) {
    let lat = f64::from(bsm.lat) / UNITS_PER_DEG;
    let lon = f64::from(bsm.lon) / UNITS_PER_DEG;
    let y = (lat - anchor.lat_deg) * METRES_PER_DEG_LAT;
    let x = (lon - anchor.lon_deg) * METRES_PER_DEG_LAT * anchor.lat_deg.to_radians().cos();
    (x as f32, y as f32)
}

/// A lossy, delayed broadcast channel. Messages arrive `latency_ms` after they
/// are sent, unless they are lost.
#[derive(Clone, Debug)]
pub struct Channel {
    latency_ms: u64,
    loss_threshold: u32,
    state: u32,
    in_flight: VecDeque<(u64, Vec<u8>)>,
}

impl Channel {
    /// A channel with `latency_ms` delay and `loss` probability in `[0, 1]`.
    /// `seed` must be non-zero.
    #[must_use]
    pub fn new(latency_ms: u64, loss: f32, seed: u32) -> Self {
        let loss = loss.clamp(0.0, 1.0);
        Self {
            latency_ms,
            loss_threshold: (f64::from(loss) * f64::from(u32::MAX)) as u32,
            state: if seed == 0 { 0x2F6E_2B1D } else { seed },
            in_flight: VecDeque::new(),
        }
    }

    fn next(&mut self) -> u32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        self.state
    }

    /// Sends `payload` at `now_ms`. It is queued for delivery, or dropped.
    /// Returns `false` if it was dropped.
    pub fn send(&mut self, now_ms: u64, payload: Vec<u8>) -> bool {
        if self.next() < self.loss_threshold {
            return false;
        }
        self.in_flight
            .push_back((now_ms + self.latency_ms, payload));
        true
    }

    /// Payloads whose delivery time has come, in the order they were sent.
    pub fn receive(&mut self, now_ms: u64) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        while self
            .in_flight
            .front()
            .is_some_and(|(due, _)| *due <= now_ms)
        {
            if let Some((_, payload)) = self.in_flight.pop_front() {
                out.push(payload);
            }
        }
        out
    }
}

/// What an injection run measured.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Report {
    /// Messages offered to the channel.
    pub offered: u32,
    /// Messages the encoder refused, for a field outside its range.
    pub refused: u32,
    /// Messages dropped by the channel.
    pub lost: u32,
    /// Messages delivered and decoded.
    pub delivered: u32,
    /// Delivered messages that failed to decode.
    pub decode_errors: u32,
    /// Largest distance between a neighbour and its decoded position, in metres.
    pub max_position_error_m: f32,
}

/// Broadcasts each neighbour's BSM every `period_ms` for `duration_ms`, through
/// `channel`, and decodes every message that arrives.
#[must_use]
pub fn inject(
    neighbours: &[Neighbour],
    anchor: Anchor,
    channel: &mut Channel,
    period_ms: u64,
    duration_ms: u64,
) -> Report {
    let mut report = Report::default();
    let mut msg_cnt: u8 = 0;
    // The true position of each message, in the order it was sent, so the
    // receiver can measure its error.
    let mut truth: VecDeque<(u64, (f32, f32))> = VecDeque::new();
    let mut now = 0;
    while now <= duration_ms {
        for n in neighbours {
            let bsm = bsm_for(n, anchor, msg_cnt);
            msg_cnt = (msg_cnt + 1) % 128;
            let mut w = BitWriter::new();
            if bsm.encode(&mut w).is_err() {
                report.refused += 1;
                continue;
            }
            report.offered += 1;
            if channel.send(now, w.into_bytes()) {
                truth.push_back((now, (n.x_m, n.y_m)));
            } else {
                report.lost += 1;
            }
        }
        for payload in channel.receive(now) {
            // Messages arrive in send order, so the front of the queue is theirs.
            let sent_truth = truth.pop_front();
            match BsmCoreData::decode(&mut BitReader::new(&payload)) {
                Ok(bsm) => {
                    report.delivered += 1;
                    if let Some((_, (x, y))) = sent_truth {
                        let (dx, dy) = position_of(&bsm, anchor);
                        let error = ((dx - x).powi(2) + (dy - y).powi(2)).sqrt();
                        report.max_position_error_m = report.max_position_error_m.max(error);
                    }
                }
                Err(_) => report.decode_errors += 1,
            }
        }
        now += period_ms;
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    const ANCHOR: Anchor = Anchor {
        lat_deg: 30.0,
        lon_deg: -97.0,
    };

    fn traffic() -> Vec<Neighbour> {
        vec![
            Neighbour {
                x_m: 120.0,
                y_m: -40.0,
                speed_mps: 11.0,
            },
            Neighbour {
                x_m: -35.5,
                y_m: 8.25,
                speed_mps: 0.0,
            },
            Neighbour {
                x_m: 0.0,
                y_m: 0.0,
                speed_mps: 25.0,
            },
        ]
    }

    #[test]
    fn a_lossless_channel_delivers_every_message_within_the_format_precision() {
        let mut channel = Channel::new(50, 0.0, 7);
        let report = inject(&traffic(), ANCHOR, &mut channel, 100, 10_000);
        assert_eq!(report.lost, 0);
        assert_eq!(report.decode_errors, 0);
        assert_eq!(
            report.delivered + channel.in_flight.len() as u32,
            report.offered
        );
        // Tenth-microdegree resolution is about 1.1 cm in latitude and 0.9 cm in
        // longitude. Allow a little more for the flat-earth approximation.
        assert!(report.max_position_error_m < 0.03, "{report:?}");
    }

    #[test]
    fn random_loss_drops_about_the_configured_fraction() {
        let mut channel = Channel::new(0, 0.1, 99);
        let report = inject(&traffic(), ANCHOR, &mut channel, 100, 60_000);
        let ratio = f64::from(report.lost) / f64::from(report.offered);
        assert!((0.08..0.12).contains(&ratio), "loss ratio {ratio}");
        assert_eq!(report.decode_errors, 0);
    }

    #[test]
    fn a_message_arrives_exactly_the_channel_latency_after_it_was_sent() {
        let mut channel = Channel::new(75, 0.0, 3);
        assert!(channel.send(1_000, vec![1, 2, 3]));
        assert!(channel.receive(1_074).is_empty());
        assert_eq!(channel.receive(1_075), vec![vec![1, 2, 3]]);
    }

    #[test]
    fn a_neighbour_outside_the_format_range_is_refused_not_sent() {
        let fast = Neighbour {
            x_m: 0.0,
            y_m: 0.0,
            speed_mps: 900.0,
        };
        let mut channel = Channel::new(0, 0.0, 1);
        let report = inject(&[fast], ANCHOR, &mut channel, 100, 1_000);
        assert!(report.refused > 0, "{report:?}");
        assert_eq!(report.offered, 0);
    }
}
