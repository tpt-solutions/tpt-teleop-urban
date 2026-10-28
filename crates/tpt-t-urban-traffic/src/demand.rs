//! Queue counts for the optimizer, from vehicle observations.
//!
//! [`queued_per_approach`] is the demand source [`crate::optimize::allocate_greens`]
//! needs. Each approach is a ray from the intersection centre. A vehicle is queued
//! on an approach when it is stopped, lies within the queue length of the stop line
//! on that ray, and is within the lane's half-width of the ray's centre line.
//!
//! The vehicles can come from perception tracks or any other source that gives a
//! position and velocity in the intersection frame, in metres and m/s.

/// A vehicle as the intersection sees it, in the intersection frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vehicle {
    /// Position, in metres.
    pub pos: [f32; 2],
    /// Velocity, in m/s.
    pub vel: [f32; 2],
}

/// Geometry for counting queues.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QueueZone {
    /// How far back from the stop line a queued vehicle may be, in metres.
    pub queue_length_m: f32,
    /// Half the lane width, in metres: the lateral tolerance around the ray.
    pub lane_half_m: f32,
    /// Speed below which a vehicle counts as stopped, in m/s.
    pub stopped_speed_m_s: f32,
}

/// Counts the stopped vehicles queued on each approach.
///
/// `approaches` holds one unit direction per phase, pointing from the intersection
/// centre out along that approach. A vehicle counts once, on the first approach
/// whose zone it is in. Vehicles at or past the centre, or beyond the queue length,
/// are not counted.
#[must_use]
pub fn queued_per_approach<const N: usize>(
    approaches: &[[f32; 2]; N],
    vehicles: &[Vehicle],
    zone: QueueZone,
) -> [u32; N] {
    let mut counts = [0u32; N];
    for v in vehicles {
        let speed = (v.vel[0] * v.vel[0] + v.vel[1] * v.vel[1]).sqrt();
        if speed > zone.stopped_speed_m_s {
            continue;
        }
        let found = approaches.iter().position(|d| {
            // Distance along the ray, and perpendicular distance from it.
            let along = v.pos[0] * d[0] + v.pos[1] * d[1];
            let across = (v.pos[0] * d[1] - v.pos[1] * d[0]).abs();
            along > 0.0 && along <= zone.queue_length_m && across <= zone.lane_half_m
        });
        if let Some(i) = found {
            counts[i] += 1;
        }
    }
    counts
}

#[cfg(test)]
mod tests {
    use super::*;

    const NORTH_SOUTH: [[f32; 2]; 2] = [[0.0, 1.0], [0.0, -1.0]];
    const ZONE: QueueZone = QueueZone {
        queue_length_m: 30.0,
        lane_half_m: 2.0,
        stopped_speed_m_s: 0.5,
    };

    fn at(x: f32, y: f32, speed: f32) -> Vehicle {
        Vehicle {
            pos: [x, y],
            vel: [speed, 0.0],
        }
    }

    #[test]
    fn counts_stopped_vehicles_on_their_own_approach() {
        let vehicles = [at(0.5, 10.0, 0.0), at(0.5, 20.0, 0.0), at(0.0, -5.0, 0.0)];
        assert_eq!(queued_per_approach(&NORTH_SOUTH, &vehicles, ZONE), [2, 1]);
    }

    #[test]
    fn moving_vehicles_are_not_queued() {
        let vehicles = [at(0.5, 10.0, 8.0)];
        assert_eq!(queued_per_approach(&NORTH_SOUTH, &vehicles, ZONE), [0, 0]);
    }

    #[test]
    fn vehicles_beyond_the_queue_or_outside_the_lane_are_not_counted() {
        let vehicles = [
            at(0.0, 31.0, 0.0), // past the queue length
            at(5.0, 10.0, 0.0), // outside the lane
            at(0.0, 0.0, 0.0),  // at the centre
        ];
        assert_eq!(queued_per_approach(&NORTH_SOUTH, &vehicles, ZONE), [0, 0]);
    }
}
