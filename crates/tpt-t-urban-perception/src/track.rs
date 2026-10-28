//! Tracking: associate detections across frames and estimate their velocity.
//!
//! Each track predicts its position with its velocity, then takes the nearest
//! detection within `gate_m` of that prediction. Matching is greedy, nearest
//! first. The track's position and velocity are updated with an alpha-beta
//! filter, which needs no covariance and is stable at a fixed `dt`. A track with
//! no detection for more than `max_misses` frames is dropped.

/// One tracked object.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Track {
    /// Stable identifier, assigned when the track starts.
    pub id: u32,
    /// Estimated position, in metres.
    pub position: [f32; 2],
    /// Estimated velocity, in metres per second.
    pub velocity: [f32; 2],
    /// Frames in which the track matched a detection.
    pub hits: u32,
    /// Consecutive frames without a match.
    pub misses: u32,
}

/// The tracker's gating and filter settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    /// Largest distance from a prediction at which a detection still matches, in metres.
    pub gate_m: f32,
    /// Position gain of the alpha-beta filter, in `(0, 1]`.
    pub alpha: f32,
    /// Velocity gain of the alpha-beta filter, in `(0, 1]`.
    pub beta: f32,
    /// Consecutive missed frames after which a track is dropped.
    pub max_misses: u32,
}

/// Tracks objects across frames.
#[derive(Clone, Debug)]
pub struct Tracker {
    config: Config,
    tracks: Vec<Track>,
    next_id: u32,
}

impl Tracker {
    /// A tracker with no tracks.
    #[must_use]
    pub const fn new(config: Config) -> Self {
        Self {
            config,
            tracks: Vec::new(),
            next_id: 1,
        }
    }

    /// The live tracks.
    #[must_use]
    pub fn tracks(&self) -> &[Track] {
        &self.tracks
    }

    /// Advances one frame of `dt` seconds with the detections in it.
    pub fn update(&mut self, detections: &[[f32; 2]], dt: f32) {
        let cfg = self.config;
        let mut taken = vec![false; detections.len()];

        // Predict every track, then match greedily, nearest pair first.
        let predicted: Vec<[f32; 2]> = self
            .tracks
            .iter()
            .map(|t| {
                [
                    t.position[0] + t.velocity[0] * dt,
                    t.position[1] + t.velocity[1] * dt,
                ]
            })
            .collect();
        let mut matched = vec![None::<usize>; self.tracks.len()];
        let mut pairs: Vec<(f32, usize, usize)> = Vec::new();
        for (ti, p) in predicted.iter().enumerate() {
            for (di, z) in detections.iter().enumerate() {
                let d = ((p[0] - z[0]).powi(2) + (p[1] - z[1]).powi(2)).sqrt();
                if d <= cfg.gate_m {
                    pairs.push((d, ti, di));
                }
            }
        }
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        for (_, ti, di) in pairs {
            if matched[ti].is_none() && !taken[di] {
                matched[ti] = Some(di);
                taken[di] = true;
            }
        }

        for (ti, track) in self.tracks.iter_mut().enumerate() {
            let p = predicted[ti];
            match matched[ti] {
                Some(di) => {
                    let z = detections[di];
                    let r = [z[0] - p[0], z[1] - p[1]];
                    track.position = [p[0] + cfg.alpha * r[0], p[1] + cfg.alpha * r[1]];
                    track.velocity = [
                        track.velocity[0] + cfg.beta * r[0] / dt,
                        track.velocity[1] + cfg.beta * r[1] / dt,
                    ];
                    track.hits += 1;
                    track.misses = 0;
                }
                None => {
                    track.position = p;
                    track.misses += 1;
                }
            }
        }
        self.tracks.retain(|t| t.misses <= cfg.max_misses);

        for (di, z) in detections.iter().enumerate() {
            if !taken[di] {
                self.tracks.push(Track {
                    id: self.next_id,
                    position: *z,
                    velocity: [0.0, 0.0],
                    hits: 1,
                    misses: 0,
                });
                self.next_id += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> Config {
        Config {
            gate_m: 1.5,
            alpha: 0.5,
            beta: 0.3,
            max_misses: 2,
        }
    }

    #[test]
    fn a_moving_object_keeps_its_id_and_its_velocity_converges() {
        let mut tracker = Tracker::new(config());
        let dt = 0.1;
        for step in 0..40 {
            let x = 2.0 * step as f32 * dt;
            tracker.update(&[[x, 5.0]], dt);
        }
        let tracks = tracker.tracks();
        assert_eq!(tracks.len(), 1, "{tracks:?}");
        assert_eq!(tracks[0].id, 1);
        assert!((tracks[0].velocity[0] - 2.0).abs() < 0.1, "{tracks:?}");
        assert!(tracks[0].velocity[1].abs() < 0.05);
    }

    #[test]
    fn two_objects_get_two_tracks() {
        let mut tracker = Tracker::new(config());
        tracker.update(&[[0.0, 0.0], [20.0, 0.0]], 0.1);
        tracker.update(&[[0.0, 0.0], [20.0, 0.0]], 0.1);
        assert_eq!(tracker.tracks().len(), 2);
    }

    #[test]
    fn a_track_is_dropped_after_too_many_missed_frames() {
        let mut tracker = Tracker::new(config());
        tracker.update(&[[0.0, 0.0]], 0.1);
        for _ in 0..3 {
            tracker.update(&[], 0.1);
        }
        assert!(tracker.tracks().is_empty());
    }

    #[test]
    fn a_detection_outside_the_gate_starts_a_new_track() {
        let mut tracker = Tracker::new(config());
        tracker.update(&[[0.0, 0.0]], 0.1);
        tracker.update(&[[10.0, 0.0]], 0.1);
        let ids: Vec<u32> = tracker.tracks().iter().map(|t| t.id).collect();
        assert!(ids.contains(&2), "{ids:?}");
    }
}
