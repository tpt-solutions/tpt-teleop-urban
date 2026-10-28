//! Interacting Multiple Model (IMM) Kalman filter over two motion models, and
//! the probabilistic trajectory cone built from it.
//!
//! State per agent is `[px, py, vx, vy]` in metres and metres per second.
//! Models:
//!
//! - **cruise**: constant velocity with low process noise.
//! - **maneuver**: constant velocity with high process noise, so the filter can
//!   follow turns and sudden braking.
//!
//! The IMM mixes the two filters each step in proportion to how well each
//! explains the measurements. The mixture is what the trajectory cone uses.
//!
//! This is the scalar reference implementation. The SIMD kernel for 100+ agents
//! (spec §4.3) is a nightly-only feature, and its output must match this code.

use crate::linalg::{Mat, add, mul, mul_vec, outer, scale, sub, transpose};

/// Position and velocity state, `[px, py, vx, vy]`.
pub type State = [f32; 4];
/// 4×4 covariance.
pub type Cov = Mat<4, 4>;

/// Process-noise acceleration standard deviation for cruise, in m/s².
pub const CRUISE_SIGMA_A: f32 = 0.3;
/// Process-noise acceleration standard deviation for maneuver, in m/s².
pub const MANEUVER_SIGMA_A: f32 = 2.5;
/// Measurement variance per position axis, in m².
pub const MEAS_VAR: f32 = 0.04;
/// Probability of switching models between steps.
pub const SWITCH_PROB: f32 = 0.05;

/// Mixing probabilities over `[cruise, maneuver]`.
pub type Mu = [f32; 2];

/// Gaussian state and covariance for one model.
#[derive(Clone, Copy, Debug)]
struct Filter {
    x: State,
    p: Cov,
}

/// Two-model IMM filter for one agent.
#[derive(Clone, Copy, Debug)]
pub struct Imm {
    filters: [Filter; 2],
    mu: Mu,
}

/// Mixed estimate across models.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimate {
    /// Mixed state, `[px, py, vx, vy]`.
    pub x: State,
    /// Mixed covariance.
    pub p: Cov,
    /// Model probabilities: cruise, then maneuver.
    pub mu: Mu,
}

/// One point on the probabilistic trajectory cone.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConePoint {
    /// Seconds ahead of the current estimate.
    pub t_s: f32,
    /// Position x, in metres.
    pub x: f32,
    /// Position y, in metres.
    pub y: f32,
    /// Radius that contains the position with `k_sigma` standard deviations
    /// along the worst axis.
    pub radius: f32,
}

impl Imm {
    /// Starts both models at `x0`, with position variance `pos_var` and velocity
    /// variance `vel_var`. The initial mix favours cruise.
    #[must_use]
    pub fn new(x0: State, pos_var: f32, vel_var: f32) -> Self {
        let p = [
            [pos_var, 0.0, 0.0, 0.0],
            [0.0, pos_var, 0.0, 0.0],
            [0.0, 0.0, vel_var, 0.0],
            [0.0, 0.0, 0.0, vel_var],
        ];
        let filter = Filter { x: x0, p };
        Self {
            filters: [filter, filter],
            mu: [0.8, 0.2],
        }
    }

    #[must_use]
    /// Model probabilities: cruise, then maneuver.
    pub const fn mu(&self) -> Mu {
        self.mu
    }

    /// Advances the filter by `dt` seconds using position measurement `z`.
    pub fn step(&mut self, z: [f32; 2], dt: f32) {
        let (mixed, c_bar) = self.mix();
        let mut likelihood = [0.0f32; 2];
        for j in 0..2 {
            let mut f = mixed[j];
            let (fm, qm) = model_matrices(j, dt);
            f = predict(&f, &fm, &qm);
            let (innovation, s) = update(&mut f, z);
            likelihood[j] = gaussian_likelihood(&innovation, &s);
            self.filters[j] = f;
        }
        let weights: [f32; 2] = core::array::from_fn(|j| likelihood[j] * c_bar[j]);
        let total: f32 = weights.iter().sum();
        if total > 0.0 && total.is_finite() {
            self.mu = weights.map(|w| w / total);
        }
    }

    /// Combined estimate across both models.
    #[must_use]
    pub fn estimate(&self) -> Estimate {
        let x = mix_state(&self.filters, &self.mu);
        let mut p = [[0.0; 4]; 4];
        for (j, f) in self.filters.iter().enumerate() {
            let d = sub_state(&f.x, &x);
            p = add(&p, &scale(&add(&f.p, &outer(&d, &d)), self.mu[j]));
        }
        Estimate { x, p, mu: self.mu }
    }

    /// Mixing step: each model restarts from a blend of both models' states,
    /// weighted by how likely each was to have been the true model.
    /// Returns the mixed filters and the predicted model probabilities.
    fn mix(&self) -> ([Filter; 2], [f32; 2]) {
        let pi = transition();
        let mut c_bar = [0.0f32; 2];
        for (j, c) in c_bar.iter_mut().enumerate() {
            *c = (0..2).map(|i| pi[i][j] * self.mu[i]).sum();
        }
        let mut mixed = self.filters;
        for j in 0..2 {
            // Mixing weights mu_{i|j} = pi_{ij} mu_i / c_bar_j.
            let w: [f32; 2] = core::array::from_fn(|i| {
                if c_bar[j] > 0.0 {
                    pi[i][j] * self.mu[i] / c_bar[j]
                } else {
                    0.5
                }
            });
            let x0 = mix_state(&self.filters, &w);
            let mut p0 = [[0.0; 4]; 4];
            for (i, f) in self.filters.iter().enumerate() {
                let d = sub_state(&f.x, &x0);
                p0 = add(&p0, &scale(&add(&f.p, &outer(&d, &d)), w[i]));
            }
            mixed[j] = Filter { x: x0, p: p0 };
        }
        (mixed, c_bar)
    }

    /// Propagates the mixture `horizon` steps ahead without measurements and
    /// fills `out` with one cone point per step. Returns how many were written,
    /// which is the smaller of `out.len()` and `horizon`. Writes no heap memory.
    pub fn cone(&self, horizon: usize, dt: f32, k_sigma: f32, out: &mut [ConePoint]) -> usize {
        let pi = transition();
        let mut filters = self.filters;
        let mut mu = self.mu;
        let steps = horizon.min(out.len());
        for (k, slot) in out.iter_mut().take(steps).enumerate() {
            let mut next_mu = [0.0f32; 2];
            for (j, nm) in next_mu.iter_mut().enumerate() {
                *nm = (0..2).map(|i| pi[i][j] * mu[i]).sum();
            }
            for (j, f) in filters.iter_mut().enumerate() {
                let (fm, qm) = model_matrices(j, dt);
                *f = predict(f, &fm, &qm);
            }
            mu = next_mu;
            let x = mix_state(&filters, &mu);
            let mut p = [[0.0; 4]; 4];
            for (j, f) in filters.iter().enumerate() {
                let d = sub_state(&f.x, &x);
                p = add(&p, &scale(&add(&f.p, &outer(&d, &d)), mu[j]));
            }
            *slot = ConePoint {
                t_s: (k + 1) as f32 * dt,
                x: x[0],
                y: x[1],
                radius: k_sigma * max_axis_std(&p),
            };
        }
        steps
    }
}

/// Model transition matrix: stay with probability `1 - SWITCH_PROB`.
fn transition() -> Mat<2, 2> {
    [
        [1.0 - SWITCH_PROB, SWITCH_PROB],
        [SWITCH_PROB, 1.0 - SWITCH_PROB],
    ]
}

/// State transition `F(dt)` and process noise `Q(dt)` for model `j`.
fn model_matrices(j: usize, dt: f32) -> (Cov, Cov) {
    let sigma = if j == 0 {
        CRUISE_SIGMA_A
    } else {
        MANEUVER_SIGMA_A
    };
    let s2 = sigma * sigma;
    let f = [
        [1.0, 0.0, dt, 0.0],
        [0.0, 1.0, 0.0, dt],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    // Discrete white-noise acceleration model, per axis.
    let (q_pp, q_pv, q_vv) = (dt.powi(4) / 4.0 * s2, dt.powi(3) / 2.0 * s2, dt * dt * s2);
    let q = [
        [q_pp, 0.0, q_pv, 0.0],
        [0.0, q_pp, 0.0, q_pv],
        [q_pv, 0.0, q_vv, 0.0],
        [0.0, q_pv, 0.0, q_vv],
    ];
    (f, q)
}

/// Kalman predict: `x = F x`, `P = F P Fᵀ + Q`.
fn predict(f: &Filter, fm: &Cov, qm: &Cov) -> Filter {
    let x = mul_vec(fm, &f.x);
    let p = add(&mul(&mul(fm, &f.p), &transpose(fm)), qm);
    Filter { x, p }
}

/// Kalman update with a position measurement. Returns the innovation and its
/// covariance, which the likelihood needs.
fn update(f: &mut Filter, z: [f32; 2]) -> ([f32; 2], [[f32; 2]; 2]) {
    // H selects the position rows, so H P = rows 0 and 1 of P and P Hᵀ = columns 0 and 1.
    let hp: Mat<2, 4> = [f.p[0], f.p[1]];
    let pht: Mat<4, 2> = core::array::from_fn(|i| [f.p[i][0], f.p[i][1]]);
    let s: [[f32; 2]; 2] = [
        [f.p[0][0] + MEAS_VAR, f.p[0][1]],
        [f.p[1][0], f.p[1][1] + MEAS_VAR],
    ];
    let det = s[0][0] * s[1][1] - s[0][1] * s[1][0];
    let inv = [
        [s[1][1] / det, -s[0][1] / det],
        [-s[1][0] / det, s[0][0] / det],
    ];
    let k = mul(&pht, &inv);
    let innovation = [z[0] - f.x[0], z[1] - f.x[1]];
    let correction = mul_vec(&k, &innovation);
    for (xi, ci) in f.x.iter_mut().zip(correction) {
        *xi += ci;
    }
    f.p = sub(&f.p, &mul(&k, &hp));
    (innovation, s)
}

/// Gaussian density of an innovation with covariance `s`, evaluated at `y`.
fn gaussian_likelihood(y: &[f32; 2], s: &[[f32; 2]; 2]) -> f32 {
    let det = s[0][0] * s[1][1] - s[0][1] * s[1][0];
    if det <= 0.0 {
        return 0.0;
    }
    // yᵀ S⁻¹ y with the 2×2 inverse written out.
    let quad =
        (s[1][1] * y[0] * y[0] - (s[0][1] + s[1][0]) * y[0] * y[1] + s[0][0] * y[1] * y[1]) / det;
    (-0.5 * quad).exp() / (2.0 * core::f32::consts::PI * det.sqrt())
}

fn mix_state(filters: &[Filter; 2], w: &[f32; 2]) -> State {
    let mut x = [0.0; 4];
    for (f, &wi) in filters.iter().zip(w) {
        for (xi, fi) in x.iter_mut().zip(f.x) {
            *xi += wi * fi;
        }
    }
    x
}

fn sub_state(a: &State, b: &State) -> State {
    core::array::from_fn(|i| a[i] - b[i])
}

/// Larger standard deviation along x or y, from the position block of `p`.
/// Uses the closed-form eigenvalue of the symmetric 2×2 block.
fn max_axis_std(p: &Cov) -> f32 {
    let (a, b, d) = (p[0][0], p[0][1], p[1][1]);
    let half_trace = 0.5 * (a + d);
    let disc = (0.5 * (a - d)).powi(2) + b * b;
    (half_trace + disc.sqrt()).max(0.0).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_track(t: usize, dt: f32) -> [f32; 2] {
        // 10 m/s along +x, starting at the origin.
        [10.0 * t as f32 * dt, 0.0]
    }

    #[test]
    fn tracks_a_straight_line_and_favours_cruise() {
        let dt = 0.1;
        let mut imm = Imm::new([0.0, 0.0, 10.0, 0.0], 1.0, 4.0);
        for t in 1..=60 {
            imm.step(straight_track(t, dt), dt);
        }
        let est = imm.estimate();
        let truth = straight_track(60, dt);
        assert!((est.x[0] - truth[0]).abs() < 0.2, "x {}", est.x[0]);
        assert!((est.x[2] - 10.0).abs() < 0.5, "vx {}", est.x[2]);
        assert!(imm.mu()[0] > imm.mu()[1], "mu {:?}", imm.mu());
    }

    #[test]
    fn mixing_probabilities_stay_normalized() {
        let dt = 0.1;
        let mut imm = Imm::new([0.0, 0.0, 5.0, 0.0], 1.0, 1.0);
        for t in 1..=40 {
            // An abrupt lateral jump should not produce NaN or lose normalization.
            let z = [5.0 * t as f32 * dt, if t > 20 { 6.0 } else { 0.0 }];
            imm.step(z, dt);
            let sum: f32 = imm.mu().iter().sum();
            assert!((sum - 1.0).abs() < 1e-4, "step {t}: sum {sum}");
            assert!(imm.mu().iter().all(|m| m.is_finite()));
        }
    }

    #[test]
    fn maneuver_model_gains_weight_after_a_turn() {
        let dt = 0.1;
        let mut imm = Imm::new([0.0, 0.0, 10.0, 0.0], 0.5, 1.0);
        // Straight for 2 s, then a sharp 90-degree turn toward +y.
        let mut x = 0.0;
        let mut y = 0.0;
        for t in 0..30 {
            x += 10.0 * dt;
            if t >= 20 {
                y += 10.0 * dt;
            }
            imm.step([x, y], dt);
        }
        assert!(imm.mu()[1] > 0.5, "maneuver mu {:?}", imm.mu());
    }

    #[test]
    fn cone_grows_with_horizon_and_covers_the_mean_path() {
        let dt = 0.1;
        let mut imm = Imm::new([0.0, 0.0, 10.0, 0.0], 0.1, 0.5);
        for t in 1..=20 {
            imm.step(straight_track(t, dt), dt);
        }
        let mut out = [ConePoint {
            t_s: 0.0,
            x: 0.0,
            y: 0.0,
            radius: 0.0,
        }; 30];
        let n = imm.cone(30, dt, 2.0, &mut out);
        assert_eq!(n, 30);
        for pair in out[..n].windows(2) {
            assert!(pair[1].radius >= pair[0].radius - 1e-4);
        }
        // Mean path moves forward about 10 m/s × t.
        assert!((out[29].x - (imm_last_x(&imm) + 10.0 * 3.0)).abs() < 1.5);
        assert!((out[29].t_s - 3.0).abs() < 1e-4);
    }

    #[test]
    fn cone_respects_output_capacity() {
        let imm = Imm::new([0.0; 4], 1.0, 1.0);
        let mut out = [ConePoint {
            t_s: 0.0,
            x: 0.0,
            y: 0.0,
            radius: 0.0,
        }; 5];
        assert_eq!(imm.cone(30, 0.1, 2.0, &mut out), 5);
    }

    fn imm_last_x(imm: &Imm) -> f32 {
        imm.estimate().x[0]
    }
}
