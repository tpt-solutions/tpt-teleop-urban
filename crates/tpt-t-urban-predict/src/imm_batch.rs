//! Batched IMM filter over many agents, in structure-of-arrays layout.
//!
//! The batch runs the same two-model IMM as [`crate::Imm`], for every agent at
//! once. Each agent's state is a column in a structure-of-arrays table, so one
//! vector holds the same quantity for eight agents.
//!
//! One generic kernel, [`step_at`], does the arithmetic. The stable build
//! instantiates it with one lane, a plain `f32`. With the `simd` feature (nightly,
//! `portable_simd`) the same kernel runs with eight lanes, one `f32x8`. The
//! operations are the same and in the same order, so the two agree bit for bit.
//! Agents past the last full group of eight are run in one lane.
//!
//! Only the per-lane transcendental (the likelihood `exp`) is applied lane by
//! lane, since `portable_simd` has no portable `exp`.

use core::ops::{Add, Div, Mul, Sub};

use crate::imm::{CRUISE_SIGMA_A, MANEUVER_SIGMA_A, MEAS_VAR, SWITCH_PROB};

/// Lane arithmetic the kernel needs. `f32` is the one-lane case. `f32x8` is the
/// eight-lane case under the `simd` feature.
pub trait Lane:
    Copy + Add<Output = Self> + Sub<Output = Self> + Mul<Output = Self> + Div<Output = Self>
{
    /// Number of agents processed per step.
    const WIDTH: usize;
    /// A lane value with `v` in every lane.
    fn splat(v: f32) -> Self;
    /// Builds a lane value from one `f32` per lane.
    fn from_fn(f: impl FnMut(usize) -> f32) -> Self;
    /// The value in lane `i`.
    fn lane(self, i: usize) -> f32;
    /// Loads `WIDTH` consecutive values.
    fn load(s: &[f32]) -> Self;
    /// Stores `WIDTH` consecutive values.
    fn store(self, s: &mut [f32]);
}

impl Lane for f32 {
    const WIDTH: usize = 1;

    fn splat(v: f32) -> Self {
        v
    }

    fn from_fn(mut f: impl FnMut(usize) -> f32) -> Self {
        f(0)
    }

    fn lane(self, _i: usize) -> f32 {
        self
    }

    fn load(s: &[f32]) -> Self {
        s[0]
    }

    fn store(self, s: &mut [f32]) {
        s[0] = self;
    }
}

#[cfg(feature = "simd")]
impl Lane for std::simd::f32x8 {
    const WIDTH: usize = 8;

    fn splat(v: f32) -> Self {
        std::simd::f32x8::splat(v)
    }

    fn from_fn(f: impl FnMut(usize) -> f32) -> Self {
        std::simd::f32x8::from_array(core::array::from_fn(f))
    }

    fn lane(self, i: usize) -> f32 {
        self.to_array()[i]
    }

    fn load(s: &[f32]) -> Self {
        std::simd::f32x8::from_slice(&s[..8])
    }

    fn store(self, s: &mut [f32]) {
        self.copy_to_slice(&mut s[..8]);
    }
}

/// The lane type the batch uses on this build.
#[cfg(feature = "simd")]
type Wide = std::simd::f32x8;
#[cfg(not(feature = "simd"))]
type Wide = f32;

// Column layout: for each model j, four state columns, then sixteen covariance
// columns in row-major order. Then one mixing-probability column per model.
const MODEL_COLS: usize = 20;
const MU_COL: usize = 2 * MODEL_COLS;
const COLS: usize = MU_COL + 2;

fn x_col(j: usize, k: usize) -> usize {
    j * MODEL_COLS + k
}

fn p_col(j: usize, r: usize, c: usize) -> usize {
    j * MODEL_COLS + 4 + r * 4 + c
}

fn mu_col(j: usize) -> usize {
    MU_COL + j
}

/// Model-switching probabilities, `PI[i][j]`, as in the scalar filter.
fn transition(i: usize, j: usize) -> f32 {
    if i == j {
        1.0 - SWITCH_PROB
    } else {
        SWITCH_PROB
    }
}

/// Discrete white-noise acceleration constants for model `j` at step `dt`.
fn noise(j: usize, dt: f32) -> (f32, f32, f32) {
    let sigma = if j == 0 {
        CRUISE_SIGMA_A
    } else {
        MANEUVER_SIGMA_A
    };
    let s2 = sigma * sigma;
    (dt.powi(4) / 4.0 * s2, dt.powi(3) / 2.0 * s2, dt * dt * s2)
}

/// IMM filters for many agents, one column per quantity.
#[derive(Clone, Debug)]
pub struct ImmBatch {
    n: usize,
    cols: Vec<Vec<f32>>,
}

impl Default for ImmBatch {
    fn default() -> Self {
        Self::new()
    }
}

impl ImmBatch {
    /// An empty batch.
    #[must_use]
    pub fn new() -> Self {
        Self {
            n: 0,
            cols: vec![Vec::new(); COLS],
        }
    }

    /// Number of agents.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.n
    }

    /// True when there are no agents.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.n == 0
    }

    /// Adds one agent starting at `x0`, with the same initial covariance and
    /// mixing as [`crate::Imm::new`].
    pub fn push(&mut self, x0: [f32; 4], pos_var: f32, vel_var: f32) {
        let p = [
            [pos_var, 0.0, 0.0, 0.0],
            [0.0, pos_var, 0.0, 0.0],
            [0.0, 0.0, vel_var, 0.0],
            [0.0, 0.0, 0.0, vel_var],
        ];
        for j in 0..2 {
            for (k, &v) in x0.iter().enumerate() {
                self.cols[x_col(j, k)].push(v);
            }
            for (r, row) in p.iter().enumerate() {
                for (c, &v) in row.iter().enumerate() {
                    self.cols[p_col(j, r, c)].push(v);
                }
            }
        }
        self.cols[mu_col(0)].push(0.8);
        self.cols[mu_col(1)].push(0.2);
        self.n += 1;
    }

    /// Advances every agent by `dt` seconds with position measurements
    /// `(zx[i], zy[i])`. Uses the widest lane the build supports.
    pub fn step(&mut self, zx: &[f32], zy: &[f32], dt: f32) {
        self.step_with::<Wide>(zx, zy, dt);
    }

    /// The same step, run in one lane per agent. The reference for the wide
    /// path.
    pub fn step_scalar(&mut self, zx: &[f32], zy: &[f32], dt: f32) {
        self.step_with::<f32>(zx, zy, dt);
    }

    fn step_with<L: Lane>(&mut self, zx: &[f32], zy: &[f32], dt: f32) {
        assert_eq!(zx.len(), self.n, "one measurement per agent");
        assert_eq!(zy.len(), self.n, "one measurement per agent");
        let full = self.n / L::WIDTH * L::WIDTH;
        let mut at = 0;
        while at < full {
            let z = [L::load(&zx[at..]), L::load(&zy[at..])];
            step_at::<L>(&mut self.cols, at, z, dt);
            at += L::WIDTH;
        }
        // Tail agents that do not fill a full lane group run one at a time.
        for i in full..self.n {
            let z = [f32::load(&zx[i..]), f32::load(&zy[i..])];
            step_at::<f32>(&mut self.cols, i, z, dt);
        }
    }

    /// Mixed position estimate of agent `i`, `[px, py]`.
    #[must_use]
    pub fn position(&self, i: usize) -> [f32; 2] {
        let mu = [self.cols[mu_col(0)][i], self.cols[mu_col(1)][i]];
        let at = |k: usize| {
            let mut v = 0.0;
            for (j, &w) in mu.iter().enumerate() {
                v += w * self.cols[x_col(j, k)][i];
            }
            v
        };
        [at(0), at(1)]
    }

    /// Model probabilities of agent `i`: cruise, then maneuver.
    #[must_use]
    pub fn mu(&self, i: usize) -> [f32; 2] {
        [self.cols[mu_col(0)][i], self.cols[mu_col(1)][i]]
    }
}

/// One IMM step for `L::WIDTH` agents starting at column index `at`.
///
/// Mirrors [`crate::Imm::step`] operation for operation. Each group of
/// expressions is commented with the scalar step it mirrors.
// The index loops mirror the scalar filter, which indexes the same arrays in the
// same order. Iterators would change nothing but the look.
#[allow(clippy::needless_range_loop)]
fn step_at<L: Lane>(cols: &mut [Vec<f32>], at: usize, z: [L; 2], dt: f32) {
    let w = L::WIDTH;
    let load = |c: usize| L::load(&cols[c][at..at + w]);

    let mut x = [[L::splat(0.0); 4]; 2];
    let mut p = [[[L::splat(0.0); 4]; 4]; 2];
    let mut mu = [L::splat(0.0); 2];
    for j in 0..2 {
        for k in 0..4 {
            x[j][k] = load(x_col(j, k));
        }
        for r in 0..4 {
            for c in 0..4 {
                p[j][r][c] = load(p_col(j, r, c));
            }
        }
        mu[j] = load(mu_col(j));
    }

    // Mixing (`Imm::mix`): predicted probabilities, then the mixed starts.
    let c_bar: [L; 2] = core::array::from_fn(|j| {
        let mut s = L::splat(0.0);
        for i in 0..2 {
            s = s + L::splat(transition(i, j)) * mu[i];
        }
        s
    });
    let mut x0 = [[L::splat(0.0); 4]; 2];
    let mut p0 = [[[L::splat(0.0); 4]; 4]; 2];
    for j in 0..2 {
        // Mixing weights mu_{i|j}, with 0.5 where the predicted probability is zero.
        let weight: [L; 2] = core::array::from_fn(|i| {
            let num = L::splat(transition(i, j)) * mu[i];
            L::from_fn(|l| {
                let c = c_bar[j].lane(l);
                if c > 0.0 { num.lane(l) / c } else { 0.5 }
            })
        });
        for k in 0..4 {
            let mut s = L::splat(0.0);
            for i in 0..2 {
                s = s + weight[i] * x[i][k];
            }
            x0[j][k] = s;
        }
        for r in 0..4 {
            for c in 0..4 {
                let mut s = L::splat(0.0);
                for i in 0..2 {
                    let d_r = x[i][r] - x0[j][r];
                    let d_c = x[i][c] - x0[j][c];
                    s = s + (p[i][r][c] + d_r * d_c) * weight[i];
                }
                p0[j][r][c] = s;
            }
        }
    }

    // Predict and update each model (`Imm::step`'s loop over `j`).
    let mut likelihood = [L::splat(0.0); 2];
    for j in 0..2 {
        let (q_pp, q_pv, q_vv) = noise(j, dt);
        let (q_pp, q_pv, q_vv) = (L::splat(q_pp), L::splat(q_pv), L::splat(q_vv));
        let dt_v = L::splat(dt);

        // Predict: x = F x0, P = F P0 Fᵀ + Q, with F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]].
        let mut f_x = x0[j];
        f_x[0] = x0[j][0] + dt_v * x0[j][2];
        f_x[1] = x0[j][1] + dt_v * x0[j][3];
        let mut fp = [[L::splat(0.0); 4]; 4];
        for c in 0..4 {
            fp[0][c] = p0[j][0][c] + dt_v * p0[j][2][c];
            fp[1][c] = p0[j][1][c] + dt_v * p0[j][3][c];
            fp[2][c] = p0[j][2][c];
            fp[3][c] = p0[j][3][c];
        }
        let mut f_p = [[L::splat(0.0); 4]; 4];
        for r in 0..4 {
            f_p[r][0] = fp[r][0] + dt_v * fp[r][2];
            f_p[r][1] = fp[r][1] + dt_v * fp[r][3];
            f_p[r][2] = fp[r][2];
            f_p[r][3] = fp[r][3];
        }
        f_p[0][0] = f_p[0][0] + q_pp;
        f_p[1][1] = f_p[1][1] + q_pp;
        f_p[0][2] = f_p[0][2] + q_pv;
        f_p[2][0] = f_p[2][0] + q_pv;
        f_p[1][3] = f_p[1][3] + q_pv;
        f_p[3][1] = f_p[3][1] + q_pv;
        f_p[2][2] = f_p[2][2] + q_vv;
        f_p[3][3] = f_p[3][3] + q_vv;

        // Update with the position measurement (`imm::update`).
        let r_v = L::splat(MEAS_VAR);
        let s00 = f_p[0][0] + r_v;
        let s01 = f_p[0][1];
        let s10 = f_p[1][0];
        let s11 = f_p[1][1] + r_v;
        let det = s00 * s11 - s01 * s10;
        let inv00 = s11 / det;
        let inv01 = (L::splat(0.0) - s01) / det;
        let inv10 = (L::splat(0.0) - s10) / det;
        let inv11 = s00 / det;
        // K = P Hᵀ S⁻¹, with P Hᵀ = the first two columns of P.
        let mut kg = [[L::splat(0.0); 2]; 4];
        for i in 0..4 {
            kg[i][0] = f_p[i][0] * inv00 + f_p[i][1] * inv10;
            kg[i][1] = f_p[i][0] * inv01 + f_p[i][1] * inv11;
        }
        let y0 = z[0] - f_x[0];
        let y1 = z[1] - f_x[1];
        let mut x_new = f_x;
        for i in 0..4 {
            x_new[i] = f_x[i] + (kg[i][0] * y0 + kg[i][1] * y1);
        }
        // P = P - K H P, with H P = the first two rows of P.
        let mut p_new = f_p;
        for i in 0..4 {
            for c in 0..4 {
                p_new[i][c] = f_p[i][c] - (kg[i][0] * f_p[0][c] + kg[i][1] * f_p[1][c]);
            }
        }

        // Gaussian likelihood of the innovation (`imm::gaussian_likelihood`).
        likelihood[j] = L::from_fn(|l| {
            let d = det.lane(l);
            if d <= 0.0 {
                return 0.0;
            }
            let (a, b) = (y0.lane(l), y1.lane(l));
            let quad = (s11.lane(l) * a * a - (s01.lane(l) + s10.lane(l)) * a * b
                + s00.lane(l) * b * b)
                / d;
            (-0.5 * quad).exp() / (2.0 * core::f32::consts::PI * d.sqrt())
        });

        for i in 0..4 {
            x[j][i] = x_new[i];
            for c in 0..4 {
                p[j][i][c] = p_new[i][c];
            }
        }
    }

    // Model probabilities (`Imm::step`): weight by likelihood, normalize if the
    // total is positive and finite, otherwise keep the previous probabilities.
    let weights: [L; 2] = core::array::from_fn(|j| likelihood[j] * c_bar[j]);
    let total = weights[0] + weights[1];
    for j in 0..2 {
        mu[j] = L::from_fn(|l| {
            let t = total.lane(l);
            if t > 0.0 && t.is_finite() {
                weights[j].lane(l) / t
            } else {
                mu[j].lane(l)
            }
        });
    }

    for j in 0..2 {
        for k in 0..4 {
            x[j][k].store(&mut cols[x_col(j, k)][at..at + w]);
        }
        for r in 0..4 {
            for c in 0..4 {
                p[j][r][c].store(&mut cols[p_col(j, r, c)][at..at + w]);
            }
        }
        mu[j].store(&mut cols[mu_col(j)][at..at + w]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Imm;

    /// Scripted measurements for an agent: a turn at step 40, then straight.
    fn measurement(agent: usize, t: usize, dt: f32) -> [f32; 2] {
        let speed = 5.0 + 0.1 * agent as f32;
        let turn = if t > 40 { 0.4 } else { 0.0 };
        let s = t as f32 * dt;
        let x = speed * s + agent as f32;
        let y = (turn * (s - 40.0 * dt)).max(0.0) * speed * 0.5 + 0.02 * (t as f32).sin();
        [x, y]
    }

    fn fleet(n: usize) -> ImmBatch {
        let mut batch = ImmBatch::new();
        for i in 0..n {
            let f = i as f32;
            batch.push([f, 0.0, 5.0 + 0.1 * f, 0.0], 1.0, 1.0);
        }
        batch
    }

    #[test]
    fn the_batch_tracks_each_agent_like_the_scalar_filter() {
        let n = 19;
        let dt = 0.1;
        let mut batch = fleet(n);
        let mut scalars: Vec<Imm> = (0..n)
            .map(|i| {
                let f = i as f32;
                Imm::new([f, 0.0, 5.0 + 0.1 * f, 0.0], 1.0, 1.0)
            })
            .collect();
        for t in 0..80 {
            let zx: Vec<f32> = (0..n).map(|i| measurement(i, t, dt)[0]).collect();
            let zy: Vec<f32> = (0..n).map(|i| measurement(i, t, dt)[1]).collect();
            batch.step(&zx, &zy, dt);
            for (i, imm) in scalars.iter_mut().enumerate() {
                imm.step([zx[i], zy[i]], dt);
            }
        }
        for (i, imm) in scalars.iter().enumerate() {
            let want = imm.estimate().x;
            let got = batch.position(i);
            assert!(
                (got[0] - want[0]).abs() < 1e-3 && (got[1] - want[1]).abs() < 1e-3,
                "agent {i}: batch {got:?} vs scalar {want:?}"
            );
            let mu = batch.mu(i);
            assert!((mu[0] - imm.mu()[0]).abs() < 1e-4, "agent {i} mu {mu:?}");
        }
    }

    #[test]
    fn the_wide_path_matches_the_one_lane_path_bit_for_bit() {
        // With the stable build both paths are the same code, so this checks the
        // tail handling and the lane count. Under `simd` it checks f32x8 against f32.
        let n = 27;
        let dt = 0.05;
        let mut wide = fleet(n);
        let mut narrow = fleet(n);
        for t in 0..60 {
            let zx: Vec<f32> = (0..n).map(|i| measurement(i, t, dt)[0]).collect();
            let zy: Vec<f32> = (0..n).map(|i| measurement(i, t, dt)[1]).collect();
            wide.step(&zx, &zy, dt);
            narrow.step_scalar(&zx, &zy, dt);
        }
        for i in 0..n {
            assert_eq!(wide.position(i), narrow.position(i), "agent {i}");
            assert_eq!(wide.mu(i), narrow.mu(i), "agent {i}");
        }
    }

    #[test]
    fn probabilities_stay_normalized_per_agent() {
        let n = 9;
        let mut batch = fleet(n);
        for t in 0..30 {
            let zx: Vec<f32> = (0..n).map(|i| measurement(i, t, 0.1)[0]).collect();
            let zy: Vec<f32> = (0..n).map(|i| measurement(i, t, 0.1)[1]).collect();
            batch.step(&zx, &zy, 0.1);
        }
        for i in 0..n {
            let mu = batch.mu(i);
            assert!((mu[0] + mu[1] - 1.0).abs() < 1e-4, "agent {i}: {mu:?}");
        }
    }
}
