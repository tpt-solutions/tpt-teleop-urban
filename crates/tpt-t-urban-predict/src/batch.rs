//! Batched constant-velocity propagation over many agents, in structure-of-arrays
//! layout.
//!
//! This is the scalar reference for the batched kernel. Each agent's position
//! advances by `v * dt`. With the `simd` feature (nightly, `portable_simd`), the
//! same arithmetic runs eight agents per instruction in `propagate_cv_simd`.
//! The two paths must agree exactly, and the tests check that.

/// Agents in structure-of-arrays form. All four slices must be the same length.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AgentsSoa {
    pub px: Vec<f32>,
    pub py: Vec<f32>,
    pub vx: Vec<f32>,
    pub vy: Vec<f32>,
}

impl AgentsSoa {
    /// Number of agents.
    #[must_use]
    pub fn len(&self) -> usize {
        self.px.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.px.is_empty()
    }

    /// Adds one agent.
    pub fn push(&mut self, px: f32, py: f32, vx: f32, vy: f32) {
        self.px.push(px);
        self.py.push(py);
        self.vx.push(vx);
        self.vy.push(vy);
    }
}

/// Scalar reference: advances every agent by `dt` with constant velocity.
pub fn propagate_cv_scalar(agents: &mut AgentsSoa, dt: f32) {
    for i in 0..agents.len() {
        agents.px[i] += agents.vx[i] * dt;
        agents.py[i] += agents.vy[i] * dt;
    }
}

/// Batched kernel, eight agents per step. Same results as
/// [`propagate_cv_scalar`], bit for bit, because each lane performs the same
/// multiply then add.
#[cfg(feature = "simd")]
pub fn propagate_cv_simd(agents: &mut AgentsSoa, dt: f32) {
    use std::simd::f32x8;

    let n = agents.len();
    let lanes = n / 8 * 8;
    let dt_v = f32x8::splat(dt);
    let mut i = 0;
    while i < lanes {
        let vx = f32x8::from_slice(&agents.vx[i..i + 8]);
        let vy = f32x8::from_slice(&agents.vy[i..i + 8]);
        let px = f32x8::from_slice(&agents.px[i..i + 8]);
        let py = f32x8::from_slice(&agents.py[i..i + 8]);
        (px + vx * dt_v).copy_to_slice(&mut agents.px[i..i + 8]);
        (py + vy * dt_v).copy_to_slice(&mut agents.py[i..i + 8]);
        i += 8;
    }
    // Tail agents that do not fill a full vector.
    propagate_cv_scalar_from(agents, lanes, dt);
}

#[cfg(feature = "simd")]
fn propagate_cv_scalar_from(agents: &mut AgentsSoa, start: usize, dt: f32) {
    for i in start..agents.len() {
        agents.px[i] += agents.vx[i] * dt;
        agents.py[i] += agents.vy[i] * dt;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fleet(n: usize) -> AgentsSoa {
        let mut a = AgentsSoa::default();
        for i in 0..n {
            let f = i as f32;
            a.push(f * 1.5, -f, 0.3 * f - 2.0, 0.7 - 0.01 * f);
        }
        a
    }

    #[test]
    fn scalar_propagation_moves_each_agent_by_velocity_times_dt() {
        let mut a = fleet(5);
        let before = a.clone();
        propagate_cv_scalar(&mut a, 0.5);
        for i in 0..a.len() {
            assert_eq!(a.px[i], before.px[i] + before.vx[i] * 0.5);
            assert_eq!(a.py[i], before.py[i] + before.vy[i] * 0.5);
        }
    }

    #[cfg(feature = "simd")]
    #[test]
    fn simd_matches_scalar_exactly_including_the_tail() {
        for n in [0, 1, 7, 8, 9, 100, 103] {
            let mut scalar = fleet(n);
            let mut simd = fleet(n);
            for _ in 0..10 {
                propagate_cv_scalar(&mut scalar, 0.1);
                propagate_cv_simd(&mut simd, 0.1);
            }
            assert_eq!(scalar, simd, "n = {n}");
        }
    }
}
