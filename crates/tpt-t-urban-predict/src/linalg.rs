//! Small fixed-size matrix helpers for the 4-state Kalman filters.
//!
//! Matrices are plain arrays so they live on the stack and never allocate.

/// Row-major matrix with `R` rows and `C` columns.
pub type Mat<const R: usize, const C: usize> = [[f32; C]; R];

/// `a * b`.
#[must_use]
pub fn mul<const A: usize, const B: usize, const C: usize>(
    a: &Mat<A, B>,
    b: &Mat<B, C>,
) -> Mat<A, C> {
    let mut out = [[0.0; C]; A];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (0..B).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

/// `aᵀ`.
#[must_use]
pub fn transpose<const R: usize, const C: usize>(a: &Mat<R, C>) -> Mat<C, R> {
    let mut out = [[0.0; R]; C];
    for (i, row) in a.iter().enumerate() {
        for (j, &v) in row.iter().enumerate() {
            out[j][i] = v;
        }
    }
    out
}

/// Element-wise `a + b`.
#[must_use]
pub fn add<const R: usize, const C: usize>(a: &Mat<R, C>, b: &Mat<R, C>) -> Mat<R, C> {
    let mut out = *a;
    for (row, brow) in out.iter_mut().zip(b.iter()) {
        for (cell, &bv) in row.iter_mut().zip(brow.iter()) {
            *cell += bv;
        }
    }
    out
}

/// Element-wise `a - b`.
#[must_use]
pub fn sub<const R: usize, const C: usize>(a: &Mat<R, C>, b: &Mat<R, C>) -> Mat<R, C> {
    let mut out = *a;
    for (row, brow) in out.iter_mut().zip(b.iter()) {
        for (cell, &bv) in row.iter_mut().zip(brow.iter()) {
            *cell -= bv;
        }
    }
    out
}

/// Scales every element by `s`.
#[must_use]
pub fn scale<const R: usize, const C: usize>(a: &Mat<R, C>, s: f32) -> Mat<R, C> {
    let mut out = *a;
    for row in &mut out {
        for cell in row.iter_mut() {
            *cell *= s;
        }
    }
    out
}

/// Outer product `v wᵀ`.
#[must_use]
pub fn outer<const N: usize>(v: &[f32; N], w: &[f32; N]) -> Mat<N, N> {
    let mut out = [[0.0; N]; N];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = v[i] * w[j];
        }
    }
    out
}

/// Matrix-vector product `a * v`.
#[must_use]
pub fn mul_vec<const R: usize, const C: usize>(a: &Mat<R, C>, v: &[f32; C]) -> [f32; R] {
    let mut out = [0.0; R];
    for (i, cell) in out.iter_mut().enumerate() {
        *cell = a[i].iter().zip(v.iter()).map(|(x, y)| x * y).sum();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn multiply_and_transpose() {
        let a: Mat<2, 3> = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
        let b: Mat<3, 1> = [[1.0], [0.0], [-1.0]];
        assert_eq!(mul(&a, &b), [[-2.0], [-2.0]]);
        assert_eq!(transpose(&a), [[1.0, 4.0], [2.0, 5.0], [3.0, 6.0]]);
    }

    #[test]
    fn outer_product_and_matrix_vector() {
        let m = outer(&[1.0, 2.0], &[3.0, 4.0]);
        assert_eq!(m, [[3.0, 4.0], [6.0, 8.0]]);
        assert_eq!(mul_vec(&m, &[1.0, 1.0]), [7.0, 14.0]);
    }
}
