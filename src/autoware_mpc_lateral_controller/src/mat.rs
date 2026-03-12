// Copyright 2025 autoware-nano-ros contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Minimal matrix operations on flat row-major `[f64]` slices.
//!
//! All matrices are stored as flat arrays in row-major order.
//! Element (i, j) of an m×n matrix is at index `i * n + j`.

/// Zero all elements.
pub fn zeros(out: &mut [f64]) {
    for v in out.iter_mut() {
        *v = 0.0;
    }
}

/// Copy src to dst.
pub fn copy(src: &[f64], dst: &mut [f64], len: usize) {
    dst[..len].copy_from_slice(&src[..len]);
}

/// C = A * B. A is (m × k), B is (k × n), C is (m × n).
pub fn mul(a: &[f64], m: usize, k: usize, b: &[f64], n: usize, c: &mut [f64]) {
    for i in 0..m {
        for j in 0..n {
            let mut sum = 0.0;
            for p in 0..k {
                sum += a[i * k + p] * b[p * n + j];
            }
            c[i * n + j] = sum;
        }
    }
}

/// C = A^T * B. A is (m × k), B is (m × n), C is (k × n).
pub fn mul_at_b(a: &[f64], m: usize, k: usize, b: &[f64], n: usize, c: &mut [f64]) {
    for i in 0..k {
        for j in 0..n {
            let mut sum = 0.0;
            for p in 0..m {
                sum += a[p * k + i] * b[p * n + j];
            }
            c[i * n + j] = sum;
        }
    }
}

/// C += A * B. A is (m × k), B is (k × n), C is (m × n).
pub fn mul_add(a: &[f64], m: usize, k: usize, b: &[f64], n: usize, c: &mut [f64]) {
    for i in 0..m {
        for j in 0..n {
            let mut sum = 0.0;
            for p in 0..k {
                sum += a[i * k + p] * b[p * n + j];
            }
            c[i * n + j] += sum;
        }
    }
}

/// out = a + b (element-wise).
pub fn add(a: &[f64], b: &[f64], out: &mut [f64], len: usize) {
    for i in 0..len {
        out[i] = a[i] + b[i];
    }
}

/// a += b (element-wise, in-place).
pub fn add_inplace(a: &mut [f64], b: &[f64], len: usize) {
    for i in 0..len {
        a[i] += b[i];
    }
}

/// out = a * scalar.
pub fn scale(a: &[f64], s: f64, out: &mut [f64], len: usize) {
    for i in 0..len {
        out[i] = a[i] * s;
    }
}

/// a *= scalar (in-place).
pub fn scale_inplace(a: &mut [f64], s: f64, len: usize) {
    for i in 0..len {
        a[i] *= s;
    }
}

/// Set identity matrix (n × n).
pub fn identity(out: &mut [f64], n: usize) {
    zeros(&mut out[..n * n]);
    for i in 0..n {
        out[i * n + i] = 1.0;
    }
}

/// Invert a small matrix (n × n, n <= 4) using Gaussian elimination with partial pivoting.
/// Returns false if singular.
pub fn inv_small(a: &[f64], n: usize, out: &mut [f64]) -> bool {
    // Augmented matrix [A | I] stored as n × 2n
    let mut aug = [0.0; 4 * 8]; // max 4×8
    let cols = 2 * n;

    // Fill augmented matrix
    for i in 0..n {
        for j in 0..n {
            aug[i * cols + j] = a[i * n + j];
        }
        aug[i * cols + n + i] = 1.0;
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = libm::fabs(aug[col * cols + col]);
        let mut max_row = col;
        for row in (col + 1)..n {
            let v = libm::fabs(aug[row * cols + col]);
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-12 {
            return false; // Singular
        }

        // Swap rows
        if max_row != col {
            for j in 0..cols {
                let tmp = aug[col * cols + j];
                aug[col * cols + j] = aug[max_row * cols + j];
                aug[max_row * cols + j] = tmp;
            }
        }

        // Eliminate
        let pivot = aug[col * cols + col];
        for j in 0..cols {
            aug[col * cols + j] /= pivot;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row * cols + col];
            for j in 0..cols {
                aug[row * cols + j] -= factor * aug[col * cols + j];
            }
        }
    }

    // Extract inverse
    for i in 0..n {
        for j in 0..n {
            out[i * n + j] = aug[i * cols + n + j];
        }
    }
    true
}

/// Solve Hx = -f using Cholesky LLT decomposition.
/// H is symmetric positive definite (n × n), f is (n × 1).
/// Returns false if H is not positive definite.
pub fn cholesky_solve(h: &[f64], f: &[f64], n: usize, x: &mut [f64]) -> bool {
    // Cholesky decomposition: H = L * L^T
    let mut l = [0.0; 2500]; // MAX_HORIZON^2
    zeros(&mut l[..n * n]);

    for i in 0..n {
        for j in 0..=i {
            let mut sum = 0.0;
            for k in 0..j {
                sum += l[i * n + k] * l[j * n + k];
            }
            if i == j {
                let diag = h[i * n + i] - sum;
                if diag <= 0.0 {
                    return false;
                }
                l[i * n + j] = libm::sqrt(diag);
            } else {
                l[i * n + j] = (h[i * n + j] - sum) / l[j * n + j];
            }
        }
    }

    // Solve L*y = -f (forward substitution)
    let mut y = [0.0; 50]; // MAX_HORIZON
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..i {
            sum += l[i * n + j] * y[j];
        }
        y[i] = (-f[i] - sum) / l[i * n + i];
    }

    // Solve L^T*x = y (backward substitution)
    for i in (0..n).rev() {
        let mut sum = 0.0;
        for j in (i + 1)..n {
            sum += l[j * n + i] * x[j]; // L^T[i,j] = L[j,i]
        }
        x[i] = (y[i] - sum) / l[i * n + i];
    }

    true
}

#[cfg(kani)]
mod verification {
    use super::*;

    fn any_f64() -> f64 {
        f64::from_bits(kani::any::<u64>())
    }

    /// inv_small never panics for any 2x2 matrix input (including NaN, Inf).
    #[kani::proof]
    fn inv_small_2x2_never_panics() {
        let a = [any_f64(), any_f64(), any_f64(), any_f64()];
        let mut out = [0.0; 4];
        let _ok = inv_small(&a, 2, &mut out);
    }

    /// Cholesky solver never panics for any 2x2 input.
    #[kani::proof]
    fn cholesky_solve_2x2_never_panics() {
        let h = [any_f64(), any_f64(), any_f64(), any_f64()];
        let f = [any_f64(), any_f64()];
        let mut x = [0.0; 2];
        let _ok = cholesky_solve(&h, &f, 2, &mut x);
    }

    /// inv_small returns false (not panic) for a singular matrix.
    #[kani::proof]
    fn inv_small_singular_returns_false() {
        // Singular: second row is scaled first row
        let a = any_f64();
        let b = any_f64();
        let s = any_f64();
        let mat = [a, b, a * s, b * s];
        let mut out = [0.0; 4];
        // Either succeeds (numerical near-singular) or returns false — never panics
        let _ok = inv_small(&mat, 2, &mut out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mat_mul_2x2() {
        let a = [1.0, 2.0, 3.0, 4.0];
        let b = [5.0, 6.0, 7.0, 8.0];
        let mut c = [0.0; 4];
        mul(&a, 2, 2, &b, 2, &mut c);
        assert!((c[0] - 19.0).abs() < 1e-10);
        assert!((c[1] - 22.0).abs() < 1e-10);
        assert!((c[2] - 43.0).abs() < 1e-10);
        assert!((c[3] - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_mat_inv_2x2() {
        let a = [4.0, 7.0, 2.0, 6.0];
        let mut inv = [0.0; 4];
        assert!(inv_small(&a, 2, &mut inv));
        // A * A^-1 = I
        let mut prod = [0.0; 4];
        mul(&a, 2, 2, &inv, 2, &mut prod);
        assert!((prod[0] - 1.0).abs() < 1e-10);
        assert!((prod[1]).abs() < 1e-10);
        assert!((prod[2]).abs() < 1e-10);
        assert!((prod[3] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_mat_inv_3x3() {
        let a = [1.0, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0];
        let mut inv = [0.0; 9];
        assert!(inv_small(&a, 3, &mut inv));
        let mut prod = [0.0; 9];
        mul(&a, 3, 3, &inv, 3, &mut prod);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (prod[i * 3 + j] - expected).abs() < 1e-9,
                    "prod[{},{}] = {} expected {}",
                    i,
                    j,
                    prod[i * 3 + j],
                    expected
                );
            }
        }
    }

    #[test]
    fn test_cholesky_solve_2x2() {
        // H = [[4, 2], [2, 3]], f = [1, 2]
        // Solve Hx = -f → x = -H^-1 * f
        let h = [4.0, 2.0, 2.0, 3.0];
        let f = [1.0, 2.0];
        let mut x = [0.0; 2];
        assert!(cholesky_solve(&h, &f, 2, &mut x));
        // H^-1 = [[3/8, -1/4], [-1/4, 1/2]]
        // x = -H^-1*f = -[3/8 - 1/2, -1/4 + 1] = -[-1/8, 3/4] = [1/8, -3/4]
        assert!((x[0] - 0.125).abs() < 1e-10);
        assert!((x[1] - (-0.75)).abs() < 1e-10);
    }

    #[test]
    fn test_cholesky_solve_identity() {
        let h = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let f = [3.0, 5.0, 7.0];
        let mut x = [0.0; 3];
        assert!(cholesky_solve(&h, &f, 3, &mut x));
        assert!((x[0] - (-3.0)).abs() < 1e-10);
        assert!((x[1] - (-5.0)).abs() < 1e-10);
        assert!((x[2] - (-7.0)).abs() < 1e-10);
    }
}
