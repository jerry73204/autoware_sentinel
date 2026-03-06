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

//! Port of autoware_interpolation from Autoware Universe.
//!
//! Provides linear interpolation, cubic spline interpolation, Akima spline
//! interpolation, SLERP for quaternions, and 2D spline interpolation for
//! trajectory points.

#![no_std]

#[cfg(feature = "std")]
extern crate std;

use geometry_msgs::msg::{Point, Quaternion};

/// Maximum number of base points for spline interpolation.
/// All fixed-size arrays use this as their capacity.
pub const MAX_POINTS: usize = 256;

// ---------------------------------------------------------------------------
// Linear interpolation
// ---------------------------------------------------------------------------

/// Linearly interpolate a single value between `src` and `dst` by `ratio`.
#[inline]
pub fn lerp(src: f64, dst: f64, ratio: f64) -> f64 {
    src + (dst - src) * ratio
}

/// Error type for interpolation operations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterpolationError {
    /// Not enough base points (need at least 2).
    NotEnoughPoints,
    /// Base keys are not strictly increasing.
    KeysNotIncreasing,
    /// Keys and values have different lengths.
    LengthMismatch,
    /// Too many points (exceeds MAX_POINTS).
    TooManyPoints,
    /// Query key is out of the base key range.
    OutOfRange,
}

/// Validate that base keys are strictly increasing.
fn validate_keys(keys: &[f64]) -> Result<(), InterpolationError> {
    if keys.len() < 2 {
        return Err(InterpolationError::NotEnoughPoints);
    }
    for i in 1..keys.len() {
        if !(keys[i] > keys[i - 1]) {
            return Err(InterpolationError::KeysNotIncreasing);
        }
    }
    Ok(())
}

/// Validate keys and values have matching lengths and keys are valid.
fn validate_keys_and_values(keys: &[f64], values: &[f64]) -> Result<(), InterpolationError> {
    validate_keys(keys)?;
    if keys.len() != values.len() {
        return Err(InterpolationError::LengthMismatch);
    }
    Ok(())
}

/// Clamp a query key to the valid range [keys.first, keys.last].
fn clamp_key(key: f64, keys: &[f64]) -> f64 {
    if key < keys[0] {
        keys[0]
    } else if key > keys[keys.len() - 1] {
        keys[keys.len() - 1]
    } else {
        key
    }
}

/// Find the segment index for a query key via linear scan.
fn find_segment(key: f64, keys: &[f64]) -> usize {
    for i in 0..keys.len() - 1 {
        if keys[i + 1] >= key {
            return i;
        }
    }
    keys.len() - 2
}

/// Linearly interpolate a single query key over base keys and values.
pub fn lerp_single(
    base_keys: &[f64],
    base_values: &[f64],
    query_key: f64,
) -> Result<f64, InterpolationError> {
    validate_keys_and_values(base_keys, base_values)?;
    let key = clamp_key(query_key, base_keys);
    let idx = find_segment(key, base_keys);
    let ratio = (key - base_keys[idx]) / (base_keys[idx + 1] - base_keys[idx]);
    Ok(lerp(base_values[idx], base_values[idx + 1], ratio))
}

/// Linearly interpolate multiple query keys.
///
/// Results are written to `output` slice. Returns the number of values written.
pub fn lerp_multi(
    base_keys: &[f64],
    base_values: &[f64],
    query_keys: &[f64],
    output: &mut [f64],
) -> Result<usize, InterpolationError> {
    validate_keys_and_values(base_keys, base_values)?;
    let n = query_keys.len().min(output.len());
    let mut seg = 0usize;
    for i in 0..n {
        let key = clamp_key(query_keys[i], base_keys);
        while seg + 1 < base_keys.len() - 1 && base_keys[seg + 1] < key {
            seg += 1;
        }
        let ratio = (key - base_keys[seg]) / (base_keys[seg + 1] - base_keys[seg]);
        output[i] = lerp(base_values[seg], base_values[seg + 1], ratio);
    }
    Ok(n)
}

// ---------------------------------------------------------------------------
// SLERP (Spherical Linear Interpolation)
// ---------------------------------------------------------------------------

/// Spherical linear interpolation between two quaternions.
///
/// Implements SLERP without depending on Eigen: uses the standard formula
/// with dot product to determine angle, handles near-parallel quaternions
/// by falling back to normalized linear interpolation.
pub fn slerp(src: &Quaternion, dst: &Quaternion, ratio: f64) -> Quaternion {
    let mut dot = src.x * dst.x + src.y * dst.y + src.z * dst.z + src.w * dst.w;

    // If dot is negative, negate one quaternion to take shortest path
    let (dx, dy, dz, dw) = if dot < 0.0 {
        dot = -dot;
        (-dst.x, -dst.y, -dst.z, -dst.w)
    } else {
        (dst.x, dst.y, dst.z, dst.w)
    };

    // Clamp dot to valid acos range
    if dot > 0.9995 {
        // Nearly parallel — use normalized LERP
        let x = src.x + (dx - src.x) * ratio;
        let y = src.y + (dy - src.y) * ratio;
        let z = src.z + (dz - src.z) * ratio;
        let w = src.w + (dw - src.w) * ratio;
        let inv_norm = 1.0 / libm::sqrt(x * x + y * y + z * z + w * w);
        return Quaternion {
            x: x * inv_norm,
            y: y * inv_norm,
            z: z * inv_norm,
            w: w * inv_norm,
        };
    }

    let theta = libm::acos(dot);
    let sin_theta = libm::sin(theta);
    let s0 = libm::sin((1.0 - ratio) * theta) / sin_theta;
    let s1 = libm::sin(ratio * theta) / sin_theta;

    Quaternion {
        x: src.x * s0 + dx * s1,
        y: src.y * s0 + dy * s1,
        z: src.z * s0 + dz * s1,
        w: src.w * s0 + dw * s1,
    }
}

/// SLERP a single query key over base keys and quaternion values.
pub fn slerp_single(
    base_keys: &[f64],
    base_values: &[Quaternion],
    query_key: f64,
) -> Result<Quaternion, InterpolationError> {
    validate_keys(base_keys)?;
    if base_keys.len() != base_values.len() {
        return Err(InterpolationError::LengthMismatch);
    }
    let key = clamp_key(query_key, base_keys);
    let idx = find_segment(key, base_keys);
    let ratio = (key - base_keys[idx]) / (base_keys[idx + 1] - base_keys[idx]);
    Ok(slerp(&base_values[idx], &base_values[idx + 1], ratio))
}

// ---------------------------------------------------------------------------
// Cubic spline interpolation
// ---------------------------------------------------------------------------

/// Cubic spline coefficients for one-dimensional interpolation.
///
/// For each segment i, the spline value at offset dx from base_keys[i] is:
/// `f(dx) = a[i]*dx³ + b[i]*dx² + c[i]*dx + d[i]`
#[derive(Debug, Clone)]
pub struct SplineInterpolation {
    base_keys: [f64; MAX_POINTS],
    a: [f64; MAX_POINTS],
    b: [f64; MAX_POINTS],
    c: [f64; MAX_POINTS],
    d: [f64; MAX_POINTS],
    len: usize,
}

impl SplineInterpolation {
    /// Fit a natural cubic spline to the given base keys and values.
    pub fn new(base_keys: &[f64], base_values: &[f64]) -> Result<Self, InterpolationError> {
        if base_keys.len() != base_values.len() {
            return Err(InterpolationError::LengthMismatch);
        }
        let n = base_keys.len();
        if n < 2 {
            return Err(InterpolationError::NotEnoughPoints);
        }
        if n > MAX_POINTS {
            return Err(InterpolationError::TooManyPoints);
        }
        validate_keys(base_keys)?;

        let mut s = SplineInterpolation {
            base_keys: [0.0; MAX_POINTS],
            a: [0.0; MAX_POINTS],
            b: [0.0; MAX_POINTS],
            c: [0.0; MAX_POINTS],
            d: [0.0; MAX_POINTS],
            len: n,
        };
        s.base_keys[..n].copy_from_slice(base_keys);
        s.calc_coefficients(base_keys, base_values);
        Ok(s)
    }

    /// Number of base points.
    pub fn size(&self) -> usize {
        self.len
    }

    fn calc_coefficients(&mut self, x: &[f64], y: &[f64]) {
        let n = x.len();
        let segs = n - 1;

        if n == 2 {
            // Linear: a=0, b=0, c=slope, d=y[0]
            self.a[0] = 0.0;
            self.b[0] = 0.0;
            self.c[0] = (y[1] - y[0]) / (x[1] - x[0]);
            self.d[0] = y[0];
            return;
        }

        // Step sizes
        let mut h = [0.0; MAX_POINTS];
        for i in 0..segs {
            h[i] = x[i + 1] - x[i];
        }

        // Build tridiagonal system for second derivatives v
        // System size = n-2 (interior points)
        let m = n - 2;
        let mut sub = [0.0; MAX_POINTS]; // sub-diagonal a
        let mut diag = [0.0; MAX_POINTS]; // main diagonal b
        let mut sup = [0.0; MAX_POINTS]; // super-diagonal c
        let mut rhs = [0.0; MAX_POINTS]; // right-hand side d

        for i in 0..m {
            if i > 0 {
                sub[i] = h[i];
            }
            diag[i] = 2.0 * (h[i] + h[i + 1]);
            if i < m - 1 {
                sup[i] = h[i + 1];
            }
            rhs[i] = 6.0 * ((y[i + 2] - y[i + 1]) / h[i + 1] - (y[i + 1] - y[i]) / h[i]);
        }

        // Solve tridiagonal system (Thomas algorithm)
        let mut v = [0.0; MAX_POINTS]; // v[0] and v[n-1] = 0 (natural spline)
        {
            let mut c_prime = [0.0; MAX_POINTS];
            let mut d_prime = [0.0; MAX_POINTS];

            c_prime[0] = sup[0] / diag[0];
            d_prime[0] = rhs[0] / diag[0];

            for i in 1..m {
                let w = diag[i] - sub[i] * c_prime[i - 1];
                c_prime[i] = if i < m - 1 { sup[i] / w } else { 0.0 };
                d_prime[i] = (rhs[i] - sub[i] * d_prime[i - 1]) / w;
            }

            // v indices are shifted by 1 (v[i+1] = solution[i])
            v[m] = d_prime[m - 1];
            for i in (0..m - 1).rev() {
                v[i + 1] = d_prime[i] - c_prime[i] * v[i + 2];
            }
        }
        // v[0] = 0 (natural), v[n-1] = 0 (natural) — already initialized

        // Calculate cubic coefficients
        for i in 0..segs {
            self.a[i] = (v[i + 1] - v[i]) / (6.0 * h[i]);
            self.b[i] = v[i] / 2.0;
            self.c[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (2.0 * v[i] + v[i + 1]) / 6.0;
            self.d[i] = y[i];
        }
    }

    /// Find segment index for a given key.
    fn get_index(&self, key: f64) -> usize {
        let keys = &self.base_keys[..self.len];
        // Binary search (lower_bound) then clamp to valid segment range
        let mut lo = 0usize;
        let mut hi = self.len;
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            if keys[mid] < key {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        // lo is now lower_bound index; clamp to [0, len-2]
        if lo == 0 {
            0
        } else if lo >= self.len - 1 {
            self.len - 2
        } else {
            lo - 1
        }
    }

    /// Evaluate the spline at a single query key.
    pub fn eval(&self, query_key: f64) -> f64 {
        let key = clamp_key(query_key, &self.base_keys[..self.len]);
        let idx = self.get_index(key);
        let dx = key - self.base_keys[idx];
        self.a[idx] * dx * dx * dx + self.b[idx] * dx * dx + self.c[idx] * dx + self.d[idx]
    }

    /// Evaluate the first derivative of the spline at a single query key.
    pub fn eval_diff(&self, query_key: f64) -> f64 {
        let key = clamp_key(query_key, &self.base_keys[..self.len]);
        let idx = self.get_index(key);
        let dx = key - self.base_keys[idx];
        3.0 * self.a[idx] * dx * dx + 2.0 * self.b[idx] * dx + self.c[idx]
    }

    /// Evaluate the second derivative of the spline at a single query key.
    pub fn eval_quad_diff(&self, query_key: f64) -> f64 {
        let key = clamp_key(query_key, &self.base_keys[..self.len]);
        let idx = self.get_index(key);
        let dx = key - self.base_keys[idx];
        6.0 * self.a[idx] * dx + 2.0 * self.b[idx]
    }

    /// Evaluate the spline at multiple query keys, writing results to output.
    pub fn eval_multi(&self, query_keys: &[f64], output: &mut [f64]) -> usize {
        let n = query_keys.len().min(output.len());
        for i in 0..n {
            output[i] = self.eval(query_keys[i]);
        }
        n
    }
}

// ---------------------------------------------------------------------------
// Akima spline interpolation
// ---------------------------------------------------------------------------

/// Akima spline coefficients for one-dimensional interpolation.
///
/// Akima splines reduce oscillations near outliers compared to natural cubic splines,
/// using a weighted average of slopes for endpoint tangents.
#[derive(Debug, Clone)]
pub struct AkimaInterpolation {
    base_keys: [f64; MAX_POINTS],
    a: [f64; MAX_POINTS],
    b: [f64; MAX_POINTS],
    c: [f64; MAX_POINTS],
    d: [f64; MAX_POINTS],
    len: usize,
}

impl AkimaInterpolation {
    /// Fit an Akima spline to the given base keys and values.
    pub fn new(base_keys: &[f64], base_values: &[f64]) -> Result<Self, InterpolationError> {
        if base_keys.len() != base_values.len() {
            return Err(InterpolationError::LengthMismatch);
        }
        let n = base_keys.len();
        if n < 2 {
            return Err(InterpolationError::NotEnoughPoints);
        }
        if n > MAX_POINTS {
            return Err(InterpolationError::TooManyPoints);
        }
        validate_keys(base_keys)?;

        let mut akima = AkimaInterpolation {
            base_keys: [0.0; MAX_POINTS],
            a: [0.0; MAX_POINTS],
            b: [0.0; MAX_POINTS],
            c: [0.0; MAX_POINTS],
            d: [0.0; MAX_POINTS],
            len: n,
        };
        akima.base_keys[..n].copy_from_slice(base_keys);

        let segs = n - 1;
        const EPSILON: f64 = 1e-5;

        // Calculate segment slopes m
        let mut m = [0.0; MAX_POINTS];
        for i in 0..segs {
            m[i] = (base_values[i + 1] - base_values[i]) / (base_keys[i + 1] - base_keys[i]);
        }

        // Calculate endpoint tangents s using Akima's weighted formula
        let mut s = [0.0; MAX_POINTS];
        for i in 0..n {
            if i == 0 {
                s[i] = m[0];
            } else if i == n - 1 {
                s[i] = m[segs - 1];
            } else if i == 1 || i == n - 2 {
                s[i] = (m[i - 1] + m[i]) / 2.0;
            } else {
                let denom = libm::fabs(m[i + 1] - m[i]) + libm::fabs(m[i - 1] - m[i - 2]);
                if libm::fabs(denom) < EPSILON {
                    s[i] = (m[i - 1] + m[i]) / 2.0;
                } else {
                    s[i] = (libm::fabs(m[i + 1] - m[i]) * m[i - 1]
                        + libm::fabs(m[i - 1] - m[i - 2]) * m[i])
                        / denom;
                }
            }
        }

        // Calculate cubic coefficients
        for i in 0..segs {
            let h = base_keys[i + 1] - base_keys[i];
            akima.a[i] = (s[i] + s[i + 1] - 2.0 * m[i]) / (h * h);
            akima.b[i] = (3.0 * m[i] - 2.0 * s[i] - s[i + 1]) / h;
            akima.c[i] = s[i];
            akima.d[i] = base_values[i];
        }

        Ok(akima)
    }

    /// Evaluate the Akima spline at a single query key.
    pub fn eval(&self, query_key: f64) -> f64 {
        let key = clamp_key(query_key, &self.base_keys[..self.len]);
        let idx = find_segment(key, &self.base_keys[..self.len]);
        let ds = key - self.base_keys[idx];
        self.d[idx] + (self.c[idx] + (self.b[idx] + self.a[idx] * ds) * ds) * ds
    }
}

// ---------------------------------------------------------------------------
// 2D Spline interpolation for trajectory points
// ---------------------------------------------------------------------------

/// 2D spline interpolation over a sequence of Point messages.
///
/// Fits three independent splines (x, y, z) parameterized by cumulative
/// arc length. Provides interpolated position, yaw, and curvature.
#[derive(Debug, Clone)]
pub struct SplineInterpolationPoints2d {
    spline_x: SplineInterpolation,
    spline_y: SplineInterpolation,
    spline_z: SplineInterpolation,
    base_s: [f64; MAX_POINTS],
    len: usize,
}

impl SplineInterpolationPoints2d {
    /// Fit 2D splines through the given points.
    ///
    /// Duplicate consecutive points (distance < 1e-6) are removed.
    pub fn new(points: &[Point]) -> Result<Self, InterpolationError> {
        if points.len() < 2 {
            return Err(InterpolationError::NotEnoughPoints);
        }

        // Extract unique points (remove overlaps)
        let mut xs = [0.0; MAX_POINTS];
        let mut ys = [0.0; MAX_POINTS];
        let mut zs = [0.0; MAX_POINTS];
        let mut count = 0usize;

        for (i, p) in points.iter().enumerate() {
            if i > 0 && count > 0 {
                let dx = p.x - xs[count - 1];
                let dy = p.y - ys[count - 1];
                if libm::fabs(dx) < 1e-6 && libm::fabs(dy) < 1e-6 {
                    continue;
                }
            }
            if count >= MAX_POINTS {
                return Err(InterpolationError::TooManyPoints);
            }
            xs[count] = p.x;
            ys[count] = p.y;
            zs[count] = p.z;
            count += 1;
        }

        if count < 2 {
            return Err(InterpolationError::NotEnoughPoints);
        }

        // Calculate cumulative arc length
        let mut base_s = [0.0; MAX_POINTS];
        for i in 1..count {
            let dx = xs[i] - xs[i - 1];
            let dy = ys[i] - ys[i - 1];
            base_s[i] = base_s[i - 1] + libm::sqrt(dx * dx + dy * dy);
        }

        let s = &base_s[..count];
        let spline_x = SplineInterpolation::new(s, &xs[..count])?;
        let spline_y = SplineInterpolation::new(s, &ys[..count])?;
        let spline_z = SplineInterpolation::new(s, &zs[..count])?;

        Ok(SplineInterpolationPoints2d {
            spline_x,
            spline_y,
            spline_z,
            base_s,
            len: count,
        })
    }

    /// Number of (unique) base points.
    pub fn size(&self) -> usize {
        self.len
    }

    /// Get the cumulative arc length at base point index.
    pub fn accumulated_length(&self, idx: usize) -> f64 {
        self.base_s[idx]
    }

    /// Total arc length.
    pub fn total_length(&self) -> f64 {
        self.base_s[self.len - 1]
    }

    /// Clamp an arc-length value to valid range.
    fn clamp_s(&self, s: f64) -> f64 {
        if s < self.base_s[0] {
            self.base_s[0]
        } else if s > self.base_s[self.len - 1] {
            self.base_s[self.len - 1]
        } else {
            s
        }
    }

    /// Get interpolated 3D point at base index + arc-length offset.
    pub fn interpolated_point(&self, idx: usize, s_offset: f64) -> Point {
        let s = self.clamp_s(self.base_s[idx] + s_offset);
        Point {
            x: self.spline_x.eval(s),
            y: self.spline_y.eval(s),
            z: self.spline_z.eval(s),
        }
    }

    /// Get interpolated yaw at base index + arc-length offset.
    pub fn interpolated_yaw(&self, idx: usize, s_offset: f64) -> f64 {
        let s = self.clamp_s(self.base_s[idx] + s_offset);
        let diff_x = self.spline_x.eval_diff(s);
        let diff_y = self.spline_y.eval_diff(s);
        libm::atan2(diff_y, diff_x)
    }

    /// Get interpolated curvature at base index + arc-length offset.
    pub fn interpolated_curvature(&self, idx: usize, s_offset: f64) -> f64 {
        let s = self.clamp_s(self.base_s[idx] + s_offset);
        let dx = self.spline_x.eval_diff(s);
        let dy = self.spline_y.eval_diff(s);
        let ddx = self.spline_x.eval_quad_diff(s);
        let ddy = self.spline_y.eval_quad_diff(s);
        let denom = libm::pow(dx * dx + dy * dy, 1.5);
        if libm::fabs(denom) < 1e-10 {
            return 0.0;
        }
        (dx * ddy - ddx * dy) / denom
    }

    /// Get all yaw values at each base point.
    pub fn yaws(&self, output: &mut [f64]) -> usize {
        let n = self.len.min(output.len());
        for i in 0..n {
            output[i] = self.interpolated_yaw(i, 0.0);
        }
        n
    }

    /// Get all curvature values at each base point.
    pub fn curvatures(&self, output: &mut [f64]) -> usize {
        let n = self.len.min(output.len());
        for i in 0..n {
            output[i] = self.interpolated_curvature(i, 0.0);
        }
        n
    }

    /// Find the base index closest to a given arc-length offset from `idx`.
    pub fn offset_index(&self, idx: usize, offset: f64) -> usize {
        let whole_s = self.base_s[idx] + offset;
        for i in 0..self.len {
            if whole_s < self.base_s[i] {
                return i;
            }
        }
        self.len - 1
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64) -> bool {
        libm::fabs(a - b) < EPS
    }

    // -- Linear interpolation --

    #[test]
    fn test_lerp_basic() {
        assert!(approx_eq(lerp(0.0, 10.0, 0.5), 5.0));
        assert!(approx_eq(lerp(0.0, 10.0, 0.0), 0.0));
        assert!(approx_eq(lerp(0.0, 10.0, 1.0), 10.0));
    }

    #[test]
    fn test_lerp_single() {
        let keys = [0.0, 1.0, 2.0, 3.0];
        let vals = [0.0, 2.0, 4.0, 6.0];
        let r = lerp_single(&keys, &vals, 1.5).unwrap();
        assert!(approx_eq(r, 3.0));
    }

    #[test]
    fn test_lerp_multi() {
        let keys = [0.0, 1.0, 2.0];
        let vals = [0.0, 10.0, 20.0];
        let queries = [0.5, 1.0, 1.5];
        let mut out = [0.0; 3];
        lerp_multi(&keys, &vals, &queries, &mut out).unwrap();
        assert!(approx_eq(out[0], 5.0));
        assert!(approx_eq(out[1], 10.0));
        assert!(approx_eq(out[2], 15.0));
    }

    #[test]
    fn test_lerp_error_not_increasing() {
        let keys = [0.0, 0.0, 1.0];
        let vals = [0.0, 1.0, 2.0];
        assert_eq!(
            lerp_single(&keys, &vals, 0.5),
            Err(InterpolationError::KeysNotIncreasing)
        );
    }

    // -- Cubic spline --

    #[test]
    fn test_spline_linear_data() {
        // Spline through linear data should reproduce linear interpolation
        let keys = [0.0, 1.0, 2.0, 3.0];
        let vals = [0.0, 1.0, 2.0, 3.0];
        let s = SplineInterpolation::new(&keys, &vals).unwrap();
        assert!(approx_eq(s.eval(0.5), 0.5));
        assert!(approx_eq(s.eval(1.5), 1.5));
        assert!(approx_eq(s.eval(2.5), 2.5));
    }

    #[test]
    fn test_spline_endpoints() {
        let keys = [0.0, 1.0, 2.0, 3.0, 4.0];
        let vals = [0.0, 1.0, 0.0, 1.0, 0.0];
        let s = SplineInterpolation::new(&keys, &vals).unwrap();
        // Should pass through base points exactly
        assert!(approx_eq(s.eval(0.0), 0.0));
        assert!(approx_eq(s.eval(1.0), 1.0));
        assert!(approx_eq(s.eval(2.0), 0.0));
        assert!(approx_eq(s.eval(3.0), 1.0));
        assert!(approx_eq(s.eval(4.0), 0.0));
    }

    #[test]
    fn test_spline_two_points() {
        let keys = [0.0, 1.0];
        let vals = [0.0, 5.0];
        let s = SplineInterpolation::new(&keys, &vals).unwrap();
        assert!(approx_eq(s.eval(0.5), 2.5));
    }

    #[test]
    fn test_spline_diff() {
        // Linear data: first derivative should be constant = 1.0
        let keys = [0.0, 1.0, 2.0, 3.0];
        let vals = [0.0, 1.0, 2.0, 3.0];
        let s = SplineInterpolation::new(&keys, &vals).unwrap();
        assert!(approx_eq(s.eval_diff(0.5), 1.0));
        assert!(approx_eq(s.eval_diff(1.5), 1.0));
    }

    // -- Akima spline --

    #[test]
    fn test_akima_passthrough() {
        let keys = [0.0, 1.0, 2.0, 3.0, 4.0];
        let vals = [0.0, 1.0, 4.0, 9.0, 16.0];
        let s = AkimaInterpolation::new(&keys, &vals).unwrap();
        // Should pass through base points
        assert!(approx_eq(s.eval(0.0), 0.0));
        assert!(approx_eq(s.eval(1.0), 1.0));
        assert!(approx_eq(s.eval(4.0), 16.0));
    }

    // -- SLERP --

    #[test]
    fn test_slerp_identity() {
        let q = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        let result = slerp(&q, &q, 0.5);
        assert!(approx_eq(result.w, 1.0));
        assert!(approx_eq(result.x, 0.0));
    }

    #[test]
    fn test_slerp_endpoints() {
        let q0 = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        // 90° rotation around Z
        let half = core::f64::consts::FRAC_PI_4;
        let q1 = Quaternion {
            x: 0.0,
            y: 0.0,
            z: libm::sin(half),
            w: libm::cos(half),
        };
        let r0 = slerp(&q0, &q1, 0.0);
        let r1 = slerp(&q0, &q1, 1.0);
        assert!(approx_eq(r0.w, q0.w));
        assert!(approx_eq(r0.z, q0.z));
        assert!(approx_eq(r1.w, q1.w));
        assert!(approx_eq(r1.z, q1.z));
    }

    // -- 2D spline --

    #[test]
    fn test_spline_2d_straight_line() {
        let points = [
            Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Point {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Point {
                x: 2.0,
                y: 0.0,
                z: 0.0,
            },
            Point {
                x: 3.0,
                y: 0.0,
                z: 0.0,
            },
        ];
        let s = SplineInterpolationPoints2d::new(&points).unwrap();
        assert_eq!(s.size(), 4);
        assert!(approx_eq(s.total_length(), 3.0));

        // Midpoint
        let p = s.interpolated_point(1, 0.5);
        assert!(approx_eq(p.x, 1.5));
        assert!(approx_eq(p.y, 0.0));

        // Yaw should be 0 for straight line along X
        let yaw = s.interpolated_yaw(0, 0.0);
        assert!(approx_eq(yaw, 0.0));

        // Curvature should be 0 for straight line
        let k = s.interpolated_curvature(1, 0.0);
        assert!(libm::fabs(k) < 1e-3);
    }

    #[test]
    fn test_spline_2d_removes_duplicates() {
        let points = [
            Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }, // duplicate
            Point {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Point {
                x: 2.0,
                y: 0.0,
                z: 0.0,
            },
        ];
        let s = SplineInterpolationPoints2d::new(&points).unwrap();
        assert_eq!(s.size(), 3); // duplicate removed
    }

    #[test]
    fn test_spline_2d_not_enough_points() {
        let points = [Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }];
        assert!(matches!(
            SplineInterpolationPoints2d::new(&points),
            Err(InterpolationError::NotEnoughPoints)
        ));
    }
}
