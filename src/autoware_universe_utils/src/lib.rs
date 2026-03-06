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

//! Port of autoware_universe_utils from Autoware Universe.
//!
//! Geometry, math, and normalization utilities used across Autoware algorithms.

#![no_std]

#[cfg(feature = "std")]
extern crate std;

use core::f64::consts::PI;
use geometry_msgs::msg::{Point, Quaternion, Vector3};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

pub const GRAVITY: f64 = 9.80665;

// ---------------------------------------------------------------------------
// Unit conversion
// ---------------------------------------------------------------------------

#[inline]
pub fn deg2rad(deg: f64) -> f64 {
    deg * PI / 180.0
}

#[inline]
pub fn rad2deg(rad: f64) -> f64 {
    rad * 180.0 / PI
}

#[inline]
pub fn kmph2mps(kmph: f64) -> f64 {
    kmph * 1000.0 / 3600.0
}

#[inline]
pub fn mps2kmph(mps: f64) -> f64 {
    mps * 3600.0 / 1000.0
}

// ---------------------------------------------------------------------------
// Angle normalization
// ---------------------------------------------------------------------------

/// Normalize radian to [min_rad, min_rad + 2*PI).
pub fn normalize_radian(rad: f64, min_rad: f64) -> f64 {
    let max_rad = min_rad + 2.0 * PI;
    let mut value = rad % (2.0 * PI);
    // Handle negative modulo
    if value < min_rad {
        value += 2.0 * PI;
    }
    if value >= max_rad {
        value -= 2.0 * PI;
    }
    value
}

/// Normalize degree to [min_deg, min_deg + 360).
pub fn normalize_degree(deg: f64, min_deg: f64) -> f64 {
    let max_deg = min_deg + 360.0;
    let mut value = deg % 360.0;
    if value < min_deg {
        value += 360.0;
    }
    if value >= max_deg {
        value -= 360.0;
    }
    value
}

// ---------------------------------------------------------------------------
// Geometry: distance functions
// ---------------------------------------------------------------------------

/// 2D Euclidean distance between two points.
#[inline]
pub fn calc_distance_2d(p1: &Point, p2: &Point) -> f64 {
    let dx = p1.x - p2.x;
    let dy = p1.y - p2.y;
    libm::sqrt(dx * dx + dy * dy)
}

/// Squared 2D Euclidean distance between two points.
#[inline]
pub fn calc_squared_distance_2d(p1: &Point, p2: &Point) -> f64 {
    let dx = p1.x - p2.x;
    let dy = p1.y - p2.y;
    dx * dx + dy * dy
}

/// 3D Euclidean distance between two points.
#[inline]
pub fn calc_distance_3d(p1: &Point, p2: &Point) -> f64 {
    let dx = p1.x - p2.x;
    let dy = p1.y - p2.y;
    let dz = p1.z - p2.z;
    libm::sqrt(dx * dx + dy * dy + dz * dz)
}

// ---------------------------------------------------------------------------
// Quaternion utilities
// ---------------------------------------------------------------------------

/// Create a Quaternion message.
pub fn create_quaternion(x: f64, y: f64, z: f64, w: f64) -> Quaternion {
    Quaternion { x, y, z, w }
}

/// Extract yaw angle from a quaternion.
///
/// Uses the standard formula: atan2(2(wz + xy), 1 - 2(y² + z²)).
pub fn get_yaw(q: &Quaternion) -> f64 {
    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    libm::atan2(siny_cosp, cosy_cosp)
}

/// Create a quaternion from yaw angle only (rotation around Z axis).
pub fn create_quaternion_from_yaw(yaw: f64) -> Quaternion {
    let half = yaw * 0.5;
    Quaternion {
        x: 0.0,
        y: 0.0,
        z: libm::sin(half),
        w: libm::cos(half),
    }
}

/// Create a quaternion from roll, pitch, yaw (ZYX intrinsic convention).
pub fn create_quaternion_from_rpy(roll: f64, pitch: f64, yaw: f64) -> Quaternion {
    let (sr, cr) = (libm::sin(roll * 0.5), libm::cos(roll * 0.5));
    let (sp, cp) = (libm::sin(pitch * 0.5), libm::cos(pitch * 0.5));
    let (sy, cy) = (libm::sin(yaw * 0.5), libm::cos(yaw * 0.5));

    Quaternion {
        x: sr * cp * cy - cr * sp * sy,
        y: cr * sp * cy + sr * cp * sy,
        z: cr * cp * sy - sr * sp * cy,
        w: cr * cp * cy + sr * sp * sy,
    }
}

/// Extract roll, pitch, yaw from a quaternion. Returns (roll, pitch, yaw).
pub fn get_rpy(q: &Quaternion) -> (f64, f64, f64) {
    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    let cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    let roll = libm::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    let sinp = 2.0 * (q.w * q.y - q.z * q.x);
    let pitch = if libm::fabs(sinp) >= 1.0 {
        libm::copysign(PI / 2.0, sinp)
    } else {
        libm::asin(sinp)
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    let yaw = libm::atan2(siny_cosp, cosy_cosp);

    (roll, pitch, yaw)
}

// ---------------------------------------------------------------------------
// Geometry: pose deviation
// ---------------------------------------------------------------------------

/// Calculate yaw deviation between two poses (target yaw - base yaw), normalized to [-PI, PI).
pub fn calc_yaw_deviation(base_yaw: f64, target_yaw: f64) -> f64 {
    normalize_radian(target_yaw - base_yaw, -PI)
}

/// Calculate lateral deviation of a target point from a base pose.
///
/// Projects the vector from base position to target point onto the base pose's
/// lateral axis (perpendicular to heading direction).
pub fn calc_lateral_deviation(base_pos: &Point, base_yaw: f64, target: &Point) -> f64 {
    let dx = target.x - base_pos.x;
    let dy = target.y - base_pos.y;
    // Lateral = cross product of unit direction vector with offset
    -libm::sin(base_yaw) * dx + libm::cos(base_yaw) * dy
}

/// Calculate longitudinal offset of a target point projected onto a base pose direction.
pub fn calc_longitudinal_deviation(base_pos: &Point, base_yaw: f64, target: &Point) -> f64 {
    let dx = target.x - base_pos.x;
    let dy = target.y - base_pos.y;
    libm::cos(base_yaw) * dx + libm::sin(base_yaw) * dy
}

// ---------------------------------------------------------------------------
// Geometry: curvature
// ---------------------------------------------------------------------------

/// Menger curvature from three points.
///
/// curvature = 2 * area(triangle) / (|p1-p2| * |p2-p3| * |p1-p3|)
pub fn calc_curvature(p1: &Point, p2: &Point, p3: &Point) -> f64 {
    let d12 = calc_distance_2d(p1, p2);
    let d23 = calc_distance_2d(p2, p3);
    let d13 = calc_distance_2d(p1, p3);

    let denom = d12 * d23 * d13;
    if denom < 1e-10 {
        return 0.0;
    }

    // Signed area via cross product
    let area2 = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    area2 / denom
}

// ---------------------------------------------------------------------------
// Geometry: interpolation helpers
// ---------------------------------------------------------------------------

/// Linearly interpolate between two points.
pub fn lerp_point(src: &Point, dst: &Point, ratio: f64) -> Point {
    Point {
        x: src.x + (dst.x - src.x) * ratio,
        y: src.y + (dst.y - src.y) * ratio,
        z: src.z + (dst.z - src.z) * ratio,
    }
}

/// Linearly interpolate between two Vector3 values.
pub fn lerp_vector3(src: &Vector3, dst: &Vector3, ratio: f64) -> Vector3 {
    Vector3 {
        x: src.x + (dst.x - src.x) * ratio,
        y: src.y + (dst.y - src.y) * ratio,
        z: src.z + (dst.z - src.z) * ratio,
    }
}

/// Calculate azimuth angle from one point to another in 2D.
pub fn calc_azimuth_angle(p_from: &Point, p_to: &Point) -> f64 {
    libm::atan2(p_to.y - p_from.y, p_to.x - p_from.x)
}

/// Calculate elevation angle from one point to another.
pub fn calc_elevation_angle(p_from: &Point, p_to: &Point) -> f64 {
    let dist_2d = calc_distance_2d(p_from, p_to);
    libm::atan2(p_to.z - p_from.z, dist_2d)
}

/// Check if driving direction from src to dst is forward
/// (angle between src heading and direction to dst < PI/2).
pub fn is_driving_forward(src_yaw: f64, src: &Point, dst: &Point) -> bool {
    let direction_yaw = calc_azimuth_angle(src, dst);
    let dev = normalize_radian(src_yaw - direction_yaw, -PI);
    libm::fabs(dev) < PI / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64) -> bool {
        libm::fabs(a - b) < EPS
    }

    #[test]
    fn test_deg2rad_and_back() {
        assert!(approx_eq(deg2rad(180.0), PI));
        assert!(approx_eq(rad2deg(PI), 180.0));
        assert!(approx_eq(deg2rad(90.0), PI / 2.0));
    }

    #[test]
    fn test_unit_conversion() {
        assert!(approx_eq(kmph2mps(3.6), 1.0));
        assert!(approx_eq(mps2kmph(1.0), 3.6));
    }

    #[test]
    fn test_normalize_radian() {
        assert!(approx_eq(normalize_radian(0.0, -PI), 0.0));
        assert!(approx_eq(normalize_radian(2.0 * PI, -PI), 0.0));
        assert!(approx_eq(normalize_radian(-2.0 * PI, -PI), 0.0));
        // 3*PI should wrap to PI (but since range is [-PI, PI), it should be -PI)
        let val = normalize_radian(3.0 * PI, -PI);
        assert!(approx_eq(val, -PI) || approx_eq(val, PI));
    }

    #[test]
    fn test_distance_2d() {
        let p1 = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let p2 = Point {
            x: 3.0,
            y: 4.0,
            z: 0.0,
        };
        assert!(approx_eq(calc_distance_2d(&p1, &p2), 5.0));
        assert!(approx_eq(calc_squared_distance_2d(&p1, &p2), 25.0));
    }

    #[test]
    fn test_distance_3d() {
        let p1 = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let p2 = Point {
            x: 1.0,
            y: 2.0,
            z: 2.0,
        };
        assert!(approx_eq(calc_distance_3d(&p1, &p2), 3.0));
    }

    #[test]
    fn test_yaw_from_quaternion() {
        let q = create_quaternion_from_yaw(PI / 4.0);
        assert!(approx_eq(get_yaw(&q), PI / 4.0));

        let q_zero = create_quaternion_from_yaw(0.0);
        assert!(approx_eq(get_yaw(&q_zero), 0.0));
    }

    #[test]
    fn test_rpy_roundtrip() {
        let roll = 0.1;
        let pitch = 0.2;
        let yaw = 0.3;
        let q = create_quaternion_from_rpy(roll, pitch, yaw);
        let (r, p, y) = get_rpy(&q);
        assert!(approx_eq(r, roll));
        assert!(approx_eq(p, pitch));
        assert!(approx_eq(y, yaw));
    }

    #[test]
    fn test_yaw_deviation() {
        assert!(approx_eq(calc_yaw_deviation(0.0, PI / 2.0), PI / 2.0));
        assert!(approx_eq(calc_yaw_deviation(PI / 2.0, 0.0), -PI / 2.0));
    }

    #[test]
    fn test_lateral_deviation() {
        let base = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let target = Point {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        // Base facing along X axis (yaw=0) → target is 1m to the left
        assert!(approx_eq(calc_lateral_deviation(&base, 0.0, &target), 1.0));
    }

    #[test]
    fn test_curvature_straight() {
        let p1 = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let p2 = Point {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let p3 = Point {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        };
        assert!(approx_eq(calc_curvature(&p1, &p2, &p3), 0.0));
    }

    #[test]
    fn test_curvature_circle() {
        // Three points on a unit circle: (1,0), (0,1), (-1,0)
        // Menger curvature = signed_cross / (d12 * d23 * d13) = 2 / 4 = 0.5
        // (This is the signed curvature from the cross product formula)
        let p1 = Point {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let p2 = Point {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        let p3 = Point {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        };
        let k = calc_curvature(&p1, &p2, &p3);
        assert!(k > 0.0, "curvature should be positive for CCW arc");
        assert!(libm::fabs(k - 0.5) < 0.01);
    }

    #[test]
    fn test_lerp_point() {
        let a = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let b = Point {
            x: 10.0,
            y: 20.0,
            z: 30.0,
        };
        let mid = lerp_point(&a, &b, 0.5);
        assert!(approx_eq(mid.x, 5.0));
        assert!(approx_eq(mid.y, 10.0));
        assert!(approx_eq(mid.z, 15.0));
    }

    #[test]
    fn test_is_driving_forward() {
        let src = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let dst_fwd = Point {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let dst_back = Point {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        };
        assert!(is_driving_forward(0.0, &src, &dst_fwd));
        assert!(!is_driving_forward(0.0, &src, &dst_back));
    }
}
