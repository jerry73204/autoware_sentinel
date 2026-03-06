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

//! Trajectory geometry utilities operating on [`TrajectoryPoint`] slices.
//!
//! Lightweight reimplementations of `autoware_motion_utils` functions that
//! work directly on the base crate's [`TrajectoryPoint`] type, avoiding
//! message crate dependencies (`geometry_msgs::msg::Point`).

use autoware_trajectory_follower_base::TrajectoryPoint;

const PI: f64 = core::f64::consts::PI;

/// Normalize an angle to [-pi, pi).
pub fn normalize_radian(angle: f64) -> f64 {
    let mut a = angle;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Distance between two trajectory points in the XY plane.
fn dist_2d(a: &TrajectoryPoint, b: &TrajectoryPoint) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    libm::sqrt(dx * dx + dy * dy)
}

/// Find the nearest trajectory point index with distance + yaw constraints.
///
/// Uses a 3-pass soft-constraint search (matching `autoware_motion_utils`):
/// 1. Both distance and yaw constraints
/// 2. Distance constraint only
/// 3. No constraints (pure nearest)
pub fn find_nearest_index(
    traj: &[TrajectoryPoint],
    ego_x: f64,
    ego_y: f64,
    ego_yaw: f64,
    dist_threshold: f64,
    yaw_threshold: f64,
) -> usize {
    if traj.is_empty() {
        return 0;
    }

    // Pass 1: both constraints
    let mut best_idx = None;
    let mut best_dist = f64::MAX;
    for (i, pt) in traj.iter().enumerate() {
        let dx = pt.x - ego_x;
        let dy = pt.y - ego_y;
        let d2 = dx * dx + dy * dy;
        let d = libm::sqrt(d2);
        let yaw_diff = libm::fabs(normalize_radian(pt.yaw - ego_yaw));
        if d <= dist_threshold && yaw_diff <= yaw_threshold && d2 < best_dist {
            best_dist = d2;
            best_idx = Some(i);
        }
    }
    if let Some(idx) = best_idx {
        return idx;
    }

    // Pass 2: distance only
    best_dist = f64::MAX;
    for (i, pt) in traj.iter().enumerate() {
        let dx = pt.x - ego_x;
        let dy = pt.y - ego_y;
        let d2 = dx * dx + dy * dy;
        let d = libm::sqrt(d2);
        if d <= dist_threshold && d2 < best_dist {
            best_dist = d2;
            best_idx = Some(i);
        }
    }
    if let Some(idx) = best_idx {
        return idx;
    }

    // Pass 3: pure nearest
    best_dist = f64::MAX;
    let mut result = 0;
    for (i, pt) in traj.iter().enumerate() {
        let dx = pt.x - ego_x;
        let dy = pt.y - ego_y;
        let d2 = dx * dx + dy * dy;
        if d2 < best_dist {
            best_dist = d2;
            result = i;
        }
    }
    result
}

/// Calculate signed arc length from `from_idx` to `to_idx` along the trajectory.
fn signed_arc_length(traj: &[TrajectoryPoint], from_idx: usize, to_idx: usize) -> f64 {
    if from_idx == to_idx {
        return 0.0;
    }
    let (lo, hi, sign) = if from_idx < to_idx {
        (from_idx, to_idx, 1.0)
    } else {
        (to_idx, from_idx, -1.0)
    };
    let mut length = 0.0;
    for i in lo..hi {
        length += dist_2d(&traj[i], &traj[i + 1]);
    }
    sign * length
}

/// Calculate the distance from the ego position to the first zero-velocity
/// point on the trajectory. Returns a large positive value if no stop point exists.
pub fn calc_stop_distance(
    traj: &[TrajectoryPoint],
    nearest_idx: usize,
    ego_x: f64,
    ego_y: f64,
) -> f64 {
    if traj.is_empty() {
        return 0.0;
    }

    // Find first zero-velocity point
    let stop_idx = traj
        .iter()
        .position(|pt| libm::fabs(pt.longitudinal_velocity_mps) < 1e-6);

    match stop_idx {
        Some(idx) => {
            // Arc length from ego's nearest segment to stop point
            let arc = signed_arc_length(traj, nearest_idx, idx);

            // Offset for projection onto the nearest segment
            let offset = if nearest_idx < traj.len().saturating_sub(1) {
                let seg = &traj[nearest_idx];
                let seg_next = &traj[nearest_idx + 1];
                let sx = seg_next.x - seg.x;
                let sy = seg_next.y - seg.y;
                let seg_len = libm::sqrt(sx * sx + sy * sy);
                if seg_len > 1e-9 {
                    let dx = ego_x - seg.x;
                    let dy = ego_y - seg.y;
                    let dot = dx * sx + dy * sy;
                    dot / seg_len
                } else {
                    0.0
                }
            } else {
                0.0
            };

            arc - offset
        }
        None => {
            // No stop point — return distance to trajectory end
            let last = traj.len() - 1;
            signed_arc_length(traj, nearest_idx, last) + 100.0
        }
    }
}

/// Compute pitch angle from trajectory elevation change over `wheel_base` distance.
pub fn calc_pitch(traj: &[TrajectoryPoint], start_idx: usize, wheel_base: f64) -> f64 {
    if traj.len() < 2 || start_idx >= traj.len() - 1 {
        return 0.0;
    }

    let mut accumulated = 0.0;
    let mut end_idx = start_idx;
    for i in start_idx..traj.len() - 1 {
        accumulated += dist_2d(&traj[i], &traj[i + 1]);
        end_idx = i + 1;
        if accumulated >= wheel_base {
            break;
        }
    }

    if end_idx == start_idx {
        return 0.0;
    }

    let p0 = &traj[start_idx];
    let p1 = &traj[end_idx];
    let dx = p1.x - p0.x;
    let dy = p1.y - p0.y;
    let dz = p1.z - p0.z;
    let horiz = libm::sqrt(dx * dx + dy * dy);
    if horiz < 1e-9 {
        return 0.0;
    }
    libm::atan2(dz, horiz)
}

/// Compute curvature at a trajectory point using the Menger formula.
///
/// Uses the three-point circumradius method. At endpoints, uses the
/// adjacent interior triplet.
pub fn calc_curvature_at(traj: &[TrajectoryPoint], idx: usize) -> f64 {
    if traj.len() < 3 {
        return 0.0;
    }
    let (i0, i1, i2) = if idx == 0 {
        (0, 1, 2)
    } else if idx >= traj.len() - 1 {
        (traj.len() - 3, traj.len() - 2, traj.len() - 1)
    } else {
        (idx - 1, idx, idx + 1)
    };

    let p0 = &traj[i0];
    let p1 = &traj[i1];
    let p2 = &traj[i2];

    let a = dist_2d(p0, p1);
    let b = dist_2d(p1, p2);
    let c = dist_2d(p0, p2);

    // Area of triangle (cross product magnitude / 2)
    let cross = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
    let area = libm::fabs(cross) / 2.0;

    let denom = a * b * c;
    if denom < 1e-12 {
        return 0.0;
    }

    // Signed curvature: positive = left turn
    let sign = if cross >= 0.0 { 1.0 } else { -1.0 };
    sign * 4.0 * area / denom
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64) -> TrajectoryPoint {
        TrajectoryPoint {
            x,
            y,
            yaw: 0.0,
            longitudinal_velocity_mps: 5.0,
            ..TrajectoryPoint::default()
        }
    }

    #[test]
    fn test_normalize_radian() {
        assert!((normalize_radian(0.0)).abs() < 1e-10);
        assert!(
            (normalize_radian(PI) - PI).abs() < 1e-10 || (normalize_radian(PI) + PI).abs() < 1e-10
        );
        assert!(
            (normalize_radian(3.0 * PI) - PI).abs() < 1e-10
                || (normalize_radian(3.0 * PI) + PI).abs() < 1e-10
        );
    }

    #[test]
    fn test_find_nearest_simple() {
        let traj = [pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0), pt(3.0, 0.0)];
        assert_eq!(find_nearest_index(&traj, 0.9, 0.0, 0.0, 3.0, 1.57), 1);
        assert_eq!(find_nearest_index(&traj, 2.8, 0.0, 0.0, 3.0, 1.57), 3);
    }

    #[test]
    fn test_calc_stop_distance_basic() {
        let mut traj = [pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0), pt(3.0, 0.0)];
        // Last point is stop
        traj[3].longitudinal_velocity_mps = 0.0;

        let d = calc_stop_distance(&traj, 0, 0.0, 0.0);
        assert!((d - 3.0).abs() < 0.1);
    }

    #[test]
    fn test_calc_pitch_flat() {
        let traj = [pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0), pt(3.0, 0.0)];
        let pitch = calc_pitch(&traj, 0, 2.7);
        assert!(pitch.abs() < 1e-10);
    }

    #[test]
    fn test_calc_pitch_incline() {
        let traj = [
            TrajectoryPoint {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                ..TrajectoryPoint::default()
            },
            TrajectoryPoint {
                x: 1.0,
                y: 0.0,
                z: 0.0,
                ..TrajectoryPoint::default()
            },
            TrajectoryPoint {
                x: 2.0,
                y: 0.0,
                z: 1.0,
                ..TrajectoryPoint::default()
            },
            TrajectoryPoint {
                x: 3.0,
                y: 0.0,
                z: 2.0,
                ..TrajectoryPoint::default()
            },
        ];
        let pitch = calc_pitch(&traj, 0, 2.7);
        assert!(pitch > 0.0); // uphill
    }

    #[test]
    fn test_calc_curvature_straight() {
        let traj = [pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0)];
        let k = calc_curvature_at(&traj, 1);
        assert!(k.abs() < 1e-10);
    }

    #[test]
    fn test_calc_curvature_circle() {
        // Points on a unit circle (radius=1, curvature=1)
        let traj = [
            TrajectoryPoint {
                x: 1.0,
                y: 0.0,
                ..TrajectoryPoint::default()
            },
            TrajectoryPoint {
                x: libm::cos(PI / 6.0),
                y: libm::sin(PI / 6.0),
                ..TrajectoryPoint::default()
            },
            TrajectoryPoint {
                x: libm::cos(PI / 3.0),
                y: libm::sin(PI / 3.0),
                ..TrajectoryPoint::default()
            },
        ];
        let k = calc_curvature_at(&traj, 1);
        assert!((k - 1.0).abs() < 0.01);
    }
}
