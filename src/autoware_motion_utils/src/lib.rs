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

//! Port of autoware_motion_utils from Autoware Universe.
//!
//! Trajectory utilities: nearest point search, curvature calculation,
//! arc length computation, overlap removal, and zero-velocity search.
//!
//! The C++ original uses templates generic over any trajectory-like container.
//! This port uses the [`TrajectoryAccessor`] trait to achieve the same
//! generality in `#![no_std]` Rust.

#![no_std]

#[cfg(feature = "std")]
extern crate std;

use autoware_universe_utils::{
    calc_distance_2d, calc_squared_distance_2d, calc_yaw_deviation, get_yaw,
};
use geometry_msgs::msg::Point;

/// Zero velocity threshold in m/s (same as C++: 1e-3).
const ZERO_VELOCITY_EPS: f64 = 1e-3;

// ---------------------------------------------------------------------------
// Trait for trajectory point access
// ---------------------------------------------------------------------------

/// Trait abstracting access to trajectory point data.
///
/// Replaces the C++ template `<class T>` pattern. Implementations exist for
/// `autoware_planning_msgs::msg::TrajectoryPoint` and for test-only types.
pub trait TrajectoryAccessor {
    /// Get the position (Point) of the trajectory point at `index`.
    fn point_at(&self, index: usize) -> &Point;

    /// Get the yaw angle at `index` (from the pose orientation).
    fn yaw_at(&self, index: usize) -> f64;

    /// Get the longitudinal velocity at `index` (m/s).
    fn velocity_at(&self, index: usize) -> f32;

    /// Number of points.
    fn len(&self) -> usize;

    /// Whether the container is empty.
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// Implement TrajectoryAccessor for a slice of TrajectoryPoint messages.
impl TrajectoryAccessor for [autoware_planning_msgs::msg::TrajectoryPoint] {
    fn point_at(&self, index: usize) -> &Point {
        &self[index].pose.position
    }

    fn yaw_at(&self, index: usize) -> f64 {
        get_yaw(&self[index].pose.orientation)
    }

    fn velocity_at(&self, index: usize) -> f32 {
        self[index].longitudinal_velocity_mps
    }

    fn len(&self) -> usize {
        <[autoware_planning_msgs::msg::TrajectoryPoint]>::len(self)
    }
}

// ---------------------------------------------------------------------------
// Nearest index search
// ---------------------------------------------------------------------------

/// Find the index of the nearest point by 2D squared distance.
///
/// Returns `None` if `points` is empty.
pub fn find_nearest_index(points: &dyn TrajectoryAccessor, target: &Point) -> Option<usize> {
    if points.is_empty() {
        return None;
    }
    let mut min_dist = f64::MAX;
    let mut min_idx = 0;
    for i in 0..points.len() {
        let d = calc_squared_distance_2d(points.point_at(i), target);
        if d < min_dist {
            min_dist = d;
            min_idx = i;
        }
    }
    Some(min_idx)
}

/// Find the nearest index with distance and yaw constraints.
///
/// Returns `None` if no point satisfies both constraints, or if `points` is empty.
pub fn find_nearest_index_with_constraints(
    points: &dyn TrajectoryAccessor,
    target: &Point,
    target_yaw: f64,
    max_dist: f64,
    max_yaw: f64,
) -> Option<usize> {
    if points.is_empty() {
        return None;
    }
    let max_sq_dist = max_dist * max_dist;
    let mut min_sq_dist = f64::MAX;
    let mut min_idx = None;

    for i in 0..points.len() {
        let sq_dist = calc_squared_distance_2d(points.point_at(i), target);
        if sq_dist > max_sq_dist || sq_dist >= min_sq_dist {
            continue;
        }
        let yaw_dev = calc_yaw_deviation(points.yaw_at(i), target_yaw);
        if libm::fabs(yaw_dev) > max_yaw {
            continue;
        }
        min_sq_dist = sq_dist;
        min_idx = Some(i);
    }
    min_idx
}

/// Find first nearest index with soft distance and yaw constraints.
///
/// Cascading search: first tries with both dist+yaw thresholds, then dist only,
/// then falls back to unconstrained nearest. Returns `None` only if empty.
pub fn find_first_nearest_index_with_soft_constraints(
    points: &dyn TrajectoryAccessor,
    target: &Point,
    target_yaw: f64,
    dist_threshold: f64,
    yaw_threshold: f64,
) -> Option<usize> {
    if points.is_empty() {
        return None;
    }

    let sq_dist_threshold = dist_threshold * dist_threshold;

    // Pass 1: both distance and yaw constraints
    {
        let mut min_sq_dist = f64::MAX;
        let mut min_idx = None;
        let mut in_constraints = false;
        for i in 0..points.len() {
            let sq_dist = calc_squared_distance_2d(points.point_at(i), target);
            let yaw_dev = calc_yaw_deviation(points.yaw_at(i), target_yaw);
            if sq_dist_threshold < sq_dist || yaw_threshold < libm::fabs(yaw_dev) {
                if in_constraints {
                    break;
                }
                continue;
            }
            if min_sq_dist <= sq_dist {
                continue;
            }
            min_sq_dist = sq_dist;
            min_idx = Some(i);
            in_constraints = true;
        }
        if let Some(idx) = min_idx {
            return Some(idx);
        }
    }

    // Pass 2: distance constraint only
    {
        let mut min_sq_dist = f64::MAX;
        let mut min_idx = None;
        let mut in_constraints = false;
        for i in 0..points.len() {
            let sq_dist = calc_squared_distance_2d(points.point_at(i), target);
            if sq_dist_threshold < sq_dist {
                if in_constraints {
                    break;
                }
                continue;
            }
            if min_sq_dist <= sq_dist {
                continue;
            }
            min_sq_dist = sq_dist;
            min_idx = Some(i);
            in_constraints = true;
        }
        if let Some(idx) = min_idx {
            return Some(idx);
        }
    }

    // Pass 3: unconstrained
    find_nearest_index(points, target)
}

/// Find nearest segment index with soft constraints.
///
/// A segment is the straight path between two consecutive points.
/// When the pose is closest to point `i`, returns `i-1` if the longitudinal
/// projection onto segment `i` is negative, otherwise returns `i`.
pub fn find_first_nearest_segment_index_with_soft_constraints(
    points: &dyn TrajectoryAccessor,
    target: &Point,
    target_yaw: f64,
    dist_threshold: f64,
    yaw_threshold: f64,
) -> Option<usize> {
    let nearest_idx = find_first_nearest_index_with_soft_constraints(
        points,
        target,
        target_yaw,
        dist_threshold,
        yaw_threshold,
    )?;

    if nearest_idx == 0 {
        return Some(0);
    }
    if nearest_idx >= points.len() - 1 {
        return Some(points.len() - 2);
    }

    let offset = calc_longitudinal_offset_to_segment(points, nearest_idx, target);
    if offset <= 0.0 {
        Some(nearest_idx - 1)
    } else {
        Some(nearest_idx)
    }
}

// ---------------------------------------------------------------------------
// Arc length
// ---------------------------------------------------------------------------

/// Calculate signed arc length between two indices along the trajectory.
///
/// Positive if `dst_idx > src_idx`, negative otherwise.
pub fn calc_signed_arc_length(
    points: &dyn TrajectoryAccessor,
    src_idx: usize,
    dst_idx: usize,
) -> f64 {
    if points.is_empty() {
        return 0.0;
    }
    if src_idx > dst_idx {
        return -calc_signed_arc_length(points, dst_idx, src_idx);
    }
    let mut dist = 0.0;
    for i in src_idx..dst_idx {
        dist += calc_distance_2d(points.point_at(i), points.point_at(i + 1));
    }
    dist
}

/// Calculate signed arc length between two points with segment offsets.
pub fn calc_signed_arc_length_between_points(
    points: &dyn TrajectoryAccessor,
    src_point: &Point,
    src_seg_idx: usize,
    dst_point: &Point,
    dst_seg_idx: usize,
) -> f64 {
    let on_traj = calc_signed_arc_length(points, src_seg_idx, dst_seg_idx);
    let src_offset = calc_longitudinal_offset_to_segment(points, src_seg_idx, src_point);
    let dst_offset = calc_longitudinal_offset_to_segment(points, dst_seg_idx, dst_point);
    on_traj - src_offset + dst_offset
}

// ---------------------------------------------------------------------------
// Longitudinal offset to segment
// ---------------------------------------------------------------------------

/// Calculate the longitudinal offset from segment start to the projection of
/// `target` onto the segment `[points[seg_idx], points[seg_idx+1]]`.
///
/// Returns `NaN` if `seg_idx` is out of bounds.
pub fn calc_longitudinal_offset_to_segment(
    points: &dyn TrajectoryAccessor,
    seg_idx: usize,
    target: &Point,
) -> f64 {
    if seg_idx + 1 >= points.len() {
        return f64::NAN;
    }
    let p_front = points.point_at(seg_idx);
    let p_back = points.point_at(seg_idx + 1);

    let seg_x = p_back.x - p_front.x;
    let seg_y = p_back.y - p_front.y;
    let tgt_x = target.x - p_front.x;
    let tgt_y = target.y - p_front.y;

    let seg_len = libm::sqrt(seg_x * seg_x + seg_y * seg_y);
    if seg_len < 1e-10 {
        return 0.0;
    }

    (seg_x * tgt_x + seg_y * tgt_y) / seg_len
}

// ---------------------------------------------------------------------------
// Curvature
// ---------------------------------------------------------------------------

/// Calculate Menger curvature at each trajectory point.
///
/// Uses three consecutive points. Endpoints copy the adjacent interior value.
/// Results are written to `output`; returns the number written.
pub fn calc_curvature(points: &dyn TrajectoryAccessor, output: &mut [f64]) -> usize {
    let n = points.len().min(output.len());
    if n < 3 {
        for v in output[..n].iter_mut() {
            *v = 0.0;
        }
        return n;
    }

    output[0] = 0.0;
    for i in 1..n - 1 {
        output[i] = autoware_universe_utils::calc_curvature(
            points.point_at(i - 1),
            points.point_at(i),
            points.point_at(i + 1),
        );
    }
    output[n - 1] = 0.0;

    // Copy adjacent values to endpoints
    output[0] = output[1];
    output[n - 1] = output[n - 2];
    n
}

// ---------------------------------------------------------------------------
// Zero velocity search
// ---------------------------------------------------------------------------

/// Search for the first zero-velocity point in range `[src_idx, dst_idx)`.
pub fn search_zero_velocity_index(
    points: &dyn TrajectoryAccessor,
    src_idx: usize,
    dst_idx: usize,
) -> Option<usize> {
    if points.is_empty() {
        return None;
    }
    let end = dst_idx.min(points.len());
    for i in src_idx..end {
        if libm::fabsf(points.velocity_at(i)) < ZERO_VELOCITY_EPS as f32 {
            return Some(i);
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Driving direction
// ---------------------------------------------------------------------------

/// Check if the trajectory starts in the forward driving direction.
///
/// Compares the heading of the first point with the direction to the second point.
/// Returns `None` if fewer than 2 points.
pub fn is_driving_forward(points: &dyn TrajectoryAccessor) -> Option<bool> {
    if points.len() < 2 {
        return None;
    }
    let src = points.point_at(0);
    let dst = points.point_at(1);
    let src_yaw = points.yaw_at(0);
    Some(autoware_universe_utils::is_driving_forward(
        src_yaw, src, dst,
    ))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple test trajectory point for testing without message dependencies.
    struct TestPoint {
        pos: Point,
        yaw: f64,
        vel: f32,
    }

    /// A simple trajectory backed by a Vec.
    struct TestTrajectory {
        points: std::vec::Vec<TestPoint>,
    }

    impl TrajectoryAccessor for TestTrajectory {
        fn point_at(&self, index: usize) -> &Point {
            &self.points[index].pos
        }
        fn yaw_at(&self, index: usize) -> f64 {
            self.points[index].yaw
        }
        fn velocity_at(&self, index: usize) -> f32 {
            self.points[index].vel
        }
        fn len(&self) -> usize {
            self.points.len()
        }
    }

    extern crate std;

    fn make_straight_trajectory(n: usize, spacing: f64, vel: f32) -> TestTrajectory {
        let points: std::vec::Vec<_> = (0..n)
            .map(|i| TestPoint {
                pos: Point {
                    x: i as f64 * spacing,
                    y: 0.0,
                    z: 0.0,
                },
                yaw: 0.0,
                vel,
            })
            .collect();
        TestTrajectory { points }
    }

    #[test]
    fn test_find_nearest_index() {
        let traj = make_straight_trajectory(5, 1.0, 1.0);
        let target = Point {
            x: 2.3,
            y: 0.1,
            z: 0.0,
        };
        assert_eq!(find_nearest_index(&traj, &target), Some(2));
    }

    #[test]
    fn test_find_nearest_index_empty() {
        let traj = TestTrajectory {
            points: std::vec![],
        };
        let target = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        assert_eq!(find_nearest_index(&traj, &target), None);
    }

    #[test]
    fn test_signed_arc_length() {
        let traj = make_straight_trajectory(5, 1.0, 1.0);
        let len = calc_signed_arc_length(&traj, 0, 4);
        assert!((len - 4.0).abs() < 1e-10);
        let neg = calc_signed_arc_length(&traj, 4, 0);
        assert!((neg + 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_longitudinal_offset() {
        let traj = make_straight_trajectory(3, 1.0, 1.0);
        let target = Point {
            x: 0.5,
            y: 0.0,
            z: 0.0,
        };
        let offset = calc_longitudinal_offset_to_segment(&traj, 0, &target);
        assert!((offset - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_curvature_straight() {
        let traj = make_straight_trajectory(5, 1.0, 1.0);
        let mut output = [0.0; 5];
        calc_curvature(&traj, &mut output);
        for k in output.iter() {
            assert!(k.abs() < 1e-10);
        }
    }

    #[test]
    fn test_search_zero_velocity() {
        let mut traj = make_straight_trajectory(5, 1.0, 1.0);
        traj.points[3].vel = 0.0;
        assert_eq!(search_zero_velocity_index(&traj, 0, 5), Some(3));
        assert_eq!(search_zero_velocity_index(&traj, 0, 3), None);
    }

    #[test]
    fn test_is_driving_forward() {
        let traj = make_straight_trajectory(3, 1.0, 1.0);
        assert_eq!(is_driving_forward(&traj), Some(true));

        // Yaw = PI means facing backward, but trajectory goes forward → not forward
        let mut backward = make_straight_trajectory(3, 1.0, 1.0);
        backward.points[0].yaw = core::f64::consts::PI;
        assert_eq!(is_driving_forward(&backward), Some(false));
    }

    #[test]
    fn test_is_driving_forward_too_few_points() {
        let traj = make_straight_trajectory(1, 1.0, 1.0);
        assert_eq!(is_driving_forward(&traj), None);
    }
}
