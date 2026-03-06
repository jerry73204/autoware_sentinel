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

//! Base traits and common types for trajectory follower controllers.
//!
//! Port of `autoware_trajectory_follower_base` from Autoware Universe.
//! Defines the interface that lateral and longitudinal controllers implement,
//! plus shared types for inter-controller synchronization.
//!
//! The controller node (10.5) converts ROS messages into [`InputData`] and
//! calls both controllers each cycle, exchanging [`LateralSyncData`] and
//! [`LongitudinalSyncData`] between them.

#![no_std]

#[cfg(feature = "std")]
extern crate std;

// ---------------------------------------------------------------------------
// Trajectory point (common representation)
// ---------------------------------------------------------------------------

/// Maximum number of trajectory points.
pub const MAX_TRAJECTORY_POINTS: usize = 256;

/// Lightweight trajectory point used as the common interface between
/// ROS message types and algorithm-specific representations.
///
/// The controller node extracts these from `autoware_planning_msgs::msg::TrajectoryPoint`.
#[derive(Debug, Clone, Copy, Default)]
pub struct TrajectoryPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Orientation as yaw angle (radians). Extracted from quaternion by the node.
    pub yaw: f64,
    pub longitudinal_velocity_mps: f64,
    pub lateral_velocity_mps: f64,
    pub acceleration_mps2: f64,
    pub heading_rate_rps: f64,
    pub front_wheel_angle_rad: f64,
}

// ---------------------------------------------------------------------------
// Input data
// ---------------------------------------------------------------------------

/// Unified input data for both controllers, extracted from ROS subscriptions.
///
/// The controller node populates this from:
/// - `/planning/scenario_planning/trajectory` → `trajectory`, `trajectory_len`
/// - `/localization/kinematic_state` (Odometry) → `current_pose_*`, `current_velocity`
/// - `/vehicle/status/steering_status` → `current_steer`
/// - `/localization/acceleration` → `current_accel`
/// - `/system/operation_mode/state` → `is_autonomous`, `is_in_transition`
pub struct InputData {
    /// Trajectory points buffer.
    pub trajectory: [TrajectoryPoint; MAX_TRAJECTORY_POINTS],
    /// Number of valid trajectory points.
    pub trajectory_len: usize,

    // Ego pose (from Odometry)
    pub current_pose_x: f64,
    pub current_pose_y: f64,
    pub current_pose_z: f64,
    pub current_pose_yaw: f64,
    pub current_pose_pitch: f64,

    // Ego velocity (from Odometry twist.twist.linear.x)
    pub current_velocity: f64,

    // Steering angle (from SteeringReport)
    pub current_steer: f64,

    // Longitudinal acceleration (from AccelWithCovarianceStamped)
    pub current_accel: f64,

    // Operation mode
    pub is_autonomous: bool,
    pub is_in_transition: bool,
}

impl InputData {
    /// Get a slice of the valid trajectory points.
    pub fn trajectory_slice(&self) -> &[TrajectoryPoint] {
        &self.trajectory[..self.trajectory_len]
    }
}

impl Default for InputData {
    fn default() -> Self {
        Self {
            trajectory: [TrajectoryPoint::default(); MAX_TRAJECTORY_POINTS],
            trajectory_len: 0,
            current_pose_x: 0.0,
            current_pose_y: 0.0,
            current_pose_z: 0.0,
            current_pose_yaw: 0.0,
            current_pose_pitch: 0.0,
            current_velocity: 0.0,
            current_steer: 0.0,
            current_accel: 0.0,
            is_autonomous: false,
            is_in_transition: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Sync data
// ---------------------------------------------------------------------------

/// Data sent from the lateral controller to the longitudinal controller.
///
/// The longitudinal controller uses `is_steer_converged` to decide whether
/// it is safe to depart from a stop (prevents jerky starts with large steer).
#[derive(Debug, Clone, Copy, Default)]
pub struct LateralSyncData {
    pub is_steer_converged: bool,
}

/// Data sent from the longitudinal controller to the lateral controller.
///
/// Currently unused — reserved for future velocity convergence signaling.
#[derive(Debug, Clone, Copy, Default)]
pub struct LongitudinalSyncData {}

// ---------------------------------------------------------------------------
// Controller outputs
// ---------------------------------------------------------------------------

/// Output from the lateral controller.
///
/// Maps to `autoware_control_msgs::msg::Lateral`:
/// - `steering_tire_angle` → `Lateral.steering_tire_angle`
/// - `steering_tire_rotation_rate` → `Lateral.steering_tire_rotation_rate`
#[derive(Debug, Clone, Copy, Default)]
pub struct LateralOutput {
    pub steering_tire_angle: f64,
    pub steering_tire_rotation_rate: f64,
    pub sync_data: LateralSyncData,
}

/// Output from the longitudinal controller.
///
/// Maps to `autoware_control_msgs::msg::Longitudinal`:
/// - `velocity` → `Longitudinal.velocity`
/// - `acceleration` → `Longitudinal.acceleration`
#[derive(Debug, Clone, Copy, Default)]
pub struct LongitudinalOutput {
    pub velocity: f64,
    pub acceleration: f64,
    pub sync_data: LongitudinalSyncData,
}

// ---------------------------------------------------------------------------
// Controller traits
// ---------------------------------------------------------------------------

/// Trait for lateral (steering) controllers.
///
/// Implementations: [`autoware_mpc_lateral_controller::MpcLateralController`]
pub trait LateralControllerBase {
    /// Check if the controller has received enough data to run.
    fn is_ready(&self, input: &InputData) -> bool;

    /// Compute the lateral control output for the current cycle.
    fn run(&mut self, input: &InputData) -> LateralOutput;

    /// Receive sync data from the longitudinal controller.
    fn sync(&mut self, data: &LongitudinalSyncData);
}

/// Trait for longitudinal (velocity/acceleration) controllers.
///
/// Implementations: [`autoware_pid_longitudinal_controller::PidLongitudinalController`]
pub trait LongitudinalControllerBase {
    /// Check if the controller has received enough data to run.
    fn is_ready(&self, input: &InputData) -> bool;

    /// Compute the longitudinal control output for the current cycle.
    fn run(&mut self, input: &InputData) -> LongitudinalOutput;

    /// Receive sync data from the lateral controller.
    fn sync(&mut self, data: &LateralSyncData);

    /// Reset sync state (called when trajectory is replanned).
    fn reset(&mut self);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_input_data_default() {
        let input = InputData::default();
        assert_eq!(input.trajectory_len, 0);
        assert_eq!(input.trajectory_slice().len(), 0);
        assert!(!input.is_autonomous);
    }

    #[test]
    fn test_input_data_trajectory_slice() {
        let mut input = InputData::default();
        input.trajectory[0] = TrajectoryPoint {
            x: 1.0,
            y: 2.0,
            longitudinal_velocity_mps: 5.0,
            ..TrajectoryPoint::default()
        };
        input.trajectory[1] = TrajectoryPoint {
            x: 3.0,
            y: 4.0,
            longitudinal_velocity_mps: 10.0,
            ..TrajectoryPoint::default()
        };
        input.trajectory_len = 2;

        let slice = input.trajectory_slice();
        assert_eq!(slice.len(), 2);
        assert!((slice[0].x - 1.0).abs() < 1e-10);
        assert!((slice[1].longitudinal_velocity_mps - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_lateral_output_default() {
        let out = LateralOutput::default();
        assert!((out.steering_tire_angle).abs() < 1e-10);
        assert!(!out.sync_data.is_steer_converged);
    }

    #[test]
    fn test_longitudinal_output_default() {
        let out = LongitudinalOutput::default();
        assert!((out.velocity).abs() < 1e-10);
        assert!((out.acceleration).abs() < 1e-10);
    }

    #[test]
    fn test_sync_data_defaults() {
        let lat_sync = LateralSyncData::default();
        assert!(!lat_sync.is_steer_converged);

        let _lon_sync = LongitudinalSyncData::default();
    }

    #[test]
    fn test_trajectory_point_default() {
        let pt = TrajectoryPoint::default();
        assert!((pt.x).abs() < 1e-10);
        assert!((pt.yaw).abs() < 1e-10);
        assert!((pt.longitudinal_velocity_mps).abs() < 1e-10);
    }
}
