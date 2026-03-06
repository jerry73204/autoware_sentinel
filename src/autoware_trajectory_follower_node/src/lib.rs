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

//! Trajectory follower controller node algorithm.
//!
//! Port of `autoware_trajectory_follower_node` from Autoware Universe.
//! Orchestrates MPC lateral and PID longitudinal controllers:
//! converts [`InputData`] → algorithm-specific inputs → runs both controllers
//! → syncs lateral↔longitudinal data → produces combined [`ControlOutput`].
//!
//! This is the algorithm layer. The actual ROS wiring (subscriptions,
//! publishers, timer) lives in the sentinel binary (`autoware_sentinel_linux`).

#![no_std]

#[cfg(feature = "std")]
extern crate std;

mod traj_utils;

use autoware_mpc_lateral_controller::mpc::{MpcTrajPoint, MpcTrajectory};
use autoware_mpc_lateral_controller::{self as mpc_ctrl, LateralInput, MpcLateralController};
use autoware_pid_longitudinal_controller::{
    self as pid_ctrl, ControlData, PidLongitudinalController, Shift,
};
use autoware_trajectory_follower_base::{
    InputData, LateralOutput, LateralSyncData, LongitudinalOutput, LongitudinalSyncData,
};
use autoware_vehicle_info_utils::VehicleInfo;
use traj_utils::{
    calc_curvature_at, calc_pitch, calc_stop_distance, find_nearest_index, normalize_radian,
};

/// Combined control output from both controllers.
#[derive(Debug, Clone, Copy, Default)]
pub struct ControlOutput {
    pub lateral: LateralOutput,
    pub longitudinal: LongitudinalOutput,
}

/// Controller node parameters.
#[derive(Debug, Clone)]
pub struct ControllerNodeParams {
    pub ctrl_period: f64,
    pub lateral: mpc_ctrl::ControllerParams,
    pub longitudinal: pid_ctrl::ControllerParams,
    /// Distance threshold for nearest-point search (m).
    pub ego_nearest_dist_threshold: f64,
    /// Yaw threshold for nearest-point search (rad).
    pub ego_nearest_yaw_threshold: f64,
}

impl Default for ControllerNodeParams {
    fn default() -> Self {
        Self {
            ctrl_period: 0.033,
            lateral: mpc_ctrl::ControllerParams::default(),
            longitudinal: pid_ctrl::ControllerParams::default(),
            ego_nearest_dist_threshold: 3.0,
            ego_nearest_yaw_threshold: 1.57,
        }
    }
}

/// Controller node: orchestrates lateral (MPC) and longitudinal (PID) controllers.
///
/// Call [`ControllerNode::update`] once per control cycle with the latest
/// [`InputData`] populated from ROS subscriptions.
pub struct ControllerNode {
    lateral: MpcLateralController,
    longitudinal: PidLongitudinalController,
    vehicle_info: VehicleInfo,
    params: ControllerNodeParams,
    prev_time: f64,
    lat_sync: LateralSyncData,
    lon_sync: LongitudinalSyncData,
}

impl ControllerNode {
    /// Create a new controller node.
    pub fn new(params: ControllerNodeParams, vehicle_info: VehicleInfo) -> Self {
        let lateral = MpcLateralController::new(params.lateral.clone(), &vehicle_info);
        let longitudinal =
            PidLongitudinalController::new(params.longitudinal.clone(), vehicle_info);

        Self {
            lateral,
            longitudinal,
            vehicle_info,
            params,
            prev_time: 0.0,
            lat_sync: LateralSyncData::default(),
            lon_sync: LongitudinalSyncData::default(),
        }
    }

    /// Check if the input data has enough trajectory points to run.
    pub fn is_ready(&self, input: &InputData) -> bool {
        input.trajectory_len >= 3
    }

    /// Run one control cycle.
    ///
    /// Returns `None` if input data is insufficient (< 3 trajectory points).
    pub fn update(&mut self, input: &InputData, current_time: f64) -> Option<ControlOutput> {
        if !self.is_ready(input) {
            return None;
        }

        let traj = input.trajectory_slice();

        // Find nearest trajectory point
        let nearest_idx = find_nearest_index(
            traj,
            input.current_pose_x,
            input.current_pose_y,
            input.current_pose_yaw,
            self.params.ego_nearest_dist_threshold,
            self.params.ego_nearest_yaw_threshold,
        );

        let dt = if self.prev_time > 0.0 {
            current_time - self.prev_time
        } else {
            self.params.ctrl_period
        };
        self.prev_time = current_time;

        // Sync from previous cycle
        self.longitudinal
            .set_steer_converged(self.lat_sync.is_steer_converged);

        // --- Run lateral (MPC) controller ---
        let lat_out = self.run_lateral(input, traj, nearest_idx);

        // --- Run longitudinal (PID) controller ---
        let lon_out = self.run_longitudinal(input, traj, nearest_idx, dt, current_time);

        // Store sync data for next cycle
        self.lat_sync = lat_out.sync_data;
        self.lon_sync = lon_out.sync_data;

        Some(ControlOutput {
            lateral: lat_out,
            longitudinal: lon_out,
        })
    }

    fn run_lateral(
        &mut self,
        input: &InputData,
        traj: &[autoware_trajectory_follower_base::TrajectoryPoint],
        nearest_idx: usize,
    ) -> LateralOutput {
        // Build MPC trajectory from input
        let mut mpc_traj = MpcTrajectory::new();
        let mut accumulated_time = 0.0;
        for i in 0..traj.len() {
            let curvature = calc_curvature_at(traj, i);
            let pt = &traj[i];
            let vx = if libm::fabs(pt.longitudinal_velocity_mps) > 0.01 {
                pt.longitudinal_velocity_mps
            } else {
                0.01 // avoid division by zero in MPC
            };
            if i > 0 {
                let dx = pt.x - traj[i - 1].x;
                let dy = pt.y - traj[i - 1].y;
                let dist = libm::sqrt(dx * dx + dy * dy);
                accumulated_time += dist / libm::fabs(vx);
            }
            if !mpc_traj.push(MpcTrajPoint {
                x: pt.x,
                y: pt.y,
                yaw: pt.yaw,
                vx,
                curvature,
                smooth_curvature: curvature,
                time: accumulated_time,
            }) {
                break; // trajectory full
            }
        }

        if mpc_traj.len < 3 {
            return LateralOutput::default();
        }

        // Compute lateral error and yaw error at nearest point
        let nearest_pt = &traj[nearest_idx];
        let lateral_error = {
            let dx = input.current_pose_x - nearest_pt.x;
            let dy = input.current_pose_y - nearest_pt.y;
            -libm::sin(nearest_pt.yaw) * dx + libm::cos(nearest_pt.yaw) * dy
        };
        let yaw_error = normalize_radian(input.current_pose_yaw - nearest_pt.yaw);

        let is_autonomous = input.is_autonomous && !input.is_in_transition;

        let mpc_input = LateralInput {
            trajectory: &mpc_traj,
            current_vel: input.current_velocity,
            current_steer: input.current_steer,
            lateral_error,
            yaw_error,
            is_autonomous,
        };

        let mpc_out = self.lateral.run(&mpc_input);

        LateralOutput {
            steering_tire_angle: mpc_out.steering_angle,
            steering_tire_rotation_rate: mpc_out.steering_rate,
            sync_data: LateralSyncData {
                is_steer_converged: mpc_out.is_steer_converged,
            },
        }
    }

    fn run_longitudinal(
        &mut self,
        input: &InputData,
        traj: &[autoware_trajectory_follower_base::TrajectoryPoint],
        nearest_idx: usize,
        dt: f64,
        current_time: f64,
    ) -> LongitudinalOutput {
        let nearest_pt = &traj[nearest_idx];

        let target_vel = nearest_pt.longitudinal_velocity_mps;
        let target_acc = nearest_pt.acceleration_mps2;

        let stop_dist = calc_stop_distance(
            traj,
            nearest_idx,
            input.current_pose_x,
            input.current_pose_y,
        );

        let slope_pitch = calc_pitch(traj, nearest_idx, self.vehicle_info.wheel_base_m);

        let shift = if target_vel >= 0.0 {
            Shift::Forward
        } else {
            Shift::Reverse
        };

        let is_under_control = input.is_autonomous && !input.is_in_transition;

        let control_data = ControlData {
            dt,
            current_vel: input.current_velocity,
            current_acc: input.current_accel,
            target_vel,
            target_acc,
            nearest_idx,
            stop_dist,
            shift,
            slope_pitch,
            nearest_point: pid_ctrl::geometry_point(nearest_pt.x, nearest_pt.y, nearest_pt.z),
            nearest_yaw: nearest_pt.yaw,
        };

        let pid_out = self
            .longitudinal
            .run(&control_data, current_time, is_under_control);

        LongitudinalOutput {
            velocity: pid_out.velocity,
            acceleration: pid_out.acceleration,
            sync_data: LongitudinalSyncData {},
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use autoware_trajectory_follower_base::TrajectoryPoint;

    fn make_straight_input(n: usize, vx: f64) -> InputData {
        let mut input = InputData::default();
        let dt = 0.1;
        for i in 0..n.min(input.trajectory.len()) {
            input.trajectory[i] = TrajectoryPoint {
                x: i as f64 * vx * dt,
                y: 0.0,
                z: 0.0,
                yaw: 0.0,
                longitudinal_velocity_mps: vx,
                acceleration_mps2: 0.0,
                ..TrajectoryPoint::default()
            };
        }
        input.trajectory_len = n.min(input.trajectory.len());
        input.is_autonomous = true;
        input
    }

    fn default_node() -> ControllerNode {
        let params = ControllerNodeParams {
            lateral: mpc_ctrl::ControllerParams {
                mpc_params: autoware_mpc_lateral_controller::mpc::MpcParams {
                    prediction_horizon: 10,
                    prediction_dt: 0.1,
                    ..autoware_mpc_lateral_controller::mpc::MpcParams::default()
                },
                ..mpc_ctrl::ControllerParams::default()
            },
            ..ControllerNodeParams::default()
        };
        ControllerNode::new(params, VehicleInfo::default())
    }

    #[test]
    fn test_not_ready_insufficient_points() {
        let node = default_node();
        let mut input = InputData::default();
        input.trajectory_len = 2; // need >= 3
        assert!(!node.is_ready(&input));
    }

    #[test]
    fn test_ready_with_enough_points() {
        let node = default_node();
        let input = make_straight_input(20, 5.0);
        assert!(node.is_ready(&input));
    }

    #[test]
    fn test_update_returns_none_when_not_ready() {
        let mut node = default_node();
        let input = InputData::default();
        assert!(node.update(&input, 1.0).is_none());
    }

    #[test]
    fn test_straight_line_produces_output() {
        let mut node = default_node();
        let mut input = make_straight_input(50, 5.0);
        input.current_velocity = 5.0;
        input.current_pose_x = 0.0;
        input.current_pose_y = 0.0;
        input.current_pose_yaw = 0.0;

        let out = node.update(&input, 1.0);
        assert!(out.is_some());
        let out = out.unwrap();
        // On a straight line with matching velocity, steering should be near zero
        assert!(libm::fabs(out.lateral.steering_tire_angle) < 0.1);
    }

    #[test]
    fn test_lateral_error_produces_steering() {
        let mut node = default_node();
        let mut input = make_straight_input(50, 5.0);
        input.current_velocity = 5.0;
        input.current_pose_x = 0.1;
        input.current_pose_y = 1.0; // 1m lateral offset
        input.current_pose_yaw = 0.0;

        let out = node.update(&input, 1.0);
        assert!(out.is_some());
        let out = out.unwrap();
        // Should steer to correct the lateral error
        assert!(libm::fabs(out.lateral.steering_tire_angle) > 0.001);
    }

    #[test]
    fn test_stopped_longitudinal() {
        let mut node = default_node();
        // Trajectory with stop at end
        let mut input = make_straight_input(50, 5.0);
        // Set last points to zero velocity (stop)
        for i in 10..50 {
            input.trajectory[i].longitudinal_velocity_mps = 0.0;
        }
        input.current_velocity = 0.0;
        input.current_pose_x = 0.0;

        let out = node.update(&input, 1.0);
        assert!(out.is_some());
    }

    #[test]
    fn test_sync_steer_convergence() {
        let mut node = default_node();
        let mut input = make_straight_input(50, 5.0);
        input.current_velocity = 5.0;

        // First call establishes state
        let out1 = node.update(&input, 1.0).unwrap();

        // Second call uses sync data from first
        let out2 = node.update(&input, 1.033).unwrap();

        // Both should produce valid outputs
        assert!(out1.longitudinal.velocity >= 0.0 || out1.longitudinal.velocity < 0.0); // just check it's a number
        assert!(out2.longitudinal.velocity >= 0.0 || out2.longitudinal.velocity < 0.0);
    }
}
