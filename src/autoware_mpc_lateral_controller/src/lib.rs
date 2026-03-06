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

//! MPC lateral controller for Autoware trajectory following.
//!
//! Port of `autoware_mpc_lateral_controller` from Autoware Universe.
//! Implements model predictive control for steering angle computation
//! using bicycle vehicle models and unconstrained QP optimization.
//!
//! # Memory
//!
//! The [`MpcWorkspace`] struct is ~200KB due to prediction matrix buffers.
//! For embedded targets with limited RAM, reduce [`mpc::MAX_HORIZON`].

#![no_std]

#[cfg(feature = "std")]
extern crate std;

pub mod mat;
pub mod mpc;
pub mod vehicle_model;

use autoware_vehicle_info_utils::VehicleInfo;
use mpc::{MpcParams, MpcTrajectory, MpcWorkspace, calculate_mpc};
use vehicle_model::{VehicleModel, VehicleModelType};

/// Controller parameters.
#[derive(Debug, Clone)]
pub struct ControllerParams {
    pub mpc_params: MpcParams,
    pub vehicle_model_type: VehicleModelType,
    pub ctrl_period: f64,
    pub admissible_position_error: f64,
    pub admissible_yaw_error_rad: f64,
    pub stop_state_entry_ego_speed: f64,
    pub stop_state_entry_target_speed: f64,
    pub converged_steer_rad: f64,
    pub keep_steer_control_until_converged: bool,
    // Low-pass filter for output steering
    pub steering_lpf_gain: f64,
}

impl Default for ControllerParams {
    fn default() -> Self {
        Self {
            mpc_params: MpcParams::default(),
            vehicle_model_type: VehicleModelType::Kinematics,
            ctrl_period: 0.033,
            admissible_position_error: 5.0,
            admissible_yaw_error_rad: 1.57,
            stop_state_entry_ego_speed: 0.001,
            stop_state_entry_target_speed: 0.001,
            converged_steer_rad: 0.1,
            keep_steer_control_until_converged: true,
            steering_lpf_gain: 0.8,
        }
    }
}

/// Lateral controller output.
#[derive(Debug, Clone, Copy, Default)]
pub struct LateralOutput {
    pub steering_angle: f64,
    pub steering_rate: f64,
    pub is_steer_converged: bool,
}

/// Input data for the lateral controller.
pub struct LateralInput<'a> {
    pub trajectory: &'a MpcTrajectory,
    pub current_vel: f64,
    pub current_steer: f64,
    pub lateral_error: f64,
    pub yaw_error: f64,
    pub is_autonomous: bool,
}

/// MPC lateral controller.
///
/// Combines vehicle model, MPC solver, and output filtering.
pub struct MpcLateralController {
    params: ControllerParams,
    vehicle_model: VehicleModel,
    workspace: MpcWorkspace,
    prev_steer_cmd: f64,
    filtered_steer: f64,
    is_initialized: bool,
}

impl MpcLateralController {
    /// Create a new controller.
    pub fn new(params: ControllerParams, vehicle_info: &VehicleInfo) -> Self {
        let vehicle_model = match params.vehicle_model_type {
            VehicleModelType::Kinematics => VehicleModel::new_kinematics(
                vehicle_info.wheel_base_m,
                vehicle_info.max_steer_angle_rad,
                params.mpc_params.steer_tau,
            ),
            VehicleModelType::KinematicsNoDelay => VehicleModel::new_kinematics_no_delay(
                vehicle_info.wheel_base_m,
                vehicle_info.max_steer_angle_rad,
            ),
            VehicleModelType::Dynamics => {
                // Use default mass distribution for dynamics model
                let total_mass = 1700.0;
                let front_ratio = 0.5;
                let mass_fl = total_mass * front_ratio / 2.0;
                let mass_fr = mass_fl;
                let mass_rl = total_mass * (1.0 - front_ratio) / 2.0;
                let mass_rr = mass_rl;
                VehicleModel::new_dynamics(
                    vehicle_info.wheel_base_m,
                    vehicle_info.max_steer_angle_rad,
                    mass_fl,
                    mass_fr,
                    mass_rl,
                    mass_rr,
                    150000.0, // cornering stiffness front
                    150000.0, // cornering stiffness rear
                )
            }
        };

        Self {
            params,
            vehicle_model,
            workspace: MpcWorkspace::new(),
            prev_steer_cmd: 0.0,
            filtered_steer: 0.0,
            is_initialized: false,
        }
    }

    /// Get the steer limit from the vehicle model.
    pub fn steer_limit(&self) -> f64 {
        self.vehicle_model.steer_lim
    }

    /// Run the lateral controller.
    pub fn run(&mut self, input: &LateralInput) -> LateralOutput {
        if !input.is_autonomous {
            self.is_initialized = false;
            return LateralOutput {
                steering_angle: input.current_steer,
                steering_rate: 0.0,
                is_steer_converged: false,
            };
        }

        // Check admissibility
        if libm::fabs(input.lateral_error) > self.params.admissible_position_error
            || libm::fabs(input.yaw_error) > self.params.admissible_yaw_error_rad
        {
            return LateralOutput {
                steering_angle: self.prev_steer_cmd,
                steering_rate: 0.0,
                is_steer_converged: false,
            };
        }

        // Check stop state
        let is_stopped = libm::fabs(input.current_vel) < self.params.stop_state_entry_ego_speed;

        if is_stopped {
            let steer_converged = libm::fabs(input.current_steer - self.prev_steer_cmd)
                < self.params.converged_steer_rad;

            if self.params.keep_steer_control_until_converged && !steer_converged {
                // Keep previous command to converge
                return LateralOutput {
                    steering_angle: self.prev_steer_cmd,
                    steering_rate: 0.0,
                    is_steer_converged: false,
                };
            }

            return LateralOutput {
                steering_angle: self.prev_steer_cmd,
                steering_rate: 0.0,
                is_steer_converged: steer_converged,
            };
        }

        // Build initial state
        let dx = self.vehicle_model.dim_x;
        let mut x0 = [0.0; vehicle_model::MAX_DIM_X];
        match self.vehicle_model.model_type {
            VehicleModelType::Kinematics => {
                x0[0] = input.lateral_error;
                x0[1] = input.yaw_error;
                x0[2] = input.current_steer;
            }
            VehicleModelType::KinematicsNoDelay => {
                x0[0] = input.lateral_error;
                x0[1] = input.yaw_error;
            }
            VehicleModelType::Dynamics => {
                x0[0] = input.lateral_error;
                x0[1] = 0.0; // lateral error rate (would need derivative)
                x0[2] = input.yaw_error;
                x0[3] = 0.0; // yaw error rate
            }
        }

        let steer_lim = self.vehicle_model.steer_lim;

        // Run MPC
        let mpc_out = calculate_mpc(
            &mut self.vehicle_model,
            &self.params.mpc_params,
            input.trajectory,
            &x0[..dx],
            steer_lim,
            self.params.ctrl_period,
            &mut self.workspace,
        );

        if !mpc_out.success {
            return LateralOutput {
                steering_angle: self.prev_steer_cmd,
                steering_rate: 0.0,
                is_steer_converged: false,
            };
        }

        // Apply low-pass filter
        let g = self.params.steering_lpf_gain;
        self.filtered_steer = if self.is_initialized {
            g * self.filtered_steer + (1.0 - g) * mpc_out.steer_cmd
        } else {
            self.is_initialized = true;
            mpc_out.steer_cmd
        };

        self.prev_steer_cmd = self.filtered_steer;

        let steer_converged =
            libm::fabs(input.current_steer - self.filtered_steer) < self.params.converged_steer_rad;

        LateralOutput {
            steering_angle: self.filtered_steer,
            steering_rate: mpc_out.steer_rate,
            is_steer_converged: steer_converged,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mpc::MpcTrajPoint;

    fn make_straight_traj(n: usize, vx: f64, dt: f64) -> MpcTrajectory {
        let mut traj = MpcTrajectory::new();
        for i in 0..n {
            traj.push(MpcTrajPoint {
                x: i as f64 * vx * dt,
                y: 0.0,
                yaw: 0.0,
                vx,
                curvature: 0.0,
                smooth_curvature: 0.0,
                time: i as f64 * dt,
            });
        }
        traj
    }

    fn default_controller() -> MpcLateralController {
        let params = ControllerParams {
            mpc_params: MpcParams {
                prediction_horizon: 10,
                prediction_dt: 0.1,
                ..MpcParams::default()
            },
            ..ControllerParams::default()
        };
        MpcLateralController::new(params, &VehicleInfo::default())
    }

    #[test]
    fn test_straight_line_no_error() {
        let mut ctrl = default_controller();
        let traj = make_straight_traj(20, 5.0, 0.1);
        let input = LateralInput {
            trajectory: &traj,
            current_vel: 5.0,
            current_steer: 0.0,
            lateral_error: 0.0,
            yaw_error: 0.0,
            is_autonomous: true,
        };
        let out = ctrl.run(&input);
        assert!(libm::fabs(out.steering_angle) < 0.05);
    }

    #[test]
    fn test_lateral_error_steers_to_correct() {
        let mut ctrl = default_controller();
        let traj = make_straight_traj(20, 5.0, 0.1);
        let input = LateralInput {
            trajectory: &traj,
            current_vel: 5.0,
            current_steer: 0.0,
            lateral_error: 1.0,
            yaw_error: 0.0,
            is_autonomous: true,
        };
        let out = ctrl.run(&input);
        // Should steer negative to correct positive lateral error
        assert!(out.steering_angle < -0.001);
    }

    #[test]
    fn test_not_autonomous_passthrough() {
        let mut ctrl = default_controller();
        let traj = make_straight_traj(20, 5.0, 0.1);
        let input = LateralInput {
            trajectory: &traj,
            current_vel: 5.0,
            current_steer: 0.3,
            lateral_error: 1.0,
            yaw_error: 0.0,
            is_autonomous: false,
        };
        let out = ctrl.run(&input);
        assert!((out.steering_angle - 0.3).abs() < 1e-10);
        assert!(!out.is_steer_converged);
    }

    #[test]
    fn test_stopped_returns_previous() {
        let mut ctrl = default_controller();
        let traj = make_straight_traj(20, 5.0, 0.1);

        // First: drive to get a steer command
        let input = LateralInput {
            trajectory: &traj,
            current_vel: 5.0,
            current_steer: 0.0,
            lateral_error: 0.5,
            yaw_error: 0.0,
            is_autonomous: true,
        };
        let out1 = ctrl.run(&input);
        let prev_steer = out1.steering_angle;

        // Now stop
        let input_stopped = LateralInput {
            trajectory: &traj,
            current_vel: 0.0,
            current_steer: prev_steer,
            lateral_error: 0.0,
            yaw_error: 0.0,
            is_autonomous: true,
        };
        let out2 = ctrl.run(&input_stopped);
        assert!((out2.steering_angle - prev_steer).abs() < 1e-10);
        assert!(out2.is_steer_converged);
    }

    #[test]
    fn test_admissibility_check() {
        let mut ctrl = default_controller();
        let traj = make_straight_traj(20, 5.0, 0.1);
        let input = LateralInput {
            trajectory: &traj,
            current_vel: 5.0,
            current_steer: 0.0,
            lateral_error: 10.0, // exceeds admissible_position_error (5.0)
            yaw_error: 0.0,
            is_autonomous: true,
        };
        let out = ctrl.run(&input);
        assert!(!out.is_steer_converged);
    }

    #[test]
    fn test_lpf_smoothing() {
        let mut ctrl = default_controller();
        let traj = make_straight_traj(20, 5.0, 0.1);

        // Two calls with different errors - output should be smoothed
        let input1 = LateralInput {
            trajectory: &traj,
            current_vel: 5.0,
            current_steer: 0.0,
            lateral_error: 1.0,
            yaw_error: 0.0,
            is_autonomous: true,
        };
        let out1 = ctrl.run(&input1);

        let input2 = LateralInput {
            trajectory: &traj,
            current_vel: 5.0,
            current_steer: 0.0,
            lateral_error: 0.0, // error gone
            yaw_error: 0.0,
            is_autonomous: true,
        };
        let out2 = ctrl.run(&input2);

        // Due to LPF, out2 should still have some steering from out1
        assert!(libm::fabs(out2.steering_angle) > 0.0);
        // But less than out1
        assert!(libm::fabs(out2.steering_angle) < libm::fabs(out1.steering_angle));
    }
}
