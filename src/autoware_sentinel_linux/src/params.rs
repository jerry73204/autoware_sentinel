// Copyright 2025 Autoware Sentinel contributors
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

//! Parameter declaration and reading for the sentinel binary (Phase 10.6).
//!
//! All algorithm parameters are declared as read-only ROS 2 parameters
//! using the rclrs-compatible [`ParameterBuilder`] API.
//! Parameter names use dotted namespaces (`<algorithm>.<param>`).
//!
//! After declaration, [`read_params()`] reads all values from the parameter
//! server into the algorithm param structs used by `SafetyIsland`.

use autoware_control_validator::ValidatorParams;
use autoware_mpc_lateral_controller::ControllerParams as MpcControllerParams;
use autoware_pid_longitudinal_controller::ControllerParams as PidControllerParams;
use autoware_pid_longitudinal_controller::pid::PidGains;
use autoware_trajectory_follower_node::ControllerNodeParams;
use autoware_vehicle_cmd_gate::GateParams;
use autoware_vehicle_cmd_gate::filter::FilterParams;
use autoware_vehicle_cmd_gate::gate::ArbiterParams;
use autoware_vehicle_info_utils::VehicleInfo;
use nros::prelude::*;

/// All algorithm parameters read from the parameter server.
pub struct SentinelParams {
    // Sensing
    pub stop_filter_vx_threshold: f64,
    pub stop_filter_wz_threshold: f64,
    pub velocity_converter_speed_scale: f64,
    pub velocity_converter_stddev_vx: f64,
    pub velocity_converter_stddev_wz: f64,
    pub twist2accel_lpf_gain: f64,

    // MRM chain
    pub watchdog_timeout_ms: u64,
    pub mrm_handler: autoware_mrm_handler::Params,
    pub emergency_stop: autoware_mrm_emergency_stop_operator::Params,
    pub comfortable_stop: autoware_mrm_comfortable_stop_operator::Params,

    // Command output
    pub gate: GateParams,
    pub shift_decider_park_on_goal: bool,

    // Validation
    pub control_validator: ValidatorParams,
    pub op_mode_mgr: autoware_operation_mode_transition_manager::Params,

    // Controller
    pub controller_node: ControllerNodeParams,
    pub vehicle_info: VehicleInfo,
}

/// Declare all sentinel parameters on the parameter server (read-only).
///
/// Uses the rclrs-compatible `ParameterBuilder` API. All parameters are
/// declared read-only with Autoware-compatible defaults.
pub fn declare_parameters(server: &mut ParameterServer) {
    // Helper macro to reduce boilerplate for read-only parameter declaration.
    macro_rules! ro {
        ($server:expr, $name:expr, $val:expr, $desc:expr) => {
            ParameterBuilder::new($server, $name)
                .default($val)
                .description($desc)
                .read_only()
                .expect(concat!("failed to declare parameter: ", $name))
        };
    }

    // ── Sensing ──────────────────────────────────────────────────────────
    ro!(
        server,
        "stop_filter.vx_threshold",
        0.1_f64,
        "Velocity stop threshold (m/s)"
    );
    ro!(
        server,
        "stop_filter.wz_threshold",
        0.02_f64,
        "Angular velocity stop threshold (rad/s)"
    );
    ro!(
        server,
        "velocity_converter.speed_scale_factor",
        1.0_f64,
        "Speed scaling factor"
    );
    ro!(
        server,
        "velocity_converter.stddev_vx",
        0.2_f64,
        "Longitudinal velocity std dev (m/s)"
    );
    ro!(
        server,
        "velocity_converter.stddev_wz",
        0.1_f64,
        "Angular velocity std dev (rad/s)"
    );
    ro!(
        server,
        "twist2accel.accel_lowpass_gain",
        0.9_f64,
        "Acceleration lowpass filter gain"
    );

    // ── Heartbeat ────────────────────────────────────────────────────────
    ro!(
        server,
        "heartbeat.timeout_ms",
        5000_i64,
        "Heartbeat timeout (ms)"
    );

    // ── MRM emergency stop ──────────────────────────────────────────────
    ro!(
        server,
        "mrm_emergency_stop.target_acceleration",
        -2.5_f64,
        "Emergency stop target acceleration (m/s^2)"
    );
    ro!(
        server,
        "mrm_emergency_stop.target_jerk",
        -1.5_f64,
        "Emergency stop target jerk (m/s^3)"
    );

    // ── MRM comfortable stop ────────────────────────────────────────────
    ro!(
        server,
        "mrm_comfortable_stop.min_acceleration",
        -1.0_f64,
        "Comfortable stop min acceleration (m/s^2)"
    );
    ro!(
        server,
        "mrm_comfortable_stop.min_jerk",
        -0.3_f64,
        "Comfortable stop min jerk (m/s^3)"
    );

    // ── MRM handler ─────────────────────────────────────────────────────
    ro!(
        server,
        "mrm_handler.stopped_velocity_threshold",
        0.001_f64,
        "Velocity below which vehicle is stopped (m/s)"
    );

    // ── Shift decider ───────────────────────────────────────────────────
    ro!(
        server,
        "shift_decider.park_on_goal",
        true,
        "Shift to park when goal reached"
    );

    // ── Vehicle command gate ────────────────────────────────────────────
    ro!(
        server,
        "vehicle_cmd_gate.heartbeat_timeout_ms",
        5000_i64,
        "Gate heartbeat timeout (ms)"
    );
    ro!(
        server,
        "vehicle_cmd_gate.stop_hold_accel",
        -1.5_f64,
        "Stop-hold acceleration (m/s^2)"
    );
    ro!(
        server,
        "vehicle_cmd_gate.emergency_accel",
        -2.4_f64,
        "Emergency acceleration (m/s^2)"
    );
    ro!(
        server,
        "vehicle_cmd_gate.vel_lim",
        25.0_f64,
        "Velocity limit (m/s)"
    );

    // ── Control validator — acceleration ────────────────────────────────
    ro!(
        server,
        "control_validator.acc_error_offset",
        0.8_f64,
        "Acceleration error offset (m/s^2)"
    );
    ro!(
        server,
        "control_validator.acc_error_scale",
        0.2_f64,
        "Acceleration error scale factor"
    );
    ro!(
        server,
        "control_validator.acc_lpf_gain",
        0.97_f64,
        "Acceleration LPF gain"
    );

    // ── Control validator — velocity ────────────────────────────────────
    ro!(
        server,
        "control_validator.rolling_back_velocity",
        0.5_f64,
        "Rolling back velocity threshold (m/s)"
    );
    ro!(
        server,
        "control_validator.over_velocity_ratio",
        0.2_f64,
        "Over-velocity ratio threshold"
    );
    ro!(
        server,
        "control_validator.over_velocity_offset",
        2.0_f64,
        "Over-velocity offset (m/s)"
    );
    ro!(
        server,
        "control_validator.vel_lpf_gain",
        0.9_f64,
        "Velocity LPF gain"
    );

    // ── Control validator — lateral jerk ────────────────────────────────
    ro!(
        server,
        "control_validator.lateral_jerk_threshold",
        10.0_f64,
        "Lateral jerk threshold (m/s^3)"
    );
    ro!(
        server,
        "control_validator.jerk_lpf_gain",
        0.8_f64,
        "Lateral jerk LPF gain"
    );

    // ── Operation mode transition manager ───────────────────────────────
    ro!(
        server,
        "op_mode.allow_autonomous_in_stopped",
        true,
        "Allow autonomous engagement when stopped"
    );
    ro!(
        server,
        "op_mode.stopped_velocity_threshold",
        0.01_f64,
        "Stopped velocity threshold (m/s)"
    );
    ro!(
        server,
        "op_mode.enable_engage_on_driving",
        false,
        "Allow engagement while driving"
    );
    ro!(
        server,
        "op_mode.acc_threshold",
        1.5_f64,
        "Acceleration threshold for engagement (m/s^2)"
    );
    ro!(
        server,
        "op_mode.speed_upper_threshold",
        10.0_f64,
        "Upper speed threshold (m/s)"
    );
    ro!(
        server,
        "op_mode.speed_lower_threshold",
        -10.0_f64,
        "Lower speed threshold (m/s)"
    );
    ro!(
        server,
        "op_mode.stable_check_duration",
        0.1_f64,
        "Stable check duration (s)"
    );

    // ── Vehicle info ────────────────────────────────────────────────────
    ro!(
        server,
        "vehicle_info.wheel_base",
        2.79_f64,
        "Wheel base (m)"
    );
    ro!(
        server,
        "vehicle_info.max_steer_angle",
        0.70_f64,
        "Max steering angle (rad)"
    );
    ro!(
        server,
        "vehicle_info.wheel_radius",
        0.383_f64,
        "Wheel radius (m)"
    );
    ro!(
        server,
        "vehicle_info.wheel_width",
        0.235_f64,
        "Wheel width (m)"
    );
    ro!(
        server,
        "vehicle_info.wheel_tread",
        1.64_f64,
        "Wheel tread (m)"
    );
    ro!(
        server,
        "vehicle_info.front_overhang",
        1.0_f64,
        "Front overhang (m)"
    );
    ro!(
        server,
        "vehicle_info.rear_overhang",
        1.1_f64,
        "Rear overhang (m)"
    );
    ro!(
        server,
        "vehicle_info.left_overhang",
        0.128_f64,
        "Left overhang (m)"
    );
    ro!(
        server,
        "vehicle_info.right_overhang",
        0.128_f64,
        "Right overhang (m)"
    );
    ro!(
        server,
        "vehicle_info.vehicle_height",
        2.5_f64,
        "Vehicle height (m)"
    );

    // ── Controller node ─────────────────────────────────────────────────
    ro!(
        server,
        "controller.ctrl_period",
        0.033_f64,
        "Control period (s)"
    );
    ro!(
        server,
        "controller.ego_nearest_dist_threshold",
        3.0_f64,
        "Nearest point distance threshold (m)"
    );
    ro!(
        server,
        "controller.ego_nearest_yaw_threshold",
        1.57_f64,
        "Nearest point yaw threshold (rad)"
    );

    // ── PID longitudinal controller ─────────────────────────────────────
    ro!(server, "pid.kp", 1.0_f64, "PID proportional gain");
    ro!(server, "pid.ki", 0.1_f64, "PID integral gain");
    ro!(server, "pid.kd", 0.0_f64, "PID derivative gain");
    ro!(server, "pid.max_acc", 3.0_f64, "Max acceleration (m/s^2)");
    ro!(server, "pid.min_acc", -5.0_f64, "Min acceleration (m/s^2)");
    ro!(server, "pid.max_jerk", 2.0_f64, "Max jerk (m/s^3)");
    ro!(server, "pid.min_jerk", -5.0_f64, "Min jerk (m/s^3)");
    ro!(
        server,
        "pid.delay_compensation_time",
        0.17_f64,
        "Delay compensation time (s)"
    );
    ro!(
        server,
        "pid.stopped_acc",
        -3.4_f64,
        "Stopped state acceleration (m/s^2)"
    );
    ro!(
        server,
        "pid.emergency_acc",
        -5.0_f64,
        "Emergency state acceleration (m/s^2)"
    );

    // ── MPC lateral controller ──────────────────────────────────────────
    ro!(
        server,
        "mpc.prediction_horizon",
        50_i64,
        "MPC prediction horizon (steps)"
    );
    ro!(
        server,
        "mpc.prediction_dt",
        0.1_f64,
        "MPC prediction time step (s)"
    );
    ro!(
        server,
        "mpc.steer_tau",
        0.27_f64,
        "Steering time constant (s)"
    );
    ro!(
        server,
        "mpc.steering_lpf_gain",
        0.8_f64,
        "Steering output LPF gain"
    );
    ro!(
        server,
        "mpc.input_delay",
        0.0_f64,
        "Input delay compensation (s)"
    );
    ro!(
        server,
        "mpc.steer_rate_lim",
        1.7321_f64,
        "Steering rate limit (rad/s)"
    );
}

/// Read all declared parameters into algorithm param structs.
///
/// Must be called after [`declare_parameters()`].
pub fn read_params(server: &ParameterServer) -> SentinelParams {
    // Convenience closures for typed access with panic on missing.
    let f = |name: &str| -> f64 {
        server
            .get_double(name)
            .unwrap_or_else(|| panic!("parameter {name} not found"))
    };
    let i = |name: &str| -> i64 {
        server
            .get_integer(name)
            .unwrap_or_else(|| panic!("parameter {name} not found"))
    };
    let b = |name: &str| -> bool {
        server
            .get_bool(name)
            .unwrap_or_else(|| panic!("parameter {name} not found"))
    };

    // Vehicle info
    let vehicle_info = VehicleInfo::new(
        f("vehicle_info.wheel_radius"),
        f("vehicle_info.wheel_width"),
        f("vehicle_info.wheel_base"),
        f("vehicle_info.wheel_tread"),
        f("vehicle_info.front_overhang"),
        f("vehicle_info.rear_overhang"),
        f("vehicle_info.left_overhang"),
        f("vehicle_info.right_overhang"),
        f("vehicle_info.vehicle_height"),
        f("vehicle_info.max_steer_angle"),
    );

    // PID longitudinal params
    let mut pid_params = PidControllerParams::default();
    pid_params.pid_gains = PidGains {
        kp: f("pid.kp"),
        ki: f("pid.ki"),
        kd: f("pid.kd"),
    };
    pid_params.max_acc = f("pid.max_acc");
    pid_params.min_acc = f("pid.min_acc");
    pid_params.max_jerk = f("pid.max_jerk");
    pid_params.min_jerk = f("pid.min_jerk");
    pid_params.delay_compensation_time = f("pid.delay_compensation_time");
    pid_params.stopped_acc = f("pid.stopped_acc");
    pid_params.emergency_acc = f("pid.emergency_acc");

    // MPC lateral params
    let mut mpc_params = MpcControllerParams::default();
    mpc_params.ctrl_period = f("controller.ctrl_period");
    mpc_params.steering_lpf_gain = f("mpc.steering_lpf_gain");
    mpc_params.mpc_params.prediction_horizon = i("mpc.prediction_horizon") as usize;
    mpc_params.mpc_params.prediction_dt = f("mpc.prediction_dt");
    mpc_params.mpc_params.steer_tau = f("mpc.steer_tau");
    mpc_params.mpc_params.input_delay = f("mpc.input_delay");
    mpc_params.mpc_params.steer_rate_lim = f("mpc.steer_rate_lim");

    // Gate params
    let mut filter_params = FilterParams::default();
    filter_params.vel_lim = f("vehicle_cmd_gate.vel_lim") as f32;
    filter_params.wheel_base = vehicle_info.wheel_base_m as f32;

    let gate = GateParams {
        filter: filter_params,
        arbiter: ArbiterParams {
            stop_hold_accel: f("vehicle_cmd_gate.stop_hold_accel") as f32,
            emergency_accel: f("vehicle_cmd_gate.emergency_accel") as f32,
        },
        heartbeat_timeout_ms: i("vehicle_cmd_gate.heartbeat_timeout_ms") as u64,
    };

    // Control validator params
    let control_validator = ValidatorParams {
        accel: autoware_control_validator::accel::AccelValidatorParams {
            e_offset: f("control_validator.acc_error_offset"),
            e_scale: f("control_validator.acc_error_scale"),
            lpf_gain: f("control_validator.acc_lpf_gain"),
        },
        velocity: autoware_control_validator::velocity::VelocityValidatorParams {
            rolling_back_velocity_th: f("control_validator.rolling_back_velocity"),
            over_velocity_ratio_th: f("control_validator.over_velocity_ratio"),
            over_velocity_offset_th: f("control_validator.over_velocity_offset"),
            lpf_gain: f("control_validator.vel_lpf_gain"),
        },
        jerk: autoware_control_validator::jerk::LateralJerkValidatorParams {
            lateral_jerk_threshold: f("control_validator.lateral_jerk_threshold"),
            wheel_base: vehicle_info.wheel_base_m,
            lpf_gain: f("control_validator.jerk_lpf_gain"),
        },
    };

    // Operation mode transition manager
    let op_mode_mgr = autoware_operation_mode_transition_manager::Params {
        allow_autonomous_in_stopped: b("op_mode.allow_autonomous_in_stopped"),
        stopped_velocity_threshold: f("op_mode.stopped_velocity_threshold"),
        enable_engage_on_driving: b("op_mode.enable_engage_on_driving"),
        acc_threshold: f("op_mode.acc_threshold"),
        speed_upper_threshold: f("op_mode.speed_upper_threshold"),
        speed_lower_threshold: f("op_mode.speed_lower_threshold"),
        stable_check_duration: f("op_mode.stable_check_duration"),
    };

    SentinelParams {
        stop_filter_vx_threshold: f("stop_filter.vx_threshold"),
        stop_filter_wz_threshold: f("stop_filter.wz_threshold"),
        velocity_converter_speed_scale: f("velocity_converter.speed_scale_factor"),
        velocity_converter_stddev_vx: f("velocity_converter.stddev_vx"),
        velocity_converter_stddev_wz: f("velocity_converter.stddev_wz"),
        twist2accel_lpf_gain: f("twist2accel.accel_lowpass_gain"),

        watchdog_timeout_ms: i("heartbeat.timeout_ms") as u64,
        mrm_handler: autoware_mrm_handler::Params {
            stopped_velocity_threshold: f("mrm_handler.stopped_velocity_threshold"),
            ..autoware_mrm_handler::Params::default()
        },
        emergency_stop: autoware_mrm_emergency_stop_operator::Params {
            target_acceleration: f("mrm_emergency_stop.target_acceleration") as f32,
            target_jerk: f("mrm_emergency_stop.target_jerk") as f32,
        },
        comfortable_stop: autoware_mrm_comfortable_stop_operator::Params {
            min_acceleration: f("mrm_comfortable_stop.min_acceleration") as f32,
            min_jerk: f("mrm_comfortable_stop.min_jerk") as f32,
        },

        gate,
        shift_decider_park_on_goal: b("shift_decider.park_on_goal"),

        control_validator,
        op_mode_mgr,

        controller_node: ControllerNodeParams {
            ctrl_period: f("controller.ctrl_period"),
            lateral: mpc_params,
            longitudinal: pid_params,
            ego_nearest_dist_threshold: f("controller.ego_nearest_dist_threshold"),
            ego_nearest_yaw_threshold: f("controller.ego_nearest_yaw_threshold"),
        },
        vehicle_info,
    }
}
