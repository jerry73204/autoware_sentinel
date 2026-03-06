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

//! Autoware Sentinel — Linux Native Safety Island Binary
//!
//! Std-enabled Linux adaptation of the Zephyr safety island application.
//! Reuses the same `SafetyIsland` struct and algorithm wiring with:
//! - Autoware-compatible topic names (for planning simulator integration)
//! - `env_logger` instead of Zephyr logger
//! - `std::time::Instant` for monotonic clock
//! - `ExecutorConfig::from_env()` for Zenoh locator configuration
//! - `spin_blocking()` instead of `spin()`
//! - Engagement flow: publishes OperationModeState and serves ChangeOperationMode
//!
//! # Usage
//!
//! ```bash
//! # Start zenoh router first:
//! ~/repos/nano-ros/build/zenohd/zenohd --listen tcp/127.0.0.1:7447
//!
//! # Run the sentinel:
//! RUST_LOG=info cargo run
//!
//! # Or with custom zenoh locator:
//! ZENOH_LOCATOR=tcp/192.168.1.100:7447 RUST_LOG=info cargo run
//! ```

mod params;

use std::cell::RefCell;
use std::sync::OnceLock;
use std::time::Instant;

use log::info;
use nros::prelude::*;

// Algorithm crates
use autoware_control_validator::ControlValidator;
use autoware_heartbeat_watchdog::HeartbeatWatchdog;
use autoware_mrm_comfortable_stop_operator::ComfortableStopOperator;
use autoware_mrm_emergency_stop_operator::EmergencyStopOperator;
use autoware_mrm_handler::MrmHandler;
use autoware_operation_mode_transition_manager::OperationModeTransitionManager;
use autoware_shift_decider::ShiftDecider;
use autoware_stop_filter::StopFilter;
use autoware_trajectory_follower_base::{InputData, TrajectoryPoint};
use autoware_trajectory_follower_node::ControllerNode;
use autoware_twist2accel::Twist2Accel;
use autoware_vehicle_cmd_gate::VehicleCmdGate;
use autoware_vehicle_cmd_gate::gate::SourceCommands;
use autoware_vehicle_velocity_converter::VehicleVelocityConverter;

// Message types
use autoware_adapi_v1_msgs::msg::{Heartbeat, MrmState, OperationModeState};
use autoware_adapi_v1_msgs::srv::{ChangeOperationMode, ChangeOperationModeResponse};
use autoware_control_msgs::msg::Control;
use autoware_planning_msgs::msg::Trajectory;
use autoware_system_msgs::msg::AutowareState;
use autoware_vehicle_msgs::msg::{
    Engage, GearCommand, GearReport, HazardLightsCommand, SteeringReport, TurnIndicatorsCommand,
    VelocityReport,
};
use geometry_msgs::msg::{Accel, AccelWithCovarianceStamped, Twist};
use nav_msgs::msg::Odometry;
use tier4_control_msgs::msg::{GateMode, IsStopped};
use tier4_external_api_msgs::msg::Emergency;
use tier4_system_msgs::msg::{MrmBehaviorStatus, OperationModeAvailability};
use tier4_vehicle_msgs::msg::VehicleEmergencyStamped;

// Debug/diagnostic message types (Phase 8.2)
use autoware_control_validator_msgs::msg::ControlValidatorStatus;
use autoware_internal_debug_msgs::msg::BoolStamped;
use autoware_internal_msgs::msg::PublishedTime;
use autoware_operation_mode_transition_manager_msgs::msg::OperationModeTransitionManagerDebug;
use autoware_vehicle_cmd_gate_msgs::msg::IsFilterActivated;
use visualization_msgs::msg::MarkerArray;

/// MRM handler state: OPERATING (emergency response active).
const MRM_STATE_OPERATING: u16 = 2;

/// AutowareState: DRIVING (autonomous engaged).
const AUTOWARE_STATE_DRIVING: u8 = 5;

/// Consecutive invalid validation frames before triggering MRM (~1s at 30 Hz).
const VALIDATION_FAILURE_THRESHOLD: u32 = 30;

/// Control period (s) for 30 Hz timer.
const DT: f32 = 1.0 / 30.0;

/// OperationModeState constant: AUTONOMOUS mode.
const OP_MODE_AUTONOMOUS: u8 = 2;

/// MrmBehaviorStatus constants.
const MRM_BEHAVIOR_AVAILABLE: u8 = 1;
const MRM_BEHAVIOR_OPERATING: u8 = 2;

/// GateMode constant: AUTO.
const GATE_MODE_AUTO: u8 = 0;

// ============================================================================
// Static shared state
// ============================================================================

/// `RefCell` wrapper that implements `Sync` for single-threaded contexts.
///
/// # Safety
///
/// All access occurs within the nros single-threaded executor. Callbacks
/// run sequentially — never concurrently — so `RefCell`'s runtime borrow
/// checks are always satisfied.
struct SyncRefCell<T>(RefCell<T>);

// SAFETY: The nros executor dispatches all callbacks (subscriptions, timers)
// sequentially in a single thread. No concurrent access is possible.
unsafe impl<T> Sync for SyncRefCell<T> {}

/// All algorithm instances and shared data for the safety island.
///
/// Statically allocated — no `Rc`, no `Arc`. Subscription callbacks
/// write latest received data; the 30 Hz timer runs the full control pipeline
/// in deterministic order: sense → MRM → gate → validate → publish.
struct SafetyIsland {
    // --- Sensing ---
    velocity_converter: VehicleVelocityConverter,
    stop_filter: StopFilter,
    twist2accel: Twist2Accel,
    prev_stamp: Option<(i32, u32)>,
    current_velocity: f64,
    is_stopped: bool,
    twist: Twist,
    accel: Accel,
    accel_covariance: [f64; 36],

    // --- Heartbeat ---
    watchdog: HeartbeatWatchdog,

    // --- MRM chain ---
    mrm_handler: MrmHandler,
    emergency_stop: EmergencyStopOperator,
    comfortable_stop: ComfortableStopOperator,

    // --- Command output ---
    cmd_gate: VehicleCmdGate,
    shift_decider: ShiftDecider,
    auto_control: Control,
    autoware_state: AutowareState,
    gear_report: GearReport,

    // --- Validation ---
    control_validator: ControlValidator,
    op_mode_mgr: OperationModeTransitionManager,

    // --- Trajectory follower (Phase 10.5) ---
    controller_node: ControllerNode,
    input_data: InputData,
    has_trajectory: bool,
    has_odometry: bool,
    has_steering: bool,
}

static ISLAND: SyncRefCell<Option<SafetyIsland>> = SyncRefCell(RefCell::new(None));

/// Borrow the safety island state for the duration of `f`.
///
/// # Panics
///
/// Panics if called before initialization or if a borrow is already active
/// (the latter is impossible in the single-threaded executor).
#[inline]
fn with_island<R>(f: impl FnOnce(&mut SafetyIsland) -> R) -> R {
    let mut guard = ISLAND.0.borrow_mut();
    f(guard.as_mut().expect("SafetyIsland not initialized"))
}

impl SafetyIsland {
    fn new(p: params::SentinelParams) -> Self {
        let mut watchdog = HeartbeatWatchdog::new(autoware_heartbeat_watchdog::Params {
            timeout_ms: p.watchdog_timeout_ms,
        });
        // Pre-seed with boot time to avoid immediate timeout.
        // Gives Autoware time to initialize and start sending heartbeats.
        watchdog.on_heartbeat(0);

        Self {
            // Sensing
            velocity_converter: VehicleVelocityConverter::new(
                p.velocity_converter_speed_scale,
                p.velocity_converter_stddev_vx,
                p.velocity_converter_stddev_wz,
            ),
            stop_filter: StopFilter::new(p.stop_filter_vx_threshold, p.stop_filter_wz_threshold),
            twist2accel: Twist2Accel::new(autoware_twist2accel::Params {
                accel_lowpass_gain: p.twist2accel_lpf_gain,
            }),
            prev_stamp: None,
            current_velocity: 0.0,
            is_stopped: true,
            twist: Twist::default(),
            accel: Accel::default(),
            accel_covariance: [0.0; 36],

            // Heartbeat
            watchdog,

            // MRM chain
            mrm_handler: MrmHandler::new(p.mrm_handler),
            emergency_stop: EmergencyStopOperator::new(p.emergency_stop),
            comfortable_stop: ComfortableStopOperator::new(p.comfortable_stop),

            // Command output
            cmd_gate: VehicleCmdGate::new(p.gate),
            shift_decider: ShiftDecider::new(p.shift_decider_park_on_goal),
            auto_control: Control::default(),
            autoware_state: AutowareState::default(),
            gear_report: GearReport::default(),

            // Validation
            control_validator: ControlValidator::new(p.control_validator),
            op_mode_mgr: OperationModeTransitionManager::new(p.op_mode_mgr),

            // Trajectory follower
            controller_node: ControllerNode::new(p.controller_node, p.vehicle_info),
            input_data: InputData::default(),
            has_trajectory: false,
            has_odometry: false,
            has_steering: false,
        }
    }

    /// Process a velocity report through the sensing pipeline.
    ///
    /// VelocityReport → VehicleVelocityConverter → StopFilter → Twist2Accel
    fn on_velocity_report(&mut self, msg: &VelocityReport) {
        let twist_cov = self.velocity_converter.convert(msg);
        let twist = &twist_cov.twist.twist;
        let filtered = self.stop_filter.apply(&twist.linear, &twist.angular);

        let stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec);
        let dt = if let Some((ps, pn)) = self.prev_stamp {
            (stamp.0 - ps) as f64 + (stamp.1 as f64 - pn as f64) * 1e-9
        } else {
            0.0
        };
        self.prev_stamp = Some(stamp);

        let filtered_twist = Twist {
            linear: filtered.linear.clone(),
            angular: filtered.angular.clone(),
        };
        let accel_output = self.twist2accel.update(&filtered_twist, dt);

        self.current_velocity = filtered_twist.linear.x;
        self.is_stopped = filtered.was_stopped;
        self.twist = filtered_twist;
        if let Some(output) = accel_output {
            self.accel = output.accel;
            self.accel_covariance = output.covariance;
        }
    }

    /// Store a trajectory message for the controller node.
    fn on_trajectory(&mut self, msg: &Trajectory) {
        let n = msg
            .points
            .len()
            .min(autoware_trajectory_follower_base::MAX_TRAJECTORY_POINTS);
        for i in 0..n {
            let pt = &msg.points[i];
            let q = &pt.pose.orientation;
            let (pitch, yaw) = quaternion_to_pitch_yaw(q.x, q.y, q.z, q.w);
            self.input_data.trajectory[i] = TrajectoryPoint {
                x: pt.pose.position.x,
                y: pt.pose.position.y,
                z: pt.pose.position.z,
                yaw,
                longitudinal_velocity_mps: pt.longitudinal_velocity_mps as f64,
                lateral_velocity_mps: pt.lateral_velocity_mps as f64,
                acceleration_mps2: pt.acceleration_mps2 as f64,
                heading_rate_rps: pt.heading_rate_rps as f64,
                front_wheel_angle_rad: pt.front_wheel_angle_rad as f64,
            };
            // Store pitch from first point for slope compensation
            if i == 0 {
                self.input_data.current_pose_pitch = pitch;
            }
        }
        self.input_data.trajectory_len = n;
        self.has_trajectory = true;
    }

    /// Store odometry for the controller node.
    fn on_odometry(&mut self, msg: &Odometry) {
        let pos = &msg.pose.pose.position;
        let q = &msg.pose.pose.orientation;
        let (pitch, yaw) = quaternion_to_pitch_yaw(q.x, q.y, q.z, q.w);

        self.input_data.current_pose_x = pos.x;
        self.input_data.current_pose_y = pos.y;
        self.input_data.current_pose_z = pos.z;
        self.input_data.current_pose_yaw = yaw;
        self.input_data.current_pose_pitch = pitch;
        self.input_data.current_velocity = msg.twist.twist.linear.x;
        self.has_odometry = true;
    }

    /// Store steering angle for the controller node.
    fn on_steering(&mut self, msg: &SteeringReport) {
        self.input_data.current_steer = msg.steering_tire_angle as f64;
        self.has_steering = true;
    }

    /// Store acceleration for the controller node.
    fn on_acceleration(&mut self, msg: &AccelWithCovarianceStamped) {
        self.input_data.current_accel = msg.accel.accel.linear.x;
    }

    /// Run the trajectory follower controller and update auto_control.
    fn run_controller(&mut self, current_time_s: f64) {
        if !self.has_trajectory || !self.has_odometry || !self.has_steering {
            return; // Not enough data yet — use external control_cmd
        }

        // Operation mode: always autonomous in sentinel
        self.input_data.is_autonomous = true;
        self.input_data.is_in_transition = false;

        if let Some(output) = self
            .controller_node
            .update(&self.input_data, current_time_s)
        {
            self.auto_control.lateral.steering_tire_angle =
                output.lateral.steering_tire_angle as f32;
            self.auto_control.lateral.steering_tire_rotation_rate =
                output.lateral.steering_tire_rotation_rate as f32;
            self.auto_control.longitudinal.velocity = output.longitudinal.velocity as f32;
            self.auto_control.longitudinal.acceleration = output.longitudinal.acceleration as f32;
        }
    }
}

/// Extract (pitch, yaw) from quaternion components.
fn quaternion_to_pitch_yaw(x: f64, y: f64, z: f64, w: f64) -> (f64, f64) {
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        f64::copysign(std::f64::consts::FRAC_PI_2, sinp)
    } else {
        sinp.asin()
    };
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);
    (pitch, yaw)
}

/// Monotonic clock in milliseconds since process start.
fn now_ms() -> u64 {
    static EPOCH: OnceLock<Instant> = OnceLock::new();
    let epoch = EPOCH.get_or_init(Instant::now);
    epoch.elapsed().as_millis() as u64
}

// ============================================================================
// Entry point
// ============================================================================

fn main() {
    env_logger::init();

    info!("Autoware Sentinel — Linux Native Safety Island");

    if let Err(e) = run() {
        log::error!("Fatal: {:?}", e);
        std::process::exit(1);
    }
}

fn run() -> Result<(), NodeError> {
    let config = ExecutorConfig::from_env().node_name("sentinel");
    let mut executor = Executor::<_, 52, 16384>::open(&config)?;

    // Register ROS 2 parameter services (~/get_parameters, ~/set_parameters, etc.)
    executor.register_parameter_services("/sentinel")?;
    info!("Parameter services registered for /sentinel");

    // Declare all algorithm parameters (read-only, Autoware-compatible defaults)
    let server = executor
        .params_mut()
        .expect("parameter services not registered");
    params::declare_parameters(server);
    let sentinel_params = params::read_params(server);
    info!("Declared {} parameters", server.len());

    // Initialize static state (before any callbacks can fire)
    *ISLAND.0.borrow_mut() = Some(SafetyIsland::new(sentinel_params));

    // --- Create publishers (node borrows executor, dropped after) ---
    let (
        mrm_state_pub,
        hazard_pub,
        gear_pub,
        control_pub,
        turn_pub,
        op_mode_pub,
        // Phase 8.1a — MRM behavior status (topics 11–13)
        mrm_estop_status_pub,
        mrm_comfy_status_pub,
        mrm_pullover_status_pub,
        // Phase 8.1b — MRM handler emergency outputs (topics 8–10)
        emergency_gear_pub,
        emergency_hazard_pub,
        emergency_turn_pub,
        // Phase 8.1c — Vehicle command gate additional (topics 3–7)
        emergency_cmd_pub,
        gate_mode_pub,
        shift_decider_gear_pub,
        is_stopped_pub,
        gate_op_mode_pub,
        // Phase 8.1d — Engagement (topics 1–2)
        engage_api_pub,
        engage_compat_pub,
        // Phase 8.1g — Emergency API (topic 14)
        emergency_api_pub,
        // Phase 8.2a — Control validator debug (topics 16–19)
        cv_debug_marker_pub,
        cv_output_markers_pub,
        cv_validation_status_pub,
        cv_virtual_wall_pub,
        // Phase 8.2b — Vehicle cmd gate filter debug (topics 20–23)
        filter_activated_pub,
        filter_flag_pub,
        filter_marker_pub,
        filter_marker_raw_pub,
        // Phase 8.2c — Remaining debug (topics 14–15)
        op_mode_debug_pub,
        published_time_pub,
    ) = {
        let mut node = executor.create_node("sentinel")?;
        (
            // Existing 6 publishers
            node.create_publisher::<MrmState>("/system/fail_safe/mrm_state")?,
            node.create_publisher::<HazardLightsCommand>("/control/command/hazard_lights_cmd")?,
            node.create_publisher::<GearCommand>("/control/command/gear_cmd")?,
            node.create_publisher::<Control>("/control/command/control_cmd")?,
            node.create_publisher::<TurnIndicatorsCommand>("/control/command/turn_indicators_cmd")?,
            node.create_publisher::<OperationModeState>("/api/operation_mode/state")?,
            // 8.1a — MRM behavior status
            node.create_publisher::<MrmBehaviorStatus>("/system/mrm/emergency_stop/status")?,
            node.create_publisher::<MrmBehaviorStatus>("/system/mrm/comfortable_stop/status")?,
            node.create_publisher::<MrmBehaviorStatus>("/system/mrm/pull_over_manager/status")?,
            // 8.1b — MRM handler emergency outputs
            node.create_publisher::<GearCommand>("/system/emergency/gear_cmd")?,
            node.create_publisher::<HazardLightsCommand>("/system/emergency/hazard_lights_cmd")?,
            node.create_publisher::<TurnIndicatorsCommand>(
                "/system/emergency/turn_indicators_cmd",
            )?,
            // 8.1c — Vehicle command gate additional
            node.create_publisher::<VehicleEmergencyStamped>("/control/command/emergency_cmd")?,
            node.create_publisher::<GateMode>("/control/gate_mode_cmd")?,
            node.create_publisher::<GearCommand>("/control/shift_decider/gear_cmd")?,
            node.create_publisher::<IsStopped>("/control/vehicle_cmd_gate/is_stopped")?,
            node.create_publisher::<OperationModeState>(
                "/control/vehicle_cmd_gate/operation_mode",
            )?,
            // 8.1d — Engagement
            node.create_publisher::<Engage>("/api/autoware/get/engage")?,
            node.create_publisher::<Engage>("/autoware/engage")?,
            // 8.1g — Emergency API
            node.create_publisher::<Emergency>("/api/autoware/get/emergency")?,
            // 8.2a — Control validator debug
            node.create_publisher::<MarkerArray>("/control/control_validator/debug/marker")?,
            node.create_publisher::<MarkerArray>("/control/control_validator/output/markers")?,
            node.create_publisher::<ControlValidatorStatus>(
                "/control/control_validator/validation_status",
            )?,
            node.create_publisher::<MarkerArray>("/control/control_validator/virtual_wall")?,
            // 8.2b — Vehicle cmd gate filter debug
            node.create_publisher::<IsFilterActivated>(
                "/control/vehicle_cmd_gate/is_filter_activated",
            )?,
            node.create_publisher::<BoolStamped>(
                "/control/vehicle_cmd_gate/is_filter_activated/flag",
            )?,
            node.create_publisher::<MarkerArray>(
                "/control/vehicle_cmd_gate/is_filter_activated/marker",
            )?,
            node.create_publisher::<MarkerArray>(
                "/control/vehicle_cmd_gate/is_filter_activated/marker_raw",
            )?,
            // 8.2c — Remaining debug
            node.create_publisher::<OperationModeTransitionManagerDebug>(
                "/control/autoware_operation_mode_transition_manager/debug_info",
            )?,
            node.create_publisher::<PublishedTime>(
                "/control/command/control_cmd/debug/published_time",
            )?,
        )
    };

    // ====================================================================
    // Sensing / input layer
    // ====================================================================
    // VelocityReport → VehicleVelocityConverter → StopFilter → Twist2Accel
    executor.add_subscription::<VelocityReport, _>("/vehicle/status/velocity_status", |msg| {
        with_island(|island| island.on_velocity_report(msg))
    })?;
    info!("Subscribed: /vehicle/status/velocity_status");

    // ====================================================================
    // Heartbeat subscription (ADAPI topic)
    // ====================================================================
    executor.add_subscription::<Heartbeat, _>("/api/system/heartbeat", |_msg| {
        with_island(|island| island.watchdog.on_heartbeat(now_ms()));
    })?;
    info!("Subscribed: /api/system/heartbeat");

    // ====================================================================
    // Autonomous command subscriptions (Autoware-compatible topics)
    // ====================================================================
    executor.add_subscription::<Control, _>("/control/trajectory_follower/control_cmd", |msg| {
        with_island(|island| island.auto_control = msg.clone())
    })?;
    executor.add_subscription::<AutowareState, _>("/autoware/state", |msg| {
        with_island(|island| island.autoware_state = msg.clone());
    })?;
    executor.add_subscription::<GearReport, _>("/vehicle/status/gear_status", |msg| {
        with_island(|island| island.gear_report = msg.clone())
    })?;
    info!("Subscribed: control_cmd, autoware_state, gear_status");

    // ====================================================================
    // Trajectory follower input subscriptions (Phase 10.5)
    // ====================================================================
    executor
        .add_subscription::<Trajectory, _>("/planning/scenario_planning/trajectory", |msg| {
            with_island(|island| island.on_trajectory(msg))
        })?;
    executor.add_subscription::<Odometry, _>("/localization/kinematic_state", |msg| {
        with_island(|island| island.on_odometry(msg))
    })?;
    executor.add_subscription::<SteeringReport, _>("/vehicle/status/steering_status", |msg| {
        with_island(|island| island.on_steering(msg))
    })?;
    executor
        .add_subscription::<AccelWithCovarianceStamped, _>("/localization/acceleration", |msg| {
            with_island(|island| island.on_acceleration(msg))
        })?;
    info!("Subscribed: trajectory, odometry, steering, acceleration (controller inputs)");

    // ====================================================================
    // Engagement flow: ChangeOperationMode service (always returns success)
    // ====================================================================
    executor.add_service::<ChangeOperationMode, _>(
        "/api/operation_mode/change_to_autonomous",
        |_request| {
            info!("ChangeOperationMode service called — returning success");
            ChangeOperationModeResponse {
                status: autoware_adapi_v1_msgs::msg::ResponseStatus {
                    success: true,
                    code: 0,
                    message: Default::default(),
                },
            }
        },
    )?;
    info!("Service: /api/operation_mode/change_to_autonomous");

    // ====================================================================
    // 30 Hz main control timer
    // ====================================================================
    executor.add_timer(TimerDuration::from_millis(33), move || {
        with_island(|island| {
            let now = now_ms();

            // ── Trajectory follower (Phase 10.5) ─────────────────────
            island.run_controller(now as f64 / 1000.0);

            // ── MRM chain ──────────────────────────────────────────────

            // Check heartbeat watchdog (state-change only)
            let watchdog_update = island.watchdog.check(now);

            island.mrm_handler.update_velocity(island.current_velocity);
            if let Some(ref availability) = watchdog_update {
                island.mrm_handler.update_availability(availability);
            }

            let mrm_output = island.mrm_handler.update();

            // Activate/deactivate stop operators
            if let Some(activate) = mrm_output.emergency_stop_operate {
                if activate {
                    island
                        .emergency_stop
                        .set_initial_velocity(island.current_velocity as f32);
                }
                island.emergency_stop.operate(activate);
            }
            if let Some(activate) = mrm_output.comfortable_stop_operate {
                if activate {
                    island
                        .comfortable_stop
                        .set_initial_velocity(island.current_velocity as f32);
                }
                island.comfortable_stop.operate(activate);
            }

            // Advance deceleration ramps
            let emergency_control = island.emergency_stop.update(DT);
            let comfortable_control = island.comfortable_stop.update(DT);

            // Select active MRM control for emergency source
            let mrm_control = if island.emergency_stop.is_operating() {
                emergency_control
            } else if island.comfortable_stop.is_operating() {
                comfortable_control
            } else {
                Control::default()
            };

            // ── Command output ─────────────────────────────────────────

            island
                .cmd_gate
                .set_system_emergency(island.mrm_handler.state() == MRM_STATE_OPERATING);
            island
                .cmd_gate
                .set_current_speed(island.current_velocity as f32);
            island
                .cmd_gate
                .set_engaged(island.autoware_state.state == AUTOWARE_STATE_DRIVING);

            // Autonomous source: control + shift decider gear
            let auto_gear = island.shift_decider.decide(
                &island.autoware_state,
                &island.auto_control,
                &island.gear_report,
            );
            island.cmd_gate.set_autonomous_commands(
                SourceCommands {
                    control: island.auto_control.clone(),
                    gear: GearCommand {
                        command: auto_gear,
                        ..Default::default()
                    },
                    turn_indicators: TurnIndicatorsCommand::default(),
                    hazard_lights: HazardLightsCommand::default(),
                },
                now,
            );

            // Emergency source: MRM operator control + MRM gear/hazard
            island.cmd_gate.set_emergency_commands(SourceCommands {
                control: mrm_control,
                gear: mrm_output.gear.clone(),
                turn_indicators: TurnIndicatorsCommand::default(),
                hazard_lights: mrm_output.hazard_lights.clone(),
            });

            let gate_output = island.cmd_gate.update(now);

            // ── Validation ─────────────────────────────────────────────

            let target_vel = gate_output.control.longitudinal.velocity as f64;
            island.control_validator.validate(
                &gate_output.control,
                island.current_velocity,
                island.accel.linear.x,
                target_vel,
                0.0, // pitch — no IMU on safety island
                DT as f64,
            );

            island.op_mode_mgr.update_velocity(island.current_velocity);
            island.op_mode_mgr.update_control_cmd(&gate_output.control);
            island.op_mode_mgr.update(DT as f64);

            // Validation failure → trigger MRM (takes effect next iteration)
            if island.control_validator.status().invalid_count >= VALIDATION_FAILURE_THRESHOLD {
                island
                    .mrm_handler
                    .update_availability(&OperationModeAvailability::default());
            }

            // ── Publish ────────────────────────────────────────────────

            mrm_state_pub.publish(&mrm_output.mrm_state).ok();
            hazard_pub.publish(&mrm_output.hazard_lights).ok();
            gear_pub.publish(&gate_output.gear).ok();
            control_pub.publish(&gate_output.control).ok();
            turn_pub.publish(&gate_output.turn_indicators).ok();

            // Engagement flow: publish OperationModeState (always autonomous-ready)
            let op_mode_state = OperationModeState {
                stamp: Default::default(),
                mode: OP_MODE_AUTONOMOUS,
                is_autoware_control_enabled: true,
                is_in_transition: false,
                is_stop_mode_available: true,
                is_autonomous_mode_available: true,
                is_local_mode_available: true,
                is_remote_mode_available: true,
            };
            op_mode_pub.publish(&op_mode_state).ok();

            // ── Phase 8.1 — Additional functional topics ────────────

            // 8.1a — MRM behavior status
            mrm_estop_status_pub
                .publish(&MrmBehaviorStatus {
                    stamp: Default::default(),
                    state: if island.emergency_stop.is_operating() {
                        MRM_BEHAVIOR_OPERATING
                    } else {
                        MRM_BEHAVIOR_AVAILABLE
                    },
                })
                .ok();
            mrm_comfy_status_pub
                .publish(&MrmBehaviorStatus {
                    stamp: Default::default(),
                    state: if island.comfortable_stop.is_operating() {
                        MRM_BEHAVIOR_OPERATING
                    } else {
                        MRM_BEHAVIOR_AVAILABLE
                    },
                })
                .ok();
            mrm_pullover_status_pub
                .publish(&MrmBehaviorStatus {
                    stamp: Default::default(),
                    state: MRM_BEHAVIOR_AVAILABLE,
                })
                .ok();

            // 8.1b — MRM handler emergency outputs
            emergency_gear_pub.publish(&mrm_output.gear).ok();
            emergency_hazard_pub.publish(&mrm_output.hazard_lights).ok();
            emergency_turn_pub
                .publish(&TurnIndicatorsCommand::default())
                .ok();

            // 8.1c — Vehicle command gate additional
            emergency_cmd_pub
                .publish(&VehicleEmergencyStamped {
                    stamp: Default::default(),
                    emergency: island.mrm_handler.state() == MRM_STATE_OPERATING,
                })
                .ok();
            gate_mode_pub
                .publish(&GateMode {
                    data: GATE_MODE_AUTO,
                })
                .ok();
            shift_decider_gear_pub
                .publish(&GearCommand {
                    command: auto_gear,
                    ..Default::default()
                })
                .ok();
            is_stopped_pub
                .publish(&IsStopped {
                    stamp: Default::default(),
                    data: island.is_stopped,
                    requested_sources: Default::default(),
                })
                .ok();
            gate_op_mode_pub.publish(&op_mode_state).ok();

            // 8.1d — Engagement
            let engaged = island.autoware_state.state == AUTOWARE_STATE_DRIVING;
            engage_api_pub
                .publish(&Engage {
                    stamp: Default::default(),
                    engage: engaged,
                })
                .ok();
            engage_compat_pub
                .publish(&Engage {
                    stamp: Default::default(),
                    engage: engaged,
                })
                .ok();

            // 8.1g — Emergency API
            emergency_api_pub
                .publish(&Emergency {
                    stamp: Default::default(),
                    emergency: island.mrm_handler.state() == MRM_STATE_OPERATING,
                })
                .ok();

            // ── Phase 8.2 — Debug/diagnostic topics ─────────────────

            // 8.2a — Control validator debug (empty markers, real status)
            let empty_markers = MarkerArray {
                markers: Default::default(),
            };
            cv_debug_marker_pub.publish(&empty_markers).ok();
            cv_output_markers_pub.publish(&empty_markers).ok();
            cv_virtual_wall_pub.publish(&empty_markers).ok();

            let cv_status = island.control_validator.status();
            cv_validation_status_pub
                .publish(&ControlValidatorStatus {
                    stamp: Default::default(),
                    is_valid_max_distance_deviation: true,
                    is_valid_acc: cv_status.is_valid_acc,
                    is_rolling_back: cv_status.is_rolling_back,
                    is_over_velocity: cv_status.is_over_velocity,
                    is_valid_lateral_jerk: cv_status.is_valid_lateral_jerk,
                    has_overrun_stop_point: false,
                    will_overrun_stop_point: false,
                    is_valid_latency: true,
                    is_valid_yaw: true,
                    is_warn_yaw: false,
                    max_distance_deviation: 0.0,
                    steering_rate: cv_status.steering_rate,
                    lateral_jerk: cv_status.lateral_jerk,
                    desired_acc: cv_status.desired_acc,
                    measured_acc: cv_status.measured_acc,
                    target_vel: cv_status.target_vel,
                    vehicle_vel: cv_status.vehicle_vel,
                    dist_to_stop: 0.0,
                    pred_dist_to_stop: 0.0,
                    nearest_trajectory_vel: 0.0,
                    latency: 0.0,
                    yaw_deviation: 0.0,
                    invalid_count: cv_status.invalid_count as i64,
                })
                .ok();

            // 8.2b — Vehicle cmd gate filter debug (all inactive)
            filter_activated_pub
                .publish(&IsFilterActivated {
                    stamp: Default::default(),
                    is_activated: false,
                    is_activated_on_steering: false,
                    is_activated_on_steering_rate: false,
                    is_activated_on_speed: false,
                    is_activated_on_acceleration: false,
                    is_activated_on_jerk: false,
                })
                .ok();
            filter_flag_pub
                .publish(&BoolStamped {
                    stamp: Default::default(),
                    data: false,
                })
                .ok();
            filter_marker_pub.publish(&empty_markers).ok();
            filter_marker_raw_pub.publish(&empty_markers).ok();

            // 8.2c — Remaining debug
            op_mode_debug_pub
                .publish(&OperationModeTransitionManagerDebug {
                    stamp: Default::default(),
                    status: Default::default(),
                    in_autoware_control: true,
                    in_transition: false,
                    is_all_ok: true,
                    engage_allowed_for_stopped_vehicle: true,
                    trajectory_available_ok: true,
                    lateral_deviation_ok: true,
                    yaw_deviation_ok: true,
                    speed_upper_deviation_ok: true,
                    speed_lower_deviation_ok: true,
                    stop_ok: true,
                    large_acceleration_ok: true,
                    large_lateral_acceleration_ok: true,
                    large_lateral_acceleration_diff_ok: true,
                    current_speed: island.current_velocity,
                    target_control_speed: 0.0,
                    target_planning_speed: 0.0,
                    target_control_acceleration: 0.0,
                    lateral_acceleration: 0.0,
                    lateral_acceleration_deviation: 0.0,
                    lateral_deviation: 0.0,
                    yaw_deviation: 0.0,
                    speed_deviation: 0.0,
                })
                .ok();
            published_time_pub
                .publish(&PublishedTime {
                    header: Default::default(),
                    published_stamp: Default::default(),
                })
                .ok();
        });
    })?;
    info!("30 Hz control loop ready");

    info!("Executor ready — spinning...");
    executor.spin_blocking(SpinOptions::default())?;

    Ok(())
}
