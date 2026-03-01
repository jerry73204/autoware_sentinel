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

use std::cell::RefCell;
use std::sync::OnceLock;
use std::time::Instant;

use log::info;
use nros::prelude::*;

// Algorithm crates
use autoware_control_validator::{ControlValidator, ValidatorParams};
use autoware_heartbeat_watchdog::HeartbeatWatchdog;
use autoware_mrm_comfortable_stop_operator::ComfortableStopOperator;
use autoware_mrm_emergency_stop_operator::EmergencyStopOperator;
use autoware_mrm_handler::MrmHandler;
use autoware_operation_mode_transition_manager::OperationModeTransitionManager;
use autoware_shift_decider::ShiftDecider;
use autoware_stop_filter::StopFilter;
use autoware_twist2accel::Twist2Accel;
use autoware_vehicle_cmd_gate::gate::SourceCommands;
use autoware_vehicle_cmd_gate::{GateParams, VehicleCmdGate};
use autoware_vehicle_velocity_converter::VehicleVelocityConverter;

// Message types
use autoware_adapi_v1_msgs::msg::{Heartbeat, MrmState, OperationModeState};
use autoware_adapi_v1_msgs::srv::{ChangeOperationMode, ChangeOperationModeResponse};
use autoware_control_msgs::msg::Control;
use autoware_system_msgs::msg::AutowareState;
use autoware_vehicle_msgs::msg::{
    GearCommand, GearReport, HazardLightsCommand, TurnIndicatorsCommand, VelocityReport,
};
use geometry_msgs::msg::{Accel, Twist};
use tier4_system_msgs::msg::OperationModeAvailability;

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
    fn new() -> Self {
        let mut watchdog =
            HeartbeatWatchdog::new(autoware_heartbeat_watchdog::Params { timeout_ms: 5000 });
        // Pre-seed with boot time to avoid immediate timeout.
        // Gives Autoware time to initialize and start sending heartbeats.
        watchdog.on_heartbeat(0);

        Self {
            // Sensing
            velocity_converter: VehicleVelocityConverter::new(1.0, 0.2, 0.1),
            stop_filter: StopFilter::new(0.1, 0.02),
            twist2accel: Twist2Accel::new(autoware_twist2accel::Params::default()),
            prev_stamp: None,
            current_velocity: 0.0,
            is_stopped: true,
            twist: Twist::default(),
            accel: Accel::default(),
            accel_covariance: [0.0; 36],

            // Heartbeat
            watchdog,

            // MRM chain
            mrm_handler: MrmHandler::new(autoware_mrm_handler::Params::default()),
            emergency_stop: EmergencyStopOperator::new(
                autoware_mrm_emergency_stop_operator::Params::default(),
            ),
            comfortable_stop: ComfortableStopOperator::new(
                autoware_mrm_comfortable_stop_operator::Params::default(),
            ),

            // Command output
            cmd_gate: VehicleCmdGate::new(GateParams {
                filter: autoware_vehicle_cmd_gate::filter::FilterParams::default(),
                arbiter: autoware_vehicle_cmd_gate::gate::ArbiterParams::default(),
                heartbeat_timeout_ms: 5000,
            }),
            shift_decider: ShiftDecider::new(true),
            auto_control: Control::default(),
            autoware_state: AutowareState::default(),
            gear_report: GearReport::default(),

            // Validation
            control_validator: ControlValidator::new(ValidatorParams::default()),
            op_mode_mgr: OperationModeTransitionManager::new(
                autoware_operation_mode_transition_manager::Params::default(),
            ),
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
    // Initialize static state (before any callbacks can fire)
    *ISLAND.0.borrow_mut() = Some(SafetyIsland::new());

    let config = ExecutorConfig::from_env().node_name("sentinel");
    let mut executor = Executor::<_, 16, 16384>::open(&config)?;

    // --- Create publishers (node borrows executor, dropped after) ---
    let (mrm_state_pub, hazard_pub, gear_pub, control_pub, turn_pub, op_mode_pub) = {
        let mut node = executor.create_node("sentinel")?;
        (
            // Autoware-compatible output topic names
            node.create_publisher::<MrmState>("/system/fail_safe/mrm_state")?,
            node.create_publisher::<HazardLightsCommand>("/control/command/hazard_lights_cmd")?,
            node.create_publisher::<GearCommand>("/control/command/gear_cmd")?,
            node.create_publisher::<Control>("/control/command/control_cmd")?,
            node.create_publisher::<TurnIndicatorsCommand>("/control/command/turn_indicators_cmd")?,
            // Engagement flow: operation mode state
            node.create_publisher::<OperationModeState>("/api/operation_mode/state")?,
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
            op_mode_pub
                .publish(&OperationModeState {
                    stamp: Default::default(),
                    mode: OP_MODE_AUTONOMOUS,
                    is_autoware_control_enabled: true,
                    is_in_transition: false,
                    is_stop_mode_available: true,
                    is_autonomous_mode_available: true,
                    is_local_mode_available: true,
                    is_remote_mode_available: true,
                })
                .ok();
        });
    })?;
    info!("30 Hz control loop ready");

    info!("Executor ready — spinning...");
    executor.spin_blocking(SpinOptions::default())?;

    Ok(())
}
