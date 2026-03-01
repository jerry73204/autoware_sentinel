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

//! Autoware Sentinel — Safety Island Application
//!
//! Composes all algorithm crates into a single Zephyr binary with nros
//! pub/sub wiring. Runs on a dedicated safety MCU to independently bring
//! the vehicle to a safe stop when the main compute fails.
//!
//! All algorithm instances and shared data live in a single statically-
//! allocated [`SafetyIsland`] struct — no heap allocation, no `Rc`, no `Arc`.
//! The nros executor is single-threaded, so `RefCell` runtime borrow
//! checking is sufficient for safe interior mutability.
//!
//! Data flow (per 30 Hz control period):
//! ```text
//! Subscriptions (event-driven):
//!   /vehicle/status/velocity_status → VelocityConverter → StopFilter → Twist2Accel
//!   /heartbeat                      → HeartbeatWatchdog
//!   /control/command/control_cmd    → store latest
//!   /system/autoware_state          → store latest
//!   /vehicle/status/gear_status     → store latest
//!
//! Timer (30 Hz, deterministic order):
//!   1. Watchdog check → MRM handler → operators
//!   2. Command gate (autonomous + emergency arbitration)
//!   3. Control validator + operation mode manager
//!   4. Publish: control, MRM state, gear, hazard lights, turn indicators
//! ```

#![no_std]

use core::cell::RefCell;

use log::info;
use nros::{Executor, ExecutorConfig, NodeError, TimerDuration};

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
use autoware_adapi_v1_msgs::msg::{Heartbeat, MrmState};
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
/// Statically allocated — no heap, no `Rc`, no `Arc`. Subscription callbacks
/// write latest received data; the 30 Hz timer runs the full control pipeline
/// in deterministic order: sense → MRM → gate → validate → publish.
struct SafetyIsland {
    // --- Sensing (6.3) ---
    velocity_converter: VehicleVelocityConverter,
    stop_filter: StopFilter,
    twist2accel: Twist2Accel,
    prev_stamp: Option<(i32, u32)>,
    current_velocity: f64,
    is_stopped: bool,
    twist: Twist,
    accel: Accel,
    accel_covariance: [f64; 36],

    // --- Heartbeat (6.4) ---
    watchdog: HeartbeatWatchdog,

    // --- MRM chain (6.4) ---
    mrm_handler: MrmHandler,
    emergency_stop: EmergencyStopOperator,
    comfortable_stop: ComfortableStopOperator,

    // --- Command output (6.5) ---
    cmd_gate: VehicleCmdGate,
    shift_decider: ShiftDecider,
    auto_control: Control,
    autoware_state: AutowareState,
    gear_report: GearReport,

    // --- Validation (6.6) ---
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
            HeartbeatWatchdog::new(autoware_heartbeat_watchdog::Params { timeout_ms: 500 });
        // Pre-seed with boot time to avoid immediate timeout.
        // Gives main compute 500 ms from boot to start sending heartbeats.
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
                heartbeat_timeout_ms: 500,
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

/// Zephyr uptime in milliseconds.
fn now_ms() -> u64 {
    zephyr::sys::uptime_get() as u64
}

// ============================================================================
// Entry point
// ============================================================================

#[unsafe(no_mangle)]
extern "C" fn rust_main() {
    unsafe {
        zephyr::set_logger().ok();
    }

    info!("Autoware Sentinel — Safety Island");
    info!("Board: {}", zephyr::kconfig::CONFIG_BOARD);

    if let Err(e) = run() {
        log::error!("Fatal: {:?}", e);
    }
}

fn run() -> Result<(), NodeError> {
    // Initialize static state (before any callbacks can fire)
    *ISLAND.0.borrow_mut() = Some(SafetyIsland::new());

    let config = ExecutorConfig::new("tcp/192.0.2.2:7447");
    let mut executor = Executor::<_, 16, 16384>::open(&config)?;

    // --- Create publishers (node borrows executor, dropped after) ---
    let (mrm_state_pub, hazard_pub, gear_pub, control_pub, turn_pub) = {
        let mut node = executor.create_node("sentinel")?;
        (
            node.create_publisher::<MrmState>("/output/mrm/state")?,
            node.create_publisher::<HazardLightsCommand>("/output/vehicle/hazard_lights")?,
            node.create_publisher::<GearCommand>("/output/vehicle/gear_cmd")?,
            node.create_publisher::<Control>("/output/vehicle/control_cmd")?,
            node.create_publisher::<TurnIndicatorsCommand>("/output/vehicle/turn_indicators")?,
        )
    };

    // ====================================================================
    // Phase 6.3 — Sensing / input layer
    // ====================================================================
    // VelocityReport → VehicleVelocityConverter → StopFilter → Twist2Accel
    executor.add_subscription::<VelocityReport, _>(
        "/vehicle/status/velocity_status",
        |msg| with_island(|island| island.on_velocity_report(msg)),
    )?;
    info!("6.3: velocity → stop_filter → twist2accel");

    // ====================================================================
    // Phase 6.4 — Heartbeat subscription
    // ====================================================================
    executor.add_subscription::<Heartbeat, _>("/heartbeat", |_msg| {
        with_island(|island| island.watchdog.on_heartbeat(now_ms()));
    })?;

    // ====================================================================
    // Phase 6.5 — Autonomous command subscriptions
    // ====================================================================
    executor.add_subscription::<Control, _>(
        "/control/command/control_cmd",
        |msg| with_island(|island| island.auto_control = msg.clone()),
    )?;
    executor.add_subscription::<AutowareState, _>(
        "/system/autoware_state",
        |msg| with_island(|island| island.autoware_state = msg.clone()),
    )?;
    executor.add_subscription::<GearReport, _>(
        "/vehicle/status/gear_status",
        |msg| with_island(|island| island.gear_report = msg.clone()),
    )?;
    info!("6.4–6.5: heartbeat + autonomous command subscriptions");

    // ====================================================================
    // Phase 6.4–6.6 — 30 Hz main control timer
    // ====================================================================
    executor.add_timer(TimerDuration::from_millis(33), move || {
        with_island(|island| {
            let now = now_ms();

            // ── 6.4: MRM chain ─────────────────────────────────────────

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

            // ── 6.5: Command output ────────────────────────────────────

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

            // ── 6.6: Validation ────────────────────────────────────────

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
        });
    })?;
    info!("6.4–6.6: 30 Hz control loop (MRM + gate + validation)");

    info!("Executor ready — spinning...");
    executor.spin(10);
}
