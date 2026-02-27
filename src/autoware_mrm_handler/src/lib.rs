#![no_std]

use autoware_adapi_v1_msgs::msg::MrmState;
use autoware_vehicle_msgs::msg::{GearCommand, HazardLightsCommand};
use tier4_system_msgs::msg::OperationModeAvailability;

// MrmState::state constants (private in generated module)
const MRM_STATE_NORMAL: u16 = 1;
const MRM_STATE_OPERATING: u16 = 2;
const MRM_STATE_SUCCEEDED: u16 = 3;
const MRM_STATE_FAILED: u16 = 4;

// MrmState::behavior constants
const BEHAVIOR_NONE: u16 = 1;
const BEHAVIOR_EMERGENCY_STOP: u16 = 2;
const BEHAVIOR_COMFORTABLE_STOP: u16 = 3;

// HazardLightsCommand::command constants
const HAZARD_ENABLE: u8 = 2;

// GearCommand::command constants
const GEAR_PARK: u8 = 22;

/// Velocity threshold (m/s) below which the vehicle is considered stopped.
const STOPPED_VELOCITY_THRESHOLD: f64 = 0.001;

/// Parameters for the MRM handler.
#[derive(Debug, Clone)]
pub struct Params {
    /// Velocity threshold (m/s) to consider vehicle stopped.
    pub stopped_velocity_threshold: f64,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            stopped_velocity_threshold: STOPPED_VELOCITY_THRESHOLD,
        }
    }
}

/// Commands emitted by the MRM handler on each update.
#[derive(Debug, Clone)]
pub struct MrmOutput {
    /// Current MRM state.
    pub mrm_state: MrmState,
    /// Hazard lights command.
    pub hazard_lights: HazardLightsCommand,
    /// Gear command (PARK when succeeded).
    pub gear: GearCommand,
    /// If `Some(true)`, activate emergency stop operator. If `Some(false)`, deactivate.
    pub emergency_stop_operate: Option<bool>,
    /// If `Some(true)`, activate comfortable stop operator. If `Some(false)`, deactivate.
    pub comfortable_stop_operate: Option<bool>,
}

/// MRM handler state machine.
///
/// Orchestrates the emergency response pipeline. Receives system health signals
/// (OperationModeAvailability) and vehicle odometry, decides which MRM behavior
/// to activate, and emits commands for the operators.
///
/// State machine:
/// ```text
/// NORMAL ──[emergency detected]──→ MRM_OPERATING
///   ↑                                    │
///   │                              ┌─────┴─────┐
///   │                              ▼           ▼
///   └──[recovered]──── MRM_SUCCEEDED    MRM_FAILED
/// ```
#[derive(Debug, Clone)]
pub struct MrmHandler {
    params: Params,
    state: u16,
    behavior: u16,
    /// Current vehicle velocity (m/s), updated from odometry.
    current_velocity: f64,
    /// Whether operation modes are currently available.
    is_operation_available: bool,
    /// Whether comfortable stop is available.
    comfortable_stop_available: bool,
    /// Whether emergency stop is available.
    emergency_stop_available: bool,
}

impl MrmHandler {
    pub fn new(params: Params) -> Self {
        Self {
            params,
            state: MRM_STATE_NORMAL,
            behavior: BEHAVIOR_NONE,
            current_velocity: 0.0,
            is_operation_available: true,
            comfortable_stop_available: false,
            emergency_stop_available: false,
        }
    }

    /// Update vehicle velocity from odometry.
    pub fn update_velocity(&mut self, velocity: f64) {
        self.current_velocity = velocity.abs();
    }

    /// Update operation mode availability from the watchdog/system.
    pub fn update_availability(&mut self, availability: &OperationModeAvailability) {
        self.is_operation_available = availability.autonomous;
        self.comfortable_stop_available = availability.comfortable_stop;
        self.emergency_stop_available = availability.emergency_stop;
    }

    /// Run the state machine and return the output commands.
    ///
    /// This should be called periodically (e.g. at 30 Hz). Returns the current
    /// MRM state, hazard/gear commands, and operator activate/deactivate signals.
    pub fn update(&mut self) -> MrmOutput {
        match self.state {
            MRM_STATE_NORMAL => self.update_normal(),
            MRM_STATE_OPERATING => self.update_operating(),
            // Terminal states — no transitions
            MRM_STATE_SUCCEEDED | MRM_STATE_FAILED => self.make_output(None, None),
            _ => self.make_output(None, None),
        }
    }

    /// Current MRM state constant.
    pub fn state(&self) -> u16 {
        self.state
    }

    /// Current MRM behavior constant.
    pub fn behavior(&self) -> u16 {
        self.behavior
    }

    /// Whether the vehicle is considered stopped.
    pub fn is_stopped(&self) -> bool {
        self.current_velocity < self.params.stopped_velocity_threshold
    }

    // ── State handlers ─────────────────────────────────────────────────

    fn update_normal(&mut self) -> MrmOutput {
        if !self.is_operation_available {
            // Emergency detected — enter MRM
            self.state = MRM_STATE_OPERATING;
            let behavior = self.select_behavior();
            self.behavior = behavior;
            return self.activate_behavior(behavior);
        }
        self.make_output(None, None)
    }

    fn update_operating(&mut self) -> MrmOutput {
        // Check if operation has recovered
        if self.is_operation_available {
            self.state = MRM_STATE_NORMAL;
            self.behavior = BEHAVIOR_NONE;
            return self.deactivate_all();
        }

        // Check if vehicle has stopped → success
        if self.is_stopped() {
            self.state = MRM_STATE_SUCCEEDED;
            return self.make_output(
                Some(false), // deactivate emergency
                Some(false), // deactivate comfortable
            );
        }

        // Check for escalation: if currently on comfortable stop but it
        // is no longer available, escalate to emergency stop
        if self.behavior == BEHAVIOR_COMFORTABLE_STOP && !self.comfortable_stop_available {
            self.behavior = BEHAVIOR_EMERGENCY_STOP;
            return self.make_output(
                Some(true),  // activate emergency
                Some(false), // deactivate comfortable
            );
        }

        // Continue current behavior
        self.make_output(None, None)
    }

    // ── Behavior selection ─────────────────────────────────────────────

    fn select_behavior(&self) -> u16 {
        if self.comfortable_stop_available {
            BEHAVIOR_COMFORTABLE_STOP
        } else {
            BEHAVIOR_EMERGENCY_STOP
        }
    }

    fn activate_behavior(&self, behavior: u16) -> MrmOutput {
        match behavior {
            BEHAVIOR_COMFORTABLE_STOP => self.make_output(Some(false), Some(true)),
            BEHAVIOR_EMERGENCY_STOP => self.make_output(Some(true), Some(false)),
            _ => self.make_output(None, None),
        }
    }

    fn deactivate_all(&self) -> MrmOutput {
        self.make_output(Some(false), Some(false))
    }

    // ── Output construction ────────────────────────────────────────────

    fn make_output(
        &self,
        emergency_stop: Option<bool>,
        comfortable_stop: Option<bool>,
    ) -> MrmOutput {
        MrmOutput {
            mrm_state: MrmState {
                state: self.state,
                behavior: self.behavior,
                ..Default::default()
            },
            hazard_lights: HazardLightsCommand {
                command: if self.state == MRM_STATE_OPERATING || self.state == MRM_STATE_SUCCEEDED {
                    HAZARD_ENABLE
                } else {
                    0
                },
                ..Default::default()
            },
            gear: GearCommand {
                command: if self.state == MRM_STATE_SUCCEEDED {
                    GEAR_PARK
                } else {
                    0
                },
                ..Default::default()
            },
            emergency_stop_operate: emergency_stop,
            comfortable_stop_operate: comfortable_stop,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn available_all() -> OperationModeAvailability {
        OperationModeAvailability {
            autonomous: true,
            stop: true,
            local: true,
            remote: true,
            emergency_stop: true,
            comfortable_stop: true,
            pull_over: true,
            ..Default::default()
        }
    }

    fn unavailable_all() -> OperationModeAvailability {
        OperationModeAvailability::default()
    }

    fn unavailable_no_comfortable() -> OperationModeAvailability {
        OperationModeAvailability {
            autonomous: false,
            emergency_stop: true,
            comfortable_stop: false,
            ..Default::default()
        }
    }

    #[test]
    fn starts_in_normal_state() {
        let handler = MrmHandler::new(Params::default());
        assert_eq!(handler.state(), MRM_STATE_NORMAL);
        assert_eq!(handler.behavior(), BEHAVIOR_NONE);
    }

    #[test]
    fn normal_to_operating_on_unavailability() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(10.0);
        handler.update_availability(&unavailable_all());

        let output = handler.update();
        assert_eq!(handler.state(), MRM_STATE_OPERATING);
        assert_eq!(output.mrm_state.state, MRM_STATE_OPERATING);
    }

    #[test]
    fn selects_comfortable_stop_when_available() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(10.0);

        // Availability with comfortable stop
        let avail = OperationModeAvailability {
            autonomous: false,
            comfortable_stop: true,
            emergency_stop: true,
            ..Default::default()
        };
        handler.update_availability(&avail);

        let output = handler.update();
        assert_eq!(handler.behavior(), BEHAVIOR_COMFORTABLE_STOP);
        assert_eq!(output.comfortable_stop_operate, Some(true));
        assert_eq!(output.emergency_stop_operate, Some(false));
    }

    #[test]
    fn selects_emergency_stop_when_comfortable_unavailable() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(10.0);
        handler.update_availability(&unavailable_no_comfortable());

        let output = handler.update();
        assert_eq!(handler.behavior(), BEHAVIOR_EMERGENCY_STOP);
        assert_eq!(output.emergency_stop_operate, Some(true));
        assert_eq!(output.comfortable_stop_operate, Some(false));
    }

    #[test]
    fn succeeds_when_vehicle_stops() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(10.0);
        handler.update_availability(&unavailable_all());
        handler.update(); // enters MRM_OPERATING

        // Vehicle decelerates to zero
        handler.update_velocity(0.0);
        let output = handler.update();

        assert_eq!(handler.state(), MRM_STATE_SUCCEEDED);
        assert_eq!(output.mrm_state.state, MRM_STATE_SUCCEEDED);
        assert_eq!(output.gear.command, GEAR_PARK);
        assert_eq!(output.hazard_lights.command, HAZARD_ENABLE);
        assert_eq!(output.emergency_stop_operate, Some(false));
        assert_eq!(output.comfortable_stop_operate, Some(false));
    }

    #[test]
    fn succeeded_is_terminal() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(10.0);
        handler.update_availability(&unavailable_all());
        handler.update(); // MRM_OPERATING

        handler.update_velocity(0.0);
        handler.update(); // MRM_SUCCEEDED

        // Further updates should not change state
        handler.update_availability(&available_all());
        handler.update();
        assert_eq!(handler.state(), MRM_STATE_SUCCEEDED);
    }

    #[test]
    fn recovery_from_operating() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(10.0);
        handler.update_availability(&unavailable_all());
        handler.update(); // MRM_OPERATING

        // System recovers
        handler.update_availability(&available_all());
        let output = handler.update();

        assert_eq!(handler.state(), MRM_STATE_NORMAL);
        assert_eq!(handler.behavior(), BEHAVIOR_NONE);
        assert_eq!(output.emergency_stop_operate, Some(false));
        assert_eq!(output.comfortable_stop_operate, Some(false));
    }

    #[test]
    fn escalation_from_comfortable_to_emergency() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(10.0);

        // Start with comfortable stop available
        let avail = OperationModeAvailability {
            autonomous: false,
            comfortable_stop: true,
            emergency_stop: true,
            ..Default::default()
        };
        handler.update_availability(&avail);
        handler.update(); // MRM_OPERATING with COMFORTABLE_STOP

        assert_eq!(handler.behavior(), BEHAVIOR_COMFORTABLE_STOP);

        // Comfortable stop becomes unavailable → escalate
        handler.update_availability(&unavailable_no_comfortable());
        let output = handler.update();

        assert_eq!(handler.behavior(), BEHAVIOR_EMERGENCY_STOP);
        assert_eq!(output.emergency_stop_operate, Some(true));
        assert_eq!(output.comfortable_stop_operate, Some(false));
    }

    #[test]
    fn hazard_lights_during_mrm() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(10.0);

        // Normal — no hazard lights
        let output = handler.update();
        assert_eq!(output.hazard_lights.command, 0);

        // Enter MRM
        handler.update_availability(&unavailable_all());
        let output = handler.update();
        assert_eq!(output.hazard_lights.command, HAZARD_ENABLE);
    }

    #[test]
    fn no_gear_park_until_succeeded() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(10.0);
        handler.update_availability(&unavailable_all());

        let output = handler.update(); // MRM_OPERATING
        assert_eq!(output.gear.command, 0);

        handler.update_velocity(0.0);
        let output = handler.update(); // MRM_SUCCEEDED
        assert_eq!(output.gear.command, GEAR_PARK);
    }

    #[test]
    fn normal_stays_normal_when_available() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_availability(&available_all());

        for _ in 0..10 {
            handler.update();
            assert_eq!(handler.state(), MRM_STATE_NORMAL);
        }
    }

    #[test]
    fn full_lifecycle() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(20.0);
        handler.update_availability(&available_all());

        // 1. Normal operation
        handler.update();
        assert_eq!(handler.state(), MRM_STATE_NORMAL);

        // 2. Heartbeat timeout — enter MRM
        let avail = OperationModeAvailability {
            autonomous: false,
            comfortable_stop: true,
            emergency_stop: true,
            ..Default::default()
        };
        handler.update_availability(&avail);
        handler.update();
        assert_eq!(handler.state(), MRM_STATE_OPERATING);
        assert_eq!(handler.behavior(), BEHAVIOR_COMFORTABLE_STOP);

        // 3. Vehicle decelerating...
        handler.update_velocity(5.0);
        handler.update();
        assert_eq!(handler.state(), MRM_STATE_OPERATING);

        // 4. Comfortable stop fails — escalate
        handler.update_availability(&unavailable_no_comfortable());
        handler.update();
        assert_eq!(handler.behavior(), BEHAVIOR_EMERGENCY_STOP);

        // 5. Vehicle stops
        handler.update_velocity(0.0);
        let output = handler.update();
        assert_eq!(handler.state(), MRM_STATE_SUCCEEDED);
        assert_eq!(output.gear.command, GEAR_PARK);
        assert_eq!(output.hazard_lights.command, HAZARD_ENABLE);

        // 6. Terminal — stays succeeded
        handler.update();
        assert_eq!(handler.state(), MRM_STATE_SUCCEEDED);
    }

    #[test]
    fn velocity_uses_absolute_value() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(-10.0);
        assert!(!handler.is_stopped());

        handler.update_velocity(-0.0005);
        assert!(handler.is_stopped());
    }

    #[test]
    fn operating_continues_without_state_change() {
        let mut handler = MrmHandler::new(Params::default());
        handler.update_velocity(10.0);
        handler.update_availability(&unavailable_all());
        handler.update(); // MRM_OPERATING

        // Subsequent updates during operation — no operator signals
        let output = handler.update();
        assert_eq!(handler.state(), MRM_STATE_OPERATING);
        assert!(output.emergency_stop_operate.is_none());
        assert!(output.comfortable_stop_operate.is_none());
    }
}
