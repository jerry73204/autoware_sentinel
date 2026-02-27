#![no_std]

use autoware_control_msgs::msg::Control;
use autoware_vehicle_msgs::msg::ControlModeReport;

/// OperationModeState msg constants for `mode_to_msg_constant`.
const OPERATION_MODE_STOP: u8 = 1;
const OPERATION_MODE_AUTONOMOUS: u8 = 2;
const OPERATION_MODE_MANUAL: u8 = 4;

/// Simplified operation modes for the safety island.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OperationMode {
    Stop,
    Autonomous,
    /// Combines upstream LOCAL and REMOTE modes.
    Manual,
}

/// Result of a mode change request.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ModeChangeResult {
    Accepted,
    Rejected,
}

/// Parameters for the operation mode transition manager.
#[derive(Debug, Clone)]
pub struct Params {
    pub allow_autonomous_in_stopped: bool,
    pub stopped_velocity_threshold: f64,
    pub enable_engage_on_driving: bool,
    pub acc_threshold: f64,
    pub speed_upper_threshold: f64,
    pub speed_lower_threshold: f64,
    /// Duration (s) of stable conditions before completing autonomous transition.
    pub stable_check_duration: f64,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            allow_autonomous_in_stopped: true,
            stopped_velocity_threshold: 0.01,
            enable_engage_on_driving: false,
            acc_threshold: 2.0,
            speed_upper_threshold: 10.0,
            speed_lower_threshold: -10.0,
            stable_check_duration: 3.0,
        }
    }
}

/// Diagnostics output from a transition manager update.
#[derive(Debug, Clone)]
pub struct TransitionDiagnostics {
    pub current_mode: OperationMode,
    pub is_in_transition: bool,
    pub is_mode_change_available: bool,
    pub is_mode_change_completed: bool,
}

/// Operation mode transition manager.
///
/// Enforces safety invariants for mode transitions. The key constraint:
/// the vehicle must be stopped to engage autonomous mode (unless
/// `enable_engage_on_driving` is set and all checks pass).
#[derive(Debug, Clone)]
pub struct OperationModeTransitionManager {
    params: Params,
    current_mode: OperationMode,
    target_mode: Option<OperationMode>,
    current_velocity: f64,
    current_acceleration_cmd: f64,
    stable_elapsed: f64,
}

impl OperationModeTransitionManager {
    pub fn new(params: Params) -> Self {
        Self {
            params,
            current_mode: OperationMode::Stop,
            target_mode: None,
            current_velocity: 0.0,
            current_acceleration_cmd: 0.0,
            stable_elapsed: 0.0,
        }
    }

    /// Update current vehicle velocity from odometry.
    pub fn update_velocity(&mut self, velocity: f64) {
        self.current_velocity = velocity;
    }

    /// Update commanded acceleration from the control command.
    pub fn update_control_cmd(&mut self, cmd: &Control) {
        self.current_acceleration_cmd = cmd.longitudinal.acceleration as f64;
    }

    /// Update vehicle control mode from hardware report.
    pub fn update_control_mode(&mut self, _report: &ControlModeReport) {
        // Stored for future node-layer use; algorithm checks use velocity/accel.
    }

    /// Request a mode change. Returns whether the request was accepted.
    pub fn request_mode_change(&mut self, mode: OperationMode) -> ModeChangeResult {
        if mode == self.current_mode {
            return ModeChangeResult::Accepted;
        }

        match mode {
            OperationMode::Autonomous => {
                if !self.is_autonomous_available() {
                    return ModeChangeResult::Rejected;
                }
                self.target_mode = Some(mode);
                self.stable_elapsed = 0.0;
                ModeChangeResult::Accepted
            }
            _ => {
                self.current_mode = mode;
                self.target_mode = None;
                ModeChangeResult::Accepted
            }
        }
    }

    /// Periodic tick. Drives transition state machine forward.
    pub fn update(&mut self, dt: f64) -> TransitionDiagnostics {
        let is_mode_change_available = self.is_autonomous_available();

        let mut is_mode_change_completed = false;
        if let Some(target) = self.target_mode {
            if target == self.current_mode {
                self.target_mode = None;
                is_mode_change_completed = true;
            } else if target == OperationMode::Autonomous {
                if self.is_autonomous_completed(dt) {
                    self.current_mode = OperationMode::Autonomous;
                    self.target_mode = None;
                    is_mode_change_completed = true;
                }
            } else {
                self.current_mode = target;
                self.target_mode = None;
                is_mode_change_completed = true;
            }
        }

        let is_in_transition =
            self.target_mode.is_some() && self.target_mode != Some(self.current_mode);

        TransitionDiagnostics {
            current_mode: self.current_mode,
            is_in_transition,
            is_mode_change_available,
            is_mode_change_completed,
        }
    }

    pub fn current_mode(&self) -> OperationMode {
        self.current_mode
    }

    /// Map an `OperationMode` to the `OperationModeState` msg constant.
    pub fn mode_to_msg_constant(mode: OperationMode) -> u8 {
        match mode {
            OperationMode::Stop => OPERATION_MODE_STOP,
            OperationMode::Autonomous => OPERATION_MODE_AUTONOMOUS,
            OperationMode::Manual => OPERATION_MODE_MANUAL,
        }
    }

    fn is_stopped(&self) -> bool {
        self.current_velocity.abs() <= self.params.stopped_velocity_threshold
    }

    fn is_autonomous_available(&self) -> bool {
        if self.is_stopped() && self.params.allow_autonomous_in_stopped {
            return true;
        }
        if !self.params.enable_engage_on_driving {
            return false;
        }
        if self.current_acceleration_cmd.abs() > self.params.acc_threshold {
            return false;
        }
        true
    }

    fn is_autonomous_completed(&mut self, dt: f64) -> bool {
        if self.is_stopped() && self.params.allow_autonomous_in_stopped {
            return true;
        }
        if !self.is_autonomous_available() {
            self.stable_elapsed = 0.0;
            return false;
        }
        self.stable_elapsed += dt;
        self.stable_elapsed >= self.params.stable_check_duration
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1.0 / 30.0;

    #[test]
    fn refuses_autonomous_when_moving() {
        let mut mgr = OperationModeTransitionManager::new(Params::default());
        mgr.update_velocity(5.0);
        let result = mgr.request_mode_change(OperationMode::Autonomous);
        assert_eq!(result, ModeChangeResult::Rejected);
    }

    #[test]
    fn allows_transition_when_stopped() {
        let mut mgr = OperationModeTransitionManager::new(Params::default());
        mgr.update_velocity(0.0);
        let result = mgr.request_mode_change(OperationMode::Autonomous);
        assert_eq!(result, ModeChangeResult::Accepted);
    }

    #[test]
    fn allows_transition_when_checks_pass() {
        let params = Params {
            enable_engage_on_driving: true,
            ..Default::default()
        };
        let mut mgr = OperationModeTransitionManager::new(params);
        mgr.update_velocity(5.0);
        let mut cmd = Control::default();
        cmd.longitudinal.acceleration = 1.0;
        mgr.update_control_cmd(&cmd);
        let result = mgr.request_mode_change(OperationMode::Autonomous);
        assert_eq!(result, ModeChangeResult::Accepted);
    }

    #[test]
    fn rejects_large_acceleration() {
        let params = Params {
            enable_engage_on_driving: true,
            ..Default::default()
        };
        let mut mgr = OperationModeTransitionManager::new(params);
        mgr.update_velocity(5.0);
        let mut cmd = Control::default();
        cmd.longitudinal.acceleration = 3.0;
        mgr.update_control_cmd(&cmd);
        let result = mgr.request_mode_change(OperationMode::Autonomous);
        assert_eq!(result, ModeChangeResult::Rejected);
    }

    #[test]
    fn transition_completes_after_stable_duration() {
        let params = Params {
            enable_engage_on_driving: true,
            stable_check_duration: 3.0,
            ..Default::default()
        };
        let mut mgr = OperationModeTransitionManager::new(params);
        mgr.update_velocity(5.0);
        let mut cmd = Control::default();
        cmd.longitudinal.acceleration = 1.0;
        mgr.update_control_cmd(&cmd);
        mgr.request_mode_change(OperationMode::Autonomous);

        let mut completed = false;
        for _ in 0..100 {
            let diag = mgr.update(DT);
            if diag.is_mode_change_completed {
                completed = true;
                break;
            }
        }
        assert!(
            completed,
            "transition should complete after stable duration"
        );
        assert_eq!(mgr.current_mode(), OperationMode::Autonomous);
    }

    #[test]
    fn transition_resets_on_instability() {
        let params = Params {
            enable_engage_on_driving: true,
            stable_check_duration: 3.0,
            ..Default::default()
        };
        let mut mgr = OperationModeTransitionManager::new(params);
        mgr.update_velocity(5.0);
        let mut cmd = Control::default();
        cmd.longitudinal.acceleration = 1.0;
        mgr.update_control_cmd(&cmd);
        mgr.request_mode_change(OperationMode::Autonomous);

        // Tick for 2 seconds (60 ticks at 30 Hz)
        for _ in 0..60 {
            mgr.update(DT);
        }
        assert_eq!(
            mgr.current_mode(),
            OperationMode::Stop,
            "should not have completed yet"
        );

        // Spike acceleration above threshold — resets stable timer
        cmd.longitudinal.acceleration = 3.0;
        mgr.update_control_cmd(&cmd);
        mgr.update(DT);

        // Restore good acceleration
        cmd.longitudinal.acceleration = 1.0;
        mgr.update_control_cmd(&cmd);

        // 80 ticks at 30 Hz = 2.67s — not enough after reset
        for _ in 0..80 {
            mgr.update(DT);
        }
        assert_eq!(
            mgr.current_mode(),
            OperationMode::Stop,
            "timer should have reset"
        );
    }

    #[test]
    fn mode_to_msg_constant_mapping() {
        assert_eq!(
            OperationModeTransitionManager::mode_to_msg_constant(OperationMode::Stop),
            1
        );
        assert_eq!(
            OperationModeTransitionManager::mode_to_msg_constant(OperationMode::Autonomous),
            2
        );
        assert_eq!(
            OperationModeTransitionManager::mode_to_msg_constant(OperationMode::Manual),
            4
        );
    }

    #[test]
    fn starts_in_stop_mode() {
        let mgr = OperationModeTransitionManager::new(Params::default());
        assert_eq!(mgr.current_mode(), OperationMode::Stop);
    }
}
