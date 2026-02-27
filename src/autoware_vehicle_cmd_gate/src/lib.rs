#![no_std]

pub mod filter;
pub mod gate;
pub mod heartbeat;

use autoware_control_msgs::msg::Control;
use autoware_vehicle_msgs::msg::{GearCommand, HazardLightsCommand, TurnIndicatorsCommand};

use filter::{FilterActivated, FilterParams, VehicleCmdFilter};
use gate::{ArbiterParams, GateMode, SourceArbiter, SourceCommands};
use heartbeat::HeartbeatMonitor;

/// Parameters for the vehicle command gate.
#[derive(Debug, Clone)]
pub struct GateParams {
    pub filter: FilterParams,
    pub arbiter: ArbiterParams,
    /// Heartbeat timeout (ms) for each command source. 0 = disabled.
    pub heartbeat_timeout_ms: u64,
}

/// Output of a gate update cycle.
#[derive(Debug, Clone)]
pub struct GateOutput {
    pub control: Control,
    pub gear: GearCommand,
    pub turn_indicators: TurnIndicatorsCommand,
    pub hazard_lights: HazardLightsCommand,
    pub diagnostics: GateDiagnostics,
}

/// Aggregated diagnostics for the gate.
#[derive(Debug, Clone, Default)]
pub struct GateDiagnostics {
    pub active_source: gate::CommandSource,
    pub is_engaged: bool,
    pub is_emergency: bool,
    pub autonomous_heartbeat_ok: bool,
    pub external_heartbeat_ok: bool,
    pub filter_activated: FilterActivated,
}

/// Vehicle command gate — top-level facade combining filter, arbiter, and heartbeats.
#[derive(Debug, Clone)]
pub struct VehicleCmdGate {
    filter: VehicleCmdFilter,
    arbiter: SourceArbiter,
    auto_heartbeat: HeartbeatMonitor,
    external_heartbeat: HeartbeatMonitor,
    prev_timestamp_ms: Option<u64>,
}

impl VehicleCmdGate {
    pub fn new(params: GateParams) -> Self {
        Self {
            filter: VehicleCmdFilter::new(params.filter),
            arbiter: SourceArbiter::new(params.arbiter),
            auto_heartbeat: HeartbeatMonitor::new(params.heartbeat_timeout_ms),
            external_heartbeat: HeartbeatMonitor::new(params.heartbeat_timeout_ms),
            prev_timestamp_ms: None,
        }
    }

    // ── Input setters ───────────────────────────────────────────────

    /// Feed the current vehicle speed (m/s) from odometry.
    pub fn set_current_speed(&mut self, speed: f32) {
        self.filter.set_current_speed(speed);
    }

    /// Feed the current physical steering angle (rad) from steering report.
    pub fn set_current_steer(&mut self, steer: f32) {
        self.filter.set_current_steer(steer);
    }

    /// Feed autonomous source commands. Records heartbeat at `now_ms`.
    pub fn set_autonomous_commands(&mut self, cmds: SourceCommands, now_ms: u64) {
        self.arbiter.set_autonomous(cmds);
        self.auto_heartbeat.on_heartbeat(now_ms);
    }

    /// Feed external (remote) source commands. Records heartbeat at `now_ms`.
    pub fn set_external_commands(&mut self, cmds: SourceCommands, now_ms: u64) {
        self.arbiter.set_external(cmds);
        self.external_heartbeat.on_heartbeat(now_ms);
    }

    /// Feed emergency source commands (from MRM operators).
    pub fn set_emergency_commands(&mut self, cmds: SourceCommands) {
        self.arbiter.set_emergency(cmds);
    }

    /// Set the gate mode (Auto / External).
    pub fn set_gate_mode(&mut self, mode: GateMode) {
        self.arbiter.set_gate_mode(mode);
    }

    /// Set engagement state.
    pub fn set_engaged(&mut self, engaged: bool) {
        self.arbiter.set_engaged(engaged);
    }

    /// Set system emergency state (MRM active with EMERGENCY_STOP behavior).
    pub fn set_system_emergency(&mut self, is_emergency: bool) {
        self.arbiter.set_system_emergency(is_emergency);
    }

    // ── Main update ─────────────────────────────────────────────────

    /// Run one gate cycle at time `now_ms`. Returns the filtered output.
    pub fn update(&mut self, now_ms: u64) -> GateOutput {
        // Compute dt
        let dt = match self.prev_timestamp_ms {
            Some(prev) => {
                let elapsed_ms = now_ms.saturating_sub(prev);
                elapsed_ms as f32 / 1000.0
            }
            None => 0.0,
        };
        self.prev_timestamp_ms = Some(now_ms);

        // Check heartbeats
        let auto_ok = !self.auto_heartbeat.is_timed_out(now_ms);
        let ext_ok = !self.external_heartbeat.is_timed_out(now_ms);

        // Select source
        let (source, cmds) = self.arbiter.select();

        // Apply filter to the selected control command
        let filter_output = if dt > 0.0 {
            self.filter.filter_all(dt, &cmds.control)
        } else {
            // First call — no filtering, just set prev_cmd
            self.filter.init_prev_cmd(&cmds.control);
            filter::FilterOutput {
                control: cmds.control.clone(),
                activated: FilterActivated::default(),
            }
        };

        GateOutput {
            control: filter_output.control,
            gear: cmds.gear,
            turn_indicators: cmds.turn_indicators,
            hazard_lights: cmds.hazard_lights,
            diagnostics: GateDiagnostics {
                active_source: source,
                is_engaged: self.arbiter.is_engaged(),
                is_emergency: self.arbiter.is_system_emergency(),
                autonomous_heartbeat_ok: auto_ok,
                external_heartbeat_ok: ext_ok,
                filter_activated: filter_output.activated,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gate::CommandSource;

    fn default_gate_params() -> GateParams {
        GateParams {
            filter: FilterParams::default(),
            arbiter: ArbiterParams::default(),
            heartbeat_timeout_ms: 500,
        }
    }

    fn make_control(vel: f32, accel: f32, steer: f32) -> Control {
        let mut cmd = Control::default();
        cmd.longitudinal.velocity = vel;
        cmd.longitudinal.acceleration = accel;
        cmd.lateral.steering_tire_angle = steer;
        cmd
    }

    fn make_source_commands(vel: f32, accel: f32, steer: f32) -> SourceCommands {
        SourceCommands {
            control: make_control(vel, accel, steer),
            gear: GearCommand::default(),
            turn_indicators: TurnIndicatorsCommand::default(),
            hazard_lights: HazardLightsCommand::default(),
        }
    }

    #[test]
    fn full_passthrough_engaged_auto_healthy_within_limits() {
        let mut gate = VehicleCmdGate::new(default_gate_params());
        gate.set_engaged(true);
        gate.set_current_speed(5.0);
        gate.set_current_steer(0.0);

        let cmds = make_source_commands(5.0, 1.0, 0.1);
        gate.set_autonomous_commands(cmds, 100);

        // First update — initializes prev_cmd
        let out = gate.update(100);
        assert_eq!(out.diagnostics.active_source, CommandSource::Autonomous);
        assert!(out.diagnostics.is_engaged);

        // Second update — filter active but within limits, should pass through
        let cmds = make_source_commands(5.0, 1.0, 0.1);
        gate.set_autonomous_commands(cmds, 133);
        let out = gate.update(133);
        assert!((out.control.longitudinal.velocity - 5.0).abs() < 0.01);
        assert!((out.control.longitudinal.acceleration - 1.0).abs() < 0.01);
        assert!((out.control.lateral.steering_tire_angle - 0.1).abs() < 0.01);
    }

    #[test]
    fn emergency_override_through_gate() {
        let mut gate = VehicleCmdGate::new(default_gate_params());
        gate.set_engaged(true);
        gate.set_system_emergency(true);

        let auto_cmds = make_source_commands(10.0, 1.0, 0.1);
        gate.set_autonomous_commands(auto_cmds, 100);

        let emrg_cmds = make_source_commands(0.0, -2.4, 0.0);
        gate.set_emergency_commands(emrg_cmds);

        let out = gate.update(100);
        assert_eq!(out.diagnostics.active_source, CommandSource::Emergency);
        // Emergency overrides autonomous
        assert_eq!(out.control.longitudinal.velocity, 0.0);
    }

    #[test]
    fn filter_clamps_excessive_command() {
        let mut gate = VehicleCmdGate::new(default_gate_params());
        gate.set_engaged(true);
        gate.set_current_speed(5.0);
        gate.set_current_steer(0.0);

        // First call to init
        let cmds = make_source_commands(5.0, 0.0, 0.0);
        gate.set_autonomous_commands(cmds, 100);
        gate.update(100);

        // Excessive velocity command — should be clamped to vel_lim (25.0)
        let cmds = make_source_commands(50.0, 0.0, 0.0);
        gate.set_autonomous_commands(cmds, 133);
        let out = gate.update(133);
        assert!(
            out.control.longitudinal.velocity <= 25.0,
            "velocity should be clamped: got {}",
            out.control.longitudinal.velocity,
        );
    }

    #[test]
    fn heartbeat_timeout_reported_in_diagnostics() {
        let mut gate = VehicleCmdGate::new(default_gate_params());
        gate.set_engaged(true);

        let cmds = make_source_commands(5.0, 0.0, 0.0);
        gate.set_autonomous_commands(cmds, 100);

        // Update well past the timeout
        let out = gate.update(700);
        assert!(
            !out.diagnostics.autonomous_heartbeat_ok,
            "heartbeat should have timed out"
        );
    }

    #[test]
    fn not_engaged_produces_stop_command() {
        let mut gate = VehicleCmdGate::new(default_gate_params());
        // Not engaged (default)
        gate.set_current_speed(5.0);

        let cmds = make_source_commands(10.0, 1.0, 0.1);
        gate.set_autonomous_commands(cmds, 100);

        let out = gate.update(100);
        assert!(!out.diagnostics.is_engaged);
        // When not engaged, longitudinal should be replaced with stop
        assert_eq!(out.control.longitudinal.velocity, 0.0);
        assert!(out.control.longitudinal.acceleration < 0.0);
    }
}
