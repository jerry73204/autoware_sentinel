//! Source arbiter — command source selection with priority-based arbitration.
//!
//! Selects which command source (autonomous, external, emergency) controls the
//! vehicle based on gate mode, engagement state, and MRM state.

use autoware_control_msgs::msg::Control;
use autoware_vehicle_msgs::msg::{GearCommand, HazardLightsCommand, TurnIndicatorsCommand};

/// Command source identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CommandSource {
    #[default]
    Autonomous,
    External,
    Emergency,
}

/// Gate operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GateMode {
    #[default]
    Auto,
    External,
}

/// Bundle of commands from a single source.
#[derive(Debug, Clone, Default)]
pub struct SourceCommands {
    pub control: Control,
    pub gear: GearCommand,
    pub turn_indicators: TurnIndicatorsCommand,
    pub hazard_lights: HazardLightsCommand,
}

/// Parameters for the source arbiter.
#[derive(Debug, Clone)]
pub struct ArbiterParams {
    /// Deceleration (m/s², negative) applied when not engaged.
    pub stop_hold_accel: f32,
    /// Deceleration (m/s², negative) for emergency stop commands.
    pub emergency_accel: f32,
}

impl Default for ArbiterParams {
    fn default() -> Self {
        Self {
            stop_hold_accel: -1.5,
            emergency_accel: -2.4,
        }
    }
}

/// Source arbiter — selects command source based on priority.
#[derive(Debug, Clone)]
pub struct SourceArbiter {
    params: ArbiterParams,
    auto_cmds: SourceCommands,
    external_cmds: SourceCommands,
    emergency_cmds: SourceCommands,
    gate_mode: GateMode,
    engaged: bool,
    system_emergency: bool,
}

impl SourceArbiter {
    pub fn new(params: ArbiterParams) -> Self {
        Self {
            params,
            auto_cmds: SourceCommands::default(),
            external_cmds: SourceCommands::default(),
            emergency_cmds: SourceCommands::default(),
            gate_mode: GateMode::Auto,
            engaged: false,
            system_emergency: false,
        }
    }

    // ── Setters ─────────────────────────────────────────────────────

    pub fn set_autonomous(&mut self, cmds: SourceCommands) {
        self.auto_cmds = cmds;
    }

    pub fn set_external(&mut self, cmds: SourceCommands) {
        self.external_cmds = cmds;
    }

    pub fn set_emergency(&mut self, cmds: SourceCommands) {
        self.emergency_cmds = cmds;
    }

    pub fn set_gate_mode(&mut self, mode: GateMode) {
        self.gate_mode = mode;
    }

    pub fn set_engaged(&mut self, engaged: bool) {
        self.engaged = engaged;
    }

    pub fn set_system_emergency(&mut self, is_emergency: bool) {
        self.system_emergency = is_emergency;
    }

    pub fn is_engaged(&self) -> bool {
        self.engaged
    }

    pub fn is_system_emergency(&self) -> bool {
        self.system_emergency
    }

    // ── Selection ───────────────────────────────────────────────────

    /// Select the active command source and return its commands.
    ///
    /// Priority:
    /// 1. Emergency (if system emergency active)
    /// 2. External (if gate mode is External)
    /// 3. Autonomous (default)
    ///
    /// If not engaged (and not emergency), longitudinal is replaced with a stop command.
    pub fn select(&self) -> (CommandSource, SourceCommands) {
        // Priority 1: Emergency overrides everything
        if self.system_emergency {
            return (CommandSource::Emergency, self.emergency_cmds.clone());
        }

        // Priority 2/3: Based on gate mode
        let (source, mut cmds) = match self.gate_mode {
            GateMode::External => (CommandSource::External, self.external_cmds.clone()),
            GateMode::Auto => (CommandSource::Autonomous, self.auto_cmds.clone()),
        };

        // If not engaged, replace longitudinal with stop hold command
        if !self.engaged {
            cmds.control.longitudinal.velocity = 0.0;
            cmds.control.longitudinal.acceleration = self.params.stop_hold_accel;
            cmds.control.longitudinal.jerk = 0.0;
        }

        (source, cmds)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_source_cmds(vel: f32, accel: f32, steer: f32) -> SourceCommands {
        let mut cmds = SourceCommands::default();
        cmds.control.longitudinal.velocity = vel;
        cmds.control.longitudinal.acceleration = accel;
        cmds.control.lateral.steering_tire_angle = steer;
        cmds
    }

    #[test]
    fn auto_mode_selects_autonomous() {
        let mut arbiter = SourceArbiter::new(ArbiterParams::default());
        arbiter.set_engaged(true);
        arbiter.set_autonomous(make_source_cmds(10.0, 1.0, 0.1));
        arbiter.set_external(make_source_cmds(5.0, 0.5, 0.2));

        let (source, cmds) = arbiter.select();
        assert_eq!(source, CommandSource::Autonomous);
        assert_eq!(cmds.control.longitudinal.velocity, 10.0);
    }

    #[test]
    fn external_mode_selects_external() {
        let mut arbiter = SourceArbiter::new(ArbiterParams::default());
        arbiter.set_engaged(true);
        arbiter.set_gate_mode(GateMode::External);
        arbiter.set_autonomous(make_source_cmds(10.0, 1.0, 0.1));
        arbiter.set_external(make_source_cmds(5.0, 0.5, 0.2));

        let (source, cmds) = arbiter.select();
        assert_eq!(source, CommandSource::External);
        assert_eq!(cmds.control.longitudinal.velocity, 5.0);
    }

    #[test]
    fn emergency_overrides_auto() {
        let mut arbiter = SourceArbiter::new(ArbiterParams::default());
        arbiter.set_engaged(true);
        arbiter.set_system_emergency(true);
        arbiter.set_autonomous(make_source_cmds(10.0, 1.0, 0.1));
        arbiter.set_emergency(make_source_cmds(0.0, -2.4, 0.0));

        let (source, cmds) = arbiter.select();
        assert_eq!(source, CommandSource::Emergency);
        assert_eq!(cmds.control.longitudinal.velocity, 0.0);
        assert_eq!(cmds.control.longitudinal.acceleration, -2.4);
    }

    #[test]
    fn emergency_overrides_external() {
        let mut arbiter = SourceArbiter::new(ArbiterParams::default());
        arbiter.set_engaged(true);
        arbiter.set_gate_mode(GateMode::External);
        arbiter.set_system_emergency(true);
        arbiter.set_external(make_source_cmds(5.0, 0.5, 0.2));
        arbiter.set_emergency(make_source_cmds(0.0, -2.4, 0.0));

        let (source, _) = arbiter.select();
        assert_eq!(source, CommandSource::Emergency);
    }

    #[test]
    fn not_engaged_produces_stop() {
        let mut arbiter = SourceArbiter::new(ArbiterParams::default());
        // Not engaged (default)
        arbiter.set_autonomous(make_source_cmds(10.0, 1.0, 0.1));

        let (source, cmds) = arbiter.select();
        assert_eq!(source, CommandSource::Autonomous);
        assert_eq!(cmds.control.longitudinal.velocity, 0.0);
        assert_eq!(cmds.control.longitudinal.acceleration, -1.5);
        // Lateral should be preserved
        assert_eq!(cmds.control.lateral.steering_tire_angle, 0.1);
    }

    #[test]
    fn emergency_bypasses_engagement_check() {
        let mut arbiter = SourceArbiter::new(ArbiterParams::default());
        // Not engaged, but emergency
        arbiter.set_system_emergency(true);
        arbiter.set_emergency(make_source_cmds(0.0, -2.4, 0.0));

        let (source, cmds) = arbiter.select();
        assert_eq!(source, CommandSource::Emergency);
        // Emergency commands passed through even when not engaged
        assert_eq!(cmds.control.longitudinal.acceleration, -2.4);
    }

    #[test]
    fn emergency_clears_and_reverts() {
        let mut arbiter = SourceArbiter::new(ArbiterParams::default());
        arbiter.set_engaged(true);
        arbiter.set_autonomous(make_source_cmds(10.0, 1.0, 0.1));
        arbiter.set_emergency(make_source_cmds(0.0, -2.4, 0.0));

        // Enter emergency
        arbiter.set_system_emergency(true);
        let (source, _) = arbiter.select();
        assert_eq!(source, CommandSource::Emergency);

        // Clear emergency
        arbiter.set_system_emergency(false);
        let (source, cmds) = arbiter.select();
        assert_eq!(source, CommandSource::Autonomous);
        assert_eq!(cmds.control.longitudinal.velocity, 10.0);
    }

    #[test]
    fn gear_and_signals_from_selected_source() {
        let mut arbiter = SourceArbiter::new(ArbiterParams::default());
        arbiter.set_engaged(true);

        let mut auto_cmds = make_source_cmds(10.0, 1.0, 0.0);
        auto_cmds.gear.command = 2; // DRIVE
        auto_cmds.turn_indicators.command = 2; // ENABLE_LEFT
        arbiter.set_autonomous(auto_cmds);

        let (_, cmds) = arbiter.select();
        assert_eq!(cmds.gear.command, 2);
        assert_eq!(cmds.turn_indicators.command, 2);
    }
}
