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

//! Port of autoware_shift_decider from Autoware Universe.
//!
//! State machine that decides the gear command based on AutowareState,
//! the control command velocity, and the current gear report.

#![no_std]

use autoware_control_msgs::msg::Control;
use autoware_system_msgs::msg::AutowareState;
use autoware_vehicle_msgs::msg::GearReport;

// Gear constants (from autoware_vehicle_msgs::msg::GearCommand)
pub mod gear {
    pub const DRIVE: u8 = 2;
    pub const REVERSE: u8 = 20;
    pub const PARK: u8 = 22;
}

// AutowareState constants
mod state {
    pub const WAITING_FOR_ROUTE: u8 = 2;
    pub const DRIVING: u8 = 5;
    pub const ARRIVED_GOAL: u8 = 6;
}

/// Velocity threshold to prevent chattering between DRIVE/REVERSE.
const VEL_THRESHOLD: f32 = 0.01;

/// Decides the gear command based on system state and control inputs.
#[derive(Debug, Clone)]
pub struct ShiftDecider {
    park_on_goal: bool,
    prev_command: u8,
}

impl ShiftDecider {
    /// Create a new shift decider.
    pub fn new(park_on_goal: bool) -> Self {
        Self {
            park_on_goal,
            prev_command: gear::PARK,
        }
    }

    /// Decide the gear command given the current inputs.
    ///
    /// Returns the gear command byte (one of the `gear::` constants).
    pub fn decide(
        &mut self,
        autoware_state: &AutowareState,
        control_cmd: &Control,
        current_gear: &GearReport,
    ) -> u8 {
        let command = if autoware_state.state == state::DRIVING {
            if control_cmd.longitudinal.velocity > VEL_THRESHOLD {
                gear::DRIVE
            } else if control_cmd.longitudinal.velocity < -VEL_THRESHOLD {
                gear::REVERSE
            } else {
                self.prev_command
            }
        } else if (autoware_state.state == state::ARRIVED_GOAL
            || autoware_state.state == state::WAITING_FOR_ROUTE)
            && self.park_on_goal
        {
            gear::PARK
        } else {
            current_gear.report
        };

        self.prev_command = command;
        command
    }
}

#[cfg(kani)]
mod verification {
    use super::*;

    /// Helper: create a symbolic f32 from arbitrary bits.
    fn any_f32() -> f32 {
        f32::from_bits(kani::any::<u32>())
    }

    fn make_state(s: u8) -> AutowareState {
        AutowareState {
            state: s,
            ..Default::default()
        }
    }

    fn make_control(velocity: f32) -> Control {
        let mut ctrl = Control::default();
        ctrl.longitudinal.velocity = velocity;
        ctrl
    }

    fn make_gear(report: u8) -> GearReport {
        GearReport {
            report,
            ..Default::default()
        }
    }

    /// Output is always a valid gear (DRIVE, REVERSE, PARK) or matches current_gear.report.
    #[kani::proof]
    fn output_is_valid_gear() {
        let autoware_state_val: u8 = kani::any();
        kani::assume(
            autoware_state_val == 2
                || autoware_state_val == 3
                || autoware_state_val == 5
                || autoware_state_val == 6,
        );

        let gear_report_val: u8 = kani::any();
        kani::assume(
            gear_report_val == gear::DRIVE
                || gear_report_val == gear::REVERSE
                || gear_report_val == gear::PARK,
        );

        let park_on_goal: bool = kani::any();
        let velocity = any_f32();
        kani::assume(!velocity.is_nan() && velocity.is_finite());

        let mut sd = ShiftDecider::new(park_on_goal);
        let result = sd.decide(
            &make_state(autoware_state_val),
            &make_control(velocity),
            &make_gear(gear_report_val),
        );

        assert!(
            result == gear::DRIVE || result == gear::REVERSE || result == gear::PARK,
            "output must be a valid gear"
        );
    }

    /// Dead-zone velocity holds previous command (DRIVE).
    #[kani::proof]
    fn dead_zone_holds_previous() {
        let park_on_goal: bool = kani::any();
        let mut sd = ShiftDecider::new(park_on_goal);

        // Step 1: establish DRIVE with forward velocity
        let result1 = sd.decide(
            &make_state(state::DRIVING),
            &make_control(1.0),
            &make_gear(gear::DRIVE),
        );
        assert!(result1 == gear::DRIVE);

        // Step 2: dead-zone velocity — must hold DRIVE
        let dz_vel = any_f32();
        kani::assume(!dz_vel.is_nan() && dz_vel.is_finite());
        kani::assume(dz_vel >= -VEL_THRESHOLD && dz_vel <= VEL_THRESHOLD);

        let result2 = sd.decide(
            &make_state(state::DRIVING),
            &make_control(dz_vel),
            &make_gear(gear::DRIVE),
        );
        assert!(result2 == gear::DRIVE, "dead-zone must hold previous DRIVE");
    }

    /// NaN velocity holds previous command (DRIVE).
    #[kani::proof]
    fn nan_velocity_holds_previous() {
        let park_on_goal: bool = kani::any();
        let mut sd = ShiftDecider::new(park_on_goal);

        // Step 1: establish DRIVE
        let result1 = sd.decide(
            &make_state(state::DRIVING),
            &make_control(1.0),
            &make_gear(gear::DRIVE),
        );
        assert!(result1 == gear::DRIVE);

        // Step 2: NaN velocity — NaN < threshold is false, NaN > threshold is false
        // → falls to dead-zone → holds previous (DRIVE)
        let result2 = sd.decide(
            &make_state(state::DRIVING),
            &make_control(f32::NAN),
            &make_gear(gear::DRIVE),
        );
        assert!(
            result2 == gear::DRIVE,
            "NaN velocity must hold previous DRIVE"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_state(s: u8) -> AutowareState {
        AutowareState {
            state: s,
            ..Default::default()
        }
    }

    fn make_control(velocity: f32) -> Control {
        let mut ctrl = Control::default();
        ctrl.longitudinal.velocity = velocity;
        ctrl
    }

    fn make_gear(report: u8) -> GearReport {
        GearReport {
            report,
            ..Default::default()
        }
    }

    #[test]
    fn driving_forward() {
        let mut sd = ShiftDecider::new(true);
        let cmd = sd.decide(
            &make_state(state::DRIVING),
            &make_control(1.0),
            &make_gear(gear::DRIVE),
        );
        assert_eq!(cmd, gear::DRIVE);
    }

    #[test]
    fn driving_reverse() {
        let mut sd = ShiftDecider::new(true);
        let cmd = sd.decide(
            &make_state(state::DRIVING),
            &make_control(-1.0),
            &make_gear(gear::DRIVE),
        );
        assert_eq!(cmd, gear::REVERSE);
    }

    #[test]
    fn driving_near_zero_holds_previous() {
        let mut sd = ShiftDecider::new(true);

        // First go forward
        sd.decide(
            &make_state(state::DRIVING),
            &make_control(1.0),
            &make_gear(gear::DRIVE),
        );

        // Then near zero → should hold DRIVE
        let cmd = sd.decide(
            &make_state(state::DRIVING),
            &make_control(0.005),
            &make_gear(gear::DRIVE),
        );
        assert_eq!(cmd, gear::DRIVE);
    }

    #[test]
    fn driving_near_zero_holds_reverse() {
        let mut sd = ShiftDecider::new(true);

        // First go reverse
        sd.decide(
            &make_state(state::DRIVING),
            &make_control(-1.0),
            &make_gear(gear::REVERSE),
        );

        // Then near zero → should hold REVERSE
        let cmd = sd.decide(
            &make_state(state::DRIVING),
            &make_control(-0.005),
            &make_gear(gear::REVERSE),
        );
        assert_eq!(cmd, gear::REVERSE);
    }

    #[test]
    fn arrived_goal_parks() {
        let mut sd = ShiftDecider::new(true);
        let cmd = sd.decide(
            &make_state(state::ARRIVED_GOAL),
            &make_control(0.0),
            &make_gear(gear::DRIVE),
        );
        assert_eq!(cmd, gear::PARK);
    }

    #[test]
    fn waiting_for_route_parks() {
        let mut sd = ShiftDecider::new(true);
        let cmd = sd.decide(
            &make_state(state::WAITING_FOR_ROUTE),
            &make_control(0.0),
            &make_gear(gear::DRIVE),
        );
        assert_eq!(cmd, gear::PARK);
    }

    #[test]
    fn arrived_goal_no_park_when_disabled() {
        let mut sd = ShiftDecider::new(false);
        let cmd = sd.decide(
            &make_state(state::ARRIVED_GOAL),
            &make_control(0.0),
            &make_gear(gear::DRIVE),
        );
        // park_on_goal=false → follow current gear
        assert_eq!(cmd, gear::DRIVE);
    }

    #[test]
    fn other_state_follows_current_gear() {
        let mut sd = ShiftDecider::new(true);
        // PLANNING state (3) → not DRIVING, not ARRIVED_GOAL/WAITING_FOR_ROUTE
        let cmd = sd.decide(
            &make_state(3), // PLANNING
            &make_control(0.0),
            &make_gear(gear::DRIVE),
        );
        assert_eq!(cmd, gear::DRIVE);
    }

    #[test]
    fn initial_prev_command_is_park() {
        let mut sd = ShiftDecider::new(true);
        // Driving at exactly zero → holds previous (PARK initially)
        let cmd = sd.decide(
            &make_state(state::DRIVING),
            &make_control(0.0),
            &make_gear(gear::DRIVE),
        );
        assert_eq!(cmd, gear::PARK);
    }

    #[test]
    fn transition_forward_to_reverse() {
        let mut sd = ShiftDecider::new(true);

        // Forward
        let cmd = sd.decide(
            &make_state(state::DRIVING),
            &make_control(5.0),
            &make_gear(gear::DRIVE),
        );
        assert_eq!(cmd, gear::DRIVE);

        // Near zero → hold DRIVE
        let cmd = sd.decide(
            &make_state(state::DRIVING),
            &make_control(0.0),
            &make_gear(gear::DRIVE),
        );
        assert_eq!(cmd, gear::DRIVE);

        // Reverse
        let cmd = sd.decide(
            &make_state(state::DRIVING),
            &make_control(-5.0),
            &make_gear(gear::DRIVE),
        );
        assert_eq!(cmd, gear::REVERSE);
    }
}
