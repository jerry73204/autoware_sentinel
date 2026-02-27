#![no_std]

use autoware_control_msgs::msg::{Control, Longitudinal};
use tier4_system_msgs::msg::MrmBehaviorStatus;

/// MrmBehaviorStatus state constants (private in generated module).
const AVAILABLE: u8 = 1;
const OPERATING: u8 = 2;

/// Parameters for the comfortable stop operator.
#[derive(Debug, Clone)]
pub struct Params {
    /// Minimum acceleration / maximum deceleration (m/s², negative). Default: -1.0
    pub min_acceleration: f32,
    /// Maximum negative jerk (m/s³, negative). Default: -0.3
    pub min_jerk: f32,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            min_acceleration: -1.0,
            min_jerk: -0.3,
        }
    }
}

/// Jerk-limited comfortable stop operator.
///
/// Same deceleration ramp as the emergency stop operator but with gentler parameters
/// for passenger comfort. On the safety island (no planner), this publishes direct
/// Control commands rather than VelocityLimit.
#[derive(Debug, Clone)]
pub struct ComfortableStopOperator {
    params: Params,
    state: u8,
    velocity: f32,
    acceleration: f32,
}

impl ComfortableStopOperator {
    pub fn new(params: Params) -> Self {
        Self {
            params,
            state: AVAILABLE,
            velocity: 0.0,
            acceleration: 0.0,
        }
    }

    /// Activate or deactivate the comfortable stop.
    pub fn operate(&mut self, activate: bool) {
        if activate {
            self.state = OPERATING;
        } else {
            self.state = AVAILABLE;
        }
    }

    /// Returns `true` if the operator is currently executing a comfortable stop.
    pub fn is_operating(&self) -> bool {
        self.state == OPERATING
    }

    /// Set the current vehicle velocity (m/s) from external odometry.
    pub fn set_initial_velocity(&mut self, velocity: f32) {
        self.velocity = velocity;
        self.acceleration = 0.0;
    }

    /// Advance the deceleration ramp by `dt` seconds. Returns the control command.
    ///
    /// Same algorithm as emergency stop but with gentler parameters:
    ///   a(t+1) = max(a(t) + min_jerk * dt, min_acceleration)
    ///   v(t+1) = max(v(t) + a(t+1) * dt, 0.0)
    pub fn update(&mut self, dt: f32) -> Control {
        if self.state != OPERATING {
            return self.passthrough_command();
        }

        let new_accel =
            (self.acceleration + self.params.min_jerk * dt).max(self.params.min_acceleration);

        let new_vel = (self.velocity + new_accel * dt).max(0.0);

        let jerk = if (self.acceleration - self.params.min_acceleration).abs() < f32::EPSILON {
            0.0
        } else {
            self.params.min_jerk
        };

        self.acceleration = new_accel;
        self.velocity = new_vel;

        let mut cmd = Control::default();
        cmd.longitudinal = Longitudinal {
            velocity: new_vel,
            acceleration: new_accel,
            jerk,
            is_defined_acceleration: true,
            is_defined_jerk: true,
            ..Default::default()
        };
        cmd
    }

    /// Current status message.
    pub fn status(&self) -> MrmBehaviorStatus {
        MrmBehaviorStatus {
            state: self.state,
            ..Default::default()
        }
    }

    /// Current velocity (m/s).
    pub fn velocity(&self) -> f32 {
        self.velocity
    }

    fn passthrough_command(&self) -> Control {
        let mut cmd = Control::default();
        cmd.longitudinal.velocity = self.velocity;
        cmd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 30.0;

    #[test]
    fn deceleration_from_20_mps_reaches_zero() {
        let mut op = ComfortableStopOperator::new(Params::default());
        op.set_initial_velocity(20.0);
        op.operate(true);

        for _ in 0..2000 {
            op.update(DT);
            if op.velocity() == 0.0 {
                break;
            }
        }

        assert_eq!(op.velocity(), 0.0, "vehicle should have stopped");
    }

    #[test]
    fn slower_than_emergency_stop() {
        // Comfortable stop should take longer than emergency stop
        let mut comfortable = ComfortableStopOperator::new(Params::default());
        comfortable.set_initial_velocity(20.0);
        comfortable.operate(true);

        let mut steps = 0u32;
        for _ in 0..2000 {
            comfortable.update(DT);
            steps += 1;
            if comfortable.velocity() == 0.0 {
                break;
            }
        }

        // Emergency stop: target_accel=-2.5, target_jerk=-1.5 → ~300 steps
        // Comfortable stop: min_accel=-1.0, min_jerk=-0.3 → should be much more
        assert!(
            steps > 300,
            "comfortable stop should take more steps than emergency: got {}",
            steps
        );
    }

    #[test]
    fn velocity_never_goes_negative() {
        let mut op = ComfortableStopOperator::new(Params::default());
        op.set_initial_velocity(5.0);
        op.operate(true);

        for _ in 0..2000 {
            op.update(DT);
            assert!(op.velocity() >= 0.0);
        }
    }

    #[test]
    fn gentler_deceleration_profile() {
        let mut op = ComfortableStopOperator::new(Params::default());
        op.set_initial_velocity(20.0);
        op.operate(true);

        // After many steps, acceleration should never exceed min_acceleration
        for _ in 0..2000 {
            op.update(DT);
            assert!(
                op.acceleration >= -1.0 - 0.001,
                "acceleration exceeded min: {}",
                op.acceleration
            );
        }
    }

    #[test]
    fn status_reflects_state() {
        let mut op = ComfortableStopOperator::new(Params::default());
        assert_eq!(op.status().state, AVAILABLE);

        op.operate(true);
        assert_eq!(op.status().state, OPERATING);

        op.operate(false);
        assert_eq!(op.status().state, AVAILABLE);
    }
}
