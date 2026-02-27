#![no_std]

use autoware_control_msgs::msg::{Control, Longitudinal};
use tier4_system_msgs::msg::MrmBehaviorStatus;

/// MrmBehaviorStatus state constants (private in generated module).
const AVAILABLE: u8 = 1;
const OPERATING: u8 = 2;

/// Parameters for the emergency stop operator.
#[derive(Debug, Clone)]
pub struct Params {
    /// Target deceleration (m/s², negative). Default: -2.5
    pub target_acceleration: f32,
    /// Jerk ramp rate (m/s³, negative). Default: -1.5
    pub target_jerk: f32,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            target_acceleration: -2.5,
            target_jerk: -1.5,
        }
    }
}

/// Jerk-limited emergency stop operator.
///
/// When activated, ramps acceleration toward `target_acceleration` at `target_jerk`,
/// and integrates velocity downward until the vehicle stops.
#[derive(Debug, Clone)]
pub struct EmergencyStopOperator {
    params: Params,
    state: u8,
    velocity: f32,
    acceleration: f32,
}

impl EmergencyStopOperator {
    pub fn new(params: Params) -> Self {
        Self {
            params,
            state: AVAILABLE,
            velocity: 0.0,
            acceleration: 0.0,
        }
    }

    /// Activate or deactivate the emergency stop.
    pub fn operate(&mut self, activate: bool) {
        if activate {
            self.state = OPERATING;
        } else {
            self.state = AVAILABLE;
        }
    }

    /// Returns `true` if the operator is currently executing an emergency stop.
    pub fn is_operating(&self) -> bool {
        self.state == OPERATING
    }

    /// Set the current vehicle velocity (m/s) from external odometry.
    /// Call this before the first `update()` after activation.
    pub fn set_initial_velocity(&mut self, velocity: f32) {
        self.velocity = velocity;
        self.acceleration = 0.0;
    }

    /// Advance the deceleration ramp by `dt` seconds. Returns the control command.
    ///
    /// The algorithm:
    ///   a(t+1) = max(a(t) + target_jerk * dt, target_acceleration)
    ///   v(t+1) = max(v(t) + a(t+1) * dt, 0.0)
    pub fn update(&mut self, dt: f32) -> Control {
        if self.state != OPERATING {
            return self.passthrough_command();
        }

        // Ramp acceleration toward target
        let new_accel =
            (self.acceleration + self.params.target_jerk * dt).max(self.params.target_acceleration);

        // Integrate velocity (clamp to zero)
        let new_vel = (self.velocity + new_accel * dt).max(0.0);

        // Determine jerk output
        let jerk = if (self.acceleration - self.params.target_acceleration).abs() < f32::EPSILON {
            0.0
        } else {
            self.params.target_jerk
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

    const DT: f32 = 1.0 / 30.0; // 30 Hz

    #[test]
    fn deceleration_from_20_mps_reaches_zero() {
        let mut op = EmergencyStopOperator::new(Params::default());
        op.set_initial_velocity(20.0);
        op.operate(true);

        for _ in 0..300 {
            op.update(DT);
            if op.velocity() == 0.0 {
                break;
            }
        }

        assert_eq!(
            op.velocity(),
            0.0,
            "vehicle should have stopped within 300 steps"
        );
    }

    #[test]
    fn velocity_never_goes_negative() {
        let mut op = EmergencyStopOperator::new(Params::default());
        op.set_initial_velocity(1.0);
        op.operate(true);

        for _ in 0..1000 {
            op.update(DT);
            assert!(op.velocity() >= 0.0, "velocity must never be negative");
        }
    }

    #[test]
    fn acceleration_ramps_to_target() {
        let mut op = EmergencyStopOperator::new(Params::default());
        op.set_initial_velocity(20.0);
        op.operate(true);

        // Run enough steps for acceleration to reach target
        // From 0 to -2.5 at jerk -1.5: needs 2.5/1.5 ≈ 1.67s ≈ 50 steps at 30Hz
        for _ in 0..60 {
            op.update(DT);
        }

        assert!(
            (op.acceleration - (-2.5f32)).abs() < 0.01,
            "acceleration should reach target: got {}",
            op.acceleration
        );
    }

    #[test]
    fn not_operating_returns_passthrough() {
        let mut op = EmergencyStopOperator::new(Params::default());
        op.set_initial_velocity(10.0);

        let cmd = op.update(DT);
        assert_eq!(cmd.longitudinal.velocity, 10.0);
        assert!(!op.is_operating());
    }

    #[test]
    fn status_reflects_state() {
        let mut op = EmergencyStopOperator::new(Params::default());
        assert_eq!(op.status().state, AVAILABLE);

        op.operate(true);
        assert_eq!(op.status().state, OPERATING);

        op.operate(false);
        assert_eq!(op.status().state, AVAILABLE);
    }

    #[test]
    fn velocity_profile_is_monotonically_decreasing() {
        let mut op = EmergencyStopOperator::new(Params::default());
        op.set_initial_velocity(20.0);
        op.operate(true);

        let mut prev_vel = 20.0;
        for _ in 0..300 {
            op.update(DT);
            assert!(
                op.velocity() <= prev_vel,
                "velocity should be monotonically decreasing: {} > {}",
                op.velocity(),
                prev_vel
            );
            prev_vel = op.velocity();
            if prev_vel == 0.0 {
                break;
            }
        }
    }

    #[test]
    fn jerk_is_zero_after_acceleration_reaches_target() {
        let mut op = EmergencyStopOperator::new(Params::default());
        op.set_initial_velocity(20.0);
        op.operate(true);

        // Run until acceleration is clamped to exactly target_acceleration.
        // At jerk=-1.5, dt=1/30: takes ceil(2.5/0.05) = 50 steps.
        // Run 60 to be safe — max() clamp guarantees exact -2.5.
        let mut cmd = Control::default();
        for _ in 0..60 {
            cmd = op.update(DT);
        }

        // Acceleration is now clamped to exactly target. Next step's jerk must be 0.
        cmd = op.update(DT);
        assert_eq!(
            cmd.longitudinal.jerk, 0.0,
            "jerk should be zero at target acceleration"
        );
    }
}
