#![no_std]

pub mod accel;
pub mod jerk;
pub mod lowpass;
pub mod velocity;

use accel::{AccelValidatorParams, AccelerationValidator};
use autoware_control_msgs::msg::Control;
use jerk::{LateralJerkValidator, LateralJerkValidatorParams};
use velocity::{VelocityValidator, VelocityValidatorParams};

/// Aggregated validation status from all validators.
#[derive(Debug, Clone, Default)]
pub struct ValidationStatus {
    pub is_valid_acc: bool,
    pub desired_acc: f64,
    pub measured_acc: f64,
    pub is_rolling_back: bool,
    pub is_over_velocity: bool,
    pub vehicle_vel: f64,
    pub target_vel: f64,
    pub is_valid_lateral_jerk: bool,
    pub lateral_jerk: f64,
    pub steering_rate: f64,
    pub invalid_count: u32,
}

/// Combined parameters for all validators.
#[derive(Debug, Clone)]
pub struct ValidatorParams {
    pub accel: AccelValidatorParams,
    pub velocity: VelocityValidatorParams,
    pub jerk: LateralJerkValidatorParams,
}

impl Default for ValidatorParams {
    fn default() -> Self {
        Self {
            accel: AccelValidatorParams::default(),
            velocity: VelocityValidatorParams::default(),
            jerk: LateralJerkValidatorParams::default(),
        }
    }
}

/// Control validator facade — runs all three validators and aggregates results.
#[derive(Debug, Clone)]
pub struct ControlValidator {
    accel_validator: AccelerationValidator,
    velocity_validator: VelocityValidator,
    jerk_validator: LateralJerkValidator,
    status: ValidationStatus,
}

impl ControlValidator {
    pub fn new(params: ValidatorParams) -> Self {
        Self {
            accel_validator: AccelerationValidator::new(params.accel),
            velocity_validator: VelocityValidator::new(params.velocity),
            jerk_validator: LateralJerkValidator::new(params.jerk),
            status: ValidationStatus::default(),
        }
    }

    /// Run all validators and update aggregated status.
    ///
    /// - `control_cmd`: the control command (extracts acceleration and steering)
    /// - `vehicle_velocity`: current vehicle velocity (m/s)
    /// - `measured_accel`: measured longitudinal acceleration (m/s²)
    /// - `target_velocity`: target velocity for overspeed check (m/s)
    /// - `pitch`: road pitch angle (rad) for gravity compensation
    /// - `dt`: time step (s) for jerk computation
    pub fn validate(
        &mut self,
        control_cmd: &Control,
        vehicle_velocity: f64,
        measured_accel: f64,
        target_velocity: f64,
        pitch: f64,
        dt: f64,
    ) -> &ValidationStatus {
        let accel_result = self.accel_validator.validate(
            control_cmd.longitudinal.acceleration as f64,
            measured_accel,
            vehicle_velocity,
            pitch,
        );

        let vel_result = self
            .velocity_validator
            .validate(vehicle_velocity, target_velocity);

        let jerk_result = self.jerk_validator.validate(
            vehicle_velocity,
            control_cmd.lateral.steering_tire_angle as f64,
            dt,
        );

        let all_valid = accel_result.is_valid
            && !vel_result.is_rolling_back
            && !vel_result.is_over_velocity
            && jerk_result.is_valid;

        if all_valid {
            self.status.invalid_count = 0;
        } else {
            self.status.invalid_count += 1;
        }

        self.status.is_valid_acc = accel_result.is_valid;
        self.status.desired_acc = accel_result.desired_acc;
        self.status.measured_acc = accel_result.measured_acc;
        self.status.is_rolling_back = vel_result.is_rolling_back;
        self.status.is_over_velocity = vel_result.is_over_velocity;
        self.status.vehicle_vel = vel_result.vehicle_vel;
        self.status.target_vel = vel_result.target_vel;
        self.status.is_valid_lateral_jerk = jerk_result.is_valid;
        self.status.lateral_jerk = jerk_result.lateral_jerk;
        self.status.steering_rate = jerk_result.steering_rate;

        &self.status
    }

    pub fn status(&self) -> &ValidationStatus {
        &self.status
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use autoware_control_msgs::msg::{Lateral, Longitudinal};

    const DT: f64 = 1.0 / 30.0;

    fn make_control(accel: f32, velocity: f32, steer: f32) -> Control {
        let mut cmd = Control::default();
        cmd.longitudinal = Longitudinal {
            acceleration: accel,
            velocity,
            ..Default::default()
        };
        cmd.lateral = Lateral {
            steering_tire_angle: steer,
            ..Default::default()
        };
        cmd
    }

    // ── Acceleration tests ───────────────────────────────────────────

    #[test]
    fn detects_acceleration_exceeding_threshold() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(3.0, 10.0, 0.0);
        for _ in 0..50 {
            v.validate(&cmd, 10.0, 6.0, 10.0, 0.0, DT);
        }
        let status = v.validate(&cmd, 10.0, 6.0, 10.0, 0.0, DT);
        assert!(
            !status.is_valid_acc,
            "large measured vs desired gap should be invalid"
        );
    }

    #[test]
    fn acceleration_within_bounds_is_valid() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(1.0, 10.0, 0.0);
        for _ in 0..50 {
            v.validate(&cmd, 10.0, 1.2, 10.0, 0.0, DT);
        }
        let status = v.validate(&cmd, 10.0, 1.2, 10.0, 0.0, DT);
        assert!(status.is_valid_acc, "close accel should be valid");
    }

    #[test]
    fn low_velocity_resets_filters() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(3.0, 0.0, 0.0);
        let status = v.validate(&cmd, 0.1, 5.0, 0.0, 0.0, DT);
        assert!(
            status.is_valid_acc,
            "low velocity should reset and return valid"
        );
    }

    #[test]
    fn gravity_compensation_applied() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        // cmd=0.0, pitch=0.1 → desired ≈ 0.98 (9.8 * 0.1)
        let cmd = make_control(0.0, 10.0, 0.0);
        for _ in 0..50 {
            v.validate(&cmd, 10.0, 0.98, 10.0, 0.1, DT);
        }
        let status = v.validate(&cmd, 10.0, 0.98, 10.0, 0.1, DT);
        assert!(
            status.is_valid_acc,
            "gravity-compensated accel should be valid"
        );
        assert!(
            status.desired_acc > 0.5,
            "desired should include gravity term, got {}",
            status.desired_acc
        );
    }

    // ── Velocity tests ───────────────────────────────────────────────

    #[test]
    fn detects_rolling_back() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(0.0, 5.0, 0.0);
        for _ in 0..50 {
            v.validate(&cmd, -1.0, 0.0, 5.0, 0.0, DT);
        }
        let status = v.validate(&cmd, -1.0, 0.0, 5.0, 0.0, DT);
        assert!(
            status.is_rolling_back,
            "opposite sign velocities should detect rolling back"
        );
    }

    #[test]
    fn detects_overspeed() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(0.0, 10.0, 0.0);
        // Threshold: |t_vel| * 1.2 + 2.0 = 10 * 1.2 + 2.0 = 14.0
        for _ in 0..50 {
            v.validate(&cmd, 15.0, 0.0, 10.0, 0.0, DT);
        }
        let status = v.validate(&cmd, 15.0, 0.0, 10.0, 0.0, DT);
        assert!(
            status.is_over_velocity,
            "vehicle 15 > threshold 14 should be overspeed"
        );
    }

    #[test]
    fn normal_velocity_is_valid() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(0.0, 10.0, 0.0);
        for _ in 0..50 {
            v.validate(&cmd, 10.0, 0.0, 10.0, 0.0, DT);
        }
        let status = v.validate(&cmd, 10.0, 0.0, 10.0, 0.0, DT);
        assert!(!status.is_rolling_back);
        assert!(!status.is_over_velocity);
    }

    #[test]
    fn both_zero_not_rolling_back() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(0.0, 0.0, 0.0);
        let status = v.validate(&cmd, 0.0, 0.0, 0.0, 0.0, DT);
        assert!(
            !status.is_rolling_back,
            "both zero should not be rolling back"
        );
    }

    // ── Lateral jerk tests ───────────────────────────────────────────

    #[test]
    fn detects_high_lateral_jerk() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd1 = make_control(0.0, 10.0, 0.0);
        v.validate(&cmd1, 10.0, 0.0, 10.0, 0.0, DT);
        for _ in 0..50 {
            v.validate(&cmd1, 10.0, 0.0, 10.0, 0.0, DT);
        }
        // Large steering change → high lateral jerk
        let cmd2 = make_control(0.0, 10.0, 1.0);
        let status = v.validate(&cmd2, 10.0, 0.0, 10.0, 0.0, DT);
        assert!(!status.is_valid_lateral_jerk, "high jerk should be invalid");
    }

    #[test]
    fn low_jerk_is_valid() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(0.0, 10.0, 0.1);
        v.validate(&cmd, 5.0, 0.0, 10.0, 0.0, DT);
        // Tiny steering change
        let cmd2 = make_control(0.0, 10.0, 0.1001);
        let status = v.validate(&cmd2, 5.0, 0.0, 10.0, 0.0, DT);
        assert!(status.is_valid_lateral_jerk, "tiny jerk should be valid");
    }

    #[test]
    fn first_call_returns_valid() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(1.0, 10.0, 0.5);
        let status = v.validate(&cmd, 10.0, 1.0, 10.0, 0.0, DT);
        assert!(
            status.is_valid_lateral_jerk,
            "first jerk call should be valid"
        );
    }

    #[test]
    fn small_dt_skipped() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(0.0, 10.0, 0.0);
        v.validate(&cmd, 5.0, 0.0, 10.0, 0.0, DT);
        // Tiny dt should skip jerk computation and return previous valid
        let cmd2 = make_control(0.0, 10.0, 1.0);
        let status = v.validate(&cmd2, 5.0, 0.0, 10.0, 0.0, 1e-6);
        assert!(
            status.is_valid_lateral_jerk,
            "small dt should return prev valid result"
        );
    }

    // ── Integration tests ────────────────────────────────────────────

    #[test]
    fn all_valid_when_within_bounds() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(1.0, 10.0, 0.1);
        for _ in 0..50 {
            v.validate(&cmd, 10.0, 1.0, 10.0, 0.0, DT);
        }
        let status = v.validate(&cmd, 10.0, 1.0, 10.0, 0.0, DT);
        assert!(status.is_valid_acc);
        assert!(!status.is_rolling_back);
        assert!(!status.is_over_velocity);
        assert!(status.is_valid_lateral_jerk);
        assert_eq!(status.invalid_count, 0);
    }

    #[test]
    fn invalid_count_increments() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd = make_control(3.0, 10.0, 0.0);
        for _ in 0..100 {
            v.validate(&cmd, 10.0, 8.0, 10.0, 0.0, DT);
        }
        let status = v.status();
        assert!(status.invalid_count > 0, "invalid count should increment");
    }

    #[test]
    fn invalid_count_resets_on_valid() {
        let mut v = ControlValidator::new(ValidatorParams::default());
        let cmd_bad = make_control(3.0, 10.0, 0.0);
        for _ in 0..100 {
            v.validate(&cmd_bad, 10.0, 8.0, 10.0, 0.0, DT);
        }
        assert!(v.status().invalid_count > 0);

        let cmd_good = make_control(1.0, 10.0, 0.1);
        for _ in 0..100 {
            v.validate(&cmd_good, 10.0, 1.0, 10.0, 0.0, DT);
        }
        assert_eq!(
            v.status().invalid_count,
            0,
            "count should reset to 0 on valid"
        );
    }
}
