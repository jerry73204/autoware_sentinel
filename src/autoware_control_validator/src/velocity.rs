use crate::lowpass::LowpassFilter1d;

/// Parameters for the velocity validator.
#[derive(Debug, Clone)]
pub struct VelocityValidatorParams {
    /// Velocity threshold for rolling-back detection (m/s). Default: 0.5
    pub rolling_back_velocity_th: f64,
    /// Ratio above target velocity for overspeed. Default: 0.2
    pub over_velocity_ratio_th: f64,
    /// Absolute offset above target velocity for overspeed (m/s). Default: 2.0
    pub over_velocity_offset_th: f64,
    /// Lowpass filter gain. Default: 0.8
    pub lpf_gain: f64,
}

impl Default for VelocityValidatorParams {
    fn default() -> Self {
        Self {
            rolling_back_velocity_th: 0.5,
            over_velocity_ratio_th: 0.2,
            over_velocity_offset_th: 2.0,
            lpf_gain: 0.8,
        }
    }
}

#[derive(Debug, Clone)]
pub struct VelocityValidatorResult {
    pub is_rolling_back: bool,
    pub is_over_velocity: bool,
    pub vehicle_vel: f64,
    pub target_vel: f64,
}

/// Validates vehicle velocity against commanded target.
#[derive(Debug, Clone)]
pub struct VelocityValidator {
    params: VelocityValidatorParams,
    vehicle_vel_lpf: LowpassFilter1d,
    target_vel_lpf: LowpassFilter1d,
}

impl VelocityValidator {
    pub fn new(params: VelocityValidatorParams) -> Self {
        let gain = params.lpf_gain;
        Self {
            params,
            vehicle_vel_lpf: LowpassFilter1d::new(gain),
            target_vel_lpf: LowpassFilter1d::new(gain),
        }
    }

    pub fn validate(
        &mut self,
        vehicle_velocity: f64,
        target_velocity: f64,
    ) -> VelocityValidatorResult {
        let v_vel = self.vehicle_vel_lpf.filter(vehicle_velocity);
        let t_vel = self.target_vel_lpf.filter(target_velocity);

        // Rolling back: opposite direction AND exceeds threshold
        let is_rolling_back = if v_vel == 0.0 && t_vel == 0.0 {
            false
        } else {
            v_vel.is_sign_positive() != t_vel.is_sign_positive()
                && v_vel.abs() > self.params.rolling_back_velocity_th
        };

        // Overspeed: vehicle velocity exceeds target + margin
        let is_over_velocity = v_vel.abs()
            > t_vel.abs() * (1.0 + self.params.over_velocity_ratio_th)
                + self.params.over_velocity_offset_th;

        VelocityValidatorResult {
            is_rolling_back,
            is_over_velocity,
            vehicle_vel: v_vel,
            target_vel: t_vel,
        }
    }
}
