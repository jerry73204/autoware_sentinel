use crate::lowpass::LowpassFilter1d;

/// Parameters for the lateral jerk validator.
#[derive(Debug, Clone)]
pub struct LateralJerkValidatorParams {
    /// Maximum lateral jerk (m/s³). Default: 10.0
    pub lateral_jerk_threshold: f64,
    /// Vehicle wheel base (m). Default: 2.79
    pub wheel_base: f64,
    /// Lowpass filter gain. Default: 0.8
    pub lpf_gain: f64,
}

impl Default for LateralJerkValidatorParams {
    fn default() -> Self {
        Self {
            lateral_jerk_threshold: 10.0,
            wheel_base: 2.79,
            lpf_gain: 0.8,
        }
    }
}

#[derive(Debug, Clone)]
pub struct LateralJerkValidatorResult {
    pub is_valid: bool,
    pub lateral_jerk: f64,
    pub steering_rate: f64,
}

/// Validates that steering commands do not produce excessive lateral jerk.
#[derive(Debug, Clone)]
pub struct LateralJerkValidator {
    params: LateralJerkValidatorParams,
    measured_vel_lpf: LowpassFilter1d,
    prev_steering: Option<f64>,
    prev_result_valid: bool,
}

impl LateralJerkValidator {
    pub fn new(params: LateralJerkValidatorParams) -> Self {
        let gain = params.lpf_gain;
        Self {
            params,
            measured_vel_lpf: LowpassFilter1d::new(gain),
            prev_steering: None,
            prev_result_valid: true,
        }
    }

    /// Validate lateral jerk from steering command rate.
    ///
    /// - `vehicle_velocity`: current vehicle speed (m/s)
    /// - `steering_cmd`: commanded steering tire angle (rad)
    /// - `dt`: time step (s)
    pub fn validate(
        &mut self,
        vehicle_velocity: f64,
        steering_cmd: f64,
        dt: f64,
    ) -> LateralJerkValidatorResult {
        let filtered_vel = self.measured_vel_lpf.filter(vehicle_velocity);

        let prev = match self.prev_steering {
            Some(prev) => prev,
            None => {
                self.prev_steering = Some(steering_cmd);
                return LateralJerkValidatorResult {
                    is_valid: true,
                    lateral_jerk: 0.0,
                    steering_rate: 0.0,
                };
            }
        };

        if dt < 1e-3 {
            self.prev_steering = Some(steering_cmd);
            return LateralJerkValidatorResult {
                is_valid: self.prev_result_valid,
                lateral_jerk: 0.0,
                steering_rate: 0.0,
            };
        }

        let steering_rate = (steering_cmd - prev) / dt;
        let lateral_jerk =
            (1.0 / self.params.wheel_base) * filtered_vel * filtered_vel * steering_rate;
        let is_valid = lateral_jerk.abs() < self.params.lateral_jerk_threshold;

        self.prev_steering = Some(steering_cmd);
        self.prev_result_valid = is_valid;

        LateralJerkValidatorResult {
            is_valid,
            lateral_jerk,
            steering_rate,
        }
    }
}
