use crate::lowpass::LowpassFilter1d;

const GRAVITY: f64 = 9.8;

/// Parameters for the acceleration validator.
#[derive(Debug, Clone)]
pub struct AccelValidatorParams {
    /// Absolute tolerance offset (m/s²). Default: 0.8
    pub e_offset: f64,
    /// Proportional tolerance scale. Default: 0.2
    pub e_scale: f64,
    /// Lowpass filter gain. Default: 0.8
    pub lpf_gain: f64,
}

impl Default for AccelValidatorParams {
    fn default() -> Self {
        Self {
            e_offset: 0.8,
            e_scale: 0.2,
            lpf_gain: 0.8,
        }
    }
}

#[derive(Debug, Clone)]
pub struct AccelValidatorResult {
    pub is_valid: bool,
    pub desired_acc: f64,
    pub measured_acc: f64,
}

/// Validates that measured acceleration tracks the commanded acceleration.
#[derive(Debug, Clone)]
pub struct AccelerationValidator {
    params: AccelValidatorParams,
    desired_acc_lpf: LowpassFilter1d,
    measured_acc_lpf: LowpassFilter1d,
}

impl AccelerationValidator {
    pub fn new(params: AccelValidatorParams) -> Self {
        let gain = params.lpf_gain;
        Self {
            params,
            desired_acc_lpf: LowpassFilter1d::new(gain),
            measured_acc_lpf: LowpassFilter1d::new(gain),
        }
    }

    /// Validate commanded vs measured acceleration.
    ///
    /// - `cmd_accel`: commanded acceleration from control (m/s²)
    /// - `measured_accel`: measured acceleration from IMU or estimator (m/s²)
    /// - `velocity`: current vehicle velocity (m/s) — resets filters at low speed
    /// - `pitch`: road pitch angle (rad) for gravity compensation
    pub fn validate(
        &mut self,
        cmd_accel: f64,
        measured_accel: f64,
        velocity: f64,
        pitch: f64,
    ) -> AccelValidatorResult {
        // At low speed, reset filters and report valid
        if velocity.abs() < 0.3 {
            self.desired_acc_lpf.set(0.0);
            self.measured_acc_lpf.set(0.0);
            return AccelValidatorResult {
                is_valid: true,
                desired_acc: 0.0,
                measured_acc: 0.0,
            };
        }

        let desired = self.desired_acc_lpf.filter(cmd_accel + GRAVITY * pitch);
        let measured = self.measured_acc_lpf.filter(measured_accel);

        let margin = (self.params.e_scale * desired).abs() + self.params.e_offset;
        let is_valid = measured >= desired - margin && measured <= desired + margin;

        AccelValidatorResult {
            is_valid,
            desired_acc: desired,
            measured_acc: measured,
        }
    }
}
