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

//! PID controller with anti-windup and per-component output limiting.
//!
//! Port of `pid.cpp` / `pid.hpp` from Autoware's PID longitudinal controller.

/// PID gains.
#[derive(Debug, Clone, Copy)]
pub struct PidGains {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

/// Per-component and total output limits.
#[derive(Debug, Clone, Copy)]
pub struct PidLimits {
    pub max_ret: f64,
    pub min_ret: f64,
    pub max_ret_p: f64,
    pub min_ret_p: f64,
    pub max_ret_i: f64,
    pub min_ret_i: f64,
    pub max_ret_d: f64,
    pub min_ret_d: f64,
}

/// Breakdown of PID output contributions (for debug).
#[derive(Debug, Clone, Copy, Default)]
pub struct PidContributions {
    pub p: f64,
    pub i: f64,
    pub d: f64,
}

/// Standard PID controller with anti-windup (integral clamping).
#[derive(Debug, Clone)]
pub struct PidController {
    gains: PidGains,
    limits: PidLimits,
    error_integral: f64,
    prev_error: f64,
    is_first_time: bool,
}

impl PidController {
    /// Create a new PID controller with the given gains and limits.
    pub fn new(gains: PidGains, limits: PidLimits) -> Self {
        Self {
            gains,
            limits,
            error_integral: 0.0,
            prev_error: 0.0,
            is_first_time: true,
        }
    }

    /// Reset the controller state (integral and derivative history).
    pub fn reset(&mut self) {
        self.error_integral = 0.0;
        self.prev_error = 0.0;
        self.is_first_time = true;
    }

    /// Update gains at runtime.
    pub fn set_gains(&mut self, gains: PidGains) {
        self.gains = gains;
    }

    /// Update limits at runtime.
    pub fn set_limits(&mut self, limits: PidLimits) {
        self.limits = limits;
    }

    /// Calculate PID output.
    ///
    /// - `error`: current error (setpoint - measurement)
    /// - `dt`: time step in seconds
    /// - `enable_integration`: whether to accumulate the integral term
    ///
    /// Returns `(output, contributions)`.
    pub fn calculate(
        &mut self,
        error: f64,
        dt: f64,
        enable_integration: bool,
    ) -> (f64, PidContributions) {
        if dt <= 0.0 {
            return (0.0, PidContributions::default());
        }

        let g = &self.gains;
        let l = &self.limits;

        // Proportional
        let ret_p = clamp(g.kp * error, l.min_ret_p, l.max_ret_p);

        // Integral with anti-windup
        if enable_integration && libm::fabs(g.ki) > 1e-10 {
            self.error_integral += error * dt;
            // Clamp integral to prevent windup
            let max_int = l.max_ret_i / g.ki;
            let min_int = l.min_ret_i / g.ki;
            self.error_integral = clamp(self.error_integral, min_int, max_int);
        }
        let ret_i = clamp(g.ki * self.error_integral, l.min_ret_i, l.max_ret_i);

        // Derivative (skip on first call)
        let ret_d = if self.is_first_time {
            self.is_first_time = false;
            0.0
        } else {
            clamp(
                g.kd * (error - self.prev_error) / dt,
                l.min_ret_d,
                l.max_ret_d,
            )
        };

        self.prev_error = error;

        let output = clamp(ret_p + ret_i + ret_d, l.min_ret, l.max_ret);
        let contributions = PidContributions {
            p: ret_p,
            i: ret_i,
            d: ret_d,
        };

        (output, contributions)
    }
}

fn clamp(val: f64, min: f64, max: f64) -> f64 {
    if val < min {
        min
    } else if val > max {
        max
    } else {
        val
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_gains() -> PidGains {
        PidGains {
            kp: 1.0,
            ki: 0.1,
            kd: 0.0,
        }
    }

    fn default_limits() -> PidLimits {
        PidLimits {
            max_ret: 1.0,
            min_ret: -1.0,
            max_ret_p: 1.0,
            min_ret_p: -1.0,
            max_ret_i: 0.3,
            min_ret_i: -0.3,
            max_ret_d: 0.0,
            min_ret_d: 0.0,
        }
    }

    #[test]
    fn test_proportional_only() {
        let gains = PidGains {
            kp: 2.0,
            ki: 0.0,
            kd: 0.0,
        };
        let mut pid = PidController::new(gains, default_limits());
        let (out, contrib) = pid.calculate(0.3, 0.01, false);
        assert!((out - 0.6).abs() < 1e-10);
        assert!((contrib.p - 0.6).abs() < 1e-10);
        assert!((contrib.i).abs() < 1e-10);
    }

    #[test]
    fn test_integral_accumulates() {
        let mut pid = PidController::new(default_gains(), default_limits());
        // Constant error of 1.0 for 10 steps at dt=0.1 → integral = 1.0
        for _ in 0..10 {
            pid.calculate(1.0, 0.1, true);
        }
        let (_, contrib) = pid.calculate(1.0, 0.1, true);
        // ki=0.1, integral ≈ 1.1 → ret_i ≈ 0.11
        assert!(contrib.i > 0.1);
    }

    #[test]
    fn test_integral_windup_clamped() {
        let mut pid = PidController::new(default_gains(), default_limits());
        // Large constant error for many steps
        for _ in 0..1000 {
            pid.calculate(100.0, 0.1, true);
        }
        let (_, contrib) = pid.calculate(100.0, 0.1, true);
        // max_ret_i = 0.3, so integral contribution should be clamped
        assert!((contrib.i - 0.3).abs() < 1e-10);
    }

    #[test]
    fn test_output_clamped() {
        let mut pid = PidController::new(default_gains(), default_limits());
        let (out, _) = pid.calculate(100.0, 0.1, false);
        assert!((out - 1.0).abs() < 1e-10); // clamped to max_ret
    }

    #[test]
    fn test_reset_clears_state() {
        let mut pid = PidController::new(default_gains(), default_limits());
        pid.calculate(1.0, 0.1, true);
        pid.calculate(1.0, 0.1, true);
        pid.reset();
        // After reset, integral should be zero and first-time flag set
        let (out, contrib) = pid.calculate(0.0, 0.1, true);
        assert!((out).abs() < 1e-10);
        assert!((contrib.i).abs() < 1e-10);
    }

    #[test]
    fn test_zero_dt_returns_zero() {
        let mut pid = PidController::new(default_gains(), default_limits());
        let (out, _) = pid.calculate(1.0, 0.0, true);
        assert!((out).abs() < 1e-10);
    }

    #[test]
    fn test_derivative_term() {
        let gains = PidGains {
            kp: 0.0,
            ki: 0.0,
            kd: 1.0,
        };
        let limits = PidLimits {
            max_ret: 100.0,
            min_ret: -100.0,
            max_ret_p: 0.0,
            min_ret_p: 0.0,
            max_ret_i: 0.0,
            min_ret_i: 0.0,
            max_ret_d: 100.0,
            min_ret_d: -100.0,
        };
        let mut pid = PidController::new(gains, limits);
        // First call: derivative skipped
        let (out1, _) = pid.calculate(0.0, 0.1, false);
        assert!((out1).abs() < 1e-10);
        // Second call: error jumps from 0 to 1, d = kd * (1-0)/0.1 = 10
        let (out2, contrib) = pid.calculate(1.0, 0.1, false);
        assert!((out2 - 10.0).abs() < 1e-10);
        assert!((contrib.d - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_integration_disabled() {
        let mut pid = PidController::new(default_gains(), default_limits());
        for _ in 0..100 {
            pid.calculate(1.0, 0.1, false);
        }
        let (_, contrib) = pid.calculate(1.0, 0.1, false);
        assert!((contrib.i).abs() < 1e-10);
    }
}
