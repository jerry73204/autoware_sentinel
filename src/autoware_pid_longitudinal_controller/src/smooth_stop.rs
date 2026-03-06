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

//! Smooth stop algorithm for gentle vehicle deceleration.
//!
//! Port of `smooth_stop.cpp` / `smooth_stop.hpp` from Autoware.
//! Manages deceleration profile when approaching a stop point,
//! using velocity history for time-to-stop estimation.

/// Maximum velocity history samples for regression.
const MAX_VEL_HISTORY: usize = 64;

/// Smooth stop parameters.
#[derive(Debug, Clone, Copy)]
pub struct SmoothStopParams {
    pub max_strong_acc: f64,
    pub min_strong_acc: f64,
    pub weak_acc: f64,
    pub weak_stop_acc: f64,
    pub strong_stop_acc: f64,
    pub min_fast_vel: f64,
    pub min_running_vel: f64,
    pub min_running_acc: f64,
    pub weak_stop_time: f64,
    pub weak_stop_dist: f64,
    pub strong_stop_dist: f64,
}

impl Default for SmoothStopParams {
    fn default() -> Self {
        Self {
            max_strong_acc: -0.5,
            min_strong_acc: -0.8,
            weak_acc: -0.3,
            weak_stop_acc: -0.8,
            strong_stop_acc: -3.4,
            min_fast_vel: 0.5,
            min_running_vel: 0.01,
            min_running_acc: 0.01,
            weak_stop_time: 0.8,
            weak_stop_dist: -0.3,
            strong_stop_dist: -0.5,
        }
    }
}

/// Velocity sample with timestamp (seconds).
#[derive(Debug, Clone, Copy)]
struct VelSample {
    time: f64,
    vel: f64,
}

/// Smooth stop controller.
///
/// Tracks velocity history and computes deceleration commands to bring
/// the vehicle to a smooth stop at the target point.
#[derive(Debug, Clone)]
pub struct SmoothStop {
    params: SmoothStopParams,
    /// Kinematic strong deceleration: -v²/(2d)
    strong_acc: f64,
    /// Velocity history ring buffer
    vel_history: [VelSample; MAX_VEL_HISTORY],
    vel_history_len: usize,
    vel_history_start: usize,
    /// Time when the vehicle was detected as stopped
    stopped_time: Option<f64>,
}

impl SmoothStop {
    pub fn new(params: SmoothStopParams) -> Self {
        Self {
            params,
            strong_acc: params.min_strong_acc,
            vel_history: [VelSample {
                time: 0.0,
                vel: 0.0,
            }; MAX_VEL_HISTORY],
            vel_history_len: 0,
            vel_history_start: 0,
            stopped_time: None,
        }
    }

    /// Initialize for a new stop command.
    ///
    /// `pred_vel`: predicted velocity at target
    /// `pred_stop_dist`: predicted distance to stop point
    pub fn init(&mut self, pred_vel: f64, pred_stop_dist: f64) {
        // Kinematic deceleration: a = -v²/(2d)
        self.strong_acc = if libm::fabs(pred_stop_dist) > 1e-6 {
            let a = -(pred_vel * pred_vel) / (2.0 * pred_stop_dist);
            clamp(a, self.params.min_strong_acc, self.params.max_strong_acc)
        } else {
            self.params.min_strong_acc
        };
        self.vel_history_len = 0;
        self.vel_history_start = 0;
        self.stopped_time = None;
    }

    /// Set parameters.
    pub fn set_params(&mut self, params: SmoothStopParams) {
        self.params = params;
    }

    /// Add a velocity sample to the history.
    pub fn add_vel_sample(&mut self, time: f64, vel: f64) {
        let idx = (self.vel_history_start + self.vel_history_len) % MAX_VEL_HISTORY;
        self.vel_history[idx] = VelSample { time, vel };
        if self.vel_history_len < MAX_VEL_HISTORY {
            self.vel_history_len += 1;
        } else {
            self.vel_history_start = (self.vel_history_start + 1) % MAX_VEL_HISTORY;
        }
    }

    /// Estimate time to stop using linear regression on velocity history.
    ///
    /// Returns `None` if not enough data or velocity is not decreasing.
    fn calc_time_to_stop(&self) -> Option<f64> {
        if self.vel_history_len < 2 {
            return None;
        }

        // Linear regression: vel = a*time + b → stop time = -b/a
        let n = self.vel_history_len as f64;
        let mut sum_t = 0.0;
        let mut sum_v = 0.0;
        let mut sum_tv = 0.0;
        let mut sum_tt = 0.0;

        for i in 0..self.vel_history_len {
            let idx = (self.vel_history_start + i) % MAX_VEL_HISTORY;
            let s = &self.vel_history[idx];
            sum_t += s.time;
            sum_v += s.vel;
            sum_tv += s.time * s.vel;
            sum_tt += s.time * s.time;
        }

        let denom = n * sum_tt - sum_t * sum_t;
        if libm::fabs(denom) < 1e-10 {
            return None;
        }

        let slope = (n * sum_tv - sum_t * sum_v) / denom;
        let intercept = (sum_v - slope * sum_t) / n;

        // Velocity must be decreasing (slope < 0) and intercept > 0
        if slope >= 0.0 || intercept <= 0.0 {
            return None;
        }

        // Time when vel = 0: 0 = slope*t + intercept → t = -intercept/slope
        let stop_time = -intercept / slope;
        // Remaining time from now (last sample time)
        let last_idx = (self.vel_history_start + self.vel_history_len - 1) % MAX_VEL_HISTORY;
        let current_time = self.vel_history[last_idx].time;
        let remaining = stop_time - current_time;

        if remaining > 0.0 {
            Some(remaining)
        } else {
            None
        }
    }

    /// Calculate smooth stop acceleration command.
    ///
    /// - `stop_dist`: signed distance to stop point (positive = ahead)
    /// - `current_vel`: current velocity (m/s)
    /// - `current_acc`: current acceleration (m/s²)
    /// - `current_time`: current time (seconds)
    pub fn calculate(
        &mut self,
        stop_dist: f64,
        current_vel: f64,
        current_acc: f64,
        current_time: f64,
    ) -> f64 {
        let p = &self.params;

        // Case 1: Strong overshoot — past stop point by large margin
        if stop_dist < p.strong_stop_dist {
            return p.strong_stop_acc;
        }

        // Case 2: Weak overshoot — slightly past stop point
        if stop_dist < p.weak_stop_dist {
            return p.weak_stop_acc;
        }

        // Case 3: Still running (above velocity or acceleration thresholds)
        let is_running = libm::fabs(current_vel) > p.min_running_vel
            || libm::fabs(current_acc) > p.min_running_acc;

        if is_running {
            self.stopped_time = None;
            let time_to_stop = self.calc_time_to_stop();
            return match time_to_stop {
                Some(t) if t > p.weak_stop_time => self.strong_acc,
                _ => p.weak_acc,
            };
        }

        // Case 4: Just stopped — hold brake briefly
        let stopped_time = *self.stopped_time.get_or_insert(current_time);
        let stopped_duration = current_time - stopped_time;

        if stopped_duration < 0.5 {
            p.weak_acc
        } else {
            p.strong_stop_acc
        }
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

    #[test]
    fn test_strong_overshoot() {
        let mut ss = SmoothStop::new(SmoothStopParams::default());
        ss.init(1.0, 5.0);
        let acc = ss.calculate(-1.0, 0.0, 0.0, 0.0);
        assert!((acc - SmoothStopParams::default().strong_stop_acc).abs() < 1e-10);
    }

    #[test]
    fn test_weak_overshoot() {
        let mut ss = SmoothStop::new(SmoothStopParams::default());
        ss.init(1.0, 5.0);
        // stop_dist between strong_stop_dist (-0.5) and weak_stop_dist (-0.3)
        let acc = ss.calculate(-0.4, 0.0, 0.0, 0.0);
        assert!((acc - SmoothStopParams::default().weak_stop_acc).abs() < 1e-10);
    }

    #[test]
    fn test_running_weak_decel() {
        let mut ss = SmoothStop::new(SmoothStopParams::default());
        ss.init(2.0, 10.0);
        // Running with no history → weak_acc
        let acc = ss.calculate(3.0, 1.0, -0.2, 0.0);
        assert!((acc - SmoothStopParams::default().weak_acc).abs() < 1e-10);
    }

    #[test]
    fn test_stopped_holds_brake() {
        let mut ss = SmoothStop::new(SmoothStopParams::default());
        ss.init(1.0, 5.0);
        // Not running, first call records stopped_time
        let acc = ss.calculate(0.5, 0.0, 0.0, 10.0);
        assert!((acc - SmoothStopParams::default().weak_acc).abs() < 1e-10);
        // After 0.5s, switches to strong_stop_acc
        let acc2 = ss.calculate(0.5, 0.0, 0.0, 10.6);
        assert!((acc2 - SmoothStopParams::default().strong_stop_acc).abs() < 1e-10);
    }

    #[test]
    fn test_kinematic_init() {
        let params = SmoothStopParams::default();
        let mut ss = SmoothStop::new(params);
        // v=2, d=10 → a = -4/20 = -0.2. Clamped to [min_strong, max_strong] = [-0.8, -0.5]
        ss.init(2.0, 10.0);
        assert!((ss.strong_acc - (-0.5)).abs() < 1e-10); // clamped to max_strong
    }

    #[test]
    fn test_time_to_stop_regression() {
        let mut ss = SmoothStop::new(SmoothStopParams::default());
        ss.init(2.0, 10.0);
        // Linearly decreasing velocity: 2.0, 1.5, 1.0, 0.5 at t=0,1,2,3
        // slope = -0.5, intercept = 2.0, stop_time = 4.0
        // At t=3, remaining = 4.0 - 3.0 = 1.0
        ss.add_vel_sample(0.0, 2.0);
        ss.add_vel_sample(1.0, 1.5);
        ss.add_vel_sample(2.0, 1.0);
        ss.add_vel_sample(3.0, 0.5);
        let t = ss.calc_time_to_stop();
        assert!(t.is_some());
        assert!((t.unwrap() - 1.0).abs() < 1e-6);
    }
}
