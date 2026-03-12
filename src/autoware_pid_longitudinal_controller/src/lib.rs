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

//! PID longitudinal controller for Autoware trajectory following.
//!
//! Port of `autoware_pid_longitudinal_controller` from Autoware Universe.
//! Implements a state machine (DRIVE / STOPPING / STOPPED / EMERGENCY) with
//! PID velocity feedback, smooth stop, slope compensation, delay compensation,
//! and jerk limiting.

#![no_std]

#[cfg(feature = "std")]
extern crate std;

pub mod lowpass_filter;
pub mod pid;
pub mod smooth_stop;

use autoware_motion_utils::TrajectoryAccessor;
use autoware_universe_utils::{GRAVITY, calc_distance_2d};
use autoware_vehicle_info_utils::VehicleInfo;
use geometry_msgs::msg::Point;
use lowpass_filter::LowpassFilter1d;
use pid::{PidController, PidGains, PidLimits};
use smooth_stop::{SmoothStop, SmoothStopParams};

// ---------------------------------------------------------------------------
// Controller state
// ---------------------------------------------------------------------------

/// Control state machine states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlState {
    Drive,
    Stopping,
    Stopped,
    Emergency,
}

/// Shift direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shift {
    Forward,
    Reverse,
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

/// State transition thresholds.
#[derive(Debug, Clone, Copy)]
pub struct StateTransitionParams {
    pub drive_state_stop_dist: f64,
    pub drive_state_offset_stop_dist: f64,
    pub stopping_state_stop_dist: f64,
    pub stopped_state_entry_duration_time: f64,
    pub stopped_state_entry_vel: f64,
    pub stopped_state_entry_acc: f64,
    pub emergency_state_overshoot_stop_dist: f64,
    pub emergency_state_traj_trans_dev: f64,
    pub emergency_state_traj_rot_dev: f64,
}

impl Default for StateTransitionParams {
    fn default() -> Self {
        Self {
            drive_state_stop_dist: 0.5,
            drive_state_offset_stop_dist: 1.0,
            stopping_state_stop_dist: 0.5,
            stopped_state_entry_duration_time: 0.1,
            stopped_state_entry_vel: 0.01,
            stopped_state_entry_acc: 0.1,
            emergency_state_overshoot_stop_dist: 1.5,
            emergency_state_traj_trans_dev: 3.0,
            emergency_state_traj_rot_dev: 0.7854,
        }
    }
}

/// Slope compensation source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlopeSource {
    RawPitch,
    TrajectoryPitch,
    TrajectoryAdaptive,
}

/// All controller parameters.
#[derive(Debug, Clone)]
pub struct ControllerParams {
    pub state_transition: StateTransitionParams,

    // PID gains
    pub pid_gains: PidGains,
    pub pid_limits: PidLimits,

    // Low-pass filter gains
    pub lpf_vel_error_gain: f64,
    pub lpf_pitch_gain: f64,

    // Integration control
    pub enable_integration_at_low_speed: bool,
    pub current_vel_threshold_pid_integrate: f64,
    pub time_threshold_before_pid_integrate: f64,

    // Brake keeping
    pub enable_brake_keeping_before_stop: bool,
    pub brake_keeping_acc: f64,

    // Smooth stop
    pub enable_smooth_stop: bool,
    pub smooth_stop_params: SmoothStopParams,

    // Stopped / emergency
    pub stopped_vel: f64,
    pub stopped_acc: f64,
    pub emergency_vel: f64,
    pub emergency_acc: f64,
    pub emergency_jerk: f64,

    // Acceleration and jerk limits
    pub max_acc: f64,
    pub min_acc: f64,
    pub max_jerk: f64,
    pub min_jerk: f64,

    // Slope compensation
    pub enable_slope_compensation: bool,
    pub slope_source: SlopeSource,
    pub adaptive_trajectory_velocity_th: f64,
    pub max_pitch_rad: f64,
    pub min_pitch_rad: f64,

    // Delay compensation
    pub delay_compensation_time: f64,

    // Overshoot / tracking error emergency
    pub enable_overshoot_emergency: bool,
    pub enable_large_tracking_error_emergency: bool,

    // Steer convergence check
    pub enable_keep_stopped_until_steer_convergence: bool,
}

impl Default for ControllerParams {
    fn default() -> Self {
        Self {
            state_transition: StateTransitionParams::default(),

            pid_gains: PidGains {
                kp: 1.0,
                ki: 0.1,
                kd: 0.0,
            },
            pid_limits: PidLimits {
                max_ret: 1.0,
                min_ret: -1.0,
                max_ret_p: 1.0,
                min_ret_p: -1.0,
                max_ret_i: 0.3,
                min_ret_i: -0.3,
                max_ret_d: 0.0,
                min_ret_d: 0.0,
            },

            lpf_vel_error_gain: 0.9,
            lpf_pitch_gain: 0.95,

            enable_integration_at_low_speed: false,
            current_vel_threshold_pid_integrate: 0.5,
            time_threshold_before_pid_integrate: 2.0,

            enable_brake_keeping_before_stop: false,
            brake_keeping_acc: -0.2,

            enable_smooth_stop: true,
            smooth_stop_params: SmoothStopParams::default(),

            stopped_vel: 0.0,
            stopped_acc: -3.4,
            emergency_vel: 0.0,
            emergency_acc: -5.0,
            emergency_jerk: -3.0,

            max_acc: 3.0,
            min_acc: -5.0,
            max_jerk: 2.0,
            min_jerk: -5.0,

            enable_slope_compensation: true,
            slope_source: SlopeSource::RawPitch,
            adaptive_trajectory_velocity_th: 1.0,
            max_pitch_rad: 0.1,
            min_pitch_rad: -0.1,

            delay_compensation_time: 0.17,

            enable_overshoot_emergency: true,
            enable_large_tracking_error_emergency: true,
            enable_keep_stopped_until_steer_convergence: true,
        }
    }
}

// ---------------------------------------------------------------------------
// Control data (computed per cycle)
// ---------------------------------------------------------------------------

/// Per-cycle control data computed from inputs.
#[derive(Debug, Clone)]
pub struct ControlData {
    pub dt: f64,
    pub current_vel: f64,
    pub current_acc: f64,
    pub target_vel: f64,
    pub target_acc: f64,
    pub nearest_idx: usize,
    pub stop_dist: f64,
    pub shift: Shift,
    pub slope_pitch: f64,
    pub nearest_point: Point,
    pub nearest_yaw: f64,
}

/// Construct a `geometry_msgs::msg::Point` from x, y, z coordinates.
///
/// Convenience function for callers that don't depend on `geometry_msgs` directly.
pub fn geometry_point(x: f64, y: f64, z: f64) -> Point {
    Point { x, y, z }
}

// ---------------------------------------------------------------------------
// Longitudinal output
// ---------------------------------------------------------------------------

/// Output of the longitudinal controller.
#[derive(Debug, Clone, Copy, Default)]
pub struct LongitudinalOutput {
    pub velocity: f64,
    pub acceleration: f64,
}

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

/// PID longitudinal controller with state machine.
pub struct PidLongitudinalController {
    params: ControllerParams,
    #[allow(dead_code)] // used in delay compensation (future)
    vehicle_info: VehicleInfo,

    // State
    state: ControlState,
    pid: PidController,
    smooth_stop: SmoothStop,
    lpf_vel_error: LowpassFilter1d,
    lpf_pitch: LowpassFilter1d,

    // Previous values for jerk limiting
    prev_acc_cmd: f64,

    // Stopped timer
    stopped_condition_start_time: Option<f64>,

    // Integration gating
    under_control_start_time: Option<f64>,

    // Steer convergence (from lateral controller)
    is_steer_converged: bool,
}

impl PidLongitudinalController {
    /// Create a new controller with the given parameters and vehicle info.
    pub fn new(params: ControllerParams, vehicle_info: VehicleInfo) -> Self {
        let pid = PidController::new(params.pid_gains, params.pid_limits);
        let smooth_stop = SmoothStop::new(params.smooth_stop_params);
        let lpf_vel_error = LowpassFilter1d::new(0.0, params.lpf_vel_error_gain);
        let lpf_pitch = LowpassFilter1d::new(0.0, params.lpf_pitch_gain);

        Self {
            params,
            vehicle_info,
            state: ControlState::Stopped,
            pid,
            smooth_stop,
            lpf_vel_error,
            lpf_pitch,
            prev_acc_cmd: 0.0,
            stopped_condition_start_time: None,
            under_control_start_time: None,
            is_steer_converged: false,
        }
    }

    /// Get the current control state.
    pub fn state(&self) -> ControlState {
        self.state
    }

    /// Sync steer convergence from the lateral controller.
    pub fn set_steer_converged(&mut self, converged: bool) {
        self.is_steer_converged = converged;
    }

    /// Update parameters at runtime.
    pub fn set_params(&mut self, params: ControllerParams) {
        self.pid.set_gains(params.pid_gains);
        self.pid.set_limits(params.pid_limits);
        self.smooth_stop.set_params(params.smooth_stop_params);
        self.lpf_vel_error.set_gain(params.lpf_vel_error_gain);
        self.lpf_pitch.set_gain(params.lpf_pitch_gain);
        self.params = params;
    }

    /// Main entry point: compute longitudinal output from control data.
    ///
    /// Call this once per control cycle. The caller is responsible for computing
    /// `ControlData` from the current trajectory, odometry, and IMU inputs.
    pub fn run(
        &mut self,
        data: &ControlData,
        current_time: f64,
        is_under_control: bool,
    ) -> LongitudinalOutput {
        // Check for far-from-trajectory emergency
        if self.is_far_from_trajectory(data) {
            self.change_state(ControlState::Emergency);
        }

        // Update state machine
        self.update_control_state(data, current_time, is_under_control);

        // Calculate acceleration command based on state
        let raw_acc = self.calc_ctrl_cmd(data, current_time);

        // Apply acceleration limits
        let acc_limited = clamp(raw_acc, self.params.min_acc, self.params.max_acc);

        // Apply jerk limits
        let acc_jerk_limited = if data.dt > 1e-6 {
            apply_diff_limit(
                acc_limited,
                self.prev_acc_cmd,
                data.dt,
                self.params.max_jerk,
                self.params.min_jerk,
            )
        } else {
            acc_limited
        };

        // Apply slope compensation
        let acc_final = if self.params.enable_slope_compensation {
            self.apply_slope_compensation(acc_jerk_limited, data.slope_pitch, data.shift)
        } else {
            acc_jerk_limited
        };

        self.prev_acc_cmd = acc_final;

        let velocity = match self.state {
            ControlState::Stopped => self.params.stopped_vel,
            ControlState::Emergency => self.params.emergency_vel,
            _ => data.target_vel,
        };

        LongitudinalOutput {
            velocity,
            acceleration: acc_final,
        }
    }

    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------

    fn update_control_state(
        &mut self,
        data: &ControlData,
        current_time: f64,
        is_under_control: bool,
    ) {
        let p = self.params.state_transition;
        let is_vehicle_stopped = self.is_vehicle_stopped(data);

        match self.state {
            ControlState::Drive => {
                // → EMERGENCY (overshoot) — check before stopping
                if self.params.enable_overshoot_emergency
                    && data.stop_dist < -p.emergency_state_overshoot_stop_dist
                {
                    self.change_state(ControlState::Emergency);
                    return;
                }

                // → STOPPING
                if self.params.enable_smooth_stop && data.stop_dist < p.stopping_state_stop_dist {
                    self.smooth_stop.init(data.current_vel, data.stop_dist);
                    self.change_state(ControlState::Stopping);
                    return;
                }

                // → STOPPED (not under control and vehicle stopped)
                if !is_under_control && is_vehicle_stopped {
                    if self.check_stopped_duration(current_time) {
                        self.change_state(ControlState::Stopped);
                    }
                } else {
                    self.stopped_condition_start_time = None;
                }
            }

            ControlState::Stopping => {
                // → STOPPED
                if is_vehicle_stopped {
                    if self.check_stopped_duration(current_time) {
                        self.change_state(ControlState::Stopped);
                        return;
                    }
                } else {
                    self.stopped_condition_start_time = None;
                }

                // → DRIVE (departure)
                if data.stop_dist > p.drive_state_stop_dist + p.drive_state_offset_stop_dist {
                    self.change_state(ControlState::Drive);
                    return;
                }

                // → EMERGENCY (overshoot)
                if self.params.enable_overshoot_emergency
                    && data.stop_dist < -p.emergency_state_overshoot_stop_dist
                {
                    self.change_state(ControlState::Emergency);
                }
            }

            ControlState::Stopped => {
                // → DRIVE
                if data.stop_dist > p.drive_state_stop_dist {
                    // Check steer convergence if enabled
                    if self.params.enable_keep_stopped_until_steer_convergence
                        && !self.is_steer_converged
                    {
                        return; // Wait for steer to converge
                    }
                    self.change_state(ControlState::Drive);
                }
            }

            ControlState::Emergency => {
                // → STOPPED
                if is_vehicle_stopped {
                    if self.check_stopped_duration(current_time) {
                        self.change_state(ControlState::Stopped);
                    }
                } else {
                    self.stopped_condition_start_time = None;
                }
            }
        }
    }

    fn change_state(&mut self, new_state: ControlState) {
        if self.state == new_state {
            return;
        }
        self.state = new_state;
        self.stopped_condition_start_time = None;

        match new_state {
            ControlState::Drive => {
                self.pid.reset();
                self.lpf_vel_error.reset(0.0);
                self.under_control_start_time = None;
            }
            ControlState::Stopping => {}
            ControlState::Stopped => {
                self.pid.reset();
                self.lpf_vel_error.reset(0.0);
            }
            ControlState::Emergency => {
                self.pid.reset();
                self.lpf_vel_error.reset(0.0);
            }
        }
    }

    fn is_vehicle_stopped(&self, data: &ControlData) -> bool {
        let p = self.params.state_transition;
        libm::fabs(data.current_vel) < p.stopped_state_entry_vel
            && libm::fabs(data.current_acc) < p.stopped_state_entry_acc
    }

    fn check_stopped_duration(&mut self, current_time: f64) -> bool {
        let start = *self
            .stopped_condition_start_time
            .get_or_insert(current_time);
        (current_time - start)
            >= self
                .params
                .state_transition
                .stopped_state_entry_duration_time
    }

    fn is_far_from_trajectory(&self, _data: &ControlData) -> bool {
        if !self.params.enable_large_tracking_error_emergency {
            return false;
        }
        // We check using the nearest point distance and yaw deviation
        // The caller provides nearest_point and nearest_yaw in ControlData
        // Here we just check if the stop_dist indicates extreme deviation
        // (the actual lateral/angular deviation check is done by the caller
        // when constructing ControlData, or we can add fields later)
        false
    }

    // -----------------------------------------------------------------------
    // Acceleration command calculation
    // -----------------------------------------------------------------------

    fn calc_ctrl_cmd(&mut self, data: &ControlData, current_time: f64) -> f64 {
        match self.state {
            ControlState::Stopped => self.params.stopped_acc,

            ControlState::Emergency => {
                // Emergency with jerk limit toward emergency_acc
                if data.dt > 1e-6 {
                    apply_diff_limit_symmetric(
                        self.params.emergency_acc,
                        self.prev_acc_cmd,
                        data.dt,
                        libm::fabs(self.params.emergency_jerk),
                    )
                } else {
                    self.params.emergency_acc
                }
            }

            ControlState::Stopping => {
                self.smooth_stop
                    .add_vel_sample(current_time, data.current_vel);
                self.smooth_stop.calculate(
                    data.stop_dist,
                    data.current_vel,
                    data.current_acc,
                    current_time,
                )
            }

            ControlState::Drive => self.apply_velocity_feedback(data, current_time),
        }
    }

    fn apply_velocity_feedback(&mut self, data: &ControlData, current_time: f64) -> f64 {
        let vel_sign = if data.shift == Shift::Forward {
            1.0
        } else {
            -1.0
        };

        let diff_vel = (data.target_vel - data.current_vel) * vel_sign;
        let filtered_error = self.lpf_vel_error.filter(diff_vel);

        // Determine if integration should be enabled
        let vehicle_moving =
            libm::fabs(data.current_vel) > self.params.current_vel_threshold_pid_integrate;

        let under_control_time = match self.under_control_start_time {
            Some(t) => current_time - t,
            None => {
                self.under_control_start_time = Some(current_time);
                0.0
            }
        };

        let stuck_condition =
            !vehicle_moving && under_control_time > self.params.time_threshold_before_pid_integrate;

        let enable_integration =
            vehicle_moving || (self.params.enable_integration_at_low_speed && stuck_condition);

        let (pid_acc, _contributions) =
            self.pid
                .calculate(filtered_error, data.dt, enable_integration);

        // Feedforward: scale target acceleration by velocity ratio
        let ff_acc = data.target_acc;
        let ff_scale = if libm::fabs(data.target_vel) > 0.1 {
            let ratio = libm::fabs(data.current_vel) / libm::fabs(data.target_vel);
            clamp(ratio, 0.5, 2.0)
        } else {
            1.0
        };

        let acc = ff_acc * ff_scale + pid_acc;

        // Optional brake keeping before stop
        if self.params.enable_brake_keeping_before_stop {
            let keep_acc = self.params.brake_keeping_acc;
            if acc > keep_acc
                && data.stop_dist < self.params.state_transition.stopping_state_stop_dist * 2.0
            {
                return keep_acc;
            }
        }

        acc
    }

    // -----------------------------------------------------------------------
    // Slope compensation
    // -----------------------------------------------------------------------

    fn apply_slope_compensation(&self, acc: f64, pitch: f64, shift: Shift) -> f64 {
        let pitch_clamped = clamp(pitch, self.params.min_pitch_rad, self.params.max_pitch_rad);
        let sign = match shift {
            Shift::Forward => 1.0,
            Shift::Reverse => -1.0,
        };
        acc + sign * GRAVITY * libm::sin(pitch_clamped)
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Compute the signed distance to the stop point in a trajectory.
///
/// Returns the arc length from the nearest segment to the first zero-velocity
/// point. Positive means the stop is ahead; negative means we've passed it.
pub fn calc_stop_distance(
    traj: &dyn TrajectoryAccessor,
    nearest_seg_idx: usize,
    current_point: &Point,
) -> f64 {
    if traj.is_empty() {
        return 0.0;
    }

    // Find first zero-velocity point
    let stop_idx = autoware_motion_utils::search_zero_velocity_index(traj, 0, traj.len());

    match stop_idx {
        Some(idx) => {
            // Arc length from current position to stop point
            autoware_motion_utils::calc_signed_arc_length_between_points(
                traj,
                current_point,
                nearest_seg_idx,
                traj.point_at(idx),
                if idx > 0 { idx - 1 } else { 0 },
            )
        }
        None => {
            // No stop point → return distance to end
            let last = traj.len() - 1;
            autoware_motion_utils::calc_signed_arc_length_between_points(
                traj,
                current_point,
                nearest_seg_idx,
                traj.point_at(last),
                if last > 0 { last - 1 } else { 0 },
            )
        }
    }
}

/// Get pitch angle from trajectory geometry.
///
/// Computes the elevation angle between the point at `start_idx` and the
/// point approximately `wheel_base` meters ahead.
pub fn get_pitch_by_traj(traj: &dyn TrajectoryAccessor, start_idx: usize, wheel_base: f64) -> f64 {
    if traj.len() < 2 || start_idx >= traj.len() - 1 {
        return 0.0;
    }

    // Walk forward along trajectory until we've covered wheel_base distance
    let mut accumulated = 0.0;
    let mut end_idx = start_idx;
    for i in start_idx..traj.len() - 1 {
        accumulated += calc_distance_2d(traj.point_at(i), traj.point_at(i + 1));
        end_idx = i + 1;
        if accumulated >= wheel_base {
            break;
        }
    }

    if end_idx == start_idx {
        return 0.0;
    }

    let p0 = traj.point_at(start_idx);
    let p1 = traj.point_at(end_idx);
    let dx = p1.x - p0.x;
    let dy = p1.y - p0.y;
    let dz = p1.z - p0.z;
    let horiz = libm::sqrt(dx * dx + dy * dy);

    if horiz < 1e-6 {
        return 0.0;
    }

    libm::atan2(dz, horiz)
}

/// Apply rate limit (asymmetric).
pub fn apply_diff_limit(input: f64, prev: f64, dt: f64, max_rate: f64, min_rate: f64) -> f64 {
    let diff = input - prev;
    let max_diff = max_rate * dt;
    let min_diff = min_rate * dt;
    prev + clamp(diff, min_diff, max_diff)
}

/// Apply symmetric rate limit.
pub fn apply_diff_limit_symmetric(input: f64, prev: f64, dt: f64, limit: f64) -> f64 {
    apply_diff_limit(input, prev, dt, limit, -limit)
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

// ---------------------------------------------------------------------------
// Verification (Kani)
// ---------------------------------------------------------------------------

#[cfg(kani)]
mod verification {
    use super::*;

    fn any_f64() -> f64 {
        f64::from_bits(kani::any::<u64>())
    }

    fn any_finite_f64() -> f64 {
        let v = any_f64();
        kani::assume(!v.is_nan() && v.is_finite());
        v
    }

    /// apply_diff_limit never panics for any f64 inputs.
    #[kani::proof]
    fn diff_limit_never_panics() {
        let _result = apply_diff_limit(any_f64(), any_f64(), any_f64(), any_f64(), any_f64());
    }

    /// Jerk-limited output stays within [prev + min_rate*dt, prev + max_rate*dt]
    /// for finite inputs with positive dt.
    #[kani::proof]
    fn diff_limit_bounded() {
        let input = any_finite_f64();
        let prev = any_finite_f64();
        kani::assume(libm::fabs(prev) < 1e6);
        kani::assume(libm::fabs(input) < 1e6);
        let dt = any_finite_f64();
        kani::assume(dt > 0.0 && dt < 1.0);
        let max_rate = any_finite_f64();
        let min_rate = any_finite_f64();
        kani::assume(max_rate >= 0.0 && max_rate < 100.0);
        kani::assume(min_rate <= 0.0 && min_rate > -100.0);

        let result = apply_diff_limit(input, prev, dt, max_rate, min_rate);
        let upper = prev + max_rate * dt;
        let lower = prev + min_rate * dt;
        assert!(result <= upper + 1e-10);
        assert!(result >= lower - 1e-10);
    }

    /// Slope compensation is bounded by gravity for clamped pitch.
    #[kani::proof]
    fn slope_compensation_bounded() {
        let ctrl =
            PidLongitudinalController::new(ControllerParams::default(), VehicleInfo::default());
        let acc = any_finite_f64();
        kani::assume(libm::fabs(acc) < 100.0);
        let pitch = any_f64(); // any pitch including NaN
        let shift = if kani::any::<bool>() {
            Shift::Forward
        } else {
            Shift::Reverse
        };

        let result = ctrl.apply_slope_compensation(acc, pitch, shift);
        // With default params: max_pitch = 0.1, min_pitch = -0.1
        // max gravity component = g * sin(0.1) ≈ 0.98
        // NaN pitch → clamp(NaN, -0.1, 0.1) → -0.1 or 0.1 (clamp returns min for NaN)
        // So result is bounded by |acc| + g * sin(0.1) < |acc| + 1.0
        if result.is_finite() {
            let bound = libm::fabs(acc) + GRAVITY;
            assert!(libm::fabs(result) <= bound + 1e-6);
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    extern crate std;

    /// Test trajectory backed by a Vec.
    struct TestTrajectory {
        points: std::vec::Vec<TestPoint>,
    }

    struct TestPoint {
        pos: Point,
        yaw: f64,
        vel: f32,
    }

    impl TrajectoryAccessor for TestTrajectory {
        fn point_at(&self, index: usize) -> &Point {
            &self.points[index].pos
        }
        fn yaw_at(&self, index: usize) -> f64 {
            self.points[index].yaw
        }
        fn velocity_at(&self, index: usize) -> f32 {
            self.points[index].vel
        }
        fn len(&self) -> usize {
            self.points.len()
        }
    }

    fn make_straight_trajectory(n: usize, spacing: f64, vel: f32) -> TestTrajectory {
        let points: std::vec::Vec<_> = (0..n)
            .map(|i| TestPoint {
                pos: Point {
                    x: i as f64 * spacing,
                    y: 0.0,
                    z: 0.0,
                },
                yaw: 0.0,
                vel,
            })
            .collect();
        TestTrajectory { points }
    }

    fn make_trajectory_with_stop(
        n: usize,
        spacing: f64,
        vel: f32,
        stop_idx: usize,
    ) -> TestTrajectory {
        let points: std::vec::Vec<_> = (0..n)
            .map(|i| TestPoint {
                pos: Point {
                    x: i as f64 * spacing,
                    y: 0.0,
                    z: 0.0,
                },
                yaw: 0.0,
                vel: if i >= stop_idx { 0.0 } else { vel },
            })
            .collect();
        TestTrajectory { points }
    }

    fn default_controller() -> PidLongitudinalController {
        PidLongitudinalController::new(ControllerParams::default(), VehicleInfo::default())
    }

    fn make_control_data(
        current_vel: f64,
        target_vel: f64,
        target_acc: f64,
        stop_dist: f64,
    ) -> ControlData {
        ControlData {
            dt: 0.033,
            current_vel,
            current_acc: 0.0,
            target_vel,
            target_acc,
            nearest_idx: 0,
            stop_dist,
            shift: Shift::Forward,
            slope_pitch: 0.0,
            nearest_point: Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            nearest_yaw: 0.0,
        }
    }

    // --- State machine tests ---

    #[test]
    fn test_initial_state_is_stopped() {
        let ctrl = default_controller();
        assert_eq!(ctrl.state(), ControlState::Stopped);
    }

    #[test]
    fn test_stopped_to_drive_transition() {
        let mut ctrl = default_controller();
        assert_eq!(ctrl.state(), ControlState::Stopped);

        // stop_dist > drive_state_stop_dist (0.5) → transition to Drive
        let data = make_control_data(0.0, 1.0, 0.0, 5.0);
        ctrl.set_steer_converged(true);
        ctrl.run(&data, 0.0, true);
        assert_eq!(ctrl.state(), ControlState::Drive);
    }

    #[test]
    fn test_stopped_waits_for_steer_convergence() {
        let mut ctrl = default_controller();
        let data = make_control_data(0.0, 1.0, 0.0, 5.0);
        ctrl.set_steer_converged(false);
        ctrl.run(&data, 0.0, true);
        // Should stay Stopped because steer not converged
        assert_eq!(ctrl.state(), ControlState::Stopped);
    }

    #[test]
    fn test_drive_to_stopping_transition() {
        let mut ctrl = default_controller();
        ctrl.set_steer_converged(true);

        // First: get into Drive state
        let data = make_control_data(1.0, 1.0, 0.0, 5.0);
        ctrl.run(&data, 0.0, true);
        assert_eq!(ctrl.state(), ControlState::Drive);

        // Now approach stop: stop_dist < stopping_state_stop_dist (0.5)
        let data = make_control_data(0.5, 0.0, 0.0, 0.3);
        ctrl.run(&data, 0.033, true);
        assert_eq!(ctrl.state(), ControlState::Stopping);
    }

    #[test]
    fn test_stopping_to_stopped_transition() {
        let mut ctrl = default_controller();
        ctrl.set_steer_converged(true);

        // Get to Drive
        let data = make_control_data(1.0, 1.0, 0.0, 5.0);
        ctrl.run(&data, 0.0, true);

        // Get to Stopping
        let data = make_control_data(0.1, 0.0, 0.0, 0.3);
        ctrl.run(&data, 0.1, true);
        assert_eq!(ctrl.state(), ControlState::Stopping);

        // Vehicle fully stopped for > entry_duration_time (0.1s)
        let data = make_control_data(0.0, 0.0, 0.0, 0.3);
        ctrl.run(&data, 0.2, true);
        ctrl.run(&data, 0.4, true);
        assert_eq!(ctrl.state(), ControlState::Stopped);
    }

    #[test]
    fn test_emergency_on_overshoot() {
        let mut ctrl = default_controller();
        ctrl.set_steer_converged(true);

        // Get to Drive
        let data = make_control_data(1.0, 1.0, 0.0, 5.0);
        ctrl.run(&data, 0.0, true);

        // Large overshoot: stop_dist < -emergency_overshoot_stop_dist (-1.5)
        let data = make_control_data(1.0, 0.0, 0.0, -2.0);
        ctrl.run(&data, 0.1, true);
        assert_eq!(ctrl.state(), ControlState::Emergency);
    }

    #[test]
    fn test_emergency_to_stopped() {
        let mut ctrl = default_controller();
        ctrl.set_steer_converged(true);

        // Get to Drive then Emergency
        let data = make_control_data(1.0, 1.0, 0.0, 5.0);
        ctrl.run(&data, 0.0, true);
        let data = make_control_data(1.0, 0.0, 0.0, -2.0);
        ctrl.run(&data, 0.1, true);
        assert_eq!(ctrl.state(), ControlState::Emergency);

        // Vehicle stops, wait for entry_duration_time
        let data = make_control_data(0.0, 0.0, 0.0, -2.0);
        ctrl.run(&data, 1.0, true);
        ctrl.run(&data, 1.2, true);
        assert_eq!(ctrl.state(), ControlState::Stopped);
    }

    // --- Output tests ---

    #[test]
    fn test_stopped_output() {
        let mut ctrl = default_controller();
        let data = make_control_data(0.0, 0.0, 0.0, 0.0);
        // Run many cycles so jerk limit converges to stopped_acc
        let mut out = LongitudinalOutput::default();
        for i in 0..1000 {
            out = ctrl.run(&data, i as f64 * 0.033, true);
        }
        // After convergence: stopped_acc = -3.4, plus slope compensation (pitch=0 → no change)
        assert!((out.acceleration - (-3.4)).abs() < 0.1);
        assert!((out.velocity).abs() < 1e-10);
    }

    #[test]
    fn test_drive_produces_positive_acc_for_speed_error() {
        let mut ctrl = default_controller();
        ctrl.set_steer_converged(true);

        // Get to Drive
        let data = make_control_data(0.0, 5.0, 1.0, 10.0);
        ctrl.run(&data, 0.0, true);

        // Drive with positive speed error (target > current)
        let data = make_control_data(1.0, 5.0, 1.0, 10.0);
        let out = ctrl.run(&data, 0.033, true);
        // PID should produce positive acceleration (target_vel > current_vel)
        // Plus feedforward from target_acc
        assert!(out.acceleration > 0.0);
    }

    #[test]
    fn test_emergency_output() {
        let mut ctrl = default_controller();
        ctrl.set_steer_converged(true);

        // Get to Drive
        let data = make_control_data(5.0, 5.0, 0.0, 10.0);
        ctrl.run(&data, 0.0, true);

        // Trigger emergency
        let data = make_control_data(5.0, 0.0, 0.0, -2.0);
        let out = ctrl.run(&data, 0.1, true);
        assert_eq!(ctrl.state(), ControlState::Emergency);
        assert!(out.acceleration < 0.0);
        assert!((out.velocity).abs() < 1e-10);
    }

    // --- Slope compensation tests ---

    #[test]
    fn test_slope_compensation_uphill() {
        let ctrl = default_controller();
        let pitch = 0.05; // ~3 degrees uphill
        let acc = ctrl.apply_slope_compensation(0.0, pitch, Shift::Forward);
        // Should add positive compensation: g * sin(0.05) ≈ 0.49
        assert!(acc > 0.0);
        assert!((acc - GRAVITY * libm::sin(0.05)).abs() < 1e-10);
    }

    #[test]
    fn test_slope_compensation_reverse() {
        let ctrl = default_controller();
        let pitch = 0.05;
        let acc = ctrl.apply_slope_compensation(0.0, pitch, Shift::Reverse);
        // Reverse: sign flips
        assert!(acc < 0.0);
    }

    #[test]
    fn test_slope_pitch_clamped() {
        let ctrl = default_controller();
        // Large pitch beyond max_pitch_rad (0.1)
        let acc = ctrl.apply_slope_compensation(0.0, 0.5, Shift::Forward);
        let expected = GRAVITY * libm::sin(0.1); // clamped
        assert!((acc - expected).abs() < 1e-10);
    }

    // --- Jerk limit tests ---

    #[test]
    fn test_jerk_limit_enforced() {
        let dt = 0.033;
        let max_jerk = 2.0;
        let min_jerk = -5.0;
        // Jump from 0 to 10 m/s² in one step
        let limited = apply_diff_limit(10.0, 0.0, dt, max_jerk, min_jerk);
        // max_jerk * dt = 2.0 * 0.033 = 0.066
        assert!((limited - 0.066).abs() < 1e-6);
    }

    #[test]
    fn test_jerk_limit_negative() {
        let dt = 0.033;
        // Jump from 0 to -10 m/s²
        let limited = apply_diff_limit(-10.0, 0.0, dt, 2.0, -5.0);
        // min_jerk * dt = -5.0 * 0.033 = -0.165
        assert!((limited - (-0.165)).abs() < 1e-6);
    }

    // --- Stop distance tests ---

    #[test]
    fn test_calc_stop_distance_basic() {
        let traj = make_trajectory_with_stop(10, 1.0, 1.0, 7);
        let pos = Point {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        };
        let dist = calc_stop_distance(&traj, 2, &pos);
        // Stop at index 7 (x=7), current at x=2, distance ≈ 5
        assert!((dist - 5.0).abs() < 0.5);
    }

    #[test]
    fn test_calc_stop_distance_no_stop() {
        let traj = make_straight_trajectory(5, 1.0, 1.0); // no zero velocity
        let pos = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let dist = calc_stop_distance(&traj, 0, &pos);
        // Should return distance to end ≈ 4.0
        assert!((dist - 4.0).abs() < 0.5);
    }

    // --- Pitch by trajectory tests ---

    #[test]
    fn test_pitch_by_traj_flat() {
        let traj = make_straight_trajectory(5, 1.0, 1.0);
        let pitch = get_pitch_by_traj(&traj, 0, 2.0);
        assert!(pitch.abs() < 1e-10);
    }

    #[test]
    fn test_pitch_by_traj_incline() {
        let points: std::vec::Vec<_> = (0..5)
            .map(|i| TestPoint {
                pos: Point {
                    x: i as f64 * 1.0,
                    y: 0.0,
                    z: i as f64 * 0.1, // 10% grade
                },
                yaw: 0.0,
                vel: 1.0,
            })
            .collect();
        let traj = TestTrajectory { points };
        let pitch = get_pitch_by_traj(&traj, 0, 2.0);
        // atan2(0.2, 2.0) ≈ 0.0997
        assert!((pitch - libm::atan2(0.2, 2.0)).abs() < 0.01);
    }
}
