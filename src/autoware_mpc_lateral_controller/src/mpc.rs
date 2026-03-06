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

//! MPC core solver: prediction matrix generation, cost formulation, QP solve.
//!
//! Port of `mpc.cpp` from Autoware's MPC lateral controller.

use crate::mat;
use crate::vehicle_model::{DiscreteMatrices, MAX_DIM_X, MAX_DIM_Y, VehicleModel};

/// Maximum prediction horizon.
pub const MAX_HORIZON: usize = 50;
/// Maximum trajectory points for MPC internal trajectory.
pub const MAX_TRAJ_POINTS: usize = 256;

// Derived array size constants
const MAX_NX: usize = MAX_DIM_X * MAX_HORIZON; // 200
const MAX_NY: usize = MAX_DIM_Y * MAX_HORIZON; // 100
const MAX_NU: usize = MAX_HORIZON; // DIM_U is always 1

/// MPC cost weights.
#[derive(Debug, Clone, Copy)]
pub struct MpcWeight {
    pub lat_error: f64,
    pub heading_error: f64,
    pub heading_error_squared_vel: f64,
    pub terminal_lat_error: f64,
    pub terminal_heading_error: f64,
    pub steering_input: f64,
    pub steering_input_squared_vel: f64,
    pub lat_jerk: f64,
    pub steer_rate: f64,
    pub steer_acc: f64,
}

impl Default for MpcWeight {
    fn default() -> Self {
        Self {
            lat_error: 1.0,
            heading_error: 0.0,
            heading_error_squared_vel: 0.3,
            terminal_lat_error: 1.0,
            terminal_heading_error: 0.1,
            steering_input: 1.0,
            steering_input_squared_vel: 0.25,
            lat_jerk: 0.1,
            steer_rate: 0.0,
            steer_acc: 0.000001,
        }
    }
}

/// MPC parameters.
#[derive(Debug, Clone)]
pub struct MpcParams {
    pub prediction_horizon: usize,
    pub prediction_dt: f64,
    pub min_prediction_length: f64,
    pub zero_ff_steer_deg: f64,
    pub input_delay: f64,
    pub acceleration_limit: f64,
    pub velocity_time_constant: f64,
    pub steer_tau: f64,
    pub nominal_weight: MpcWeight,
    pub low_curvature_weight: MpcWeight,
    pub low_curvature_thresh: f64,
    pub steer_rate_lim: f64,
}

impl Default for MpcParams {
    fn default() -> Self {
        Self {
            prediction_horizon: 50,
            prediction_dt: 0.1,
            min_prediction_length: 5.0,
            zero_ff_steer_deg: 0.5,
            input_delay: 0.0,
            acceleration_limit: 2.0,
            velocity_time_constant: 0.3,
            steer_tau: 0.27,
            nominal_weight: MpcWeight::default(),
            low_curvature_weight: MpcWeight {
                lat_error: 0.1,
                lat_jerk: 0.0,
                ..MpcWeight::default()
            },
            low_curvature_thresh: 0.0,
            steer_rate_lim: libm::tan(60.0 * core::f64::consts::PI / 180.0),
        }
    }
}

/// Trajectory point for MPC internal use.
#[derive(Debug, Clone, Copy, Default)]
pub struct MpcTrajPoint {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub vx: f64,
    pub curvature: f64,
    pub smooth_curvature: f64,
    pub time: f64,
}

/// Fixed-capacity MPC trajectory.
pub struct MpcTrajectory {
    pub points: [MpcTrajPoint; MAX_TRAJ_POINTS],
    pub len: usize,
}

impl MpcTrajectory {
    pub fn new() -> Self {
        Self {
            points: [MpcTrajPoint::default(); MAX_TRAJ_POINTS],
            len: 0,
        }
    }

    pub fn push(&mut self, pt: MpcTrajPoint) -> bool {
        if self.len >= MAX_TRAJ_POINTS {
            return false;
        }
        self.points[self.len] = pt;
        self.len += 1;
        true
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }
}

/// MPC output.
#[derive(Debug, Clone, Copy, Default)]
pub struct MpcOutput {
    pub steer_cmd: f64,
    pub steer_rate: f64,
    pub success: bool,
}

/// MPC solver workspace. Contains all pre-allocated buffers.
///
/// This struct is large (~200KB) due to the prediction matrices.
/// For embedded targets, reduce `MAX_HORIZON`.
pub struct MpcWorkspace {
    // Prediction matrices (stored flat, row-major)
    aex: [f64; MAX_NX * MAX_DIM_X],
    bex: [f64; MAX_NX * MAX_NU],
    wex: [f64; MAX_NX],
    // CB = Cex * Bex (DIM_Y*N × N)
    cb: [f64; MAX_NY * MAX_NU],
    // QCB = Q_diag * CB
    qcb: [f64; MAX_NY * MAX_NU],
    // Cost matrices
    q_diag: [f64; MAX_NY],
    r1_diag: [f64; MAX_NU],
    r2: [f64; MAX_NU * MAX_NU],
    uref: [f64; MAX_NU],
    // QP matrices
    h: [f64; MAX_NU * MAX_NU],
    f_vec: [f64; MAX_NU],
    // Solution
    u_opt: [f64; MAX_NU],
    // Temporaries
    caw: [f64; MAX_NY],   // Cex * (Aex * x0 + Wex)
    ax_w: [f64; MAX_NX],  // Aex * x0 + Wex
    dm: DiscreteMatrices, // per-step discrete matrices
    // Input delay buffer
    input_buffer: [f64; MAX_HORIZON],
    input_buffer_len: usize,
}

impl MpcWorkspace {
    pub fn new() -> Self {
        Self {
            aex: [0.0; MAX_NX * MAX_DIM_X],
            bex: [0.0; MAX_NX * MAX_NU],
            wex: [0.0; MAX_NX],
            cb: [0.0; MAX_NY * MAX_NU],
            qcb: [0.0; MAX_NY * MAX_NU],
            q_diag: [0.0; MAX_NY],
            r1_diag: [0.0; MAX_NU],
            r2: [0.0; MAX_NU * MAX_NU],
            uref: [0.0; MAX_NU],
            h: [0.0; MAX_NU * MAX_NU],
            f_vec: [0.0; MAX_NU],
            u_opt: [0.0; MAX_NU],
            caw: [0.0; MAX_NY],
            ax_w: [0.0; MAX_NX],
            dm: DiscreteMatrices::default(),
            input_buffer: [0.0; MAX_HORIZON],
            input_buffer_len: 0,
        }
    }
}

/// Run MPC: compute optimal steering from trajectory and current state.
///
/// - `model`: vehicle model (will be mutated to set velocity/curvature per step)
/// - `params`: MPC parameters
/// - `traj`: reference trajectory (time-resampled at prediction_dt)
/// - `x0`: initial state vector (dim_x elements)
/// - `steer_lim`: steering angle limit [rad]
/// - `ctrl_period`: control period [s]
/// - `ws`: pre-allocated workspace
pub fn calculate_mpc(
    model: &mut VehicleModel,
    params: &MpcParams,
    traj: &MpcTrajectory,
    x0: &[f64],
    steer_lim: f64,
    _ctrl_period: f64,
    ws: &mut MpcWorkspace,
) -> MpcOutput {
    let n = params.prediction_horizon.min(MAX_HORIZON);
    let dx = model.dim_x;
    let dy = model.dim_y;

    if traj.len < n + 1 || n < 2 {
        return MpcOutput::default();
    }

    // Determine prediction dt (ensure min_prediction_length coverage)
    let prediction_dt = {
        let total_time = traj.points[n.min(traj.len - 1)].time - traj.points[0].time;
        if total_time > 1e-6 {
            (total_time / (n - 1) as f64).max(params.prediction_dt)
        } else {
            params.prediction_dt
        }
    };

    // Apply delay compensation to initial state
    let mut x0_delayed = [0.0; MAX_DIM_X];
    x0_delayed[..dx].copy_from_slice(&x0[..dx]);
    delay_compensate(model, params, &traj, &mut x0_delayed, prediction_dt, ws);

    // Generate MPC prediction matrices and cost
    generate_mpc_matrix(model, params, traj, n, prediction_dt, ws);

    // Build H and f for QP
    build_qp(dx, dy, n, &x0_delayed, ws);

    // Solve: u = -H^-1 * f
    let solved = mat::cholesky_solve(&ws.h, &ws.f_vec, n, &mut ws.u_opt);

    if !solved {
        return MpcOutput::default();
    }

    // Extract first input and clamp
    let u0 = clamp(ws.u_opt[0], -steer_lim, steer_lim);

    // Estimate steering rate
    let steer_rate = if n >= 2 {
        let u1 = clamp(ws.u_opt[1], -steer_lim, steer_lim);
        (u1 - u0) / prediction_dt
    } else {
        0.0
    };

    // Store in input buffer for delay compensation
    if ws.input_buffer_len < MAX_HORIZON {
        ws.input_buffer[ws.input_buffer_len] = u0;
        ws.input_buffer_len += 1;
    } else {
        // Shift buffer
        for i in 0..MAX_HORIZON - 1 {
            ws.input_buffer[i] = ws.input_buffer[i + 1];
        }
        ws.input_buffer[MAX_HORIZON - 1] = u0;
    }

    MpcOutput {
        steer_cmd: u0,
        steer_rate,
        success: true,
    }
}

/// Generate prediction matrices and cost matrices.
fn generate_mpc_matrix(
    model: &mut VehicleModel,
    params: &MpcParams,
    traj: &MpcTrajectory,
    n: usize,
    dt: f64,
    ws: &mut MpcWorkspace,
) {
    let dx = model.dim_x;
    let dy = model.dim_y;

    // Zero workspace
    mat::zeros(&mut ws.aex[..dx * n * dx]);
    mat::zeros(&mut ws.bex[..dx * n * n]);
    mat::zeros(&mut ws.wex[..dx * n]);
    mat::zeros(&mut ws.cb[..dy * n * n]);
    mat::zeros(&mut ws.q_diag[..dy * n]);
    mat::zeros(&mut ws.r1_diag[..n]);
    mat::zeros(&mut ws.r2[..n * n]);
    mat::zeros(&mut ws.uref[..n]);

    for i in 0..n {
        let pt = &traj.points[i.min(traj.len - 1)];
        let vx = libm::fmax(libm::fabs(pt.vx), 0.01);
        let curv = pt.curvature;

        model.set_state(vx, curv);
        model.calculate_discrete_matrix(dt, &mut ws.dm);

        let ad = &ws.dm.ad;
        let bd = &ws.dm.bd;
        let cd = &ws.dm.cd;
        let wd = &ws.dm.wd;

        // --- Build Aex ---
        // Aex[i] = Ad_i * Aex[i-1] (or Ad_i for i=0)
        let row_start = i * dx;
        if i == 0 {
            ws.aex[..dx * dx].copy_from_slice(&ad[..dx * dx]);
        } else {
            let prev_start = (i - 1) * dx;
            // tmp = Ad * Aex[prev]
            for r in 0..dx {
                for c in 0..dx {
                    let mut sum = 0.0;
                    for k in 0..dx {
                        sum += ad[r * dx + k] * ws.aex[(prev_start + k) * dx + c];
                    }
                    ws.aex[(row_start + r) * dx + c] = sum;
                }
            }
        }

        // --- Build Bex ---
        // Bex[i,i] = Bd_i
        for r in 0..dx {
            ws.bex[(row_start + r) * n + i] = bd[r];
        }
        // Bex[i,j] = Ad_i * Bex[i-1,j] for j < i
        if i > 0 {
            let prev_start = (i - 1) * dx;
            for j in 0..i {
                for r in 0..dx {
                    let mut sum = 0.0;
                    for k in 0..dx {
                        sum += ad[r * dx + k] * ws.bex[(prev_start + k) * n + j];
                    }
                    ws.bex[(row_start + r) * n + j] = sum;
                }
            }
        }

        // --- Build Wex ---
        if i == 0 {
            ws.wex[..dx].copy_from_slice(&wd[..dx]);
        } else {
            let prev_start = (i - 1) * dx;
            for r in 0..dx {
                let mut sum = 0.0;
                for k in 0..dx {
                    sum += ad[r * dx + k] * ws.wex[prev_start + k];
                }
                ws.wex[row_start + r] = sum + wd[r];
            }
        }

        // --- Build CB (= Cex * Bex block row i) ---
        // CB[i*dy..(i+1)*dy, 0..n] = Cd * Bex[i*dx..(i+1)*dx, 0..n]
        for r in 0..dy {
            for j in 0..n {
                let mut sum = 0.0;
                for k in 0..dx {
                    sum += cd[r * dx + k] * ws.bex[(row_start + k) * n + j];
                }
                ws.cb[(i * dy + r) * n + j] = sum;
            }
        }

        // --- Build cost weights ---
        let w = select_weight(params, curv);
        let is_terminal = i == n - 1;

        let q_lat = if is_terminal {
            w.terminal_lat_error
        } else {
            w.lat_error
        };
        let q_yaw = if is_terminal {
            w.terminal_heading_error
        } else {
            w.heading_error + vx * vx * w.heading_error_squared_vel
        };
        ws.q_diag[i * dy] = q_lat;
        ws.q_diag[i * dy + 1] = q_yaw;

        ws.r1_diag[i] = w.steering_input + vx * vx * w.steering_input_squared_vel;

        // --- Feedforward reference ---
        let u_ref = libm::atan(model.wheelbase * pt.smooth_curvature);
        let u_ref = if libm::fabs(u_ref) < params.zero_ff_steer_deg * core::f64::consts::PI / 180.0
        {
            0.0
        } else {
            u_ref
        };
        ws.uref[i] = u_ref;
    }

    // Add lateral jerk, steer rate, and steer acceleration penalties to R2
    let w = select_weight(params, traj.points[0].curvature);
    add_steer_weight_r2(ws, n, dt, &w);
}

/// Select weight based on curvature.
fn select_weight(params: &MpcParams, curvature: f64) -> &MpcWeight {
    if params.low_curvature_thresh > 0.0 && libm::fabs(curvature) < params.low_curvature_thresh {
        &params.low_curvature_weight
    } else {
        &params.nominal_weight
    }
}

/// Add steer rate and steer acceleration penalties to R2.
fn add_steer_weight_r2(ws: &mut MpcWorkspace, n: usize, dt: f64, w: &MpcWeight) {
    if n < 2 {
        return;
    }
    let dt2 = dt * dt;

    // Steer rate penalty: D_rate = steer_rate/dt² * [[1,-1],[-1,1]]
    if w.steer_rate > 0.0 {
        let s = w.steer_rate / dt2;
        for i in 0..n - 1 {
            ws.r2[i * n + i] += s;
            ws.r2[i * n + (i + 1)] -= s;
            ws.r2[(i + 1) * n + i] -= s;
            ws.r2[(i + 1) * n + (i + 1)] += s;
        }
    }

    // Steer acceleration penalty
    if w.steer_acc > 0.0 && n >= 3 {
        let s = w.steer_acc / (dt2 * dt2);
        for i in 1..n - 1 {
            ws.r2[(i - 1) * n + (i - 1)] += s;
            ws.r2[(i - 1) * n + i] -= 2.0 * s;
            ws.r2[(i - 1) * n + (i + 1)] += s;
            ws.r2[i * n + (i - 1)] -= 2.0 * s;
            ws.r2[i * n + i] += 4.0 * s;
            ws.r2[i * n + (i + 1)] -= 2.0 * s;
            ws.r2[(i + 1) * n + (i - 1)] += s;
            ws.r2[(i + 1) * n + i] -= 2.0 * s;
            ws.r2[(i + 1) * n + (i + 1)] += s;
        }
    }

    // Lateral jerk penalty
    if w.lat_jerk > 0.0 {
        for i in 0..n - 1 {
            // Get velocity at this step (approximate from cost weights)
            let vx = 1.0; // Simplified; in C++ this uses per-step velocity
            let j = vx * vx * w.lat_jerk / dt2;
            ws.r2[i * n + i] += j;
            ws.r2[i * n + (i + 1)] -= j;
            ws.r2[(i + 1) * n + i] -= j;
            ws.r2[(i + 1) * n + (i + 1)] += j;
        }
    }
}

/// Build QP cost matrices H and f from prediction matrices.
///
/// H = CB^T * diag(Q) * CB + diag(R1) + R2
/// f = (C*(A*x0 + W))^T * diag(Q) * CB - Uref^T * diag(R1)
fn build_qp(dx: usize, dy: usize, n: usize, x0: &[f64], ws: &mut MpcWorkspace) {
    let ny = dy * n;

    // Compute ax_w = Aex * x0 + Wex (length: dx*n)
    for i in 0..dx * n {
        let mut sum = 0.0;
        for k in 0..dx {
            sum += ws.aex[i * dx + k] * x0[k];
        }
        ws.ax_w[i] = sum + ws.wex[i];
    }

    // Compute caw = Cex * ax_w (length: ny)
    // Cex is block diagonal with blocks Cd at each step
    // Since we computed CB incrementally, we need to compute Cex * ax_w similarly
    // caw[i*dy..(i+1)*dy] = Cd_i * ax_w[i*dx..(i+1)*dx]
    // We use the Cd stored in the last dm computation, but actually Cd doesn't change
    // between models for the same model type. For simplicity, recompute from CB structure.
    // Actually, Cd is the same for all steps of the same model, so we can extract it.
    let cd = &ws.dm.cd;
    for i in 0..n {
        for r in 0..dy {
            let mut sum = 0.0;
            for k in 0..dx {
                sum += cd[r * dx + k] * ws.ax_w[i * dx + k];
            }
            ws.caw[i * dy + r] = sum;
        }
    }

    // Compute QCB: diag(Q) * CB
    for i in 0..ny {
        let q = ws.q_diag[i];
        for j in 0..n {
            ws.qcb[i * n + j] = q * ws.cb[i * n + j];
        }
    }

    // H = CB^T * QCB + diag(R1) + R2
    // H is n×n, CB is ny×n, QCB is ny×n
    mat::mul_at_b(&ws.cb, ny, n, &ws.qcb, n, &mut ws.h);
    // Add R1 diagonal
    for i in 0..n {
        ws.h[i * n + i] += ws.r1_diag[i];
    }
    // Add R2
    mat::add_inplace(&mut ws.h, &ws.r2, n * n);

    // f = caw^T * QCB - Uref^T * diag(R1)
    // f is n×1
    // f[j] = sum_i { caw[i] * QCB[i,j] } - Uref[j] * R1[j]
    for j in 0..n {
        let mut sum = 0.0;
        for i in 0..ny {
            sum += ws.caw[i] * ws.qcb[i * n + j];
        }
        ws.f_vec[j] = sum - ws.uref[j] * ws.r1_diag[j];
    }
}

/// Delay compensation: propagate state forward by input_delay using stored inputs.
fn delay_compensate(
    model: &mut VehicleModel,
    params: &MpcParams,
    traj: &MpcTrajectory,
    x0: &mut [f64],
    dt: f64,
    ws: &mut MpcWorkspace,
) {
    if params.input_delay <= 0.0 || ws.input_buffer_len == 0 {
        return;
    }

    let dx = model.dim_x;
    let delay_steps = libm::ceil(params.input_delay / dt) as usize;
    let steps = delay_steps.min(ws.input_buffer_len);

    let mut dm = DiscreteMatrices::default();
    let mut x_next = [0.0; MAX_DIM_X];

    for i in 0..steps {
        let buf_idx = if ws.input_buffer_len > steps {
            ws.input_buffer_len - steps + i
        } else {
            i
        };
        let u = ws.input_buffer[buf_idx];

        let pt = &traj.points[i.min(traj.len - 1)];
        model.set_state(libm::fmax(libm::fabs(pt.vx), 0.01), pt.curvature);
        model.calculate_discrete_matrix(dt, &mut dm);

        // x_next = Ad * x + Bd * u + Wd
        mat::mul(&dm.ad, dx, dx, x0, 1, &mut x_next);
        for r in 0..dx {
            x_next[r] += dm.bd[r] * u + dm.wd[r];
        }
        x0[..dx].copy_from_slice(&x_next[..dx]);
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

    fn make_straight_traj(n: usize, vx: f64, dt: f64) -> MpcTrajectory {
        let mut traj = MpcTrajectory::new();
        for i in 0..n {
            traj.push(MpcTrajPoint {
                x: i as f64 * vx * dt,
                y: 0.0,
                yaw: 0.0,
                vx,
                curvature: 0.0,
                smooth_curvature: 0.0,
                time: i as f64 * dt,
            });
        }
        traj
    }

    #[test]
    fn test_straight_line_zero_steer() {
        let mut model = VehicleModel::new_kinematics(2.74, 0.7, 0.27);
        let params = MpcParams {
            prediction_horizon: 10,
            prediction_dt: 0.1,
            ..MpcParams::default()
        };
        let traj = make_straight_traj(20, 5.0, 0.1);
        let x0 = [0.0, 0.0, 0.0]; // no error
        let mut ws = MpcWorkspace::new();

        let out = calculate_mpc(&mut model, &params, &traj, &x0, 0.7, 0.033, &mut ws);
        assert!(out.success);
        // On a straight line with zero error, steer should be near zero
        assert!(
            libm::fabs(out.steer_cmd) < 0.05,
            "steer_cmd = {}",
            out.steer_cmd
        );
    }

    #[test]
    fn test_lateral_error_correction() {
        let mut model = VehicleModel::new_kinematics(2.74, 0.7, 0.27);
        let params = MpcParams {
            prediction_horizon: 10,
            prediction_dt: 0.1,
            ..MpcParams::default()
        };
        let traj = make_straight_traj(20, 5.0, 0.1);
        // Lateral error of 1.0m to the left
        let x0 = [1.0, 0.0, 0.0];
        let mut ws = MpcWorkspace::new();

        let out = calculate_mpc(&mut model, &params, &traj, &x0, 0.7, 0.033, &mut ws);
        assert!(out.success);
        // Should steer right (negative) to correct left error
        assert!(
            out.steer_cmd < -0.001,
            "expected negative steer, got {}",
            out.steer_cmd
        );
    }

    #[test]
    fn test_yaw_error_correction() {
        let mut model = VehicleModel::new_kinematics(2.74, 0.7, 0.27);
        let params = MpcParams {
            prediction_horizon: 10,
            prediction_dt: 0.1,
            ..MpcParams::default()
        };
        let traj = make_straight_traj(20, 5.0, 0.1);
        // Yaw error of 0.1 rad (heading left of trajectory)
        let x0 = [0.0, 0.1, 0.0];
        let mut ws = MpcWorkspace::new();

        let out = calculate_mpc(&mut model, &params, &traj, &x0, 0.7, 0.033, &mut ws);
        assert!(out.success);
        // Should steer right to correct yaw error
        assert!(
            out.steer_cmd < -0.001,
            "expected negative steer, got {}",
            out.steer_cmd
        );
    }

    #[test]
    fn test_steer_clamped_to_limit() {
        let mut model = VehicleModel::new_kinematics(2.74, 0.7, 0.27);
        let params = MpcParams {
            prediction_horizon: 10,
            prediction_dt: 0.1,
            ..MpcParams::default()
        };
        let traj = make_straight_traj(20, 5.0, 0.1);
        // Huge lateral error
        let x0 = [100.0, 0.0, 0.0];
        let steer_lim = 0.5;
        let mut ws = MpcWorkspace::new();

        let out = calculate_mpc(&mut model, &params, &traj, &x0, steer_lim, 0.033, &mut ws);
        assert!(out.success);
        assert!(libm::fabs(out.steer_cmd) <= steer_lim + 1e-10);
    }

    #[test]
    fn test_kinematics_no_delay_model() {
        let mut model = VehicleModel::new_kinematics_no_delay(2.74, 0.7);
        let params = MpcParams {
            prediction_horizon: 10,
            prediction_dt: 0.1,
            ..MpcParams::default()
        };
        let traj = make_straight_traj(20, 5.0, 0.1);
        let x0 = [0.5, 0.0]; // lateral error
        let mut ws = MpcWorkspace::new();

        let out = calculate_mpc(&mut model, &params, &traj, &x0, 0.7, 0.033, &mut ws);
        assert!(out.success);
        assert!(out.steer_cmd < -0.001);
    }

    #[test]
    fn test_empty_trajectory_fails() {
        let mut model = VehicleModel::new_kinematics(2.74, 0.7, 0.27);
        let params = MpcParams::default();
        let traj = MpcTrajectory::new();
        let x0 = [0.0, 0.0, 0.0];
        let mut ws = MpcWorkspace::new();

        let out = calculate_mpc(&mut model, &params, &traj, &x0, 0.7, 0.033, &mut ws);
        assert!(!out.success);
    }

    #[test]
    fn test_curved_trajectory_feedforward() {
        let mut model = VehicleModel::new_kinematics(2.74, 0.7, 0.27);
        let params = MpcParams {
            prediction_horizon: 10,
            prediction_dt: 0.1,
            zero_ff_steer_deg: 0.0, // Don't zero out feedforward
            ..MpcParams::default()
        };

        // Curved trajectory (constant curvature = 0.05 = R=20m)
        let curv = 0.05;
        let mut traj = MpcTrajectory::new();
        for i in 0..20 {
            let t = i as f64 * 0.1;
            traj.push(MpcTrajPoint {
                x: t * 5.0,
                y: 0.0,
                yaw: 0.0,
                vx: 5.0,
                curvature: curv,
                smooth_curvature: curv,
                time: t,
            });
        }

        let x0 = [0.0, 0.0, 0.0];
        let mut ws = MpcWorkspace::new();

        let out = calculate_mpc(&mut model, &params, &traj, &x0, 0.7, 0.033, &mut ws);
        assert!(out.success);
        // Should produce positive steering for left curve
        let expected_ff = libm::atan(2.74 * curv); // ≈ 0.137 rad
        assert!(
            out.steer_cmd > 0.05,
            "expected positive steer for left curve, got {}",
            out.steer_cmd
        );
        // Should be in the ballpark of the feedforward
        assert!(
            (out.steer_cmd - expected_ff).abs() < 0.2,
            "steer {} far from feedforward {}",
            out.steer_cmd,
            expected_ff
        );
    }
}
