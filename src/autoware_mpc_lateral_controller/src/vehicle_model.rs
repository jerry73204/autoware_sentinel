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

//! Bicycle vehicle models for MPC prediction.
//!
//! Port of the vehicle model hierarchy from Autoware's MPC lateral controller.
//! Three models: kinematics (with steering delay), kinematics (no delay), dynamics.

use crate::mat;

/// Maximum state dimension across all models.
pub const MAX_DIM_X: usize = 4;
/// Output dimension (lateral error, yaw error).
pub const MAX_DIM_Y: usize = 2;
/// Input dimension (steering command).
pub const MAX_DIM_U: usize = 1;

/// Discrete-time model matrices for one prediction step.
///
/// x[k+1] = Ad * x[k] + Bd * u[k] + Wd
/// y[k] = Cd * x[k]
#[derive(Clone)]
pub struct DiscreteMatrices {
    /// State transition (dim_x × dim_x), row-major.
    pub ad: [f64; MAX_DIM_X * MAX_DIM_X],
    /// Input matrix (dim_x × dim_u), row-major.
    pub bd: [f64; MAX_DIM_X * MAX_DIM_U],
    /// Output matrix (dim_y × dim_x), row-major.
    pub cd: [f64; MAX_DIM_Y * MAX_DIM_X],
    /// Disturbance vector (dim_x × 1).
    pub wd: [f64; MAX_DIM_X],
}

impl Default for DiscreteMatrices {
    fn default() -> Self {
        Self {
            ad: [0.0; MAX_DIM_X * MAX_DIM_X],
            bd: [0.0; MAX_DIM_X * MAX_DIM_U],
            cd: [0.0; MAX_DIM_Y * MAX_DIM_X],
            wd: [0.0; MAX_DIM_X],
        }
    }
}

/// Vehicle model type selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleModelType {
    /// Kinematics with first-order steering delay (dim_x=3).
    Kinematics,
    /// Kinematics without steering delay (dim_x=2).
    KinematicsNoDelay,
    /// Nonlinear dynamics with tire slip (dim_x=4).
    Dynamics,
}

/// Vehicle model that computes discrete-time state-space matrices.
pub struct VehicleModel {
    pub model_type: VehicleModelType,
    pub dim_x: usize,
    pub dim_y: usize,
    pub dim_u: usize,
    pub wheelbase: f64,
    pub steer_lim: f64,
    // Kinematics: steering time constant
    pub steer_tau: f64,
    // Dynamics: vehicle params
    pub mass: f64,
    pub lf: f64,
    pub lr: f64,
    pub iz: f64,
    pub cf: f64,
    pub cr: f64,
    // Runtime state
    pub velocity: f64,
    pub curvature: f64,
}

impl VehicleModel {
    /// Create a kinematics bicycle model with steering delay.
    pub fn new_kinematics(wheelbase: f64, steer_lim: f64, steer_tau: f64) -> Self {
        Self {
            model_type: VehicleModelType::Kinematics,
            dim_x: 3,
            dim_y: 2,
            dim_u: 1,
            wheelbase,
            steer_lim,
            steer_tau,
            mass: 0.0,
            lf: 0.0,
            lr: 0.0,
            iz: 0.0,
            cf: 0.0,
            cr: 0.0,
            velocity: 0.0,
            curvature: 0.0,
        }
    }

    /// Create a kinematics bicycle model without steering delay.
    pub fn new_kinematics_no_delay(wheelbase: f64, steer_lim: f64) -> Self {
        Self {
            model_type: VehicleModelType::KinematicsNoDelay,
            dim_x: 2,
            dim_y: 2,
            dim_u: 1,
            wheelbase,
            steer_lim,
            steer_tau: 0.0,
            mass: 0.0,
            lf: 0.0,
            lr: 0.0,
            iz: 0.0,
            cf: 0.0,
            cr: 0.0,
            velocity: 0.0,
            curvature: 0.0,
        }
    }

    /// Create a dynamics bicycle model.
    pub fn new_dynamics(
        wheelbase: f64,
        steer_lim: f64,
        mass_fl: f64,
        mass_fr: f64,
        mass_rl: f64,
        mass_rr: f64,
        cf: f64,
        cr: f64,
    ) -> Self {
        let mass = mass_fl + mass_fr + mass_rl + mass_rr;
        let lf = wheelbase * (1.0 - (mass_fl + mass_fr) / mass);
        let lr = wheelbase * (1.0 - (mass_rl + mass_rr) / mass);
        let iz = lf * lf * (mass_fl + mass_fr) + lr * lr * (mass_rl + mass_rr);
        Self {
            model_type: VehicleModelType::Dynamics,
            dim_x: 4,
            dim_y: 2,
            dim_u: 1,
            wheelbase,
            steer_lim,
            steer_tau: 0.0,
            mass,
            lf,
            lr,
            iz,
            cf,
            cr,
            velocity: 0.0,
            curvature: 0.0,
        }
    }

    /// Set the current velocity and curvature for linearization.
    pub fn set_state(&mut self, velocity: f64, curvature: f64) {
        self.velocity = velocity;
        self.curvature = curvature;
    }

    /// Compute discrete-time matrices using bilinear (Tustin) discretization.
    pub fn calculate_discrete_matrix(&self, dt: f64, out: &mut DiscreteMatrices) {
        match self.model_type {
            VehicleModelType::Kinematics => self.discrete_kinematics(dt, out),
            VehicleModelType::KinematicsNoDelay => self.discrete_kinematics_no_delay(dt, out),
            VehicleModelType::Dynamics => self.discrete_dynamics(dt, out),
        }
    }

    fn discrete_kinematics(&self, dt: f64, out: &mut DiscreteMatrices) {
        let n = 3;
        let v = self.velocity;
        let wb = self.wheelbase;
        let tau = self.steer_tau;

        // Reference steering from curvature
        let delta_r = libm::atan(wb * self.curvature);
        let cos_dr = libm::cos(delta_r);
        let cos_dr2 = cos_dr * cos_dr;

        // Continuous A, B, W matrices
        let mut a_c = [0.0; 9]; // 3×3
        a_c[0 * 3 + 1] = v;
        a_c[1 * 3 + 2] = v / (wb * cos_dr2);
        a_c[2 * 3 + 2] = -1.0 / tau;

        let mut b_c = [0.0; 3]; // 3×1
        b_c[2] = 1.0 / tau;

        let mut w_c = [0.0; 3]; // 3×1
        w_c[1] = -v * delta_r / (wb * cos_dr2);

        // C matrix (2×3)
        let mut c = [0.0; 6];
        c[0 * 3 + 0] = 1.0; // y[0] = e
        c[1 * 3 + 1] = 1.0; // y[1] = th

        bilinear_discretize(&a_c, &b_c, &w_c, &c, n, 2, dt, out);
    }

    fn discrete_kinematics_no_delay(&self, dt: f64, out: &mut DiscreteMatrices) {
        let n = 2;
        let v = self.velocity;
        let wb = self.wheelbase;

        let delta_r = libm::atan(wb * self.curvature);
        let cos_dr = libm::cos(delta_r);
        let cos_dr2 = cos_dr * cos_dr;

        // Continuous A (2×2)
        let mut a_c = [0.0; 4];
        a_c[0 * 2 + 1] = v;

        // B (2×1)
        let mut b_c = [0.0; 2];
        b_c[1] = v / (wb * cos_dr2);

        // W (2×1)
        let mut w_c = [0.0; 2];
        w_c[1] = -v * delta_r / (wb * cos_dr2);

        // C (2×2) = identity
        let mut c = [0.0; 4];
        c[0] = 1.0;
        c[3] = 1.0;

        bilinear_discretize(&a_c, &b_c, &w_c, &c, n, 2, dt, out);
    }

    fn discrete_dynamics(&self, dt: f64, out: &mut DiscreteMatrices) {
        let n = 4;
        let v = libm::fmax(self.velocity, 0.01);
        let m = self.mass;
        let lf = self.lf;
        let lr = self.lr;
        let iz = self.iz;
        let cf = self.cf;
        let cr = self.cr;

        // Continuous A (4×4)
        let mut a_c = [0.0; 16];
        a_c[0 * 4 + 1] = 1.0;
        a_c[1 * 4 + 1] = -(cf + cr) / (m * v);
        a_c[1 * 4 + 2] = (cf + cr) / m;
        a_c[1 * 4 + 3] = (lr * cr - lf * cf) / (m * v);
        a_c[2 * 4 + 3] = 1.0;
        a_c[3 * 4 + 1] = (lr * cr - lf * cf) / (iz * v);
        a_c[3 * 4 + 2] = (lf * cf - lr * cr) / iz;
        a_c[3 * 4 + 3] = -(lf * lf * cf + lr * lr * cr) / (iz * v);

        // B (4×1)
        let mut b_c = [0.0; 4];
        b_c[1] = cf / m;
        b_c[3] = lf * cf / iz;

        // W (4×1)
        let mut w_c = [0.0; 4];
        w_c[1] = ((lr * cr - lf * cf) / (m * v) - v) * self.curvature;
        w_c[3] = -(lf * lf * cf + lr * lr * cr) / (iz * v) * self.curvature;

        // C (2×4)
        let mut c = [0.0; 8];
        c[0 * 4 + 0] = 1.0; // y[0] = e
        c[1 * 4 + 2] = 1.0; // y[1] = th

        bilinear_discretize(&a_c, &b_c, &w_c, &c, n, 2, dt, out);
    }
}

/// Bilinear (Tustin) discretization.
///
/// Ad = (I - dt/2 * A)^-1 * (I + dt/2 * A)
/// Bd = (I - dt/2 * A)^-1 * B * dt
/// Wd = (I - dt/2 * A)^-1 * W * dt
/// Cd = C (unchanged)
fn bilinear_discretize(
    a_c: &[f64],
    b_c: &[f64],
    w_c: &[f64],
    c: &[f64],
    dim_x: usize,
    dim_y: usize,
    dt: f64,
    out: &mut DiscreteMatrices,
) {
    let nn = dim_x * dim_x;

    // M = I - dt/2 * A
    let mut m_mat = [0.0; MAX_DIM_X * MAX_DIM_X];
    mat::identity(&mut m_mat, dim_x);
    for i in 0..nn {
        m_mat[i] -= 0.5 * dt * a_c[i];
    }

    // M_inv
    let mut m_inv = [0.0; MAX_DIM_X * MAX_DIM_X];
    if !mat::inv_small(&m_mat, dim_x, &mut m_inv) {
        // Fallback: identity (should not happen with reasonable dt)
        mat::identity(&mut out.ad, dim_x);
        mat::zeros(&mut out.bd[..dim_x]);
        mat::zeros(&mut out.wd[..dim_x]);
        mat::copy(c, &mut out.cd, dim_y * dim_x);
        return;
    }

    // N = I + dt/2 * A
    let mut n_mat = [0.0; MAX_DIM_X * MAX_DIM_X];
    mat::identity(&mut n_mat, dim_x);
    for i in 0..nn {
        n_mat[i] += 0.5 * dt * a_c[i];
    }

    // Ad = M_inv * N
    mat::mul(&m_inv, dim_x, dim_x, &n_mat, dim_x, &mut out.ad);

    // Bd = M_inv * B * dt
    let mut b_dt = [0.0; MAX_DIM_X];
    mat::scale(b_c, dt, &mut b_dt, dim_x);
    mat::mul(&m_inv, dim_x, dim_x, &b_dt, 1, &mut out.bd);

    // Wd = M_inv * W * dt
    let mut w_dt = [0.0; MAX_DIM_X];
    mat::scale(w_c, dt, &mut w_dt, dim_x);
    mat::mul(&m_inv, dim_x, dim_x, &w_dt, 1, &mut out.wd);

    // Cd = C (unchanged)
    mat::copy(c, &mut out.cd, dim_y * dim_x);
}

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

    /// Kinematics discretization never panics for any velocity/curvature.
    #[kani::proof]
    fn kinematics_discrete_never_panics() {
        let mut model = VehicleModel::new_kinematics(2.74, 0.7, 0.27);
        let vel = any_finite_f64();
        kani::assume(vel > 0.0 && vel < 100.0);
        let curv = any_finite_f64();
        kani::assume(libm::fabs(curv) < 1.0);
        let dt = any_finite_f64();
        kani::assume(dt > 0.001 && dt < 1.0);

        model.set_state(vel, curv);
        let mut dm = DiscreteMatrices::default();
        model.calculate_discrete_matrix(dt, &mut dm);
    }

    /// KinematicsNoDelay discretization never panics.
    #[kani::proof]
    fn kinematics_no_delay_discrete_never_panics() {
        let mut model = VehicleModel::new_kinematics_no_delay(2.74, 0.7);
        let vel = any_finite_f64();
        kani::assume(vel > 0.0 && vel < 100.0);
        let curv = any_finite_f64();
        kani::assume(libm::fabs(curv) < 1.0);
        let dt = any_finite_f64();
        kani::assume(dt > 0.001 && dt < 1.0);

        model.set_state(vel, curv);
        let mut dm = DiscreteMatrices::default();
        model.calculate_discrete_matrix(dt, &mut dm);
    }

    /// Dynamics discretization never panics for realistic parameters.
    #[kani::proof]
    fn dynamics_discrete_never_panics() {
        let mut model =
            VehicleModel::new_dynamics(2.74, 0.7, 400.0, 400.0, 450.0, 450.0, 150000.0, 150000.0);
        let vel = any_finite_f64();
        kani::assume(vel > 0.01 && vel < 100.0);
        let curv = any_finite_f64();
        kani::assume(libm::fabs(curv) < 1.0);
        let dt = any_finite_f64();
        kani::assume(dt > 0.001 && dt < 1.0);

        model.set_state(vel, curv);
        let mut dm = DiscreteMatrices::default();
        model.calculate_discrete_matrix(dt, &mut dm);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kinematics_dimensions() {
        let model = VehicleModel::new_kinematics(2.74, 0.7, 0.27);
        assert_eq!(model.dim_x, 3);
        assert_eq!(model.dim_y, 2);
        assert_eq!(model.dim_u, 1);
    }

    #[test]
    fn test_kinematics_no_delay_dimensions() {
        let model = VehicleModel::new_kinematics_no_delay(2.74, 0.7);
        assert_eq!(model.dim_x, 2);
    }

    #[test]
    fn test_dynamics_dimensions() {
        let model =
            VehicleModel::new_dynamics(2.74, 0.7, 400.0, 400.0, 450.0, 450.0, 150000.0, 150000.0);
        assert_eq!(model.dim_x, 4);
    }

    #[test]
    fn test_kinematics_discrete_at_zero_curvature() {
        let mut model = VehicleModel::new_kinematics(2.74, 0.7, 0.27);
        model.set_state(5.0, 0.0);
        let mut dm = DiscreteMatrices::default();
        model.calculate_discrete_matrix(0.1, &mut dm);

        // Ad should be close to identity + dt*A_c for small dt
        // Ad[0,0] should be close to 1
        assert!((dm.ad[0] - 1.0).abs() < 0.1);
        // Wd should be near zero with zero curvature
        assert!(dm.wd[0].abs() < 1e-6);
        assert!(dm.wd[2].abs() < 1e-6);
    }

    #[test]
    fn test_kinematics_no_delay_discrete() {
        let mut model = VehicleModel::new_kinematics_no_delay(2.74, 0.7);
        model.set_state(5.0, 0.0);
        let mut dm = DiscreteMatrices::default();
        model.calculate_discrete_matrix(0.1, &mut dm);
        assert!((dm.ad[0] - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_dynamics_discrete() {
        let mut model =
            VehicleModel::new_dynamics(2.74, 0.7, 400.0, 400.0, 450.0, 450.0, 150000.0, 150000.0);
        model.set_state(10.0, 0.0);
        let mut dm = DiscreteMatrices::default();
        model.calculate_discrete_matrix(0.1, &mut dm);
        // Ad diagonal should be close to 1 for small dt
        assert!((dm.ad[0 * 4 + 0] - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_state_propagation_straight() {
        // On a straight road (curvature=0), zero initial state should stay at zero
        let mut model = VehicleModel::new_kinematics(2.74, 0.7, 0.27);
        model.set_state(5.0, 0.0);
        let mut dm = DiscreteMatrices::default();
        model.calculate_discrete_matrix(0.1, &mut dm);

        let x = [0.0, 0.0, 0.0]; // zero lateral error, zero yaw error, zero steer
        let u = [0.0]; // zero steering command
        let mut x_next = [0.0; 3];
        // x_next = Ad * x + Bd * u + Wd
        mat::mul(&dm.ad, 3, 3, &x, 1, &mut x_next);
        // Add Bd*u (zero) and Wd (near-zero for zero curvature)
        for i in 0..3 {
            x_next[i] += dm.bd[i] * u[0] + dm.wd[i];
        }
        // Should remain near zero
        assert!(x_next[0].abs() < 1e-6);
        assert!(x_next[1].abs() < 1e-6);
    }
}
