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

//! Port of autoware_vehicle_velocity_converter from Autoware Core.
//!
//! Converts `VelocityReport` to `TwistWithCovarianceStamped`, mapping
//! longitudinal/lateral velocity and heading rate to twist fields with
//! a diagonal covariance matrix.

#![no_std]

use autoware_vehicle_msgs::msg::VelocityReport;
use geometry_msgs::msg::{Twist, TwistWithCovariance, TwistWithCovarianceStamped, Vector3};

/// Converts VelocityReport messages into TwistWithCovarianceStamped.
#[derive(Debug, Clone)]
pub struct VehicleVelocityConverter {
    speed_scale_factor: f64,
    stddev_vx: f64,
    stddev_wz: f64,
}

impl VehicleVelocityConverter {
    /// Create a new converter with the given parameters.
    pub fn new(speed_scale_factor: f64, stddev_vx: f64, stddev_wz: f64) -> Self {
        Self {
            speed_scale_factor,
            stddev_vx,
            stddev_wz,
        }
    }

    /// Convert a VelocityReport into a TwistWithCovarianceStamped.
    ///
    /// Field mapping:
    /// - `longitudinal_velocity * speed_scale_factor` → `twist.linear.x`
    /// - `lateral_velocity` → `twist.linear.y`
    /// - `heading_rate` → `twist.angular.z`
    ///
    /// Covariance: 6x6 diagonal with `[stddev_vx², 10000, 10000, 10000, 10000, stddev_wz²]`.
    pub fn convert(&self, report: &VelocityReport) -> TwistWithCovarianceStamped {
        let mut covariance = [0.0f64; 36];
        covariance[0] = self.stddev_vx * self.stddev_vx; // [0,0]
        covariance[7] = 10000.0; // [1,1]
        covariance[14] = 10000.0; // [2,2]
        covariance[21] = 10000.0; // [3,3]
        covariance[28] = 10000.0; // [4,4]
        covariance[35] = self.stddev_wz * self.stddev_wz; // [5,5]

        TwistWithCovarianceStamped {
            header: report.header.clone(),
            twist: TwistWithCovariance {
                twist: Twist {
                    linear: Vector3 {
                        x: report.longitudinal_velocity as f64 * self.speed_scale_factor,
                        y: report.lateral_velocity as f64,
                        z: 0.0,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: report.heading_rate as f64,
                    },
                },
                covariance,
            },
        }
    }
}

#[cfg(kani)]
mod verification {
    use super::*;

    /// Helper: create a symbolic f64 from arbitrary bits.
    fn any_f64() -> f64 {
        f64::from_bits(kani::any::<u64>())
    }

    /// Helper: create a symbolic finite f64.
    fn any_finite_f64() -> f64 {
        let v = any_f64();
        kani::assume(!v.is_nan() && v.is_finite());
        v
    }

    /// Helper: create a symbolic f32 from arbitrary bits.
    fn any_f32() -> f32 {
        f32::from_bits(kani::any::<u32>())
    }

    /// Helper: create a symbolic finite f32.
    fn any_finite_f32() -> f32 {
        let v = any_f32();
        kani::assume(!v.is_nan() && v.is_finite());
        v
    }

    /// Covariance diagonal entries are non-negative; off-diagonal entries are zero.
    #[kani::proof]
    fn covariance_valid_diagonal() {
        let stddev_vx = any_finite_f64();
        let stddev_wz = any_finite_f64();
        let conv = VehicleVelocityConverter::new(1.0, stddev_vx, stddev_wz);

        let report = VelocityReport {
            header: Default::default(),
            longitudinal_velocity: 0.0,
            lateral_velocity: 0.0,
            heading_rate: 0.0,
        };

        let result = conv.convert(&report);
        let cov = &result.twist.covariance;

        // Diagonal entries are non-negative (squares)
        assert!(cov[0] >= 0.0);
        assert!(cov[7] >= 0.0);
        assert!(cov[14] >= 0.0);
        assert!(cov[21] >= 0.0);
        assert!(cov[28] >= 0.0);
        assert!(cov[35] >= 0.0);

        // Off-diagonal entries are zero
        assert!(cov[1] == 0.0);
        assert!(cov[6] == 0.0);
        assert!(cov[2] == 0.0);
        assert!(cov[12] == 0.0);
    }

    /// f32 → f64 cast preserves sign for non-zero values.
    #[kani::proof]
    fn f32_cast_preserves_sign() {
        let input = any_finite_f32();
        kani::assume(input != 0.0);

        let conv = VehicleVelocityConverter::new(1.0, 0.2, 0.1);
        let report = VelocityReport {
            header: Default::default(),
            longitudinal_velocity: input,
            lateral_velocity: 0.0,
            heading_rate: 0.0,
        };

        let result = conv.convert(&report);
        let output = result.twist.twist.linear.x;
        assert!(output.is_sign_positive() == input.is_sign_positive());
    }

    /// `convert` never panics for any input (including NaN, Inf, subnormals).
    #[kani::proof]
    fn convert_never_panics() {
        let conv = VehicleVelocityConverter::new(any_f64(), any_f64(), any_f64());

        let report = VelocityReport {
            header: Default::default(),
            longitudinal_velocity: any_f32(),
            lateral_velocity: any_f32(),
            heading_rate: any_f32(),
        };

        let _result = conv.convert(&report);
    }

    /// Output linear.x and angular.z are not NaN when inputs are finite.
    #[kani::proof]
    fn convert_output_nan_check() {
        let scale = any_finite_f64();
        let conv = VehicleVelocityConverter::new(scale, 0.2, 0.1);

        let report = VelocityReport {
            header: Default::default(),
            longitudinal_velocity: any_finite_f32(),
            lateral_velocity: any_finite_f32(),
            heading_rate: any_finite_f32(),
        };

        let result = conv.convert(&report);
        assert!(!result.twist.twist.linear.x.is_nan());
        assert!(!result.twist.twist.angular.z.is_nan());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_converter() -> VehicleVelocityConverter {
        VehicleVelocityConverter::new(1.0, 0.2, 0.1)
    }

    fn make_report(longitudinal: f32, lateral: f32, heading_rate: f32) -> VelocityReport {
        VelocityReport {
            header: Default::default(),
            longitudinal_velocity: longitudinal,
            lateral_velocity: lateral,
            heading_rate,
        }
    }

    #[test]
    fn field_mapping() {
        let conv = default_converter();
        let report = make_report(10.0, 0.5, 0.3);
        let result = conv.convert(&report);

        assert_eq!(result.twist.twist.linear.x, 10.0);
        assert_eq!(result.twist.twist.linear.y, 0.5);
        assert_eq!(result.twist.twist.linear.z, 0.0);
        assert_eq!(result.twist.twist.angular.x, 0.0);
        assert_eq!(result.twist.twist.angular.y, 0.0);
        // f32 → f64 cast: 0.3f32 as f64 != 0.3f64
        assert_eq!(result.twist.twist.angular.z, 0.3f32 as f64);
    }

    #[test]
    fn speed_scale_factor() {
        let conv = VehicleVelocityConverter::new(2.0, 0.2, 0.1);
        let report = make_report(5.0, 0.0, 0.0);
        let result = conv.convert(&report);

        assert_eq!(result.twist.twist.linear.x, 10.0);
    }

    #[test]
    fn covariance_diagonal() {
        let conv = VehicleVelocityConverter::new(1.0, 0.2, 0.1);
        let report = make_report(0.0, 0.0, 0.0);
        let result = conv.convert(&report);

        let cov = &result.twist.covariance;
        let eps = 1e-12;

        // stddev_vx² = 0.04
        assert!((cov[0] - 0.04).abs() < eps);
        assert!((cov[7] - 10000.0).abs() < eps);
        assert!((cov[14] - 10000.0).abs() < eps);
        assert!((cov[21] - 10000.0).abs() < eps);
        assert!((cov[28] - 10000.0).abs() < eps);
        // stddev_wz² = 0.01
        assert!((cov[35] - 0.01).abs() < eps);

        // Off-diagonal should be zero
        assert_eq!(cov[1], 0.0);
        assert_eq!(cov[6], 0.0);
    }

    #[test]
    fn header_preserved() {
        let conv = default_converter();
        let mut report = make_report(1.0, 0.0, 0.0);
        report.header.frame_id = "base_link".try_into().unwrap();
        let result = conv.convert(&report);

        assert_eq!(result.header.frame_id.as_str(), "base_link");
    }

    #[test]
    fn zero_input() {
        let conv = default_converter();
        let report = make_report(0.0, 0.0, 0.0);
        let result = conv.convert(&report);

        assert_eq!(result.twist.twist.linear.x, 0.0);
        assert_eq!(result.twist.twist.linear.y, 0.0);
        assert_eq!(result.twist.twist.angular.z, 0.0);
    }
}
