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

//! Port of autoware_vehicle_info_utils from Autoware Universe.
//!
//! Vehicle geometry parameters used by controllers and planners.
//! Default values are from Autoware's sample vehicle configuration.

#![no_std]

const MIN_POSITIVE: f64 = 1e-6;

/// Vehicle geometry and kinematics parameters.
///
/// Base parameters describe the physical vehicle. Derived parameters are
/// computed from base parameters and represent offsets relative to the
/// rear axle center on the ground plane.
#[derive(Debug, Clone, Copy)]
pub struct VehicleInfo {
    // Base parameters
    pub wheel_radius_m: f64,
    pub wheel_width_m: f64,
    pub wheel_base_m: f64,
    pub wheel_tread_m: f64,
    pub front_overhang_m: f64,
    pub rear_overhang_m: f64,
    pub left_overhang_m: f64,
    pub right_overhang_m: f64,
    pub vehicle_height_m: f64,
    pub max_steer_angle_rad: f64,

    // Derived parameters (relative to rear axle center)
    pub vehicle_length_m: f64,
    pub vehicle_width_m: f64,
    pub min_longitudinal_offset_m: f64,
    pub max_longitudinal_offset_m: f64,
    pub min_lateral_offset_m: f64,
    pub max_lateral_offset_m: f64,
    pub min_height_offset_m: f64,
    pub max_height_offset_m: f64,
}

impl VehicleInfo {
    /// Create a `VehicleInfo` from base parameters, computing all derived values.
    ///
    /// `wheel_base_m` and `max_steer_angle_rad` are clamped to at least 1e-6
    /// to avoid division by zero in controllers.
    pub fn new(
        wheel_radius_m: f64,
        wheel_width_m: f64,
        wheel_base_m: f64,
        wheel_tread_m: f64,
        front_overhang_m: f64,
        rear_overhang_m: f64,
        left_overhang_m: f64,
        right_overhang_m: f64,
        vehicle_height_m: f64,
        max_steer_angle_rad: f64,
    ) -> Self {
        let wb = if libm::fabs(wheel_base_m) < MIN_POSITIVE {
            MIN_POSITIVE
        } else {
            wheel_base_m
        };
        let msa = if libm::fabs(max_steer_angle_rad) < MIN_POSITIVE {
            MIN_POSITIVE
        } else {
            max_steer_angle_rad
        };

        VehicleInfo {
            wheel_radius_m,
            wheel_width_m,
            wheel_base_m: wb,
            wheel_tread_m,
            front_overhang_m,
            rear_overhang_m,
            left_overhang_m,
            right_overhang_m,
            vehicle_height_m,
            max_steer_angle_rad: msa,
            vehicle_length_m: front_overhang_m + wb + rear_overhang_m,
            vehicle_width_m: wheel_tread_m + left_overhang_m + right_overhang_m,
            min_longitudinal_offset_m: -rear_overhang_m,
            max_longitudinal_offset_m: front_overhang_m + wb,
            min_lateral_offset_m: -(wheel_tread_m / 2.0 + right_overhang_m),
            max_lateral_offset_m: wheel_tread_m / 2.0 + left_overhang_m,
            min_height_offset_m: 0.0,
            max_height_offset_m: vehicle_height_m,
        }
    }
}

impl Default for VehicleInfo {
    /// Default values from Autoware's sample vehicle (matching actuation_porting defaults).
    fn default() -> Self {
        Self::new(
            0.39, // wheel_radius_m
            0.42, // wheel_width_m
            2.74, // wheel_base_m
            1.63, // wheel_tread_m
            1.0,  // front_overhang_m
            1.03, // rear_overhang_m
            0.1,  // left_overhang_m
            0.1,  // right_overhang_m
            2.5,  // vehicle_height_m
            0.70, // max_steer_angle_rad
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64) -> bool {
        libm::fabs(a - b) < EPS
    }

    #[test]
    fn test_default_values() {
        let v = VehicleInfo::default();
        assert!(approx_eq(v.wheel_base_m, 2.74));
        assert!(approx_eq(v.wheel_tread_m, 1.63));
        assert!(approx_eq(v.max_steer_angle_rad, 0.70));
    }

    #[test]
    fn test_derived_parameters() {
        let v = VehicleInfo::default();
        // vehicle_length = front_overhang + wheel_base + rear_overhang
        assert!(approx_eq(v.vehicle_length_m, 1.0 + 2.74 + 1.03));
        // vehicle_width = wheel_tread + left_overhang + right_overhang
        assert!(approx_eq(v.vehicle_width_m, 1.63 + 0.1 + 0.1));
        // min_longitudinal_offset = -rear_overhang
        assert!(approx_eq(v.min_longitudinal_offset_m, -1.03));
        // max_longitudinal_offset = front_overhang + wheel_base
        assert!(approx_eq(v.max_longitudinal_offset_m, 1.0 + 2.74));
    }

    #[test]
    fn test_zero_wheel_base_clamped() {
        let v = VehicleInfo::new(0.39, 0.42, 0.0, 1.63, 1.0, 1.03, 0.1, 0.1, 2.5, 0.70);
        assert!(v.wheel_base_m >= MIN_POSITIVE);
    }

    #[test]
    fn test_zero_steer_angle_clamped() {
        let v = VehicleInfo::new(0.39, 0.42, 2.74, 1.63, 1.0, 1.03, 0.1, 0.1, 2.5, 0.0);
        assert!(v.max_steer_angle_rad >= MIN_POSITIVE);
    }

    #[test]
    fn test_lateral_offsets_symmetric_when_equal_overhang() {
        let v = VehicleInfo::default(); // left=right=0.1
        assert!(approx_eq(
            libm::fabs(v.min_lateral_offset_m),
            libm::fabs(v.max_lateral_offset_m)
        ));
    }
}
