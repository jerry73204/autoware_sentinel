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

//! Port of autoware_stop_filter from Autoware Core.
//!
//! Zeros twist components when the vehicle is considered stopped
//! (linear.x and angular.z below their respective thresholds).

#![no_std]

use geometry_msgs::msg::Vector3;

/// Result of applying the stop filter.
#[derive(Debug, Clone, PartialEq)]
pub struct FilterResult {
    pub linear: Vector3,
    pub angular: Vector3,
    pub was_stopped: bool,
}

/// Filters velocity to zero when the vehicle is considered stopped.
///
/// A vehicle is stopped when both `|vx| < vx_threshold` and `|wz| < wz_threshold`.
/// When stopped, all twist components are zeroed. When moving, the input is passed through.
#[derive(Debug, Clone)]
pub struct StopFilter {
    vx_threshold: f64,
    wz_threshold: f64,
}

impl StopFilter {
    /// Create a new stop filter with the given thresholds.
    pub fn new(vx_threshold: f64, wz_threshold: f64) -> Self {
        Self {
            vx_threshold,
            wz_threshold,
        }
    }

    /// Check if the vehicle is stopped based on linear.x and angular.z velocities.
    fn is_stopped(&self, linear: &Vector3, angular: &Vector3) -> bool {
        linear.x.abs() < self.vx_threshold && angular.z.abs() < self.wz_threshold
    }

    /// Apply the stop filter.
    ///
    /// If stopped: returns zeroed linear and angular vectors.
    /// If moving: passes through the input unchanged.
    pub fn apply(&self, linear: &Vector3, angular: &Vector3) -> FilterResult {
        let was_stopped = self.is_stopped(linear, angular);

        if was_stopped {
            FilterResult {
                linear: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                was_stopped,
            }
        } else {
            FilterResult {
                linear: linear.clone(),
                angular: angular.clone(),
                was_stopped,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VX_THRESH: f64 = 0.1;
    const WZ_THRESH: f64 = 0.02;

    fn filter() -> StopFilter {
        StopFilter::new(VX_THRESH, WZ_THRESH)
    }

    #[test]
    fn stopped_when_below_both_thresholds() {
        let f = filter();
        let linear = Vector3 {
            x: 0.05,
            y: 1.0,
            z: 2.0,
        };
        let angular = Vector3 {
            x: 0.5,
            y: 0.5,
            z: 0.01,
        };
        let result = f.apply(&linear, &angular);

        assert!(result.was_stopped);
        assert_eq!(result.linear.x, 0.0);
        assert_eq!(result.linear.y, 0.0);
        assert_eq!(result.linear.z, 0.0);
        assert_eq!(result.angular.x, 0.0);
        assert_eq!(result.angular.y, 0.0);
        assert_eq!(result.angular.z, 0.0);
    }

    #[test]
    fn not_stopped_when_vx_above_threshold() {
        let f = filter();
        let linear = Vector3 {
            x: 0.5,
            y: 1.0,
            z: 2.0,
        };
        let angular = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.01,
        };
        let result = f.apply(&linear, &angular);

        assert!(!result.was_stopped);
        assert_eq!(result.linear.x, 0.5);
        assert_eq!(result.linear.y, 1.0);
        assert_eq!(result.linear.z, 2.0);
        assert_eq!(result.angular.z, 0.01);
    }

    #[test]
    fn not_stopped_when_wz_above_threshold() {
        let f = filter();
        let linear = Vector3 {
            x: 0.05,
            y: 0.0,
            z: 0.0,
        };
        let angular = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.5,
        };
        let result = f.apply(&linear, &angular);

        assert!(!result.was_stopped);
        assert_eq!(result.linear.x, 0.05);
        assert_eq!(result.angular.z, 0.5);
    }

    #[test]
    fn stopped_at_zero() {
        let f = filter();
        let linear = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let angular = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let result = f.apply(&linear, &angular);

        assert!(result.was_stopped);
        assert_eq!(result.linear.x, 0.0);
        assert_eq!(result.angular.z, 0.0);
    }

    #[test]
    fn negative_velocity_below_threshold_is_stopped() {
        let f = filter();
        let linear = Vector3 {
            x: -0.05,
            y: 0.0,
            z: 0.0,
        };
        let angular = Vector3 {
            x: 0.0,
            y: 0.0,
            z: -0.01,
        };
        let result = f.apply(&linear, &angular);

        assert!(result.was_stopped);
    }

    #[test]
    fn negative_velocity_above_threshold_passes_through() {
        let f = filter();
        let linear = Vector3 {
            x: -0.5,
            y: 3.0,
            z: 4.0,
        };
        let angular = Vector3 {
            x: 1.0,
            y: 2.0,
            z: -0.05,
        };
        let result = f.apply(&linear, &angular);

        assert!(!result.was_stopped);
        assert_eq!(result.linear.x, -0.5);
        assert_eq!(result.linear.y, 3.0);
        assert_eq!(result.angular.z, -0.05);
    }
}
