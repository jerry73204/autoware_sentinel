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

//! First-order low-pass filter.
//!
//! Port of `lowpass_filter.hpp` from Autoware.
//! `y[n] = gain * y[n-1] + (1 - gain) * x[n]`

/// First-order IIR low-pass filter.
///
/// `gain` controls smoothing: 0.0 = no filtering, 1.0 = infinite smoothing.
#[derive(Debug, Clone, Copy)]
pub struct LowpassFilter1d {
    value: f64,
    gain: f64,
}

impl LowpassFilter1d {
    /// Create a new filter with initial value and gain ∈ [0, 1].
    pub fn new(initial_value: f64, gain: f64) -> Self {
        Self {
            value: initial_value,
            gain,
        }
    }

    /// Apply the filter to a new input sample.
    pub fn filter(&mut self, input: f64) -> f64 {
        self.value = self.gain * self.value + (1.0 - self.gain) * input;
        self.value
    }

    /// Get the current filtered value.
    pub fn value(&self) -> f64 {
        self.value
    }

    /// Reset to a new value.
    pub fn reset(&mut self, value: f64) {
        self.value = value;
    }

    /// Update the filter gain.
    pub fn set_gain(&mut self, gain: f64) {
        self.gain = gain;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_gain_passthrough() {
        let mut f = LowpassFilter1d::new(0.0, 0.0);
        assert!((f.filter(5.0) - 5.0).abs() < 1e-10);
        assert!((f.filter(3.0) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_high_gain_smoothing() {
        let mut f = LowpassFilter1d::new(0.0, 0.99);
        let out = f.filter(100.0);
        // With gain=0.99: 0.99*0 + 0.01*100 = 1.0
        assert!((out - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_converges_to_constant_input() {
        let mut f = LowpassFilter1d::new(0.0, 0.9);
        for _ in 0..1000 {
            f.filter(10.0);
        }
        assert!((f.value() - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_reset() {
        let mut f = LowpassFilter1d::new(0.0, 0.9);
        f.filter(5.0);
        f.reset(0.0);
        assert!((f.value()).abs() < 1e-10);
    }
}
