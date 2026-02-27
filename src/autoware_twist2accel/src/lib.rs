#![no_std]

use geometry_msgs::msg::{Accel, Twist, Vector3};

/// First-order lowpass filter.
#[derive(Debug, Clone)]
pub struct LowpassFilter1d {
    gain: f64,
    value: Option<f64>,
}

impl LowpassFilter1d {
    pub fn new(gain: f64) -> Self {
        Self { gain, value: None }
    }

    pub fn filter(&mut self, u: f64) -> f64 {
        let x = match self.value {
            Some(prev) => self.gain * prev + (1.0 - self.gain) * u,
            None => u,
        };
        self.value = Some(x);
        x
    }

    pub fn reset(&mut self) {
        self.value = None;
    }
}

/// Parameters for the twist-to-acceleration converter.
#[derive(Debug, Clone)]
pub struct Params {
    /// Lowpass filter gain (0.0–1.0). Higher = more smoothing. Default: 0.9
    pub accel_lowpass_gain: f64,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            accel_lowpass_gain: 0.9,
        }
    }
}

/// Output of a Twist2Accel update.
#[derive(Debug, Clone)]
pub struct Twist2AccelOutput {
    pub accel: Accel,
    pub covariance: [f64; 36],
}

/// Finite-difference acceleration estimator with lowpass filtering.
///
/// Computes acceleration by differentiating consecutive twist measurements,
/// then applies a first-order lowpass filter independently on each axis.
#[derive(Debug, Clone)]
pub struct Twist2Accel {
    prev_twist: Option<Twist>,
    lpf_alx: LowpassFilter1d,
    lpf_aly: LowpassFilter1d,
    lpf_alz: LowpassFilter1d,
    lpf_aax: LowpassFilter1d,
    lpf_aay: LowpassFilter1d,
    lpf_aaz: LowpassFilter1d,
}

impl Twist2Accel {
    pub fn new(params: Params) -> Self {
        let gain = params.accel_lowpass_gain;
        Self {
            prev_twist: None,
            lpf_alx: LowpassFilter1d::new(gain),
            lpf_aly: LowpassFilter1d::new(gain),
            lpf_alz: LowpassFilter1d::new(gain),
            lpf_aax: LowpassFilter1d::new(gain),
            lpf_aay: LowpassFilter1d::new(gain),
            lpf_aaz: LowpassFilter1d::new(gain),
        }
    }

    /// Estimate acceleration from consecutive twist measurements.
    ///
    /// Returns `None` on the first call (no previous twist available).
    /// `dt` is clamped to a minimum of 1.0e-3 seconds.
    pub fn update(&mut self, twist: &Twist, dt: f64) -> Option<Twist2AccelOutput> {
        let prev = self.prev_twist.replace(twist.clone())?;

        let dt = if dt < 1.0e-3 { 1.0e-3 } else { dt };

        // Finite-difference raw accelerations
        let raw_alx = (twist.linear.x - prev.linear.x) / dt;
        let raw_aly = (twist.linear.y - prev.linear.y) / dt;
        let raw_alz = (twist.linear.z - prev.linear.z) / dt;
        let raw_aax = (twist.angular.x - prev.angular.x) / dt;
        let raw_aay = (twist.angular.y - prev.angular.y) / dt;
        let raw_aaz = (twist.angular.z - prev.angular.z) / dt;

        let accel = Accel {
            linear: Vector3 {
                x: self.lpf_alx.filter(raw_alx),
                y: self.lpf_aly.filter(raw_aly),
                z: self.lpf_alz.filter(raw_alz),
            },
            angular: Vector3 {
                x: self.lpf_aax.filter(raw_aax),
                y: self.lpf_aay.filter(raw_aay),
                z: self.lpf_aaz.filter(raw_aaz),
            },
        };

        // Covariance: diagonal [1.0, 1.0, 1.0, 0.05, 0.05, 0.05]
        let mut covariance = [0.0f64; 36];
        covariance[0] = 1.0;
        covariance[7] = 1.0;
        covariance[14] = 1.0;
        covariance[21] = 0.05;
        covariance[28] = 0.05;
        covariance[35] = 0.05;

        Some(Twist2AccelOutput { accel, covariance })
    }

    /// Reset all state. The next `update` call will return `None`.
    pub fn reset(&mut self) {
        self.prev_twist = None;
        self.lpf_alx.reset();
        self.lpf_aly.reset();
        self.lpf_alz.reset();
        self.lpf_aax.reset();
        self.lpf_aay.reset();
        self.lpf_aaz.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1.0 / 30.0;

    fn make_twist(lx: f64, ly: f64, lz: f64, ax: f64, ay: f64, az: f64) -> Twist {
        Twist {
            linear: Vector3 {
                x: lx,
                y: ly,
                z: lz,
            },
            angular: Vector3 {
                x: ax,
                y: ay,
                z: az,
            },
        }
    }

    #[test]
    fn velocity_ramp_converges() {
        let mut t2a = Twist2Accel::new(Params::default());
        let steps = 150; // 5s at 30 Hz
        let mut last_output = None;
        for i in 0..=steps {
            let vx = 1.0 * (i as f64) * DT; // v = a*t with a = 1.0 m/s²
            let twist = make_twist(vx, 0.0, 0.0, 0.0, 0.0, 0.0);
            last_output = t2a.update(&twist, DT);
        }
        let output = last_output.unwrap();
        assert!(
            (output.accel.linear.x - 1.0).abs() < 0.15,
            "expected ~1.0 m/s², got {}",
            output.accel.linear.x
        );
    }

    #[test]
    fn lowpass_attenuates_noise() {
        let mut t2a = Twist2Accel::new(Params::default());
        let delta = 0.1;
        let mut filtered_values = [0.0f64; 100];
        let mut idx = 0;

        for i in 0..102 {
            let noise = if i % 2 == 0 { delta } else { -delta };
            let twist = make_twist(10.0 + noise, 0.0, 0.0, 0.0, 0.0, 0.0);
            if let Some(output) = t2a.update(&twist, DT) {
                if idx < 100 {
                    filtered_values[idx] = output.accel.linear.x;
                    idx += 1;
                }
            }
        }

        // Raw acceleration would alternate at ±2*delta/DT = ±6.0, variance = 36.0
        let raw_variance = (2.0 * delta / DT) * (2.0 * delta / DT);
        let filtered_variance: f64 =
            filtered_values[..idx].iter().map(|x| x * x).sum::<f64>() / idx as f64;

        assert!(
            filtered_variance < raw_variance,
            "filtered variance ({:.4}) should be less than raw variance ({:.4})",
            filtered_variance,
            raw_variance
        );
    }

    #[test]
    fn first_call_returns_none() {
        let mut t2a = Twist2Accel::new(Params::default());
        let twist = make_twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(t2a.update(&twist, DT).is_none());
    }

    #[test]
    fn constant_velocity_zero_accel() {
        let mut t2a = Twist2Accel::new(Params::default());
        let twist = make_twist(5.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for _ in 0..100 {
            t2a.update(&twist, DT);
        }
        let output = t2a.update(&twist, DT).unwrap();
        assert!(
            output.accel.linear.x.abs() < 1e-6,
            "constant velocity should yield ~0 accel, got {}",
            output.accel.linear.x
        );
    }

    #[test]
    fn covariance_diagonal() {
        let mut t2a = Twist2Accel::new(Params::default());
        let t1 = make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let t2 = make_twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        t2a.update(&t1, DT);
        let output = t2a.update(&t2, DT).unwrap();

        assert_eq!(output.covariance[0], 1.0);
        assert_eq!(output.covariance[7], 1.0);
        assert_eq!(output.covariance[14], 1.0);
        assert_eq!(output.covariance[21], 0.05);
        assert_eq!(output.covariance[28], 0.05);
        assert_eq!(output.covariance[35], 0.05);
        assert_eq!(output.covariance[1], 0.0);
        assert_eq!(output.covariance[6], 0.0);
    }

    #[test]
    fn min_dt_clamped() {
        let mut t2a = Twist2Accel::new(Params::default());
        let t1 = make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let t2 = make_twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        t2a.update(&t1, 0.0);
        let output = t2a.update(&t2, 1e-9).unwrap();
        assert!(output.accel.linear.x.is_finite(), "accel should be finite");
        // With dt clamped to 1e-3: raw = 1.0 / 0.001 = 1000.0
        assert!(
            (output.accel.linear.x - 1000.0).abs() < 0.01,
            "expected ~1000.0, got {}",
            output.accel.linear.x
        );
    }

    #[test]
    fn reset_clears_state() {
        let mut t2a = Twist2Accel::new(Params::default());
        let t1 = make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let t2 = make_twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        t2a.update(&t1, DT);
        assert!(t2a.update(&t2, DT).is_some());

        t2a.reset();
        let t3 = make_twist(2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(t2a.update(&t3, DT).is_none());
    }

    #[test]
    fn axes_independent() {
        let mut t2a = Twist2Accel::new(Params::default());
        let t1 = make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let t2 = make_twist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        t2a.update(&t1, DT);
        let output = t2a.update(&t2, DT).unwrap();

        assert!(
            output.accel.linear.x.abs() > 1.0,
            "linear.x should be non-zero"
        );
        assert!(
            output.accel.linear.y.abs() < 1e-10,
            "linear.y should be zero"
        );
        assert!(
            output.accel.linear.z.abs() < 1e-10,
            "linear.z should be zero"
        );
        assert!(
            output.accel.angular.x.abs() < 1e-10,
            "angular.x should be zero"
        );
        assert!(
            output.accel.angular.y.abs() < 1e-10,
            "angular.y should be zero"
        );
        assert!(
            output.accel.angular.z.abs() < 1e-10,
            "angular.z should be zero"
        );
    }
}
