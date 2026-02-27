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

    /// Force the filter state to a specific value.
    pub fn set(&mut self, value: f64) {
        self.value = Some(value);
    }
}
