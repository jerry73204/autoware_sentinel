//! Per-source heartbeat monitor for the vehicle command gate.
//!
//! Simple timeout-based liveness check. Each command source should call
//! `on_heartbeat()` when a message arrives; the gate checks `is_timed_out()`
//! on each update cycle.

/// Heartbeat monitor for a single command source.
#[derive(Debug, Clone)]
pub struct HeartbeatMonitor {
    timeout_ms: u64,
    last_received_ms: Option<u64>,
}

impl HeartbeatMonitor {
    /// Create a new monitor. `timeout_ms = 0` disables the monitor (never times out).
    pub fn new(timeout_ms: u64) -> Self {
        Self {
            timeout_ms,
            last_received_ms: None,
        }
    }

    /// Record that a heartbeat was received at `now_ms`.
    pub fn on_heartbeat(&mut self, now_ms: u64) {
        self.last_received_ms = Some(now_ms);
    }

    /// Check whether the heartbeat has timed out at current time `now_ms`.
    ///
    /// Returns `true` if:
    /// - timeout is enabled (timeout_ms > 0), AND
    /// - either no heartbeat was ever received, OR the elapsed time exceeds the timeout.
    pub fn is_timed_out(&self, now_ms: u64) -> bool {
        if self.timeout_ms == 0 {
            return false; // Disabled
        }

        match self.last_received_ms {
            Some(last) => now_ms.saturating_sub(last) >= self.timeout_ms,
            None => true, // No heartbeat ever received
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timeout_fires_after_threshold() {
        let mut hb = HeartbeatMonitor::new(500);
        hb.on_heartbeat(1000);

        assert!(!hb.is_timed_out(1400)); // 400ms < 500ms
        assert!(hb.is_timed_out(1500)); // 500ms >= 500ms
        assert!(hb.is_timed_out(2000)); // 1000ms >= 500ms
    }

    #[test]
    fn no_false_positives() {
        let mut hb = HeartbeatMonitor::new(500);
        hb.on_heartbeat(1000);

        for t in (1000..1500).step_by(10) {
            assert!(!hb.is_timed_out(t), "false positive at t={}", t);
        }
    }

    #[test]
    fn recovery_after_new_heartbeat() {
        let mut hb = HeartbeatMonitor::new(500);
        hb.on_heartbeat(1000);
        assert!(hb.is_timed_out(1500));

        hb.on_heartbeat(1600);
        assert!(!hb.is_timed_out(1600));
        assert!(!hb.is_timed_out(2000));
        assert!(hb.is_timed_out(2100));
    }

    #[test]
    fn disabled_when_timeout_zero() {
        let hb = HeartbeatMonitor::new(0);
        // Never times out, even with no heartbeat
        assert!(!hb.is_timed_out(0));
        assert!(!hb.is_timed_out(999_999));
    }

    #[test]
    fn no_heartbeat_means_timed_out() {
        let hb = HeartbeatMonitor::new(500);
        assert!(hb.is_timed_out(0));
        assert!(hb.is_timed_out(1000));
    }
}
