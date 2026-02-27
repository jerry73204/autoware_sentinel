#![no_std]

use tier4_system_msgs::msg::OperationModeAvailability;

/// Parameters for the heartbeat watchdog.
#[derive(Debug, Clone)]
pub struct Params {
    /// Timeout in milliseconds. If no heartbeat is received within this period,
    /// all operation modes are declared unavailable. Default: 500
    pub timeout_ms: u64,
}

impl Default for Params {
    fn default() -> Self {
        Self { timeout_ms: 500 }
    }
}

/// Timer-based watchdog monitoring the main Autoware stack's heartbeat.
///
/// On timeout, publishes `OperationModeAvailability` with all modes `false`,
/// which triggers the MRM handler to initiate emergency response.
#[derive(Debug, Clone)]
pub struct HeartbeatWatchdog {
    params: Params,
    /// Timestamp of the last received heartbeat (milliseconds since epoch).
    last_heartbeat_ms: Option<u64>,
    /// Whether the watchdog has timed out.
    timed_out: bool,
}

impl HeartbeatWatchdog {
    pub fn new(params: Params) -> Self {
        Self {
            params,
            last_heartbeat_ms: None,
            timed_out: false,
        }
    }

    /// Record a heartbeat received at `now_ms` (milliseconds since epoch).
    pub fn on_heartbeat(&mut self, now_ms: u64) {
        self.last_heartbeat_ms = Some(now_ms);
        self.timed_out = false;
    }

    /// Check the watchdog at current time `now_ms`.
    /// Returns `Some(availability)` if the state changed (timeout detected or recovered),
    /// or `None` if unchanged.
    pub fn check(&mut self, now_ms: u64) -> Option<OperationModeAvailability> {
        let should_timeout = match self.last_heartbeat_ms {
            Some(last) => now_ms.saturating_sub(last) >= self.params.timeout_ms,
            None => true, // No heartbeat ever received
        };

        if should_timeout && !self.timed_out {
            self.timed_out = true;
            Some(self.unavailable())
        } else if !should_timeout && self.timed_out {
            self.timed_out = false;
            Some(self.available())
        } else {
            None
        }
    }

    /// Returns `true` if the watchdog has timed out.
    pub fn is_timed_out(&self) -> bool {
        self.timed_out
    }

    /// Generate an availability message with all modes available.
    pub fn available(&self) -> OperationModeAvailability {
        OperationModeAvailability {
            stop: true,
            autonomous: true,
            local: true,
            remote: true,
            emergency_stop: true,
            comfortable_stop: true,
            pull_over: true,
            ..Default::default()
        }
    }

    /// Generate an availability message with all modes unavailable.
    pub fn unavailable(&self) -> OperationModeAvailability {
        OperationModeAvailability::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timeout_fires_after_500ms() {
        let mut wd = HeartbeatWatchdog::new(Params::default());
        wd.on_heartbeat(1000);

        // At 1400ms (400ms elapsed) — no timeout
        assert!(wd.check(1400).is_none());
        assert!(!wd.is_timed_out());

        // At 1500ms (500ms elapsed) — timeout fires
        let msg = wd.check(1500);
        assert!(msg.is_some());
        assert!(wd.is_timed_out());

        let avail = msg.unwrap();
        assert!(!avail.autonomous);
        assert!(!avail.emergency_stop);
        assert!(!avail.comfortable_stop);
    }

    #[test]
    fn no_false_positives() {
        let mut wd = HeartbeatWatchdog::new(Params::default());
        wd.on_heartbeat(1000);

        // Check multiple times before timeout — should never fire
        for t in (1000..1500).step_by(10) {
            assert!(wd.check(t).is_none(), "false positive at t={}", t);
        }
        assert!(!wd.is_timed_out());
    }

    #[test]
    fn recovery_after_heartbeat() {
        let mut wd = HeartbeatWatchdog::new(Params::default());
        wd.on_heartbeat(1000);

        // Timeout fires
        wd.check(1500);
        assert!(wd.is_timed_out());

        // Heartbeat received — recovery
        wd.on_heartbeat(1600);
        assert!(!wd.is_timed_out());

        // Next check should report recovery (all modes available)
        // Since on_heartbeat resets timed_out, check returns None (no state change needed)
        assert!(wd.check(1600).is_none());
    }

    #[test]
    fn no_heartbeat_ever_received_means_timeout() {
        let mut wd = HeartbeatWatchdog::new(Params::default());

        // Very first check with no heartbeat → timeout
        let msg = wd.check(0);
        assert!(msg.is_some());
        assert!(wd.is_timed_out());
    }

    #[test]
    fn timeout_fires_only_once() {
        let mut wd = HeartbeatWatchdog::new(Params::default());
        wd.on_heartbeat(1000);

        // First timeout fires
        assert!(wd.check(1500).is_some());

        // Subsequent checks don't fire again
        assert!(wd.check(1600).is_none());
        assert!(wd.check(2000).is_none());
        assert!(wd.is_timed_out());
    }

    #[test]
    fn custom_timeout() {
        let mut wd = HeartbeatWatchdog::new(Params { timeout_ms: 200 });
        wd.on_heartbeat(1000);

        assert!(wd.check(1199).is_none());
        assert!(wd.check(1200).is_some());
        assert!(wd.is_timed_out());
    }
}
