//! ROS 2 process helpers for integration tests
//!
//! Provides wrappers for running `ros2 topic pub/echo` and other CLI commands
//! with `rmw_zenoh_cpp` transport, matching the nano-ros nros-tests patterns.

use crate::process::{kill_process_group, set_new_process_group};
use crate::{TestError, TestResult};
use std::process::{Child, Command, Stdio};
use std::time::Duration;

/// Default ROS 2 distro
pub const DEFAULT_ROS_DISTRO: &str = "humble";

/// Check if ROS 2 is available
pub fn is_ros2_available() -> bool {
    Command::new("bash")
        .args(["-c", "source /opt/ros/humble/setup.bash && ros2 --help"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if rmw_zenoh_cpp is available
pub fn is_rmw_zenoh_available() -> bool {
    Command::new("bash")
        .args([
            "-c",
            "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -q rmw_zenoh_cpp",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if Autoware message packages are available
pub fn is_autoware_msgs_available() -> bool {
    Command::new("bash")
        .args([
            "-c",
            "source /opt/ros/humble/setup.bash && \
             source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null && \
             ros2 pkg list | grep -q autoware_vehicle_msgs",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Require ROS 2 + rmw_zenoh + Autoware msgs for a test
pub fn require_ros2_autoware() -> bool {
    if !is_ros2_available() {
        eprintln!("Skipping test: ROS 2 not available");
        return false;
    }
    if !is_rmw_zenoh_available() {
        eprintln!("Skipping test: rmw_zenoh_cpp not available");
        return false;
    }
    if !is_autoware_msgs_available() {
        eprintln!("Skipping test: Autoware message packages not available");
        return false;
    }
    true
}

/// Get ROS 2 + Autoware environment setup command with custom locator
pub fn ros2_env_setup_with_locator(locator: &str) -> String {
    format!(
        "source /opt/ros/{}/setup.bash && \
         source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null && \
         export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
         export ZENOH_CONFIG_OVERRIDE='mode=\"client\";connect/endpoints=[\"{}\"]'",
        DEFAULT_ROS_DISTRO, locator
    )
}

/// Managed ROS 2 process with automatic cleanup.
pub struct Ros2Process {
    handle: Child,
    #[allow(dead_code)]
    name: String,
}

impl Ros2Process {
    /// Spawn a bash command in its own process group.
    fn spawn_bash(cmd: &str, name: impl Into<String>) -> TestResult<Self> {
        let name = name.into();
        let mut command = Command::new("bash");
        command
            .args(["-c", cmd])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(unix)]
        set_new_process_group(&mut command);
        let handle = command
            .spawn()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to start {name}: {e}")))?;
        Ok(Self { handle, name })
    }

    /// Start `ros2 topic echo` subscriber
    pub fn topic_echo(topic: &str, msg_type: &str, locator: &str) -> TestResult<Self> {
        let env_setup = ros2_env_setup_with_locator(locator);
        let cmd = format!(
            "{env_setup} && timeout 15 ros2 topic echo {topic} {msg_type} \
             --qos-reliability best_effort"
        );
        Self::spawn_bash(&cmd, format!("ros2 topic echo {topic}"))
    }

    /// Start `ros2 topic pub` publisher
    pub fn topic_pub(
        topic: &str,
        msg_type: &str,
        data: &str,
        rate: u32,
        locator: &str,
    ) -> TestResult<Self> {
        let env_setup = ros2_env_setup_with_locator(locator);
        let cmd = format!(
            "{env_setup} && timeout 15 ros2 topic pub -r {rate} {topic} {msg_type} \"{data}\" \
             --qos-reliability best_effort"
        );
        Self::spawn_bash(&cmd, format!("ros2 topic pub {topic}"))
    }

    /// Wait for output with timeout, capturing both stdout and stderr.
    pub fn wait_for_all_output(&mut self, timeout: Duration) -> TestResult<String> {
        use std::io::Read;
        #[cfg(unix)]
        use std::os::unix::io::AsRawFd;

        let start = std::time::Instant::now();
        let mut output = String::new();

        let mut stdout = self.handle.stdout.take();
        let mut stderr = self.handle.stderr.take();

        #[cfg(unix)]
        {
            if let Some(ref out) = stdout {
                let fd = out.as_raw_fd();
                unsafe {
                    let flags = libc::fcntl(fd, libc::F_GETFL);
                    libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
                }
            }
            if let Some(ref err) = stderr {
                let fd = err.as_raw_fd();
                unsafe {
                    let flags = libc::fcntl(fd, libc::F_GETFL);
                    libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
                }
            }
        }

        let mut buf = [0u8; 4096];

        loop {
            if start.elapsed() > timeout {
                kill_process_group(&mut self.handle);
                if output.is_empty() {
                    return Err(TestError::Timeout);
                }
                break;
            }

            match self.handle.try_wait() {
                Ok(Some(_)) => {
                    if let Some(ref mut out) = stdout {
                        let _ = out.read_to_string(&mut output);
                    }
                    if let Some(ref mut err) = stderr {
                        let _ = err.read_to_string(&mut output);
                    }
                    break;
                }
                Ok(None) => {
                    if let Some(ref mut out) = stdout {
                        if let Ok(n) = out.read(&mut buf)
                            && n > 0
                        {
                            output.push_str(&String::from_utf8_lossy(&buf[..n]));
                        }
                    }
                    if let Some(ref mut err) = stderr {
                        if let Ok(n) = err.read(&mut buf)
                            && n > 0
                        {
                            output.push_str(&String::from_utf8_lossy(&buf[..n]));
                        }
                    }
                    #[cfg(unix)]
                    {
                        let remaining = timeout.saturating_sub(start.elapsed());
                        let ms = remaining.as_millis().min(500) as i32;
                        let mut fds = Vec::new();
                        if let Some(ref out) = stdout {
                            fds.push(libc::pollfd {
                                fd: out.as_raw_fd(),
                                events: libc::POLLIN,
                                revents: 0,
                            });
                        }
                        if let Some(ref err) = stderr {
                            fds.push(libc::pollfd {
                                fd: err.as_raw_fd(),
                                events: libc::POLLIN,
                                revents: 0,
                            });
                        }
                        if !fds.is_empty() {
                            unsafe {
                                libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, ms);
                            }
                        }
                    }
                    #[cfg(not(unix))]
                    std::thread::sleep(Duration::from_millis(50));
                }
                Err(_) => break,
            }
        }

        Ok(output)
    }

    /// Kill the process group
    pub fn kill(&mut self) {
        kill_process_group(&mut self.handle);
    }
}

impl Drop for Ros2Process {
    fn drop(&mut self) {
        self.kill();
    }
}
