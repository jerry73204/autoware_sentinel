//! ZenohRouter fixture for managing zenohd process
//!
//! Provides automatic startup and cleanup of the zenoh router daemon.
//! Uses ephemeral ports for parallel-safe test execution.

use crate::process::{kill_process_group, zenohd_binary_path};
use crate::{TestError, TestResult, wait_for_port};
use std::net::TcpStream;
use std::process::Child;
use std::time::Duration;

/// Allocate an ephemeral port from the OS.
fn allocate_ephemeral_port() -> std::io::Result<u16> {
    let listener = std::net::TcpListener::bind("127.0.0.1:0")?;
    let port = listener.local_addr()?.port();
    drop(listener);
    Ok(port)
}

/// Kill any process listening on the given TCP port.
///
/// Orphaned zenohd processes can survive across test runs when nextest
/// SIGKILL's a test process (preventing Drop from running).
fn kill_listeners_on_port(port: u16) {
    if TcpStream::connect(format!("127.0.0.1:{}", port)).is_err() {
        return;
    }
    eprintln!(
        "WARNING: port {} already in use — killing orphaned process",
        port
    );
    let _ = std::process::Command::new("fuser")
        .args(["-k", &format!("{}/tcp", port)])
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status();
    let start = std::time::Instant::now();
    while start.elapsed() < Duration::from_secs(5) {
        if TcpStream::connect(format!("127.0.0.1:{}", port)).is_err() {
            return;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    eprintln!("WARNING: port {} still in use after kill attempt", port);
}

/// Managed zenohd router process.
///
/// Automatically starts zenohd on creation and kills it on drop.
/// Uses OS-assigned ephemeral ports for parallel test execution.
pub struct ZenohRouter {
    handle: Child,
    port: u16,
}

impl ZenohRouter {
    /// Start a new zenohd router on the specified port.
    pub fn start(port: u16) -> TestResult<Self> {
        kill_listeners_on_port(port);

        let locator = format!("tcp/0.0.0.0:{}", port);

        let mut cmd = std::process::Command::new(zenohd_binary_path());
        cmd.args(["--listen", &locator, "--no-multicast-scouting"])
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null());
        #[cfg(unix)]
        crate::process::set_new_process_group(&mut cmd);
        let handle = cmd.spawn()?;

        if !wait_for_port(port, Duration::from_secs(10)) {
            return Err(TestError::Timeout);
        }

        Ok(Self { handle, port })
    }

    /// Start a router on an OS-assigned ephemeral port (parallel-safe).
    pub fn start_unique() -> TestResult<Self> {
        let port = allocate_ephemeral_port()
            .map_err(|e| TestError::ProcessFailed(format!("Failed to allocate port: {}", e)))?;
        Self::start(port)
    }

    /// Get the locator string for connecting to this router.
    pub fn locator(&self) -> String {
        format!("tcp/127.0.0.1:{}", self.port)
    }

    /// Get the port number.
    pub fn port(&self) -> u16 {
        self.port
    }
}

impl Drop for ZenohRouter {
    fn drop(&mut self) {
        kill_process_group(&mut self.handle);
    }
}

/// rstest fixture for zenohd on an OS-assigned ephemeral port.
#[rstest::fixture]
pub fn zenohd_unique() -> ZenohRouter {
    ZenohRouter::start_unique().expect("Failed to start zenohd")
}
