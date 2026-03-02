//! Transport smoke tests (Phase 7.2)
//!
//! Verifies bidirectional message flow between the sentinel Linux binary
//! and ROS 2 (`rmw_zenoh_cpp`) through a shared zenohd router.
//!
//! ## Test architecture
//!
//! ```text
//! ┌──────────────────┐     ┌─────────────┐     ┌──────────────────────┐
//! │ Sentinel (Linux) │────▶│   zenohd    │◀────│ ROS 2 (rmw_zenoh_cpp)│
//! │  zenoh-pico      │     │ :ephemeral  │     │  ros2 topic pub/echo │
//! │  (Zenoh client)  │     │ (router)    │     │  (Zenoh client)      │
//! └──────────────────┘     └─────────────┘     └──────────────────────┘
//! ```

use sentinel_tests::count_pattern;
use sentinel_tests::fixtures::{
    Ros2Process, ZenohRouter, require_ros2_autoware, sentinel_binary, start_sentinel, zenohd_unique,
};

use rstest::rstest;
use std::path::PathBuf;
use std::time::Duration;

// =============================================================================
// Detection Tests
// =============================================================================

#[test]
fn test_zenohd_available() {
    let available = sentinel_tests::process::is_zenohd_available();
    eprintln!("zenohd available: {}", available);
    assert!(
        available,
        "zenohd binary not found — build with `just build-zenohd` in nano-ros"
    );
}

#[test]
fn test_ros2_detection() {
    let available = sentinel_tests::ros2::is_ros2_available();
    eprintln!("ROS 2 available: {}", available);
}

#[test]
fn test_rmw_zenoh_detection() {
    let available = sentinel_tests::ros2::is_rmw_zenoh_available();
    eprintln!("rmw_zenoh_cpp available: {}", available);
}

#[test]
fn test_autoware_msgs_detection() {
    let available = sentinel_tests::ros2::is_autoware_msgs_available();
    eprintln!("Autoware msgs available: {}", available);
}

// =============================================================================
// Sentinel startup
// =============================================================================

/// Verify sentinel connects to zenohd and prints ready message.
#[rstest]
fn test_sentinel_starts(zenohd_unique: ZenohRouter, sentinel_binary: PathBuf) {
    let locator = zenohd_unique.locator();
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");
    // start_sentinel already waits for "Executor ready"
    // ManagedProcess drop kills sentinel + zenohd drop kills router
}

// =============================================================================
// Sentinel → ROS 2 (sentinel publishes, ROS 2 echoes)
// =============================================================================

/// Verify ROS 2 receives Control messages published by sentinel at 30 Hz.
#[rstest]
fn test_sentinel_to_ros2_control(zenohd_unique: ZenohRouter, sentinel_binary: PathBuf) {
    if !require_ros2_autoware() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start ROS 2 echo first (subscribes to sentinel's output topic)
    eprintln!("Starting ros2 topic echo /control/command/control_cmd ...");
    let mut ros2_echo = Ros2Process::topic_echo(
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
        &locator,
    )
    .expect("Failed to start ros2 topic echo");

    // Give ROS 2 time to subscribe
    std::thread::sleep(Duration::from_secs(3));

    // Start sentinel
    eprintln!("Starting sentinel...");
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    // Collect output for 5 seconds
    std::thread::sleep(Duration::from_secs(5));
    let output = ros2_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    eprintln!(
        "ROS 2 echo output ({} bytes):\n{}",
        output.len(),
        &output[..output.len().min(500)]
    );

    // Sentinel publishes Control at 30 Hz; expect at least 1 message in 5s
    let msg_count = count_pattern(&output, "stamp:");
    eprintln!("Control messages received by ROS 2: {}", msg_count);
    assert!(
        msg_count > 0,
        "ROS 2 did not receive any Control messages from sentinel"
    );
}

/// Verify ROS 2 receives OperationModeState from sentinel (engagement flow).
#[rstest]
fn test_sentinel_publishes_operation_mode(zenohd_unique: ZenohRouter, sentinel_binary: PathBuf) {
    if !require_ros2_autoware() {
        return;
    }

    let locator = zenohd_unique.locator();

    eprintln!("Starting ros2 topic echo /api/operation_mode/state ...");
    let mut ros2_echo = Ros2Process::topic_echo(
        "/api/operation_mode/state",
        "autoware_adapi_v1_msgs/msg/OperationModeState",
        &locator,
    )
    .expect("Failed to start ros2 topic echo");

    std::thread::sleep(Duration::from_secs(3));

    eprintln!("Starting sentinel...");
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    std::thread::sleep(Duration::from_secs(5));
    let output = ros2_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    eprintln!(
        "OperationModeState output ({} bytes):\n{}",
        output.len(),
        &output[..output.len().min(500)]
    );

    // Check for mode=2 (AUTONOMOUS) in the output
    let has_mode = output.contains("mode: 2");
    let has_autoware_control = output.contains("is_autoware_control_enabled: true");
    eprintln!(
        "mode=AUTONOMOUS: {}, autoware_control_enabled: {}",
        has_mode, has_autoware_control
    );
    assert!(
        has_mode,
        "OperationModeState not received or mode != AUTONOMOUS"
    );
}

// =============================================================================
// ROS 2 → Sentinel (ROS 2 publishes, sentinel receives)
// =============================================================================

/// Verify sentinel receives VelocityReport published by ROS 2.
///
/// We can't directly observe the sentinel receiving messages (it only logs
/// at debug level), but we can verify the sentinel doesn't crash when
/// receiving real ROS 2 messages and continues publishing output.
#[rstest]
fn test_ros2_to_sentinel_velocity(zenohd_unique: ZenohRouter, sentinel_binary: PathBuf) {
    if !require_ros2_autoware() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start sentinel first
    eprintln!("Starting sentinel...");
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    // Publish VelocityReport from ROS 2
    eprintln!("Publishing VelocityReport from ROS 2...");
    let _ros2_pub = Ros2Process::topic_pub(
        "/vehicle/status/velocity_status",
        "autoware_vehicle_msgs/msg/VelocityReport",
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, \
         longitudinal_velocity: 5.0, lateral_velocity: 0.0, heading_rate: 0.0}",
        10,
        &locator,
    )
    .expect("Failed to start ros2 topic pub");

    // Simultaneously echo sentinel's output to verify it's still publishing
    eprintln!("Starting ros2 topic echo for sentinel output...");
    let mut ros2_echo = Ros2Process::topic_echo(
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
        &locator,
    )
    .expect("Failed to start ros2 topic echo");

    // Let them communicate
    std::thread::sleep(Duration::from_secs(5));

    let output = ros2_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    // Sentinel should still be publishing Control messages
    let msg_count = count_pattern(&output, "stamp:");
    eprintln!(
        "Control messages after VelocityReport injection: {}",
        msg_count
    );
    assert!(
        msg_count > 0,
        "Sentinel stopped publishing after receiving VelocityReport"
    );
}

// =============================================================================
// Bidirectional round-trip
// =============================================================================

/// Full bidirectional test: ROS 2 publishes velocity, sentinel processes it
/// and publishes control, ROS 2 echoes the control output.
#[rstest]
fn test_bidirectional_round_trip(zenohd_unique: ZenohRouter, sentinel_binary: PathBuf) {
    if !require_ros2_autoware() {
        return;
    }

    let locator = zenohd_unique.locator();

    // 1. Start sentinel
    eprintln!("Starting sentinel...");
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    // 2. Start ROS 2 echo on sentinel output
    eprintln!("Starting ros2 topic echo...");
    let mut ros2_echo = Ros2Process::topic_echo(
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
        &locator,
    )
    .expect("Failed to start ros2 topic echo");

    std::thread::sleep(Duration::from_secs(2));

    // 3. Start ROS 2 velocity publisher
    eprintln!("Publishing VelocityReport...");
    let _ros2_pub = Ros2Process::topic_pub(
        "/vehicle/status/velocity_status",
        "autoware_vehicle_msgs/msg/VelocityReport",
        "{header: {stamp: {sec: 1, nanosec: 0}, frame_id: ''}, \
         longitudinal_velocity: 10.0, lateral_velocity: 0.0, heading_rate: 0.0}",
        10,
        &locator,
    )
    .expect("Failed to start ros2 topic pub");

    // 4. Also publish heartbeat to keep sentinel healthy
    eprintln!("Publishing heartbeat...");
    let _ros2_hb = Ros2Process::topic_pub(
        "/api/system/heartbeat",
        "autoware_adapi_v1_msgs/msg/Heartbeat",
        "{}",
        10,
        &locator,
    )
    .expect("Failed to start heartbeat publisher");

    // 5. Let them communicate for 5 seconds
    std::thread::sleep(Duration::from_secs(5));

    let output = ros2_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    let msg_count = count_pattern(&output, "stamp:");
    eprintln!("Round-trip Control messages: {}", msg_count);
    assert!(
        msg_count > 0,
        "No Control messages received in bidirectional round-trip test"
    );
}
