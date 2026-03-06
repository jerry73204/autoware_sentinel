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

// =============================================================================
// Topic parity (Phase 8.1f + 8.2d)
// =============================================================================

/// All 30 topics that the sentinel must publish (6 original + 14 functional + 10 debug).
const SENTINEL_TOPICS: &[(&str, &str)] = &[
    // Original 6
    (
        "/system/fail_safe/mrm_state",
        "autoware_adapi_v1_msgs/msg/MrmState",
    ),
    (
        "/control/command/hazard_lights_cmd",
        "autoware_vehicle_msgs/msg/HazardLightsCommand",
    ),
    (
        "/control/command/gear_cmd",
        "autoware_vehicle_msgs/msg/GearCommand",
    ),
    (
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
    ),
    (
        "/control/command/turn_indicators_cmd",
        "autoware_vehicle_msgs/msg/TurnIndicatorsCommand",
    ),
    (
        "/api/operation_mode/state",
        "autoware_adapi_v1_msgs/msg/OperationModeState",
    ),
    // Phase 8.1 — 14 functional topics
    (
        "/system/mrm/emergency_stop/status",
        "tier4_system_msgs/msg/MrmBehaviorStatus",
    ),
    (
        "/system/mrm/comfortable_stop/status",
        "tier4_system_msgs/msg/MrmBehaviorStatus",
    ),
    (
        "/system/mrm/pull_over_manager/status",
        "tier4_system_msgs/msg/MrmBehaviorStatus",
    ),
    (
        "/system/emergency/gear_cmd",
        "autoware_vehicle_msgs/msg/GearCommand",
    ),
    (
        "/system/emergency/hazard_lights_cmd",
        "autoware_vehicle_msgs/msg/HazardLightsCommand",
    ),
    (
        "/system/emergency/turn_indicators_cmd",
        "autoware_vehicle_msgs/msg/TurnIndicatorsCommand",
    ),
    (
        "/control/command/emergency_cmd",
        "tier4_vehicle_msgs/msg/VehicleEmergencyStamped",
    ),
    ("/control/gate_mode_cmd", "tier4_control_msgs/msg/GateMode"),
    (
        "/control/shift_decider/gear_cmd",
        "autoware_vehicle_msgs/msg/GearCommand",
    ),
    (
        "/control/vehicle_cmd_gate/is_stopped",
        "tier4_control_msgs/msg/IsStopped",
    ),
    (
        "/control/vehicle_cmd_gate/operation_mode",
        "autoware_adapi_v1_msgs/msg/OperationModeState",
    ),
    (
        "/api/autoware/get/engage",
        "autoware_vehicle_msgs/msg/Engage",
    ),
    ("/autoware/engage", "autoware_vehicle_msgs/msg/Engage"),
    (
        "/api/autoware/get/emergency",
        "tier4_external_api_msgs/msg/Emergency",
    ),
    // Phase 8.2 — 10 debug/diagnostic topics
    (
        "/control/control_validator/debug/marker",
        "visualization_msgs/msg/MarkerArray",
    ),
    (
        "/control/control_validator/output/markers",
        "visualization_msgs/msg/MarkerArray",
    ),
    (
        "/control/control_validator/validation_status",
        "autoware_control_validator/msg/ControlValidatorStatus",
    ),
    (
        "/control/control_validator/virtual_wall",
        "visualization_msgs/msg/MarkerArray",
    ),
    (
        "/control/vehicle_cmd_gate/is_filter_activated",
        "autoware_vehicle_cmd_gate/msg/IsFilterActivated",
    ),
    (
        "/control/vehicle_cmd_gate/is_filter_activated/flag",
        "autoware_internal_debug_msgs/msg/BoolStamped",
    ),
    (
        "/control/vehicle_cmd_gate/is_filter_activated/marker",
        "visualization_msgs/msg/MarkerArray",
    ),
    (
        "/control/vehicle_cmd_gate/is_filter_activated/marker_raw",
        "visualization_msgs/msg/MarkerArray",
    ),
    (
        "/control/autoware_operation_mode_transition_manager/debug_info",
        "autoware_operation_mode_transition_manager/msg/OperationModeTransitionManagerDebug",
    ),
    (
        "/control/command/control_cmd/debug/published_time",
        "autoware_internal_msgs/msg/PublishedTime",
    ),
];

/// Verify sentinel publishes all 30 topics (Phase 8.1f + 8.2d).
///
/// Starts sentinel, then checks each topic in parallel using a bash script
/// that runs `ros2 topic echo --once` with a timeout for each topic.
#[rstest]
fn test_sentinel_topic_parity(zenohd_unique: ZenohRouter, sentinel_binary: PathBuf) {
    if !require_ros2_autoware() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start sentinel
    eprintln!("Starting sentinel...");
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    // Give sentinel time to establish all publishers
    std::thread::sleep(Duration::from_secs(3));

    let env_setup = sentinel_tests::ros2::ros2_env_setup_with_locator(&locator);

    // Check all topics in parallel: spawn one `ros2 topic echo --once` per topic,
    // collect results. Each gets 15s timeout.
    let mut script = format!(
        "#!/bin/bash\nset -eo pipefail\n{env_setup}\n\n\
         RESULT_DIR=$(mktemp -d)\n\
         PIDS=()\n\n"
    );

    for (i, (topic, msg_type)) in SENTINEL_TOPICS.iter().enumerate() {
        script.push_str(&format!(
            "( timeout 15 ros2 topic echo --once {topic} {msg_type} \
               --qos-reliability best_effort > /dev/null 2>&1 \
               && echo OK > \"$RESULT_DIR/{i}\" \
               || echo FAIL > \"$RESULT_DIR/{i}\" ) &\n\
             PIDS+=($!)\n"
        ));
    }

    script.push_str(
        "\n# Wait for all background jobs\n\
         for pid in \"${PIDS[@]}\"; do wait \"$pid\" 2>/dev/null; done\n\n\
         # Report results\n\
         PASS=0\nFAIL=0\nFAILED_TOPICS=\"\"\n",
    );

    for (i, (topic, _)) in SENTINEL_TOPICS.iter().enumerate() {
        script.push_str(&format!(
            "if [ \"$(cat \"$RESULT_DIR/{i}\" 2>/dev/null)\" = \"OK\" ]; then\n\
             \x20 PASS=$((PASS+1))\n\
             else\n\
             \x20 FAIL=$((FAIL+1))\n\
             \x20 FAILED_TOPICS=\"$FAILED_TOPICS  {topic}\\n\"\n\
             fi\n"
        ));
    }

    script.push_str(
        "echo \"TOPICS_PASS=$PASS\"\n\
         echo \"TOPICS_FAIL=$FAIL\"\n\
         if [ -n \"$FAILED_TOPICS\" ]; then\n\
         \x20 echo \"FAILED:\"\n\
         \x20 echo -e \"$FAILED_TOPICS\"\n\
         fi\n\
         rm -rf \"$RESULT_DIR\"\n\
         [ \"$FAIL\" -eq 0 ]\n",
    );

    eprintln!("Checking {} topics in parallel...", SENTINEL_TOPICS.len());

    let check_output = std::process::Command::new("bash")
        .args(["-c", &script])
        .output()
        .expect("Failed to spawn topic checker");

    let output = String::from_utf8_lossy(&check_output.stdout).to_string();

    eprintln!("{}", output);

    // Parse results
    let pass_count: usize = output
        .lines()
        .find(|l| l.starts_with("TOPICS_PASS="))
        .and_then(|l| l.strip_prefix("TOPICS_PASS="))
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);

    let fail_count: usize = output
        .lines()
        .find(|l| l.starts_with("TOPICS_FAIL="))
        .and_then(|l| l.strip_prefix("TOPICS_FAIL="))
        .and_then(|v| v.parse().ok())
        .unwrap_or(SENTINEL_TOPICS.len());

    eprintln!(
        "Topic parity: {}/{} passed, {} failed",
        pass_count,
        SENTINEL_TOPICS.len(),
        fail_count
    );

    assert_eq!(
        fail_count,
        0,
        "{} of {} sentinel topics were not published. See FAILED list above.",
        fail_count,
        SENTINEL_TOPICS.len()
    );
}

// =============================================================================
// Parameter services (Phase 10.6d)
// =============================================================================

/// Expected parameter names declared by the sentinel.
const SENTINEL_PARAMS: &[&str] = &[
    "stop_filter.vx_threshold",
    "stop_filter.wz_threshold",
    "velocity_converter.speed_scale_factor",
    "twist2accel.accel_lowpass_gain",
    "heartbeat.timeout_ms",
    "mrm_emergency_stop.target_acceleration",
    "mrm_comfortable_stop.min_acceleration",
    "mrm_handler.stopped_velocity_threshold",
    "shift_decider.park_on_goal",
    "vehicle_cmd_gate.vel_lim",
    "control_validator.acc_lpf_gain",
    "control_validator.vel_lpf_gain",
    "op_mode.stable_check_duration",
    "vehicle_info.wheel_base",
    "controller.ctrl_period",
    "pid.kp",
    "mpc.prediction_horizon",
];

/// Verify `ros2 param list /sentinel` returns all declared parameters.
#[rstest]
fn test_sentinel_param_list(zenohd_unique: ZenohRouter, sentinel_binary: PathBuf) {
    if !require_ros2_autoware() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start sentinel
    eprintln!("Starting sentinel...");
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    // Give sentinel time to register parameter services
    std::thread::sleep(Duration::from_secs(3));

    let env_setup = sentinel_tests::ros2::ros2_env_setup_with_locator(&locator);

    // Run `ros2 param list /sentinel`
    let output = std::process::Command::new("bash")
        .args([
            "-c",
            &format!("{env_setup}\ntimeout 15 ros2 param list /sentinel 2>&1"),
        ])
        .output()
        .expect("Failed to spawn ros2 param list");

    let stdout = String::from_utf8_lossy(&output.stdout).to_string();
    let stderr = String::from_utf8_lossy(&output.stderr).to_string();
    eprintln!("ros2 param list stdout:\n{stdout}");
    if !stderr.is_empty() {
        eprintln!("ros2 param list stderr:\n{stderr}");
    }

    // Verify all expected parameters are listed
    let mut missing = Vec::new();
    for param in SENTINEL_PARAMS {
        if !stdout.lines().any(|l| l.trim() == *param) {
            missing.push(*param);
        }
    }

    assert!(
        missing.is_empty(),
        "Missing parameters from `ros2 param list /sentinel`:\n  {}",
        missing.join("\n  ")
    );

    // Count total parameters (should be 56)
    let param_count = stdout.lines().filter(|l| !l.trim().is_empty()).count();
    eprintln!("Total parameters listed: {param_count}");
    assert!(
        param_count >= 56,
        "Expected at least 56 parameters, got {param_count}"
    );
}

/// Verify `ros2 param get /sentinel <name>` returns the correct default value.
#[rstest]
fn test_sentinel_param_get(zenohd_unique: ZenohRouter, sentinel_binary: PathBuf) {
    if !require_ros2_autoware() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start sentinel
    eprintln!("Starting sentinel...");
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    std::thread::sleep(Duration::from_secs(3));

    let env_setup = sentinel_tests::ros2::ros2_env_setup_with_locator(&locator);

    // Spot-check a few parameter values
    let checks: &[(&str, &str)] = &[
        ("stop_filter.vx_threshold", "0.1"),
        ("heartbeat.timeout_ms", "5000"),
        ("shift_decider.park_on_goal", "True"),
        ("vehicle_info.wheel_base", "2.79"),
        ("pid.kp", "1.0"),
    ];

    for (name, expected_substr) in checks {
        let output = std::process::Command::new("bash")
            .args([
                "-c",
                &format!("{env_setup}\ntimeout 10 ros2 param get /sentinel {name} 2>&1"),
            ])
            .output()
            .expect("Failed to spawn ros2 param get");

        let stdout = String::from_utf8_lossy(&output.stdout).to_string();
        eprintln!("ros2 param get /sentinel {name}: {}", stdout.trim());

        assert!(
            stdout.contains(expected_substr),
            "Parameter {name}: expected value containing '{expected_substr}', got: {stdout}"
        );
    }
}
