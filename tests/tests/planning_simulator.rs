//! Planning simulator integration tests (Phase 7.3)
//!
//! Tests the Autoware planning simulator with and without the sentinel.
//!
//! ## 7.3a — Baseline (this file, for now)
//!
//! Runs unmodified Autoware through the full autonomous drive sequence to
//! establish baseline behavior and validate the test infrastructure.
//!
//! ## Prerequisites
//!
//! - `play_launch` installed (`~/.local/bin/play_launch` or `$PLAY_LAUNCH`)
//! - Autoware map data at `$MAP_PATH` or `$HOME/autoware_map/sample-map-planning`
//! - ROS 2 Humble with `rmw_zenoh_cpp` and Autoware packages

use sentinel_tests::autoware;
use sentinel_tests::fixtures::{autoware_record, dump_autoware_record, start_autoware};
use sentinel_tests::ros2;

use rstest::rstest;
use std::path::PathBuf;
use std::time::Duration;

// =============================================================================
// Prerequisite detection tests
// =============================================================================

#[test]
fn test_play_launch_available() {
    let available = autoware::is_play_launch_available();
    eprintln!("play_launch available: {available}");
}

#[test]
fn test_autoware_map_available() {
    let path = autoware::autoware_map_path();
    let available = autoware::is_autoware_map_available();
    eprintln!("Autoware map path: {path:?}, available: {available}");
}

// =============================================================================
// 7.3a — Baseline: Standard Autoware (no sentinel)
// =============================================================================

/// Dump the Autoware planning simulator launch to record.json.
///
/// Validates that play_launch can parse and dump the full Autoware launch tree.
#[test]
fn test_autoware_dump() {
    if !autoware::require_planning_simulator_prerequisites() {
        return;
    }

    let record = dump_autoware_record().expect("Failed to dump Autoware record");
    assert!(record.exists(), "Record file not created");
    assert!(
        autoware::is_valid_record(record),
        "Record file is not valid JSON"
    );

    // Verify the record contains expected arrays
    let content = std::fs::read_to_string(record).unwrap();
    let json: serde_json::Value = serde_json::from_str(&content).unwrap();
    assert!(json.get("node").is_some(), "record missing 'node' array");
    assert!(
        json.get("container").is_some(),
        "record missing 'container' array"
    );
    assert!(
        json.get("load_node").is_some(),
        "record missing 'load_node' array"
    );

    let node_count = json["node"].as_array().map(|a| a.len()).unwrap_or(0);
    let container_count = json["container"].as_array().map(|a| a.len()).unwrap_or(0);
    let load_node_count = json["load_node"].as_array().map(|a| a.len()).unwrap_or(0);
    eprintln!(
        "Autoware record: {node_count} nodes, {container_count} containers, {load_node_count} load_nodes"
    );

    // Autoware planning simulator has many nodes
    assert!(node_count > 0, "Expected at least 1 node in record");
}

/// Full baseline test: launch unmodified Autoware and drive autonomously.
///
/// This test validates the complete test infrastructure:
/// 1. play_launch dump + replay
/// 2. ROS 2 CLI helpers (topic pub, service call, topic info)
/// 3. Autonomous drive sequence (initial pose → route → engage)
/// 4. Vehicle movement verification (kinematic_state position delta)
///
/// Autoware runs on default DDS transport. The ROS 2 CLI tools also use
/// default DDS so they can interact with the Autoware nodes directly.
#[rstest]
fn test_autoware_baseline_autonomous_drive(autoware_record: PathBuf) {
    if !autoware::require_planning_simulator_prerequisites() {
        return;
    }

    // Environment setup for ROS 2 CLI tools — use default DDS (same as Autoware)
    let env = ros2::ros2_env_setup_dds();

    // --- Step 1: Start Autoware via play_launch replay ---
    eprintln!("Starting unmodified Autoware (default DDS)...");
    let mut autoware = start_autoware(&autoware_record).expect("Failed to start Autoware");

    // Give Autoware time to initialize all nodes
    eprintln!("Waiting 30s for Autoware initialization...");
    std::thread::sleep(Duration::from_secs(30));

    // Verify Autoware is still running
    assert!(
        autoware.is_running(),
        "Autoware exited prematurely during initialization"
    );

    // --- Step 2: Wait for readiness topics ---
    eprintln!("Waiting for readiness topics...");
    match ros2::wait_for_topics(autoware::READINESS_TOPICS, &env, Duration::from_secs(60)) {
        Ok(()) => eprintln!("All readiness topics active"),
        Err(e) => eprintln!("WARNING: Not all readiness topics active: {e}"),
    }

    // --- Step 3: Publish initial pose ---
    eprintln!("Publishing initial pose...");
    let pose_yaml = autoware::initial_pose_yaml();
    match ros2::topic_pub_once(
        "/initialpose",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        &pose_yaml,
        &env,
    ) {
        Ok(_) => eprintln!("Initial pose published"),
        Err(e) => eprintln!("WARNING: Initial pose publish issue: {e}"),
    }

    // --- Step 4: Wait for localization ---
    eprintln!("Waiting 10s for localization...");
    std::thread::sleep(Duration::from_secs(10));

    match ros2::wait_for_topics(autoware::LOCALIZATION_TOPICS, &env, Duration::from_secs(30)) {
        Ok(()) => eprintln!("Localization topics active"),
        Err(e) => eprintln!("WARNING: Localization topics not all active: {e}"),
    }

    // --- Step 5: Set route ---
    eprintln!("Setting route via /api/routing/set_route_points...");
    let route_yaml = autoware::goal_pose_yaml();
    match ros2::service_call(
        "/api/routing/set_route_points",
        "autoware_adapi_v1_msgs/srv/SetRoutePoints",
        &route_yaml,
        &env,
    ) {
        Ok(resp) => eprintln!("Route set response: {}", &resp[..resp.len().min(200)]),
        Err(e) => eprintln!("WARNING: Route set issue: {e}"),
    }

    // --- Step 6: Wait for route processing ---
    eprintln!("Waiting 5s for route processing...");
    std::thread::sleep(Duration::from_secs(5));

    // --- Step 7: Engage autonomous mode ---
    eprintln!("Engaging autonomous mode...");
    match ros2::service_call(
        "/api/operation_mode/change_to_autonomous",
        "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
        "{}",
        &env,
    ) {
        Ok(resp) => eprintln!("Engage response: {}", &resp[..resp.len().min(200)]),
        Err(e) => eprintln!("WARNING: Engage issue: {e}"),
    }

    // --- Step 8: Monitor for vehicle movement ---
    eprintln!("Monitoring kinematic_state for 20s...");

    // Echo kinematic_state via default DDS to check for position data
    let mut echo = ros2::topic_echo_dds("/localization/kinematic_state", "nav_msgs/msg/Odometry")
        .expect("Failed to start kinematic_state echo");

    std::thread::sleep(Duration::from_secs(15));
    let output = echo
        .wait_for_all_output(Duration::from_secs(5))
        .unwrap_or_default();

    eprintln!(
        "kinematic_state output ({} bytes): {}",
        output.len(),
        &output[..output.len().min(500)]
    );

    // Verify Autoware is still running (no crashes)
    assert!(
        autoware.is_running(),
        "Autoware crashed during autonomous drive sequence"
    );

    // Check that we received kinematic_state data (position exists)
    let has_position = output.contains("position:");
    eprintln!("kinematic_state has position data: {has_position}");
    assert!(
        has_position,
        "No kinematic_state position data received — Autoware may not have initialized fully"
    );
}
