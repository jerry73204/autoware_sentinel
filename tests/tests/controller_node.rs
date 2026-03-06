//! Controller node integration tests (Phase 10.5c)
//!
//! Verifies that the sentinel's trajectory follower controller produces
//! control output when given trajectory, odometry, and steering inputs.
//!
//! ## Test architecture
//!
//! ```text
//! ┌────────────────┐     ┌─────────┐     ┌──────────────────────┐
//! │ Sentinel        │────▶│ zenohd  │◀────│ ROS 2 (rmw_zenoh_cpp)│
//! │ (controller +  │     │         │     │  pub: trajectory,    │
//! │  MRM chain)    │     │         │     │       odometry,      │
//! │                │     │         │     │       steering       │
//! │ publishes:     │     │         │     │  echo: control_cmd   │
//! │  control_cmd   │     │         │     │                      │
//! └────────────────┘     └─────────┘     └──────────────────────┘
//! ```

use sentinel_tests::count_pattern;
use sentinel_tests::fixtures::{
    Ros2Process, ZenohRouter, require_ros2_autoware, sentinel_binary, start_sentinel, zenohd_unique,
};

use rstest::rstest;
use std::path::PathBuf;
use std::time::Duration;

/// Build a YAML string for a Trajectory message with a straight-line path.
///
/// Creates a trajectory with `n` points along the X axis, each spaced 1m apart,
/// starting at `(start_x, 0, 0)` with the given target velocity.
fn straight_trajectory_yaml(start_x: f64, n: usize, velocity: f64) -> String {
    let mut points = String::new();
    for i in 0..n {
        let x = start_x + i as f64;
        // Quaternion (0,0,0,1) = yaw=0 (facing +X)
        points.push_str(&format!(
            "{{pose: {{position: {{x: {x}, y: 0.0, z: 0.0}}, \
              orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, \
              longitudinal_velocity_mps: {velocity}, lateral_velocity_mps: 0.0, \
              acceleration_mps2: 0.0, heading_rate_rps: 0.0, \
              front_wheel_angle_rad: 0.0, rear_wheel_angle_rad: 0.0, \
              time_from_start: {{sec: 0, nanosec: 0}}}}"
        ));
        if i < n - 1 {
            points.push_str(", ");
        }
    }
    format!("{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: map}}, points: [{points}]}}")
}

/// Odometry YAML: ego at given (x, y) with yaw=0, moving at `vx` m/s.
fn odometry_yaml(x: f64, y: f64, vx: f64) -> String {
    format!(
        "{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: map}}, \
          child_frame_id: base_link, \
          pose: {{pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, \
          orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, \
          covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}, \
          twist: {{twist: {{linear: {{x: {vx}, y: 0.0, z: 0.0}}, \
          angular: {{x: 0.0, y: 0.0, z: 0.0}}}}, \
          covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}}}"
    )
}

/// SteeringReport YAML.
fn steering_yaml(angle: f64) -> String {
    format!("{{stamp: {{sec: 0, nanosec: 0}}, steering_tire_angle: {angle}}}")
}

/// AccelWithCovarianceStamped YAML.
fn accel_yaml(ax: f64) -> String {
    format!(
        "{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: base_link}}, \
          accel: {{accel: {{linear: {{x: {ax}, y: 0.0, z: 0.0}}, \
          angular: {{x: 0.0, y: 0.0, z: 0.0}}}}, \
          covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}}}"
    )
}

// =============================================================================
// Controller integration tests
// =============================================================================

/// Verify sentinel produces control output when given trajectory + odometry + steering.
///
/// Publishes a straight-line trajectory at 5 m/s, places the ego on the path
/// at rest, and verifies that the control_cmd output contains non-trivial values
/// (the PID longitudinal controller should command positive acceleration to reach
/// the target velocity).
#[rstest]
fn test_controller_produces_output(zenohd_unique: ZenohRouter, sentinel_binary: PathBuf) {
    if !require_ros2_autoware() {
        return;
    }

    let locator = zenohd_unique.locator();

    // 1. Start sentinel
    eprintln!("Starting sentinel...");
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    // 2. Start echo on control output
    eprintln!("Starting ros2 topic echo for control_cmd...");
    let mut ros2_echo = Ros2Process::topic_echo(
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
        &locator,
    )
    .expect("Failed to start ros2 topic echo");

    std::thread::sleep(Duration::from_secs(2));

    // 3. Publish controller inputs: trajectory (20 points, 5 m/s), odometry (at origin, stopped),
    //    steering (0 rad), acceleration (0 m/s²)
    let traj_yaml = straight_trajectory_yaml(0.0, 20, 5.0);
    let odom_yaml = odometry_yaml(0.0, 0.0, 0.0);
    let steer_yaml = steering_yaml(0.0);
    let accel_yaml = accel_yaml(0.0);

    eprintln!("Publishing controller inputs...");
    let _pub_traj = Ros2Process::topic_pub(
        "/planning/scenario_planning/trajectory",
        "autoware_planning_msgs/msg/Trajectory",
        &traj_yaml,
        10,
        &locator,
    )
    .expect("Failed to publish trajectory");

    let _pub_odom = Ros2Process::topic_pub(
        "/localization/kinematic_state",
        "nav_msgs/msg/Odometry",
        &odom_yaml,
        10,
        &locator,
    )
    .expect("Failed to publish odometry");

    let _pub_steer = Ros2Process::topic_pub(
        "/vehicle/status/steering_status",
        "autoware_vehicle_msgs/msg/SteeringReport",
        &steer_yaml,
        10,
        &locator,
    )
    .expect("Failed to publish steering");

    let _pub_accel = Ros2Process::topic_pub(
        "/localization/acceleration",
        "geometry_msgs/msg/AccelWithCovarianceStamped",
        &accel_yaml,
        10,
        &locator,
    )
    .expect("Failed to publish acceleration");

    // 4. Wait for the controller to process inputs (needs a few timer cycles)
    std::thread::sleep(Duration::from_secs(6));

    let output = ros2_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    eprintln!(
        "Control output ({} bytes):\n{}",
        output.len(),
        &output[..output.len().min(1000)]
    );

    // 5. Verify we received control messages
    let msg_count = count_pattern(&output, "stamp:");
    eprintln!("Control messages received: {}", msg_count);
    assert!(msg_count > 0, "No Control messages received from sentinel");

    // The sentinel always publishes control_cmd at 30 Hz (even without controller).
    // With the controller active, the PID should produce non-zero acceleration
    // to reach the 5 m/s target from standstill.
    // Check that at least one message has a non-zero acceleration field.
    let has_nonzero_accel = output.lines().any(|line| {
        if let Some(pos) = line.find("acceleration:") {
            let rest = &line[pos + "acceleration:".len()..];
            let trimmed = rest.trim();
            // Parse the float value after "acceleration:"
            if let Ok(val) = trimmed.parse::<f64>() {
                val.abs() > 0.001
            } else {
                false
            }
        } else {
            false
        }
    });

    eprintln!("Has non-zero acceleration: {}", has_nonzero_accel);
    assert!(
        has_nonzero_accel,
        "Controller did not produce non-zero acceleration — \
         expected PID to accelerate toward 5 m/s target"
    );
}

/// Verify sentinel still publishes control when only partial inputs are provided.
///
/// Without all 3 required inputs (trajectory, odometry, steering), the controller
/// should NOT run, but the sentinel should still publish default/external control_cmd
/// from its 30 Hz timer.
#[rstest]
fn test_controller_graceful_without_all_inputs(
    zenohd_unique: ZenohRouter,
    sentinel_binary: PathBuf,
) {
    if !require_ros2_autoware() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start sentinel
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    // Only publish odometry — no trajectory or steering
    let _pub_odom = Ros2Process::topic_pub(
        "/localization/kinematic_state",
        "nav_msgs/msg/Odometry",
        &odometry_yaml(0.0, 0.0, 1.0),
        10,
        &locator,
    )
    .expect("Failed to publish odometry");

    std::thread::sleep(Duration::from_secs(3));

    // Echo control output
    let mut ros2_echo = Ros2Process::topic_echo(
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
        &locator,
    )
    .expect("Failed to start ros2 topic echo");

    std::thread::sleep(Duration::from_secs(5));

    let output = ros2_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    // Sentinel should still publish control_cmd (default/zeros from gate)
    let msg_count = count_pattern(&output, "stamp:");
    eprintln!(
        "Control messages without full controller inputs: {}",
        msg_count
    );
    assert!(
        msg_count > 0,
        "Sentinel stopped publishing control_cmd when controller has partial inputs"
    );
}

/// Verify controller reacts to lateral offset — ego offset from trajectory should
/// produce non-zero steering command.
#[rstest]
fn test_controller_lateral_correction(zenohd_unique: ZenohRouter, sentinel_binary: PathBuf) {
    if !require_ros2_autoware() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start sentinel
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    // Start echo
    let mut ros2_echo = Ros2Process::topic_echo(
        "/control/command/control_cmd",
        "autoware_control_msgs/msg/Control",
        &locator,
    )
    .expect("Failed to start ros2 topic echo");

    std::thread::sleep(Duration::from_secs(2));

    // Trajectory along X axis at y=0
    let traj_yaml = straight_trajectory_yaml(0.0, 30, 3.0);
    // Ego at y=1.0 (1m lateral offset), moving at 3 m/s along X
    let odom_yaml = odometry_yaml(5.0, 1.0, 3.0);
    let steer_yaml = steering_yaml(0.0);
    let accel_yaml = accel_yaml(0.0);

    let _pub_traj = Ros2Process::topic_pub(
        "/planning/scenario_planning/trajectory",
        "autoware_planning_msgs/msg/Trajectory",
        &traj_yaml,
        10,
        &locator,
    )
    .expect("Failed to publish trajectory");

    let _pub_odom = Ros2Process::topic_pub(
        "/localization/kinematic_state",
        "nav_msgs/msg/Odometry",
        &odom_yaml,
        10,
        &locator,
    )
    .expect("Failed to publish odometry");

    let _pub_steer = Ros2Process::topic_pub(
        "/vehicle/status/steering_status",
        "autoware_vehicle_msgs/msg/SteeringReport",
        &steer_yaml,
        10,
        &locator,
    )
    .expect("Failed to publish steering");

    let _pub_accel = Ros2Process::topic_pub(
        "/localization/acceleration",
        "geometry_msgs/msg/AccelWithCovarianceStamped",
        &accel_yaml,
        10,
        &locator,
    )
    .expect("Failed to publish acceleration");

    // Wait for controller to produce output
    std::thread::sleep(Duration::from_secs(6));

    let output = ros2_echo
        .wait_for_all_output(Duration::from_secs(3))
        .unwrap_or_default();

    eprintln!(
        "Lateral correction output ({} bytes):\n{}",
        output.len(),
        &output[..output.len().min(1000)]
    );

    let msg_count = count_pattern(&output, "stamp:");
    eprintln!("Control messages received: {}", msg_count);
    assert!(msg_count > 0, "No Control messages received from sentinel");

    // With 1m lateral offset, the MPC should command a non-zero steering angle
    let has_nonzero_steer = output.lines().any(|line| {
        if let Some(pos) = line.find("steering_tire_angle:") {
            let rest = &line[pos + "steering_tire_angle:".len()..];
            let trimmed = rest.trim();
            if let Ok(val) = trimmed.parse::<f64>() {
                val.abs() > 0.001
            } else {
                false
            }
        } else {
            false
        }
    });

    eprintln!("Has non-zero steering: {}", has_nonzero_steer);
    assert!(
        has_nonzero_steer,
        "Controller did not produce non-zero steering angle — \
         expected MPC to correct 1m lateral offset"
    );
}
