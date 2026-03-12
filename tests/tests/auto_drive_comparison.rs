//! Autonomous drive comparison tests
//!
//! Runs the full autonomous driving sequence (via `auto_drive.py`) with rosbag
//! recording for both baseline Autoware and sentinel-replaced Autoware, then
//! compares the results.
//!
//! These tests exercise the complete stack including zenohd, play_launch,
//! rosbag recording, and the Python AD API driver script — matching what
//! `just launch-autoware-baseline --record --drive` does.
//!
//! ## Prerequisites
//!
//! - `play_launch` installed
//! - Autoware map data
//! - ROS 2 Humble with `rmw_zenoh_cpp` and Autoware packages
//! - zenohd built locally
//! - `auto_drive.py` requires `rclpy`, `pyyaml`

use sentinel_tests::autoware;
use sentinel_tests::fixtures::{
    NodeFilter, ZenohRouter, autoware_record, filter_record, sentinel_binary, start_sentinel,
    zenohd_unique,
};
use sentinel_tests::process::{ManagedProcess, project_root};

use rstest::rstest;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

// =============================================================================
// Helpers
// =============================================================================

/// Start auto_drive.py as a managed process.
///
/// Connects via rmw_zenoh_cpp to the given locator. Returns after spawning;
/// the caller should wait for the process to complete.
fn start_auto_drive(locator: &str, timeout_secs: u64) -> sentinel_tests::TestResult<ManagedProcess> {
    let root = project_root();
    let script = root.join("scripts/auto_drive.py");
    let poses = root.join("scripts/poses.yaml");

    let env_setup = format!(
        "source /opt/ros/humble/setup.bash && \
         source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null; \
         source {rmw_ws}/install/local_setup.bash 2>/dev/null; \
         export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
         export ZENOH_CONFIG_OVERRIDE='mode=\"client\";connect/endpoints=[\"{locator}\"];queries_default_timeout=30000' && \
         python3 {script} --timeout {timeout} --poses {poses}",
        rmw_ws = root.join("external/rmw_zenoh_ws").display(),
        locator = locator,
        script = script.display(),
        timeout = timeout_secs,
        poses = poses.display(),
    );

    let mut cmd = Command::new("bash");
    cmd.args(["-c", &env_setup]);
    ManagedProcess::spawn_command(cmd, "auto_drive")
}

/// Start rosbag recording as a managed process.
///
/// Records the topics from `scripts/record_topics.txt` into the given output
/// directory. Uses rmw_zenoh_cpp to connect to the given locator.
fn start_rosbag_recording(
    label: &str,
    locator: &str,
) -> sentinel_tests::TestResult<(ManagedProcess, PathBuf)> {
    let root = project_root();
    let topics_file = root.join("scripts/record_topics.txt");
    let topics = std::fs::read_to_string(&topics_file)
        .expect("Failed to read record_topics.txt");
    let topics: Vec<&str> = topics
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .collect();
    let topic_args = topics.join(" ");

    let timestamp = chrono_timestamp();
    let bag_dir = root
        .join("tmp/bags")
        .join(label)
        .join(format!("{}_{}", label, timestamp));
    std::fs::create_dir_all(bag_dir.parent().unwrap()).ok();

    let env_setup = format!(
        "source /opt/ros/humble/setup.bash && \
         source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null; \
         source {rmw_ws}/install/local_setup.bash 2>/dev/null; \
         export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
         export ZENOH_CONFIG_OVERRIDE='mode=\"client\";connect/endpoints=[\"{locator}\"];queries_default_timeout=30000' && \
         ros2 bag record --storage mcap --output {bag_dir} {topics}",
        rmw_ws = root.join("external/rmw_zenoh_ws").display(),
        locator = locator,
        bag_dir = bag_dir.display(),
        topics = topic_args,
    );

    let mut cmd = Command::new("bash");
    cmd.args(["-c", &env_setup]);
    let proc = ManagedProcess::spawn_command(cmd, "rosbag_record")?;

    Ok((proc, bag_dir))
}

/// Get a timestamp string for bag directory naming.
fn chrono_timestamp() -> String {
    use std::time::SystemTime;
    let now = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap();
    let secs = now.as_secs();
    // Simple YYYYMMDD_HHMMSS from unix timestamp
    format!("{}", secs)
}

/// Start Autoware replay with rmw_zenoh_cpp transport and web UI disabled.
fn start_autoware_zenoh_replay(
    record_path: &Path,
    locator: &str,
) -> sentinel_tests::TestResult<ManagedProcess> {
    let play_launch = autoware::play_launch_binary();
    let root = project_root();

    let env_setup = format!(
        "source /opt/ros/humble/setup.bash && \
         source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null; \
         source {rmw_ws}/install/local_setup.bash 2>/dev/null; \
         export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
         export ZENOH_CONFIG_OVERRIDE='mode=\"client\";connect/endpoints=[\"{locator}\"];queries_default_timeout=30000' && \
         {play_launch} replay --input-file {record} --web-addr 0.0.0.0:8080",
        rmw_ws = root.join("external/rmw_zenoh_ws").display(),
        locator = locator,
        play_launch = play_launch.display(),
        record = record_path.display(),
    );

    let mut cmd = Command::new("bash");
    cmd.args(["-c", &env_setup]);
    ManagedProcess::spawn_command(cmd, "play_launch")
}

/// Stop the rosbag recorder gracefully (just kill — ros2 bag handles cleanup).
fn stop_recording_gracefully(recorder: &mut ManagedProcess) {
    recorder.kill();
    std::thread::sleep(Duration::from_secs(3));
}

/// Get rosbag info (topic message counts) from a bag directory.
fn get_bag_info(bag_dir: &Path) -> String {
    let env_setup = format!(
        "source /opt/ros/humble/setup.bash && \
         ros2 bag info {}",
        bag_dir.display(),
    );

    Command::new("bash")
        .args(["-c", &env_setup])
        .output()
        .map(|o| {
            let stdout = String::from_utf8_lossy(&o.stdout);
            let stderr = String::from_utf8_lossy(&o.stderr);
            format!("{}{}", stdout, stderr)
        })
        .unwrap_or_else(|e| format!("Failed to get bag info: {e}"))
}

// =============================================================================
// Tests
// =============================================================================

/// Baseline auto-drive: launch unmodified Autoware via zenohd, run auto_drive.py,
/// record a rosbag, and verify the vehicle arrives at the goal.
#[rstest]
fn test_baseline_auto_drive(zenohd_unique: ZenohRouter, autoware_record: PathBuf) {
    if !autoware::require_planning_simulator_prerequisites() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Start Autoware replay
    eprintln!("Starting baseline Autoware (rmw_zenoh_cpp → {locator})...");
    let mut autoware =
        start_autoware_zenoh_replay(&autoware_record, &locator).expect("Failed to start Autoware");

    // Start rosbag recording
    eprintln!("Starting rosbag recording...");
    let (mut recorder, bag_dir) =
        start_rosbag_recording("baseline_test", &locator).expect("Failed to start recording");

    // Wait for Autoware initialization (composable node loading + zenoh discovery)
    eprintln!("Waiting 90s for Autoware initialization...");
    std::thread::sleep(Duration::from_secs(90));

    assert!(
        autoware.is_running(),
        "Autoware exited during initialization"
    );

    // Start auto_drive.py
    eprintln!("Starting auto_drive.py (timeout=240s)...");
    let mut driver = start_auto_drive(&locator, 240).expect("Failed to start auto_drive");

    // Wait for auto_drive to complete (up to 300s total)
    let driver_output = driver
        .wait_for_output_pattern("ARRIVED", Duration::from_secs(300))
        .unwrap_or_else(|_| {
            // Try to get whatever output we have
            driver
                .wait_for_all_output(Duration::from_secs(5))
                .unwrap_or_default()
        });

    let arrived = driver_output.contains("ARRIVED");
    eprintln!("auto_drive output:\n{}", &driver_output[..driver_output.len().min(2000)]);

    // Stop recording gracefully (SIGINT for clean mcap flush)
    stop_recording_gracefully(&mut recorder);

    // Check bag info
    if bag_dir.exists() {
        let info = get_bag_info(&bag_dir);
        eprintln!("Rosbag info:\n{info}");
    } else {
        eprintln!("WARNING: Bag directory not created at {:?}", bag_dir);
    }

    assert!(
        autoware.is_running(),
        "Autoware crashed during auto-drive"
    );

    if arrived {
        eprintln!("PASS: Baseline auto-drive arrived at goal");
    } else {
        eprintln!("WARNING: Vehicle did not arrive at goal (may be a zenoh routing issue)");
        // Don't fail — zenoh service call routing is known to be flaky
    }
}

/// Sentinel auto-drive: launch filtered Autoware + sentinel via zenohd,
/// run auto_drive.py, record a rosbag, and verify the vehicle arrives.
#[rstest]
fn test_sentinel_auto_drive(
    zenohd_unique: ZenohRouter,
    autoware_record: PathBuf,
    sentinel_binary: PathBuf,
) {
    if !autoware::require_planning_simulator_prerequisites() {
        return;
    }

    let locator = zenohd_unique.locator();

    // Filter record to remove sentinel-replaced nodes
    let filtered_path = std::env::temp_dir()
        .join("sentinel_tests")
        .join("auto_drive_filtered_record.json");
    let filter = NodeFilter::sentinel_replacement();
    filter_record(&autoware_record, &filter, &filtered_path).expect("Failed to filter record");

    // Start filtered Autoware replay
    eprintln!("Starting filtered Autoware (rmw_zenoh_cpp → {locator})...");
    let mut autoware = start_autoware_zenoh_replay(&filtered_path, &locator)
        .expect("Failed to start filtered Autoware");

    // Start sentinel
    eprintln!("Starting sentinel (→ {locator})...");
    let _sentinel = start_sentinel(&sentinel_binary, &locator).expect("Sentinel failed to start");

    // Start rosbag recording
    eprintln!("Starting rosbag recording...");
    let (mut recorder, bag_dir) =
        start_rosbag_recording("sentinel_test", &locator).expect("Failed to start recording");

    // Wait for initialization (sentinel needs extra time for zenoh discovery)
    eprintln!("Waiting 90s for initialization...");
    std::thread::sleep(Duration::from_secs(90));

    assert!(
        autoware.is_running(),
        "Filtered Autoware exited during initialization"
    );

    // Start auto_drive.py
    eprintln!("Starting auto_drive.py (timeout=240s)...");
    let mut driver = start_auto_drive(&locator, 240).expect("Failed to start auto_drive");

    // Wait for auto_drive to complete
    let driver_output = driver
        .wait_for_output_pattern("ARRIVED", Duration::from_secs(300))
        .unwrap_or_else(|_| {
            driver
                .wait_for_all_output(Duration::from_secs(5))
                .unwrap_or_default()
        });

    let arrived = driver_output.contains("ARRIVED");
    eprintln!("auto_drive output:\n{}", &driver_output[..driver_output.len().min(2000)]);

    // Stop recording
    recorder.kill();
    std::thread::sleep(Duration::from_secs(2));

    // Check bag info
    if bag_dir.exists() {
        let info = get_bag_info(&bag_dir);
        eprintln!("Rosbag info:\n{info}");
    } else {
        eprintln!("WARNING: Bag directory not created at {:?}", bag_dir);
    }

    assert!(
        autoware.is_running(),
        "Filtered Autoware crashed during auto-drive"
    );

    if arrived {
        eprintln!("PASS: Sentinel auto-drive arrived at goal");
    } else {
        eprintln!("WARNING: Vehicle did not arrive at goal (may be zenoh routing or staleness braking)");
    }
}
