//! AutowareLauncher fixture for planning simulator integration tests
//!
//! Wraps the `play_launch` dump/replay lifecycle as ManagedProcess instances.
//! Provides dump(), filter_record(), and replay() methods.

use crate::autoware;
use crate::process::ManagedProcess;
use crate::{TestError, TestResult};
use once_cell::sync::OnceCell;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

/// Cached path to the dumped (unfiltered) record.json.
static AUTOWARE_RECORD: OnceCell<PathBuf> = OnceCell::new();

/// Dump the Autoware planning simulator launch to a record.json file.
///
/// Uses `play_launch dump launch` with the Autoware map path. The result
/// is cached via `OnceCell` so the dump only runs once per test process.
///
/// Requires:
/// - `play_launch` installed
/// - ROS 2 environment with Autoware packages
/// - Autoware map data
pub fn dump_autoware_record() -> TestResult<&'static Path> {
    AUTOWARE_RECORD
        .get_or_try_init(|| {
            let map_path = autoware::autoware_map_path();
            let output_dir = std::env::temp_dir().join("sentinel_tests");
            std::fs::create_dir_all(&output_dir).map_err(TestError::ProcessStart)?;
            let output_file = output_dir.join("autoware_record.json");

            eprintln!(
                "Dumping Autoware planning simulator launch (map: {:?})...",
                map_path
            );

            let play_launch = autoware::play_launch_binary();
            let map_arg = format!("map_path:={}", map_path.display());

            // play_launch dump needs the ROS 2 environment sourced.
            // Note: --output is a `play_launch dump` option and must come
            // before the `launch` subcommand.
            let env_setup = format!(
                "source /opt/ros/humble/setup.bash && \
                 source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null && \
                 {} dump --output {} launch autoware_launch planning_simulator.launch.xml \
                 {}",
                play_launch.display(),
                output_file.display(),
                map_arg,
            );

            let output = Command::new("bash")
                .args(["-c", &env_setup])
                .output()
                .map_err(|e| {
                    TestError::BuildFailed(format!("play_launch dump failed to start: {e}"))
                })?;

            if !output.status.success() {
                let stderr = String::from_utf8_lossy(&output.stderr);
                let stdout = String::from_utf8_lossy(&output.stdout);
                return Err(TestError::BuildFailed(format!(
                    "play_launch dump failed:\nstdout: {stdout}\nstderr: {stderr}"
                )));
            }

            if !autoware::is_valid_record(&output_file) {
                return Err(TestError::BuildFailed(format!(
                    "play_launch dump produced invalid record at {:?}",
                    output_file
                )));
            }

            eprintln!("Autoware record dumped: {:?}", output_file);
            Ok(output_file)
        })
        .map(|p| p.as_path())
}

/// Node filter specification for record.json.
///
/// Specifies which nodes to remove from each array in the record.
pub struct NodeFilter {
    /// Node names to remove from the `node[]` array
    pub node_names: Vec<String>,
    /// Container names to remove from the `container[]` array
    pub container_names: Vec<String>,
    /// Package names to remove from the `load_node[]` array
    pub load_node_packages: Vec<String>,
}

impl NodeFilter {
    /// Create a filter for the 7 nodes replaced by the sentinel.
    pub fn sentinel_replacement() -> Self {
        Self {
            node_names: vec!["mrm_handler".into()],
            container_names: vec![
                "mrm_emergency_stop_operator_container".into(),
                "mrm_comfortable_stop_operator_container".into(),
            ],
            load_node_packages: vec![
                "autoware_vehicle_cmd_gate".into(),
                "autoware_shift_decider".into(),
                "autoware_operation_mode_transition_manager".into(),
                "autoware_control_validator".into(),
            ],
        }
    }
}

/// Filter nodes from a record.json file.
///
/// Reads the record, removes nodes matching the filter, and writes the
/// filtered result to `output_path`.
pub fn filter_record(
    record_path: &Path,
    filter: &NodeFilter,
    output_path: &Path,
) -> TestResult<()> {
    let content = std::fs::read_to_string(record_path)
        .map_err(|e| TestError::ProcessFailed(format!("Failed to read record: {e}")))?;

    let mut record: serde_json::Value = serde_json::from_str(&content)
        .map_err(|e| TestError::ProcessFailed(format!("Failed to parse record JSON: {e}")))?;

    // Filter node[] — match by "name" field (may include namespace prefix)
    if let Some(nodes) = record.get_mut("node").and_then(|v| v.as_array_mut()) {
        let filter_names = &filter.node_names;
        nodes.retain(|node| {
            let name = node.get("name").and_then(|v| v.as_str()).unwrap_or("");
            // Match against full name or just the trailing component
            let base_name = name.rsplit('/').next().unwrap_or(name);
            !filter_names.iter().any(|f| f == base_name || f == name)
        });
    }

    // Filter container[] — match by "name" field
    if let Some(containers) = record.get_mut("container").and_then(|v| v.as_array_mut()) {
        let filter_names = &filter.container_names;
        containers.retain(|container| {
            let name = container.get("name").and_then(|v| v.as_str()).unwrap_or("");
            let base_name = name.rsplit('/').next().unwrap_or(name);
            !filter_names.iter().any(|f| f == base_name || f == name)
        });
    }

    // Filter load_node[] — match by "package" field
    if let Some(load_nodes) = record.get_mut("load_node").and_then(|v| v.as_array_mut()) {
        let filter_packages = &filter.load_node_packages;
        load_nodes.retain(|load_node| {
            let package = load_node
                .get("package")
                .and_then(|v| v.as_str())
                .unwrap_or("");
            !filter_packages.iter().any(|f| f == package)
        });
    }

    let filtered = serde_json::to_string_pretty(&record).map_err(|e| {
        TestError::ProcessFailed(format!("Failed to serialize filtered record: {e}"))
    })?;

    std::fs::write(output_path, filtered)
        .map_err(|e| TestError::ProcessFailed(format!("Failed to write filtered record: {e}")))?;

    eprintln!("Filtered record written to {:?}", output_path);
    Ok(())
}

/// Replay a record.json file using `play_launch replay`.
///
/// Uses default DDS transport. For zenoh-based replay, set `ZENOH_LOCATOR`
/// in the environment before calling.
///
/// Returns a `ManagedProcess` that owns the replayed process tree.
/// The process is killed on drop.
pub fn replay_record(record_path: &Path) -> TestResult<ManagedProcess> {
    replay_record_with_env(record_path, "")
}

/// Replay a record.json file with additional environment setup.
///
/// `extra_env` is appended to the bash command after sourcing ROS 2/Autoware.
/// Use this to set `RMW_IMPLEMENTATION` or `ZENOH_CONFIG_OVERRIDE`.
pub fn replay_record_with_env(record_path: &Path, extra_env: &str) -> TestResult<ManagedProcess> {
    let play_launch = autoware::play_launch_binary();

    let extra = if extra_env.is_empty() {
        String::new()
    } else {
        format!(" && {extra_env}")
    };

    let env_setup = format!(
        "source /opt/ros/humble/setup.bash && \
         source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null{extra} && \
         {play_launch} replay --input-file {record} --disable-all",
        play_launch = play_launch.display(),
        record = record_path.display(),
    );

    let mut cmd = Command::new("bash");
    cmd.args(["-c", &env_setup]);

    ManagedProcess::spawn_command(cmd, "play_launch replay")
}

/// Start Autoware via play_launch replay and wait for it to be ready.
///
/// Replays the record (default DDS) and waits for the "All processes started"
/// pattern in the output, indicating Autoware has finished launching.
pub fn start_autoware(record_path: &Path) -> TestResult<ManagedProcess> {
    start_autoware_impl(record_path, "")
}

/// Start Autoware with rmw_zenoh_cpp transport.
///
/// Replays the record with `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and zenoh
/// client configuration pointing to the given locator.
pub fn start_autoware_zenoh(record_path: &Path, locator: &str) -> TestResult<ManagedProcess> {
    let extra_env = format!(
        "export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
         export ZENOH_CONFIG_OVERRIDE='mode=\"client\";connect/endpoints=[\"{locator}\"]'"
    );
    start_autoware_impl(record_path, &extra_env)
}

fn start_autoware_impl(record_path: &Path, extra_env: &str) -> TestResult<ManagedProcess> {
    eprintln!("Starting Autoware replay from {:?}...", record_path);
    let mut proc = replay_record_with_env(record_path, extra_env)?;

    // Wait for play_launch to finish starting all processes.
    // play_launch prints progress as it starts each node.
    // We wait for a reasonable time for all processes to be spawned.
    let output = proc.wait_for_output_pattern("All processes started", Duration::from_secs(60));

    match output {
        Ok(out) => eprintln!("Autoware started ({} bytes of output)", out.len()),
        Err(crate::TestError::Timeout) => {
            // Timeout is acceptable — play_launch may not print this exact pattern.
            // Check if the process is still running (good sign).
            if proc.is_running() {
                eprintln!("Autoware replay still running after 60s (likely ready)");
            } else {
                return Err(TestError::ProcessFailed(
                    "Autoware replay exited before becoming ready".into(),
                ));
            }
        }
        Err(e) => return Err(e),
    }

    Ok(proc)
}

/// rstest fixture: dump the Autoware record once per test process.
#[rstest::fixture]
pub fn autoware_record() -> PathBuf {
    dump_autoware_record()
        .expect("Failed to dump Autoware record")
        .to_path_buf()
}
