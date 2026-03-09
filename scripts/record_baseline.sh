#!/usr/bin/env bash
#
# Record baseline Autoware topic data for behavioral verification (Phase 9.1a).
#
# Starts unmodified Autoware in the planning simulator using default DDS transport,
# runs the autonomous drive sequence via a Python ROS 2 node (auto_drive.py),
# and records 20 functional output topics as an MCAP rosbag.
#
# Output: tmp/bags/baseline/ (MCAP format)
#
# Prerequisites:
#   - ROS 2 Humble with Autoware 1.5.0 installed
#   - play_launch installed
#
# Usage: just record-autoware-baseline
#        or: bash scripts/record_baseline.sh [DRIVE_DURATION] [OUTPUT_DIR]

set -euo pipefail

# --- Configuration ---
DRIVE_DURATION="${1:-30}"      # seconds to record after engaging
OUTPUT_DIR="${2:-tmp/bags/baseline}"
INIT_WAIT="${INIT_WAIT:-60}"   # seconds to wait for Autoware initialization
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

RECORD_FILE="$REPO_ROOT/tmp/launch/autoware_record.json"

# 20 functional output topics from the 7 replaced nodes
TOPICS=(
    /control/command/control_cmd
    /control/command/gear_cmd
    /control/command/hazard_lights_cmd
    /control/command/turn_indicators_cmd
    /control/command/emergency_cmd
    /control/shift_decider/gear_cmd
    /control/gate_mode_cmd
    /control/vehicle_cmd_gate/is_stopped
    /control/vehicle_cmd_gate/operation_mode
    /system/fail_safe/mrm_state
    /system/mrm/emergency_stop/status
    /system/mrm/comfortable_stop/status
    /system/emergency/gear_cmd
    /system/emergency/hazard_lights_cmd
    /system/emergency/turn_indicators_cmd
    /api/autoware/get/engage
    /autoware/engage
    /api/autoware/get/emergency
    /api/operation_mode/state
    /control/control_validator/validation_status
)

# --- Transport ---
# RMW_IMPLEMENTATION should be set in the environment (e.g. .envrc).
# Unset ZENOH_SESSION_CONFIG_URI to avoid interfering with CLI tools.
unset ZENOH_SESSION_CONFIG_URI 2>/dev/null || true

# --- Cleanup trap ---
PIDS=()
cleanup() {
    echo ""
    echo "=== Cleaning up ==="
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    sleep 2
    # Force kill any remaining
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT

# --- Helpers ---
start_background() {
    local name="$1"
    shift
    echo "  Starting $name..."
    "$@" &
    local pid=$!
    PIDS+=("$pid")
    echo "  $name PID: $pid"
}

# --- Preflight checks ---
echo "=== Phase 9.1a: Record Baseline Autoware ==="
echo ""

if ! command -v play_launch &>/dev/null; then
    echo "ERROR: play_launch not found. Install it first."
    exit 1
fi

if ! command -v ros2 &>/dev/null; then
    echo "ERROR: ros2 not found. Source ROS 2 environment first."
    exit 1
fi

if [ ! -f "$SCRIPT_DIR/auto_drive.py" ]; then
    echo "ERROR: scripts/auto_drive.py not found."
    exit 1
fi

# --- Step 1: Dump Autoware record if needed ---
if [ ! -f "$RECORD_FILE" ]; then
    echo "=== Step 1: Dumping Autoware record ==="
    mkdir -p "$(dirname "$RECORD_FILE")"
    play_launch dump --output "$RECORD_FILE" \
        launch autoware_launch planning_simulator.launch.xml \
        map_path:=/opt/autoware/1.5.0/share/autoware_test_utils/test_map
    echo "  Record: $RECORD_FILE"
else
    echo "=== Step 1: Using existing Autoware record ==="
    echo "  Record: $RECORD_FILE"
fi

# --- Step 2: Start baseline Autoware (default DDS) ---
echo ""
echo "=== Step 2: Starting baseline Autoware (CycloneDDS) ==="
start_background "Autoware" \
    play_launch replay --input-file "$RECORD_FILE" --web-addr 0.0.0.0:8080

echo "  Waiting ${INIT_WAIT}s for Autoware initialization..."
sleep "$INIT_WAIT"

# --- Step 3: Prepare output directory ---
echo ""
echo "=== Step 3: Preparing output directory ==="
# ros2 bag record -o wants to create the directory itself, so remove it if it exists
rm -rf "${OUTPUT_DIR:?}"
mkdir -p "$(dirname "$OUTPUT_DIR")"
echo "  Output: $OUTPUT_DIR"

# --- Step 4: Start rosbag recording ---
echo ""
echo "=== Step 4: Starting rosbag recording ==="
TOPIC_ARGS=""
for topic in "${TOPICS[@]}"; do
    TOPIC_ARGS="$TOPIC_ARGS $topic"
done

start_background "rosbag" ros2 bag record \
    --storage mcap \
    --output "$OUTPUT_DIR" \
    $TOPIC_ARGS

sleep 5
echo "  Recording ${#TOPICS[@]} topics..."

# --- Step 5: Autonomous drive sequence (Python ROS 2 node) ---
echo ""
echo "=== Step 5: Autonomous drive sequence (auto_drive.py) ==="
# Use --no-wait-arrival: the script engages autonomous mode and exits,
# then we record for DRIVE_DURATION seconds while the vehicle drives.
python3 "$SCRIPT_DIR/auto_drive.py" --no-wait-arrival --timeout 60 || {
    echo "  WARNING: auto_drive.py exited with error (continuing recording anyway)"
}

# --- Step 6: Record during drive ---
echo ""
echo "=== Step 6: Recording for ${DRIVE_DURATION}s ==="
sleep "$DRIVE_DURATION"

# --- Step 7: Summary ---
echo ""
echo "=== Step 7: Recording complete ==="
echo ""
echo "Bag files in $OUTPUT_DIR:"
ls -lh "$OUTPUT_DIR"/ 2>/dev/null || echo "  (no files found)"
echo ""

# Check bag info if available
if ls "$OUTPUT_DIR"/*.mcap &>/dev/null; then
    echo "Bag info:"
    ros2 bag info "$OUTPUT_DIR" 2>/dev/null || true
fi

echo ""
echo "=== Baseline recording complete ==="
echo "Output: $OUTPUT_DIR"
