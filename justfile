# justfile for autoware-nano-ros

set dotenv-load

packages := "autoware_stop_filter autoware_vehicle_velocity_converter autoware_shift_decider autoware_mrm_emergency_stop_operator autoware_mrm_comfortable_stop_operator autoware_heartbeat_watchdog autoware_mrm_handler autoware_vehicle_cmd_gate autoware_twist2accel autoware_control_validator autoware_operation_mode_transition_manager autoware_interpolation autoware_universe_utils autoware_vehicle_info_utils autoware_motion_utils autoware_pid_longitudinal_controller autoware_mpc_lateral_controller autoware_trajectory_follower_base autoware_trajectory_follower_node"

kani_packages := "autoware_stop_filter autoware_vehicle_velocity_converter autoware_shift_decider autoware_mrm_emergency_stop_operator"

workspace_dir := "../autoware-sentinel-workspace"
env_script := workspace_dir / "env.sh"
zenohd := "external/zenoh/target/fast/zenohd"
session_config := ".config/zenoh_session.json5"
router_config := ".config/zenoh_router.json5"

# Default recipe - show available commands
default:
    @just --list

# Generate message bindings (sentinel_linux is the superset; workspace patches share its generated/)
generate-bindings:
    #!/usr/bin/env bash
    set -eo pipefail
    source scripts/activate_autoware.sh
    echo "=== autoware_sentinel_linux ==="
    (cd "src/autoware_sentinel_linux" && cargo nano-ros generate-rust --force)
    echo "=== autoware_sentinel (Zephyr) ==="
    (cd "src/autoware_sentinel" && cargo nano-ros generate-rust --force)

# Build all packages (generates bindings first) + Zephyr + Linux sentinel
build: generate-bindings build-zephyr build-sentinel-linux build-rmw-zenoh
    cargo build --workspace --tests

# Build Zephyr application (native_sim)
build-zephyr:
    #!/usr/bin/env bash
    set -eo pipefail
    source {{ env_script }}
    cd {{ workspace_dir }}
    west build -b native_sim/native/64 autoware-sentinel/src/autoware_sentinel -d build/sentinel

# Build Linux sentinel binary
build-sentinel-linux:
    cargo build -p autoware_sentinel_linux

# Run Linux sentinel binary
run-sentinel-linux:
    cargo run -p autoware_sentinel_linux

# Test all packages (unit tests)
test:
    cargo test --workspace

# Run integration tests with nextest
test-integration:
    cd tests && cargo nextest run

# Run integration tests (transport smoke only)
test-transport:
    cd tests && cargo nextest run -E 'binary(transport_smoke)'

# Run planning simulator integration tests only
test-planning:
    cd tests && cargo nextest run -E 'binary(planning_simulator)'

# Run auto-drive comparison tests (baseline vs sentinel)
test-auto-drive:
    cd tests && cargo nextest run -E 'binary(auto_drive_comparison)'

# Dump Autoware planning simulator launch to record.json
dump-autoware map_path="/opt/autoware/1.5.0/share/autoware_test_utils/test_map":
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    mkdir -p tmp/launch
    echo "Dumping Autoware planning simulator (map: {{ map_path }})..."
    play_launch dump --output tmp/launch/autoware_record.json \
        launch autoware_launch planning_simulator.launch.xml \
        map_path:={{ map_path }}
    echo "Record written to tmp/launch/autoware_record.json"

# Filter play_launch record to remove 7 sentinel-replaced nodes
filter-autoware: dump-autoware
    scripts/filter_autoware_record.sh tmp/launch/autoware_record.json tmp/launch/autoware_record_filtered.json

# Launch baseline Autoware planning simulator (unmodified)
[arg("record", long="record", value="true")]
[arg("drive", long="drive", value="true")]
[arg("timeout", long="timeout")]
[arg("poses", long="poses")]
launch-autoware-baseline $record="false" $drive="false" $timeout="120" $poses="scripts/poses.yaml": dump-autoware
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    export ZENOH_SESSION_CONFIG_URI="$(pwd)/{{ session_config }}"

    JOBS=(
        '{{ zenohd }} --config {{ router_config }} < /dev/null'
        'play_launch replay --input-file tmp/launch/autoware_record.json --web-addr 0.0.0.0:8080'
    )

    if [ "$record" = "true" ]; then
        JOBS+=("scripts/record_bag.sh baseline")
    fi

    if [ "$drive" = "true" ]; then
        JOBS+=("sleep 90 && python3 scripts/auto_drive.py --timeout $timeout --poses $poses")
    fi

    echo "=== Baseline Autoware ==="
    parallel --line-buffer --halt now,done=1 --delay 2 ::: "${JOBS[@]}"

# Launch sentinel binary with zenohd (no Autoware)
launch-sentinel: build-sentinel-linux
    #!/usr/bin/env bash
    set -eo pipefail
    SENTINEL="$(pwd)/target/debug/autoware_sentinel_linux"

    echo "=== Sentinel + zenohd ($ZENOH_LOCATOR) ==="
    parallel --line-buffer --halt now,done=1 --delay 2 ::: \
      '{{ zenohd }} --config {{ router_config }} < /dev/null' \
      "$SENTINEL"

# Launch filtered Autoware + sentinel (7 nodes replaced by sentinel binary)
[arg("record", long="record", value="true")]
[arg("drive", long="drive", value="true")]
[arg("timeout", long="timeout")]
[arg("poses", long="poses")]
launch-autoware-sentinel $record="false" $drive="false" $timeout="120" $poses="scripts/poses.yaml": filter-autoware build-sentinel-linux
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    export ZENOH_SESSION_CONFIG_URI="$(pwd)/{{ session_config }}"

    FILTERED=tmp/launch/autoware_record_filtered.json
    SENTINEL="$(pwd)/target/debug/autoware_sentinel_linux"

    JOBS=(
        '{{ zenohd }} --config {{ router_config }} < /dev/null'
        "play_launch replay --input-file $FILTERED --web-addr 0.0.0.0:8080"
        "$SENTINEL"
    )

    if [ "$record" = "true" ]; then
        JOBS+=("scripts/record_bag.sh sentinel")
    fi

    if [ "$drive" = "true" ]; then
        JOBS+=("sleep 90 && python3 scripts/auto_drive.py --timeout $timeout --poses $poses")
    fi

    echo "=== Autoware + Sentinel ==="
    parallel --line-buffer --halt now,done=1 --delay 2 ::: "${JOBS[@]}"

# Format all packages
format:
    cargo fmt --all

# Check formatting on all packages
format-check:
    cargo fmt --all -- --check

# Cross-compile check all algorithm crates (excludes sentinel_linux which requires std)
cross-check:
    cargo check --workspace --exclude autoware_sentinel_linux --target thumbv7em-none-eabihf

# CI: format-check, cross-check, and test
ci: format-check cross-check test

# Clean all packages
clean:
    cargo clean

# Run Kani verification on all harness crates
verify-kani:
    parallel --tag --line-buffer --halt now,fail=1 \
      'cd src/{} && cargo kani' ::: {{ kani_packages }}

# Run Verus verification
verify-verus:
    cd src/verification && ~/.verus/verus-main/source/target-verus/release/verus src/lib.rs

# Build rmw_zenoh_cpp from source
build-rmw-zenoh:
    scripts/build_rmw_zenoh.sh

# Rebuild zenohd from source
build-zenohd:
    cd external/zenoh && cargo build --profile fast -p zenohd

# Run zenohd router (needed for rmw_zenoh_cpp)
run-zenohd:
    {{ zenohd }} --config {{ router_config }}

# Run all verification
verify: verify-kani verify-verus

# Setup TAP network for Zephyr native_sim (requires sudo)
setup-tap-network:
    sudo scripts/zephyr/setup-network.sh

# Tear down TAP network (requires sudo)
teardown-tap-network:
    sudo scripts/zephyr/setup-network.sh --down

# Run Zephyr sentinel (native_sim) with zenohd on bridge network
run-sentinel-zephyr: build-zephyr
    #!/usr/bin/env bash
    set -eo pipefail
    ZEPHYR_BIN="{{ workspace_dir }}/build/sentinel/zephyr/zephyr.exe"
    if [ ! -f "$ZEPHYR_BIN" ]; then
        echo "Error: Zephyr binary not found at $ZEPHYR_BIN"
        echo "Run: just build-zephyr"
        exit 1
    fi

    echo "=== Zephyr Sentinel + zenohd (bridge: 192.0.2.2:7447) ==="
    echo "NOTE: TAP network must be set up first: just setup-tap-network"
    parallel --line-buffer --halt now,done=1 --delay 2 ::: \
      '{{ zenohd }} --listen tcp/0.0.0.0:7447' \
      "$ZEPHYR_BIN"

# Capture initial + goal poses from RViz and save to a file
capture-poses output="scripts/poses.yaml":
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    python3 scripts/capture_poses.py -o {{ output }}

# Run autonomous drive sequence (init pose → route → engage → wait for arrival)
auto-drive timeout="120" poses="scripts/poses.yaml":
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    python3 scripts/auto_drive.py --timeout {{ timeout }} --poses {{ poses }}

# Run Zephyr native_sim integration tests (Phase 7.4)
test-zephyr:
    cd tests && cargo nextest run -E 'binary(zephyr_native_sim)'
