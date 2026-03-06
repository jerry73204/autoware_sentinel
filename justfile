# justfile for autoware-nano-ros

packages := "autoware_stop_filter autoware_vehicle_velocity_converter autoware_shift_decider autoware_mrm_emergency_stop_operator autoware_mrm_comfortable_stop_operator autoware_heartbeat_watchdog autoware_mrm_handler autoware_vehicle_cmd_gate autoware_twist2accel autoware_control_validator autoware_operation_mode_transition_manager autoware_interpolation autoware_universe_utils autoware_vehicle_info_utils autoware_motion_utils autoware_pid_longitudinal_controller autoware_mpc_lateral_controller autoware_trajectory_follower_base autoware_trajectory_follower_node"

kani_packages := "autoware_stop_filter autoware_vehicle_velocity_converter autoware_shift_decider autoware_mrm_emergency_stop_operator"

workspace_dir := "../autoware-sentinel-workspace"
env_script := workspace_dir / "env.sh"
zenohd := "external/zenoh/target/fast/zenohd"
locator := "tcp/127.0.0.1:7447"
session_config := ".config/zenoh_session.json5"
router_config := ".config/zenoh_router.json5"

# Default recipe - show available commands
default:
    @just --list

# Generate bindings for all packages
generate-bindings:
    #!/usr/bin/env bash
    set -eo pipefail
    source scripts/activate_autoware.sh
    parallel --tag --line-buffer --halt now,fail=1 \
      'cd src/{} && cargo nano-ros generate-rust --config --nano-ros-path ../../../nano-ros/packages/core --force' \
      ::: {{ packages }}
    echo "=== autoware_sentinel ==="
    (cd "src/autoware_sentinel" && cargo nano-ros generate-rust --force)
    echo "=== autoware_sentinel_linux ==="
    (cd "src/autoware_sentinel_linux" && cargo nano-ros generate-rust --config --nano-ros-path ../../../nano-ros/packages/core --force)

# Build all packages (generates bindings first) + Zephyr + Linux sentinel
build: generate-bindings build-zephyr build-sentinel-linux build-rmw-zenoh
    parallel --tag --line-buffer --halt now,fail=1 \
      'cd src/{} && cargo build --tests' ::: {{ packages }}

# Build Zephyr application (native_sim)
build-zephyr:
    #!/usr/bin/env bash
    set -eo pipefail
    source {{ env_script }}
    cd {{ workspace_dir }}
    west build -b native_sim/native/64 autoware-sentinel/src/autoware_sentinel -d build/sentinel

# Build Linux sentinel binary
build-sentinel-linux:
    cd src/autoware_sentinel_linux && ZPICO_MAX_PUBLISHERS=32 ZPICO_MAX_LIVELINESS=52 cargo build

# Run Linux sentinel binary
run-sentinel-linux:
    cd src/autoware_sentinel_linux && ZPICO_MAX_PUBLISHERS=32 ZPICO_MAX_LIVELINESS=52 RUST_LOG=info cargo run

# Test all packages (unit tests)
test:
    parallel --tag --line-buffer --halt now,fail=1 \
      'cd src/{} && cargo test' ::: {{ packages }}

# Run integration tests with nextest
test-integration:
    cd tests && cargo nextest run

# Run integration tests (transport smoke only)
test-transport:
    cd tests && cargo nextest run -E 'binary(transport_smoke)'

# Run planning simulator integration tests only
test-planning:
    cd tests && cargo nextest run -E 'binary(planning_simulator)'

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

# Launch baseline Autoware planning simulator (unmodified, rmw_zenoh_cpp)
launch-autoware-baseline: dump-autoware
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_SESSION_CONFIG_URI="$(pwd)/{{ session_config }}"

    echo "=== Baseline Autoware (rmw_zenoh_cpp → {{ locator }}) ==="
    parallel --line-buffer --halt now,done=1 --delay 2 ::: \
      '{{ zenohd }} --config {{ router_config }}' \
      'play_launch replay --input-file tmp/launch/autoware_record.json --web-addr 0.0.0.0:8080'

# Launch sentinel binary with zenohd (no Autoware)
launch-sentinel: build-sentinel-linux
    #!/usr/bin/env bash
    set -eo pipefail
    SENTINEL="$(pwd)/src/autoware_sentinel_linux/target/debug/autoware_sentinel_linux"

    echo "=== Sentinel + zenohd ({{ locator }}) ==="
    parallel --line-buffer --halt now,done=1 --delay 2 ::: \
      '{{ zenohd }} --config {{ router_config }}' \
      "RUST_LOG=info ZENOH_LOCATOR={{ locator }} ZPICO_MAX_PUBLISHERS=32 ZPICO_MAX_LIVELINESS=52 $SENTINEL"

# Launch filtered Autoware + sentinel (7 nodes replaced by sentinel binary)
launch-autoware-sentinel: filter-autoware build-sentinel-linux
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_SESSION_CONFIG_URI="$(pwd)/{{ session_config }}"

    FILTERED=tmp/launch/autoware_record_filtered.json
    SENTINEL="$(pwd)/src/autoware_sentinel_linux/target/debug/autoware_sentinel_linux"

    echo "=== Autoware + Sentinel (rmw_zenoh_cpp → {{ locator }}) ==="
    parallel --line-buffer --halt now,done=1 --delay 2 ::: \
      '{{ zenohd }} --config {{ router_config }}' \
      "play_launch replay --input-file $FILTERED --web-addr 0.0.0.0:8080" \
      "RUST_LOG=info ZENOH_LOCATOR={{ locator }} ZPICO_MAX_PUBLISHERS=32 ZPICO_MAX_LIVELINESS=52 $SENTINEL"

# Format all packages
format:
    parallel --tag --line-buffer 'cd src/{} && cargo fmt' ::: {{ packages }}

# Check formatting on all packages
format-check:
    parallel --tag --line-buffer --halt now,fail=1 \
      'cd src/{} && cargo fmt -- --check' ::: {{ packages }}

# Cross-compile check all packages
cross-check:
    parallel --tag --line-buffer --halt now,fail=1 \
      'cd src/{} && cargo check --target thumbv7em-none-eabihf' ::: {{ packages }}

# CI: format-check, cross-check, and test
ci: format-check cross-check test

# Clean all packages
clean:
    parallel --tag --line-buffer 'cd src/{} && just clean' ::: {{ packages }}

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

# Run all verification
verify: verify-kani verify-verus
