# justfile for autoware-nano-ros

packages := "autoware_stop_filter autoware_vehicle_velocity_converter autoware_shift_decider autoware_mrm_emergency_stop_operator autoware_mrm_comfortable_stop_operator autoware_heartbeat_watchdog autoware_mrm_handler autoware_vehicle_cmd_gate autoware_twist2accel autoware_control_validator autoware_operation_mode_transition_manager"

workspace_dir := "../autoware-sentinel-workspace"
env_script := workspace_dir / "env.sh"

# Default recipe - show available commands
default:
    @just --list

# Generate bindings for all packages
generate-bindings:
    #!/usr/bin/env bash
    set -eo pipefail
    source scripts/activate_autoware.sh
    for pkg in {{ packages }}; do
        echo "=== $pkg ==="
        (cd "src/$pkg" && cargo nano-ros generate-rust --config --nano-ros-path ../../../nano-ros/packages/core --force)
    done
    echo "=== autoware_sentinel ==="
    (cd "src/autoware_sentinel" && cargo nano-ros generate-rust --force)
    echo "=== autoware_sentinel_linux ==="
    (cd "src/autoware_sentinel_linux" && cargo nano-ros generate-rust --config --nano-ros-path ../../../nano-ros/packages/core --force)

# Build all packages (generates bindings first) + Zephyr + Linux sentinel
build: generate-bindings build-zephyr build-sentinel-linux
    for pkg in {{ packages }}; do echo "=== $pkg ===" && (cd "src/$pkg" && cargo build --tests); done

# Build Zephyr application (native_sim)
build-zephyr:
    #!/usr/bin/env bash
    set -eo pipefail
    source {{ env_script }}
    cd {{ workspace_dir }}
    west build -b native_sim/native/64 autoware-sentinel/src/autoware_sentinel -d build/sentinel

# Build Linux sentinel binary
build-sentinel-linux:
    cd src/autoware_sentinel_linux && ZPICO_MAX_PUBLISHERS=32 cargo build

# Run Linux sentinel binary
run-sentinel-linux:
    cd src/autoware_sentinel_linux && ZPICO_MAX_PUBLISHERS=32 RUST_LOG=info cargo run

# Test all packages (unit tests)
test:
    for pkg in {{ packages }}; do echo "=== $pkg ===" && (cd "src/$pkg" && cargo test); done

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
    mkdir -p /tmp/sentinel_tests
    echo "Dumping Autoware planning simulator (map: {{ map_path }})..."
    play_launch dump --output /tmp/sentinel_tests/autoware_record.json \
        launch autoware_launch planning_simulator.launch.xml \
        map_path:={{ map_path }}
    echo "Record written to /tmp/sentinel_tests/autoware_record.json"

# Launch baseline Autoware planning simulator (unmodified)
launch-autoware-baseline: dump-autoware
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true
    echo "Replaying baseline Autoware..."
    play_launch replay --input-file /tmp/sentinel_tests/autoware_record.json --web-addr 0.0.0.0:8080

# Launch modified Autoware planning simulator (7 sentinel nodes removed + sentinel binary)
launch-autoware-modified: dump-autoware build-sentinel-linux
    #!/usr/bin/env bash
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/local_setup.bash 2>/dev/null || true

    RECORD=/tmp/sentinel_tests/autoware_record.json
    FILTERED=/tmp/sentinel_tests/autoware_record_filtered.json
    ZENOHD="$HOME/repos/nano-ros/build/zenohd/zenohd"
    SENTINEL="$(pwd)/src/autoware_sentinel_linux/target/debug/autoware_sentinel_linux"
    LOCATOR="tcp/127.0.0.1:7447"

    # Clean up all child processes on exit
    cleanup() {
        echo "Cleaning up..."
        kill $ZENOHD_PID $AUTOWARE_PID $SENTINEL_PID 2>/dev/null
        wait 2>/dev/null
    }
    trap cleanup EXIT INT TERM

    # --- Step 1: Filter record ---
    echo "Filtering 7 sentinel-replaced nodes from record..."
    jq '
      .node |= [.[] | select(.name == null or ((.name | split("/") | last) != "mrm_handler"))]
      | .container |= [.[] | select(.name == null or (
          (.name | split("/") | last) != "mrm_emergency_stop_operator_container"
          and (.name | split("/") | last) != "mrm_comfortable_stop_operator_container"
        ))]
      | .load_node |= [.[] | select(
          .package != "autoware_vehicle_cmd_gate"
          and .package != "autoware_shift_decider"
          and .package != "autoware_operation_mode_transition_manager"
          and .package != "autoware_control_validator"
        )]
    ' "$RECORD" > "$FILTERED"
    echo "Filtered record written to $FILTERED"

    # --- Step 2: Start zenohd router ---
    echo "Starting zenohd on $LOCATOR..."
    "$ZENOHD" --listen "$LOCATOR" &
    ZENOHD_PID=$!
    sleep 2

    # --- Step 3: Start filtered Autoware with rmw_zenoh_cpp ---
    echo "Starting filtered Autoware (rmw_zenoh_cpp → $LOCATOR)..."
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"$LOCATOR\"]"
    play_launch replay --input-file "$FILTERED" --web-addr 0.0.0.0:8080 &
    AUTOWARE_PID=$!

    # --- Step 4: Start sentinel ---
    echo "Starting sentinel (→ $LOCATOR)..."
    RUST_LOG=info ZENOH_LOCATOR="$LOCATOR" "$SENTINEL" &
    SENTINEL_PID=$!

    echo ""
    echo "=== All processes running ==="
    echo "  zenohd:   PID $ZENOHD_PID ($LOCATOR)"
    echo "  Autoware: PID $AUTOWARE_PID (filtered, rmw_zenoh_cpp)"
    echo "  Sentinel: PID $SENTINEL_PID"
    echo "  Web UI:   http://0.0.0.0:8080"
    echo ""
    echo "Press Ctrl-C to stop all processes."
    wait

# Format all packages
format:
    for pkg in {{ packages }}; do echo "=== $pkg ===" && (cd "src/$pkg" && cargo fmt); done

# Check formatting on all packages
format-check:
    for pkg in {{ packages }}; do echo "=== $pkg ===" && (cd "src/$pkg" && cargo fmt -- --check); done

# Cross-compile check all packages
cross-check:
    for pkg in {{ packages }}; do echo "=== $pkg ===" && (cd "src/$pkg" && cargo check --target thumbv7em-none-eabihf); done

# CI: format-check, cross-check, and test
ci: format-check cross-check test

# Clean all packages
clean:
    for pkg in {{ packages }}; do echo "=== $pkg ===" && (cd "src/$pkg" && just clean); done

# Run Kani verification on all harness crates
verify-kani:
    for pkg in autoware_stop_filter autoware_vehicle_velocity_converter autoware_shift_decider autoware_mrm_emergency_stop_operator; do \
        echo "=== Kani: $pkg ===" && (cd "src/$pkg" && cargo kani); \
    done

# Run Verus verification
verify-verus:
    cd src/verification && ~/.verus/verus-main/source/target-verus/release/verus src/lib.rs

# Run all verification
verify: verify-kani verify-verus
