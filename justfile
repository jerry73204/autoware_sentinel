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

# Build all packages (generates bindings first) + Zephyr application
build: generate-bindings build-zephyr
    for pkg in {{ packages }}; do echo "=== $pkg ===" && (cd "src/$pkg" && cargo build); done

# Build Zephyr application (native_sim)
build-zephyr:
    #!/usr/bin/env bash
    set -eo pipefail
    source {{ env_script }}
    cd {{ workspace_dir }}
    west build -b native_sim/native/64 autoware-sentinel/src/autoware_sentinel -d build/sentinel

# Test all packages
test:
    for pkg in {{ packages }}; do echo "=== $pkg ===" && (cd "src/$pkg" && cargo test); done

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
