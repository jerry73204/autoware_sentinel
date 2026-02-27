# justfile for autoware-nano-ros

packages := "autoware_stop_filter autoware_vehicle_velocity_converter autoware_shift_decider autoware_mrm_emergency_stop_operator autoware_mrm_comfortable_stop_operator autoware_heartbeat_watchdog autoware_mrm_handler"

# Default recipe - show available commands
default:
    @just --list

# Generate bindings for all packages
generate-bindings:
    #!/usr/bin/env bash
    set -euo pipefail
    source scripts/activate_autoware.sh
    for pkg in {{ packages }}; do
        echo "=== $pkg ==="
        (cd "src/$pkg" && cargo nano-ros generate-rust --config --nano-ros-path ../../../nano-ros/packages/core --force)
    done

# Build all packages
build:
    for pkg in {{ packages }}; do echo "=== $pkg ===" && (cd "src/$pkg" && cargo build); done

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
