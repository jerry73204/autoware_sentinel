#!/usr/bin/env bash
# Build rmw_zenoh_cpp from external/rmw_zenoh_ws/src/rmw_zenoh/ submodule.
# Installs to external/rmw_zenoh_ws/install/ for overlay sourcing.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$REPO_DIR/external/rmw_zenoh_ws"

# Source ROS 2 underlay
source /opt/ros/humble/setup.bash

cd "$WS_DIR"

echo "=== Building rmw_zenoh_cpp from source ($(pwd)) ==="
colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+

echo "=== Build complete: source $WS_DIR/install/local_setup.bash ==="
