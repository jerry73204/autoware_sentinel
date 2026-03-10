#!/usr/bin/env bash
# Record a rosbag with sentinel-related topics.
# Usage: scripts/record_bag.sh <label>
#   label: "baseline" or "sentinel" (used in directory name)
#
# Bags are saved to tmp/bags/<label>/<label>_YYYY-MM-DD_HHMMSS/
# A symlink tmp/bags/<label>/latest -> most recent recording

set -eo pipefail

LABEL="${1:?Usage: $0 <label>}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOPICS_FILE="$SCRIPT_DIR/record_topics.txt"

if [ ! -f "$TOPICS_FILE" ]; then
    echo "Error: $TOPICS_FILE not found" >&2
    exit 1
fi

TOPICS=$(grep -v '^#' "$TOPICS_FILE" | grep -v '^$' | tr '\n' ' ')
TIMESTAMP=$(date +%Y-%m-%d_%H%M%S)
BAG_DIR="tmp/bags/${LABEL}/${LABEL}_${TIMESTAMP}"

mkdir -p "tmp/bags/${LABEL}"
echo "=== Recording to ${BAG_DIR} ==="
ros2 bag record --storage mcap --output "$BAG_DIR" $TOPICS &
PID=$!

# Update latest symlink
ln -sfn "${LABEL}_${TIMESTAMP}" "tmp/bags/${LABEL}/latest"

# Forward signals to ros2 bag record for clean shutdown
trap "kill $PID 2>/dev/null; wait $PID 2>/dev/null" INT TERM
wait $PID
