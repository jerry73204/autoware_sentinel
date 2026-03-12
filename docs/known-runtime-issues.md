# Known Runtime Issues

Runtime issues observed during sentinel-Autoware integration testing.

## zenoh-pico subscription dropouts

**Symptom:** ~20-second gaps in controller data during autonomous driving.

**Impact:** Vehicle receives stale trajectory/control commands, causing overshoot or
control validator over-velocity triggering MRM cascade.

**Mitigation:** Staleness guard in `autoware_sentinel_linux` (`EXTERNAL_CONTROL_STALE_MS = 500`).
When external control data is >500ms old, sentinel replaces the stale command with gentle braking
(accel = -1.5 m/s², velocity = current). This prevents overshoot during dropouts.

**Root cause:** Unknown. zenoh-pico intermittently stops receiving subscription data through the
zenohd router even though the publisher (rmw_zenoh_cpp) is still active.

## zenoh discovery delay

**Symptom:** 14-20 seconds for zenoh-pico to discover rmw_zenoh_cpp publishers through
the zenohd router after startup.

**Impact:** Sentinel receives no data for the first 14-20s, causing it to operate on stale
or default values. Combined with staleness braking, this slows autonomous drive arrival from
~25s (baseline) to 42-56s.

**Root cause:** zenoh interest-driven routing propagation latency. zenoh-pico must declare
interests, which propagate through the router to rmw_zenoh_cpp, which then begins publishing
to the matching subscribers. Under resource contention this can exceed 20s.

## Intermittent consecutive service call failures

**Symptom:** Alternating success/failure on back-to-back `ros2 service call` to sentinel
queryables (1st ok, 2nd fail, 3rd ok pattern).

**Impact:** Low. Only one `ChangeOperationMode` call is needed at Autoware startup.
Single calls work reliably.

**Root cause:** Suspected zenoh router query routing cache/state issue. All calls trigger
the queryable handler and reply with correct attachment, but the 2nd reply is not delivered
to the caller.

## ADAPI composable node load failures

**Symptom:** `play_launch` sometimes fails to load 2-6 ADAPI adaptor composable nodes
within the 30-second timeout during Autoware startup.

**Impact:** Missing ADAPI endpoints (e.g., `/api/operation_mode/state`). Non-deterministic,
depends on system load and resource contention.

**Workaround:** Sentinel publishes its own `/api/operation_mode/state` and `/autoware/state`
topics, so the missing ADAPI adaptors are not critical when running with sentinel.

## ros2 CLI service call crash

**Symptom:** `ros2 service call` (C++ rcl) crashes with
`failed to initialize wait set: the given context is not valid`.

**Impact:** Cannot use `ros2 service call` CLI tool with rmw_zenoh_cpp. Does not affect
programmatic service calls (Python rclpy works fine).

**Workaround:** Use Python rclpy for service calls, or `ros2 param` commands which use
a different code path.
