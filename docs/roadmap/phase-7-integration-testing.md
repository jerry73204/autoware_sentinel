# Phase 7: Integration Testing

**Status:** Not started
**Depends on:** Phase 6 (Zephyr application), specifically 6.3–6.7 (algorithm wiring)
**Goal:** Validate the safety island end-to-end against the real Autoware planning simulator,
proving that the ported algorithms correctly replace the original Autoware system/control nodes
and can drive a simulated vehicle autonomously.

## Description

Phases 1–5 built and verified 11 algorithm libraries in isolation. Phase 6 composed them into
a single Zephyr application. This phase closes the loop by testing the safety island against
real Autoware infrastructure — first with a Linux-native binary for ease of development, then
with the actual Zephyr `native_sim` build.

Integration testing proves:
1. **Transport compatibility** — nros (zenoh-pico) and ROS 2 (`rmw_zenoh_cpp`) exchange
   CDR-encoded messages through a shared zenohd router
2. **Topic wiring correctness** — the sentinel subscribes to correct Autoware topics and
   publishes commands that the planning simulator accepts
3. **Functional correctness** — the complete MRM chain, command gating, and validation work
   end-to-end with a planning simulator driving a vehicle
4. **Engagement flow** — the sentinel satisfies Autoware's prerequisites for autonomous mode
   (`OperationModeState`, `ChangeOperationMode` service)

### Nodes Replaced by the Sentinel

The sentinel replaces 7 Autoware system/control nodes:

| Node | Full ROS Path | Package |
|------|---------------|---------|
| vehicle_cmd_gate | `/control/vehicle_cmd_gate` | `autoware_vehicle_cmd_gate` |
| mrm_handler | `/system/mrm_handler` | `autoware_mrm_handler` |
| mrm_emergency_stop_operator | `/system/mrm_emergency_stop_operator/...` | `autoware_mrm_emergency_stop_operator` |
| mrm_comfortable_stop_operator | `/system/mrm_comfortable_stop_operator/...` | `autoware_mrm_comfortable_stop_operator` |
| shift_decider | `/control/shift_decider` | (in control container) |
| control_validator | `/control/control_validator` | `autoware_control_validator` |
| operation_mode_transition_manager | `/control/autoware_operation_mode_transition_manager` | `autoware_operation_mode_transition_manager` |

## Subphases

### 7.1 — Linux native binary (`autoware_sentinel_linux`)

- [ ] Package created and builds
- [ ] Topic names mapped to Autoware conventions
- [ ] Engagement flow (OperationModeState + ChangeOperationMode service)
- [ ] Root justfile recipes added

Create `src/autoware_sentinel_linux/`, a std-enabled Linux binary that reuses the
`SafetyIsland` struct and algorithm wiring from the Zephyr `lib.rs` with only platform-layer
changes.

**Target:** `src/autoware_sentinel_linux/`

**Platform differences from Zephyr sentinel:**

| Aspect | Zephyr Sentinel | Linux Binary |
|--------|-----------------|--------------|
| Package name | `rustapp` | `autoware_sentinel_linux` |
| Crate type | `staticlib` (linked into Zephyr) | `bin` (standalone executable) |
| Entry point | `extern "C" fn rust_main()` | `fn main()` |
| Logging | `zephyr::set_logger()` | `env_logger::init()` |
| Clock | `zephyr::sys::uptime_get()` | `std::time::Instant` + `OnceLock` |
| Executor config | `ExecutorConfig::new("tcp/192.0.2.2:7447")` | `ExecutorConfig::from_env().node_name("sentinel")` |
| Spin | `executor.spin(10)` (no_std) | `executor.spin_blocking(SpinOptions::default())` (std) |
| nros features | `["rmw-zenoh", "platform-zephyr"]` | `["std", "rmw-zenoh", "platform-posix", "ros-humble"]` |
| Zephyr dep | `zephyr = "0.1.0"` | Removed; add `env_logger = "0.11"` |
| `#![no_std]` | Yes | No (removed) |
| Heartbeat timeout | 500 ms | 5000 ms (Autoware init is slow) |

**Topic name mapping (subscriptions):**

| Zephyr Topic | Linux Binary (Autoware-Compatible) | Message Type | Reason |
|--------------|------------------------------------|--------------|--------|
| `/control/command/control_cmd` | `/control/trajectory_follower/control_cmd` | `Control` | Subscribe to gate's input, not output |
| `/heartbeat` | `/api/system/heartbeat` | `Heartbeat` | ADAPI heartbeat topic |
| `/system/autoware_state` | `/autoware/state` | `AutowareState` | Compat node publishes here |
| `/vehicle/status/velocity_status` | `/vehicle/status/velocity_status` | `VelocityReport` | Unchanged |
| `/vehicle/status/gear_status` | `/vehicle/status/gear_status` | `GearReport` | Unchanged |

**Topic name mapping (publications):**

| Zephyr Topic | Linux Binary (Autoware-Compatible) | Message Type | Consumer |
|--------------|------------------------------------|--------------|----------|
| `/output/vehicle/control_cmd` | `/control/command/control_cmd` | `Control` | simple_planning_simulator |
| `/output/vehicle/gear_cmd` | `/control/command/gear_cmd` | `GearCommand` | simple_planning_simulator |
| `/output/vehicle/turn_indicators` | `/control/command/turn_indicators_cmd` | `TurnIndicatorsCommand` | simple_planning_simulator |
| `/output/vehicle/hazard_lights` | `/control/command/hazard_lights_cmd` | `HazardLightsCommand` | simple_planning_simulator |
| `/output/mrm/state` | `/system/fail_safe/mrm_state` | `MrmState` | System nodes |

**Additional publishers (engagement flow):**

| Topic | Message Type | Rate | Purpose |
|-------|-------------|------|---------|
| `/api/operation_mode/state` | `OperationModeState` | 30 Hz | Reports `mode=AUTONOMOUS`, `is_autoware_control_enabled=true`, all modes available |

**Additional services (engagement flow):**

| Service Topic | Type | Behavior |
|---------------|------|----------|
| `/api/operation_mode/change_to_autonomous` | `ChangeOperationMode` | Always returns success |

**New files:**

- `src/autoware_sentinel_linux/Cargo.toml` — bin crate, same 11 algorithm + 7 message deps,
  nros with `["std", "rmw-zenoh", "platform-posix", "ros-humble"]`
- `src/autoware_sentinel_linux/src/main.rs` — adapted from Zephyr `lib.rs`
- `src/autoware_sentinel_linux/package.xml` — same 7 message dependencies
- `src/autoware_sentinel_linux/justfile` — generate, build, run recipes
- `src/autoware_sentinel_linux/.cargo/config.toml` — nros patches (9 entries) + generated
  message patches
- `src/autoware_sentinel_linux/.gitignore` — `/target`, `/generated/`

**Root justfile additions:**

- `build-sentinel-linux` — build the Linux binary
- `run-sentinel-linux` — run with `RUST_LOG=info`
- Add to `generate-bindings` recipe

**Acceptance criteria:**

- [ ] `cd src/autoware_sentinel_linux && just build` succeeds
- [ ] `just run-sentinel-linux` starts and prints log output (exits cleanly when no zenohd)
- [ ] Binary reuses all 11 algorithm crates from Phases 1–4
- [ ] `OperationModeState` published at 30 Hz with `mode=AUTONOMOUS`
- [ ] `ChangeOperationMode` service returns success
- [ ] Topic names match Autoware conventions per mapping tables above

### 7.2 — Transport smoke test

- [ ] Bidirectional message flow verified (nros ↔ ROS 2 via shared zenohd)
- [ ] CDR encoding compatible between rmw_zenoh_cpp and nros

Verify that the sentinel Linux binary and ROS 2 nodes can exchange messages through a shared
Zenoh router. Both sides connect as Zenoh clients — no DDS bridge needed.

**Transport architecture:**

```
┌──────────────────┐     ┌─────────────┐     ┌──────────────────────┐
│ Sentinel (Linux) │────▶│   zenohd    │◀────│ ROS 2 (rmw_zenoh_cpp)│
│  zenoh-pico      │     │ :7447       │     │  ros2 topic pub/echo │
│  (Zenoh client)  │     │ (router)    │     │  (Zenoh client)      │
└──────────────────┘     └─────────────┘     └──────────────────────┘
```

**Test procedure:**

1. Start zenohd (version-matched to `ros-humble-zenoh-cpp-vendor` 0.1.8):
   ```bash
   ~/repos/nano-ros/build/zenohd/zenohd --listen tcp/127.0.0.1:7447
   ```

2. Start sentinel:
   ```bash
   cd src/autoware_sentinel_linux && RUST_LOG=info cargo run
   ```

3. From a ROS 2 terminal:
   ```bash
   source /opt/ros/humble/setup.bash
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'

   # Publish velocity → sentinel should log receipt
   ros2 topic pub /vehicle/status/velocity_status \
     autoware_vehicle_msgs/msg/VelocityReport '{...}'

   # Echo sentinel output → should see Control messages
   ros2 topic echo /control/command/control_cmd
   ```

**CDR protocol details:**

- Key expression format (Humble): `<domain_id>/<topic_name>/<type_name>/TypeHashNotSupported`
- CDR format: Little-endian with 4-byte encapsulation header `[0x00, 0x01, 0x00, 0x00]`
- RMW attachment: 33 bytes (sequence_number, timestamp, gid)

**Key constraint:** Use locally-built zenohd v1.6.2 (`~/repos/nano-ros/build/zenohd/zenohd`)
to match the version pinned by `ros-humble-zenoh-cpp-vendor` 0.1.8. The system-installed
zenohd 1.7.2 may have protocol incompatibilities.

**Acceptance criteria:**

- [ ] Sentinel receives `VelocityReport` published from `ros2 topic pub`
- [ ] `ros2 topic echo` receives `Control` messages published by sentinel
- [ ] No deserialization errors in sentinel logs
- [ ] No deserialization errors in ROS 2 terminal
- [ ] Message round-trip latency < 10 ms on localhost

### 7.3 — Autoware planning simulator integration

- [ ] Autoware launches with `rmw_zenoh_cpp`
- [ ] Replaced nodes disabled
- [ ] Sentinel drives simulated vehicle through autonomous mission
- [ ] Integration test script created

Full end-to-end test: launch the Autoware planning simulator, disable the 7 replaced nodes,
start the sentinel, and run an autonomous driving mission.

**Test procedure:**

1. Start zenohd:
   ```bash
   ~/repos/nano-ros/build/zenohd/zenohd --listen tcp/0.0.0.0:7447
   ```

2. Launch Autoware with `rmw_zenoh_cpp`:
   ```bash
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
   ros2 launch autoware_launch planning_simulator.launch.xml \
     map_path:=/path/to/map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
   ```

3. Disable replaced nodes (choose one approach):

   **Approach A (recommended) — Edit launch record before replay:**
   ```bash
   play_launch dump launch autoware_launch planning_simulator.launch.xml \
     map_path:=/path/to/map --output full_record.json

   jq '
     .node |= [.[] | select(.name != "mrm_handler")] |
     .container |= [.[] | select(
       .name != "mrm_emergency_stop_operator_container" and
       .name != "mrm_comfortable_stop_operator_container")]
   ' full_record.json > filtered_record.json

   play_launch replay --input-file filtered_record.json
   ```

   **Approach B — Stop nodes post-launch via Web API:**
   ```bash
   curl -X POST http://127.0.0.1:8080/api/nodes/mrm_handler/stop
   curl -X POST http://127.0.0.1:8080/api/nodes/vehicle_cmd_gate/stop
   # ... repeat for all 7 nodes
   ```

   **Approach C — Lifecycle shutdown:**
   ```bash
   ros2 lifecycle set /system/mrm_handler shutdown
   # ... for lifecycle-managed nodes
   ```

4. Start sentinel:
   ```bash
   just run-sentinel-linux
   ```

5. Run autonomous drive test:
   - Set initial pose in RViz (or programmatically)
   - Set goal destination
   - Engage autonomous mode
   - Observe vehicle following planned trajectory

**Integration test script (`scripts/integration_test.sh`):**

Automates the above procedure with process management and cleanup:
- Starts zenohd, Autoware, and sentinel as background processes
- Waits for initialization (node discovery or timeout)
- Disables replaced nodes
- Runs autonomous driving sequence
- Traps signals for cleanup (kills background processes on exit)
- Reports pass/fail based on observable criteria

Reference: `~/repos/play_launch/tests/fixtures/autoware/scripts/test_autonomous_drive.py`

**Key interactions with the planning simulator:**

The `simple_planning_simulator` node (`/simulation/autoware_simple_planning_simulator_node`):
- Subscribes to: `/control/command/control_cmd`, `/control/command/gear_cmd`,
  `/control/command/turn_indicators_cmd`, `/control/command/hazard_lights_cmd`
- Publishes: `/vehicle/status/velocity_status`, `/vehicle/status/gear_status`,
  `/vehicle/status/steering_status`, `/vehicle/status/control_mode`

This creates a closed loop: sentinel publishes commands → simulator updates vehicle state →
sentinel receives updated velocity → sentinel gates next command.

**Acceptance criteria:**

- [ ] Autoware planning simulator launches successfully with `rmw_zenoh_cpp`
- [ ] All 7 replaced nodes successfully disabled without affecting other nodes
- [ ] Sentinel receives `VelocityReport` at simulator's publish rate
- [ ] Sentinel publishes `Control` at 30 Hz
- [ ] Planning simulator vehicle follows the planned trajectory
- [ ] No MRM triggered during normal operation (heartbeat healthy)
- [ ] `scripts/integration_test.sh` automates the full test sequence
- [ ] Test completes within 5 minutes (including Autoware startup)

### 7.4 — Zephyr native_sim integration (optional/stretch)

- [ ] TAP networking configured
- [ ] Zephyr `native_sim` binary exchanges messages with Autoware

Run the actual Zephyr `native_sim` binary (from Phase 6) against the Autoware planning
simulator via TAP networking. This proves the real Zephyr build works end-to-end, not just
the Linux adaptation.

**TAP network topology:**

| Interface | IP Address | Role |
|-----------|------------|------|
| `zeth-br` (bridge) | 192.0.2.2/24 | Host (zenohd, ROS 2) |
| `zeth0` (TAP) | — | Zephyr application |
| Zephyr app | 192.0.2.1 | Safety island |

**Setup:**
```bash
# Create TAP interfaces (requires sudo)
sudo ~/repos/nano-ros/scripts/zephyr/setup-network.sh

# Start zenohd on bridge interface (accessible from TAP)
~/repos/nano-ros/build/zenohd/zenohd --listen tcp/0.0.0.0:7447

# Build and run Zephyr sentinel
source ../autoware-sentinel-workspace/env.sh
cd ../autoware-sentinel-workspace
west build -b native_sim/native/64 autoware-sentinel/src/autoware_sentinel
west build -t run
```

The Zephyr sentinel's `prj.conf` has `CONFIG_NROS_ZENOH_LOCATOR="tcp/192.0.2.2:7447"`, which
points to the bridge interface where zenohd listens.

**Note:** The Zephyr build uses Autoware Sentinel topic names (`/output/vehicle/control_cmd`
etc.), not the Autoware-compatible names from 7.1. A topic remapping layer or Zenoh plugin
would be needed for full simulator integration — or the Zephyr `prj.conf` / `lib.rs` could
be updated with configurable topic names.

**Acceptance criteria:**

- [ ] TAP networking setup succeeds (`zeth-br`, `zeth0`)
- [ ] Zephyr `native_sim` binary connects to zenohd on bridge interface
- [ ] Bidirectional message flow between Zephyr app and `ros2 topic pub/echo`
- [ ] Teardown script cleans up TAP interfaces

## Acceptance Criteria (Phase-Level)

- [ ] Linux sentinel binary builds and runs against Autoware planning simulator
- [ ] All 7 replaced system/control nodes successfully supplanted
- [ ] Vehicle completes autonomous driving mission with sentinel in the loop
- [ ] Transport compatibility proven (nros zenoh-pico ↔ rmw_zenoh_cpp ↔ zenohd)
- [ ] Engagement flow works (OperationModeState + ChangeOperationMode)
- [ ] Integration test script automates the full test sequence
- [ ] `just ci` still passes (no regressions in algorithm crate tests)

## References

- `src/autoware_sentinel/src/lib.rs` — SafetyIsland struct and Zephyr wiring (copy for Linux)
- `src/autoware_sentinel/Cargo.toml` — dependency template
- `src/autoware_sentinel/.cargo/config.toml` — nros patch entries to replicate
- `~/repos/nano-ros/examples/native/rust/zenoh/talker/src/main.rs` — Linux nros pattern
- `~/repos/nano-ros/build/zenohd/zenohd` — version-matched zenohd (v1.6.2)
- `~/repos/nano-ros/packages/testing/nros-tests/tests/rmw_interop.rs` — ROS 2 interop tests
- `~/repos/play_launch/tests/fixtures/autoware/scripts/test_autonomous_drive.py` — autonomous
  driving test sequence
- `docs/roadmap/phase-6-zephyr-application.md` — Zephyr application (predecessor phase)
