# Phase 10: Improvements from actuation_porting Analysis

**Status:** Not started
**Depends on:** Phase 8 (topic parity), Phase 6 (Zephyr application)
**Goal:** Incorporate lessons learned from ARM's actuation_porting project to improve the
sentinel's Zephyr deployment, clock handling, and parameter management.

## Background

ARM's [actuation_porting](https://github.com/oguzkaganozt/actuation_porting) project ports
Autoware's trajectory follower (MPC lateral + PID longitudinal controllers) to Zephyr RTOS
using CycloneDDS — no ROS 2 runtime. Key differences from our approach:

| Aspect | actuation_porting (ARM) | autoware-sentinel (ours) |
|--------|------------------------|--------------------------|
| Language | C++ (Autoware code recompiled) | Rust (`#![no_std]`, rewritten) |
| Middleware | CycloneDDS (raw DDS) | nano-ros + Zenoh |
| Ported scope | Control (MPC+PID) | Safety monitoring (MRM, gate, validators) |
| Zephyr version | 3.6.0 | 3.7.0 |
| Target HW | ARM Cortex-R52 (NXP S32Z), FVP | ARM Cortex-M (thumbv7em) |
| Message gen | IDL → C via `idlc` | `.msg` → Rust via `cargo nano-ros generate` |

The projects are **complementary**: their actuation module handles the control loop, our
sentinel handles safety monitoring. Together they cover both sides of a safety island.

### What they do well

1. **SNTP clock synchronization** — wall-clock time at startup for ROS-compatible timestamps
2. **Embedded DDS buffer tuning** — 8KB receive, 2KB chunks, 1400B max message (MTU-friendly)
3. **In-memory parameter storage** — `declare_parameter<T>()` / `get_parameter<T>()` without RPC
4. **Rosbag-based output comparison** — record DDS outputs, diff against Autoware baselines
5. **Board-specific Zephyr overlays** — clean separation of board configs (FVP, NXP S32Z)

### Where our approach is stronger

1. **Type/memory safety** — Rust `#![no_std]` with `heapless` containers vs C++ raw pointers
2. **Formal verification** — 13 Kani harnesses + Verus proofs (they have none)
3. **Integration testing** — nextest + ManagedProcess + ephemeral ports vs manual DDS pub/sub
4. **Zenoh transport** — lighter than CycloneDDS for embedded, native rmw_zenoh_cpp bridge
5. **Topic name compatibility** — transparent via rmw_zenoh_cpp (they manually prepend `"rt/"`)

## Subphases

### 10.1 — Clock synchronization for Zephyr sentinel

The ARM project initializes wall-clock time via SNTP at startup (`Clock::init_clock_via_sntp()`)
so that published ROS timestamps are meaningful. Our Zephyr sentinel currently uses monotonic
time only, which means timestamps in published messages don't correspond to wall-clock time.

- [ ] 10.1a — Add SNTP client to Zephyr sentinel startup
  - Use Zephyr's `net_sntp` subsystem (`CONFIG_SNTP=y`)
  - Configure NTP server via Kconfig (default: pool.ntp.org or configurable)
  - Set system clock once at boot, before executor starts
  - Fallback: if SNTP fails, log warning and continue with monotonic time

- [ ] 10.1b — ROS timestamp generation from wall clock
  - `Clock::now()` returns `builtin_interfaces::msg::Time` with sec + nanosec
  - Use Zephyr `k_uptime_get()` + SNTP offset for wall-clock conversion
  - Ensure nanosecond precision (Zephyr kernel tick → ns conversion)

- [ ] 10.1c — Verify timestamp compatibility
  - Publish a stamped message from sentinel, echo with `ros2 topic echo`
  - Confirm timestamps are within 1s of ROS 2 node timestamps
  - Add integration test: compare sentinel timestamp vs `ros2 topic echo --field header.stamp`

### 10.2 — Zenoh transport tuning for embedded

The ARM project's CycloneDDS configuration has well-tuned buffer sizes for embedded targets.
We should document and expose equivalent tuning for zenoh-pico in nano-ros.

- [ ] 10.2a — Document zpico compile-time constants
  - Catalog existing `ZPICO_*` environment variables and their defaults
  - Document buffer sizes, queue depths, discovery timeouts
  - Add to nano-ros `docs/guides/embedded-tuning.md`

- [ ] 10.2b — Benchmark sentinel memory usage
  - Measure RSS/heap usage of sentinel Linux binary under load
  - Profile zpico buffer allocation on Zephyr (stack + heap)
  - Compare against ARM project's 1MB heap allocation
  - Document findings and recommended configurations per target

- [ ] 10.2c — MTU-aware message fragmentation
  - ARM project caps at 1400 bytes per message (Ethernet MTU - headers)
  - Verify zenoh-pico handles large messages (e.g., Trajectory with 100 points)
  - Document max message sizes and fragmentation behavior

### 10.3 — Parameter storage for algorithm tuning

The ARM project has a lightweight in-memory parameter system (`std::unordered_map<string, variant>`)
that allows runtime tuning without recompilation. Our sentinel hardcodes all algorithm parameters
as Rust constants. While this is simpler and verifiable, it prevents tuning on deployed hardware.

- [ ] 10.3a — Design parameter API for nano-ros
  - Lightweight `declare_parameter()` / `get_parameter()` on Node
  - Storage: `heapless::FnvIndexMap<&str, ParamValue, N>` (no heap)
  - `ParamValue` enum: `Bool(bool)`, `Int(i64)`, `Float(f64)`, `String(heapless::String<64>)`
  - Default values set at declaration time
  - Read-only after init (no runtime parameter server needed for safety island)
  - This is a nano-ros feature, tracked separately in nano-ros roadmap (Phase 64)

- [ ] 10.3b — Make sentinel algorithm parameters configurable
  - Replace hardcoded constants with parameter lookups in sentinel main
  - Parameters set from defaults at startup (same values as current constants)
  - Future: load from config file or environment variables on Linux
  - Future: load from Zephyr Kconfig or devicetree on embedded

- [ ] 10.3c — Parameter equivalence audit (overlaps with Phase 9.4)
  - Extract Autoware default parameter values from launch YAML files
  - Compare against sentinel hardcoded constants
  - Fix any mismatches

### 10.4 — Board overlay infrastructure for real hardware

The ARM project has clean board overlay files for FVP and NXP S32Z. Our Zephyr application
only targets `native_sim`. Adding real hardware support requires similar overlay infrastructure.

- [ ] 10.4a — Survey target boards
  - Primary: STM32H7 (Cortex-M7, Ethernet, 1MB+ RAM) — best match for safety MCU
  - Secondary: NXP S32Z (Cortex-R52) — same target as ARM project, direct comparison
  - Tertiary: nRF5340 (Cortex-M33, BLE/Thread) — constrained IoT use case
  - Document minimum requirements: RAM, flash, networking, timer resolution

- [ ] 10.4b — Create board overlay for primary target
  - `src/autoware_sentinel/boards/<board>.overlay` — device tree
  - `src/autoware_sentinel/boards/<board>.conf` — Kconfig
  - Enable Ethernet, configure network stack sizes
  - Test: `west build -b <board> autoware-sentinel/src/autoware_sentinel`

- [ ] 10.4c — CI cross-compilation check
  - Add `just build-zephyr-<board>` recipe
  - Verify build succeeds in CI (no runtime test needed initially)

### 10.5 — Rosbag-based output comparison (overlaps with Phase 9.1)

The ARM project records DDS outputs and diffs against known-good Autoware baselines. This
validates functional equivalence. Our Phase 9 plans the same approach — this subphase
adds lessons from their implementation.

- [ ] 10.5a — Study ARM project's rosbag test format
  - Their `test/rosbag_test/` has YAML-serialized control commands
  - `test_control_cmd_outputs/` contains expected outputs for FVP, S32Z, and original Autoware
  - Adopt similar YAML-based expected output format for our comparison tooling

- [ ] 10.5b — Implement comparison in Phase 9 scripts
  - Integrate YAML baseline format into `scripts/compare_bags.py` (from Phase 9.1c)
  - Per-field tolerance configuration (float epsilon, timestamp tolerance)
  - Human-readable diff output

## Implementation Notes

### Priority ordering

10.1 (clock sync) and 10.2 (transport tuning) are prerequisites for real Zephyr deployment.
10.3 (parameters) and 10.4 (board overlays) are needed for hardware bring-up.
10.5 (rosbag comparison) overlaps with Phase 9 and can be done in parallel.

### Relationship to nano-ros changes

Items 10.2a (documentation) and 10.3a (parameter API) require changes in the nano-ros repo.
These are tracked as nano-ros Phase 64. All other items are autoware-sentinel changes.

## Acceptance Criteria

- [ ] Sentinel publishes wall-clock timestamps (SNTP or fallback monotonic)
- [ ] zpico embedded tuning guide exists with recommended configurations
- [ ] Parameter API designed and documented (nano-ros Phase 64)
- [ ] At least one real hardware board overlay compiles
- [ ] Rosbag comparison format adopted from ARM project patterns
- [ ] No regressions in existing tests

## References

- ARM actuation_porting: `external/actuation_porting/`
- ARM DDS config: `external/actuation_porting/actuation_module/include/common/dds/config.hpp`
- ARM clock: `external/actuation_porting/actuation_module/include/common/clock/clock.hpp`
- ARM node: `external/actuation_porting/actuation_module/include/common/node/node.hpp`
- ARM rosbag tests: `external/actuation_porting/actuation_module/test/rosbag_test/`
- nano-ros Phase 64: `~/repos/nano-ros/docs/roadmap/phase-64-parameter-api-embedded-tuning.md`
