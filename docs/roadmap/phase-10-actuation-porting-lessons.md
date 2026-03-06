# Phase 10: Actuation Porting & Infrastructure Improvements

**Status:** In progress (10.1, 10.2 complete)
**Depends on:** Phase 8 (topic parity), Phase 6 (Zephyr application)
**Goal:** Port Autoware's trajectory follower algorithms (PID longitudinal + MPC lateral
controllers) from ARM's actuation_porting project into Rust `#![no_std]` crates, and improve
the sentinel's Zephyr deployment infrastructure (clock, parameters, board overlays).

## Background

ARM's [actuation_porting](https://github.com/oguzkaganozt/actuation_porting) project ports
Autoware's trajectory follower (MPC lateral + PID longitudinal controllers) to Zephyr RTOS
using CycloneDDS — no ROS 2 runtime. Key differences from our approach:

| Aspect         | actuation_porting (ARM)        | autoware-sentinel (ours)                    |
|----------------|--------------------------------|---------------------------------------------|
| Language       | C++ (Autoware code recompiled) | Rust (`#![no_std]`, rewritten)              |
| Middleware     | CycloneDDS (raw DDS)           | nano-ros + Zenoh                            |
| Ported scope   | Control (MPC+PID)              | Safety monitoring (MRM, gate, validators)   |
| Zephyr version | 3.6.0                          | 3.7.0                                       |
| Target HW      | ARM Cortex-R52 (NXP S32Z), FVP | ARM Cortex-M (thumbv7em)                    |
| Message gen    | IDL → C via `idlc`             | `.msg` → Rust via `cargo nano-ros generate` |

The projects are **complementary**: their actuation module handles the control loop, our
sentinel handles safety monitoring. Together they cover both sides of a safety island.

### What they do well

1. **SNTP clock synchronization** — wall-clock time at startup for ROS-compatible timestamps
2. **Embedded DDS buffer tuning** — 8KB receive, 2KB chunks, 1400B max message (MTU-friendly)
3. **Rosbag-based output comparison** — record DDS outputs, diff against Autoware baselines
4. **Board-specific Zephyr overlays** — clean separation of board configs (FVP, NXP S32Z)
5. **Complete control pipeline** — MPC + PID + interpolation + motion utils in one binary

### Where our approach is stronger

1. **Type/memory safety** — Rust `#![no_std]` with `heapless` containers vs C++ raw pointers
2. **Formal verification** — 13 Kani harnesses + Verus proofs (they have none)
3. **Integration testing** — nextest + ManagedProcess + ephemeral ports vs manual DDS pub/sub
4. **Zenoh transport** — lighter than CycloneDDS for embedded, native rmw_zenoh_cpp bridge
5. **Topic name compatibility** — transparent via rmw_zenoh_cpp (they manually prepend `"rt/"`)
6. **Parameter system** — nano-ros has full `ParameterServer` with typed accessors, descriptors,
   range constraints, read-only enforcement, and ROS 2 parameter services (`ros2 param set/get`)

### Algorithm inventory from actuation_porting

| Component                            | C++ LoC    | Complexity | no_std Feasibility | Dependencies  |
|--------------------------------------|------------|------------|--------------------|---------------|
| autoware_pid_longitudinal_controller | 2,458      | Medium     | **High**           | Pure math     |
| autoware_mpc_lateral_controller      | 4,909      | High       | **Medium**         | Eigen, OSQP   |
| autoware_osqp_interface              | 870        | Medium     | **Medium**         | OSQP C lib    |
| autoware_interpolation               | 1,033      | Low        | **High**           | Eigen (basic) |
| autoware_motion_utils                | 714        | Low        | **High**           | Geometry only |
| autoware_universe_utils              | 2,007      | Low-Med    | **High**           | Geometry/math |
| autoware_vehicle_info_utils          | 237        | Trivial    | **High**           | Parameters    |
| autoware_trajectory_follower_base    | 285        | Trivial    | **High**           | Trait defs    |
| autoware_trajectory_follower_node    | 535        | Low        | **High**           | Node wiring   |
| **Total**                            | **13,048** |            |                    |               |

## Subphases

### 10.1 — Foundation math libraries ✓

Port the supporting math/geometry libraries that both controllers depend on.

**Status:** Complete — 4 crates, 40 tests, all cross-compile to thumbv7em-none-eabihf.

#### 10.1a — Interpolation library (`src/autoware_interpolation/`) ✓

- [x] Linear interpolation — `lerp()`, `lerp_single()`, `lerp_multi()`
- [x] Spline interpolation — `SplineInterpolation` with Thomas algorithm tridiagonal solver
  - Fixed-size arrays `[f64; MAX_POINTS]` where MAX_POINTS=256
  - `eval()`, `eval_diff()`, `eval_quad_diff()`, `eval_multi()`
- [x] Akima spline interpolation — `AkimaInterpolation` (weighted slope, reduces oscillations)
- [x] Spherical linear interpolation (SLERP) — `slerp()`, `slerp_single()` (pure Rust, no Eigen)
- [x] 2D trajectory spline — `SplineInterpolationPoints2d` with arc-length parameterization
  - `interpolated_point()`, `interpolated_yaw()`, `interpolated_curvature()`
- [x] 14 unit tests

#### 10.1b — Motion utilities (`src/autoware_motion_utils/`) ✓

- [x] `TrajectoryAccessor` trait — replaces C++ template `<class T>` pattern
- [x] `find_nearest_index()` — nearest point by 2D squared distance
- [x] `find_nearest_index_with_constraints()` — distance + yaw constraints
- [x] `find_first_nearest_index_with_soft_constraints()` — cascading 3-pass search
- [x] `find_first_nearest_segment_index_with_soft_constraints()` — segment-level search
- [x] `calc_signed_arc_length()`, `calc_signed_arc_length_between_points()`
- [x] `calc_longitudinal_offset_to_segment()` — dot product projection
- [x] `calc_curvature()` — Menger curvature at each point
- [x] `search_zero_velocity_index()` — find stopping points
- [x] `is_driving_forward()` — determine driving direction
- [x] 8 unit tests

#### 10.1c — Universe utilities (`src/autoware_universe_utils/`) ✓

- [x] Constants: `GRAVITY`
- [x] Unit conversion: `deg2rad`, `rad2deg`, `kmph2mps`, `mps2kmph`
- [x] Angle normalization: `normalize_radian`, `normalize_degree`
- [x] Distance: `calc_distance_2d`, `calc_squared_distance_2d`, `calc_distance_3d`
- [x] Quaternion: `create_quaternion`, `get_yaw`, `create_quaternion_from_yaw`,
  `create_quaternion_from_rpy`, `get_rpy`
- [x] Pose deviation: `calc_yaw_deviation`, `calc_lateral_deviation`, `calc_longitudinal_deviation`
- [x] Curvature: `calc_curvature` (Menger formula)
- [x] Interpolation: `lerp_point`, `lerp_vector3`
- [x] Direction: `calc_azimuth_angle`, `calc_elevation_angle`, `is_driving_forward`
- [x] 13 unit tests

#### 10.1d — Vehicle info (`src/autoware_vehicle_info_utils/`) ✓

- [x] `VehicleInfo` struct: 10 base params + 8 derived params
- [x] `new()` constructor with clamping (wheel_base, max_steer_angle ≥ 1e-6)
- [x] Default values from Autoware's sample vehicle config
- [x] 5 unit tests

### 10.2 — PID longitudinal controller (`src/autoware_pid_longitudinal_controller/`) ✓

Port from `external/actuation_porting/actuation_module/src/autoware/autoware_pid_longitudinal_controller/`.
This is the highest-value port — pure math, no external C library dependencies.

**Status:** Complete — 37 tests, cross-compiles to thumbv7em-none-eabihf.

- [x] 10.2a — PID controller core (`src/pid.rs`)
  - Standard PID with anti-windup (integral clamping) and per-component limits
  - `PidGains`, `PidLimits`, `PidContributions` structs
  - 8 unit tests

- [x] 10.2b — Smooth stop algorithm (`src/smooth_stop.rs`)
  - Ring buffer velocity history (64 samples)
  - Linear regression time-to-stop estimation
  - Kinematic deceleration: `a = -v²/(2d)` with clamping
  - Weak/strong braking phases based on distance and velocity
  - 6 unit tests

- [x] 10.2c — Lowpass filter (`src/lowpass_filter.rs`)
  - First-order IIR: `y[n] = gain*y[n-1] + (1-gain)*x[n]`
  - 4 unit tests

- [x] 10.2d — Longitudinal controller state machine (`src/lib.rs`)
  - States: DRIVE → STOPPING → STOPPED → EMERGENCY
  - DRIVE: PID velocity feedback + feedforward scaling + brake keeping
  - STOPPING: SmoothStop algorithm with velocity history
  - STOPPED: Configurable braking force (-3.4 m/s²)
  - EMERGENCY: Hard braking (-5.0 m/s²) with jerk limit
  - Slope compensation (pitch-based gravity compensation, configurable source)
  - Jerk limiting (asymmetric rate limiter on acceleration output)
  - Integration gating (velocity threshold + stuck detection)
  - Steer convergence check (blocks departure until lateral converged)
  - 30+ configurable parameters via `ControllerParams`

- [x] 10.2e — Unit tests (19 controller-level tests)
  - State machine transitions (Drive→Stopping→Stopped, Emergency→Stopped)
  - Emergency on overshoot
  - Steer convergence blocking
  - Slope compensation (uphill, reverse, pitch clamping)
  - Jerk limit enforcement
  - Stop distance calculation
  - Trajectory pitch computation

- [ ] 10.2f — Kani verification harnesses (future)

**Message dependencies:** `autoware_control_msgs`, `autoware_planning_msgs`,
`autoware_vehicle_msgs`, `geometry_msgs`, `nav_msgs`

### 10.3 — Trajectory follower base traits (`src/autoware_trajectory_follower_base/`)

Port the trait definitions that both controllers implement.

- [ ] `LateralControllerBase` trait with `fn run(&mut self, input: &InputData) -> LateralOutput`
- [ ] `LongitudinalControllerBase` trait with `fn run(&mut self, input: &InputData) -> LongitudinalOutput`
- [ ] `InputData` struct: trajectory, odometry, steering, acceleration, operation mode
- [ ] `LateralOutput`: steering angle, steering rate, sync data
- [ ] `LongitudinalOutput`: velocity, acceleration, jerk, sync data
- [ ] `SyncData`: lateral-to-longitudinal and vice-versa data exchange

**Source reference:**
- `autoware_trajectory_follower_base/src/lateral_controller_base.cpp`
- `autoware_trajectory_follower_base/src/longitudinal_controller_base.cpp`
- ~285 LoC total

### 10.4 — MPC lateral controller (`src/autoware_mpc_lateral_controller/`)

Port from `external/actuation_porting/actuation_module/src/autoware/autoware_mpc_lateral_controller/`.
This is the most complex component due to matrix operations and QP solver dependency.

- [ ] 10.4a — Vehicle models
  - `BicycleKinematics` — simplified steering model (most commonly used)
  - `BicycleDynamics` — physics-based with tire slip
  - `BicycleKinematicsNoDelay` — simplified without steering delay
  - State vector: [lateral_pos, yaw, velocity, steering_angle]
  - Discrete-time linearization around reference trajectory
  - Eigen matrix operations → use fixed-size arrays or `nalgebra` (behind feature flag)
  - Source: `vehicle_model/` headers

- [ ] 10.4b — MPC core solver
  - Trajectory filtering and resampling
  - State linearization around reference path
  - Cost matrices Q (tracking error), R (steering effort) generation
  - Prediction horizon: 5-10 steps, configurable
  - QP problem formulation: `minimize u'Hu + f'u subject to lb <= u <= ub`
  - Source: `mpc.cpp` (~877 LoC)

- [ ] 10.4c — QP solver interface
  - Trait: `QpSolverInterface` with `fn solve(H, f, A, lb, ub) -> Solution`
  - Option A: **Unconstrained solver** (already exists in ARM code as fallback) — simplest,
    solves `Hx = -f` directly, no inequality constraints. Suitable for safety island where
    steering limits are enforced downstream.
  - Option B: **OSQP FFI** — wrap the OSQP C library via `cc` crate. Full constrained QP.
    Adds C dependency but proven solver.
  - Option C: **Pure Rust QP** — use `osqp-rust` or implement simplified active-set solver.
    Avoids C dependency but limited ecosystem for `no_std`.
  - **Recommendation:** Start with unconstrained solver (Option A) for initial port, add
    OSQP FFI (Option B) behind a feature flag for full fidelity.
  - Source: `osqp_interface.cpp` (~870 LoC)

- [ ] 10.4d — Steering predictor and offset estimator
  - First-order steering dynamics prediction
  - Self-calibration for steering bias
  - Butterworth lowpass filters
  - Source: `steering_predictor.cpp`, `steering_offset_estimator.hpp`

- [ ] 10.4e — MPC lateral controller orchestration
  - Trajectory preprocessing (smoothing, curvature calculation)
  - Control loop: linearize → build QP → solve → filter output
  - 30+ configurable parameters
  - Source: `mpc_lateral_controller.cpp` (~900 LoC)

- [ ] 10.4f — Unit tests and verification
  - Steering angle tracking convergence
  - Lateral deviation bounded
  - Vehicle model consistency (kinematics vs dynamics)
  - QP solver correctness (known-solution inputs)
  - Kani: bounded steering output, panic freedom

**Matrix operation strategy:**

The MPC controller uses Eigen3 for matrix operations (state: 4-6D, prediction matrices up to
~50x50). Options for `no_std` Rust:

1. **Fixed-size arrays** — manual matrix multiply for small dimensions. Works for vehicle
   model (4x4), awkward for prediction horizon matrices.
2. **`nalgebra`** — mature Rust linear algebra library. Supports `no_std` with `alloc` or
   fixed-size matrices (`SMatrix`). Best option for prediction horizon matrices.
3. **Custom fixed-horizon solver** — if prediction horizon is compile-time constant (e.g., 10),
   all matrices are fixed-size and can use stack allocation.

**Recommendation:** Use `nalgebra` with `no_std` + fixed-size `SMatrix<f64, N, N>` where
N = prediction horizon (compile-time const generic). This avoids heap allocation while
supporting the required matrix operations.

### 10.5 — Controller node wiring (`src/autoware_trajectory_follower_node/`)

Wire the lateral and longitudinal controllers into a nano-ros node binary.

- [ ] 10.5a — Controller node algorithm
  - Subscribe to 5 input topics (trajectory, odometry, steering, acceleration, operation mode)
  - Create `InputData` from subscribed messages
  - Run lateral controller → `LateralOutput`
  - Run longitudinal controller → `LongitudinalOutput`
  - Sync controllers (lateral↔longitudinal data exchange)
  - Publish combined `Control` command
  - Timer at configurable period (30ms for Linux, 150ms for embedded)
  - Source: `controller_node.cpp` (~535 LoC)

- [ ] 10.5b — Integration into sentinel binary
  - Add controller node to `src/autoware_sentinel_linux/src/main.rs`
  - Wire subscriptions and publisher into existing executor
  - Update `ZPICO_MAX_PUBLISHERS`, `ZPICO_MAX_SUBSCRIBERS`, `ZPICO_MAX_LIVELINESS`

- [ ] 10.5c — Integration tests
  - New test in `tests/tests/` verifying controller output
  - Publish known trajectory + odometry, verify control command output
  - Compare against ARM project's expected outputs in
    `external/actuation_porting/actuation_module/test/rosbag_test/test_control_cmd_outputs/`

### 10.6 — Sentinel parameter migration

nano-ros already has a full `ParameterServer` with typed accessors, descriptors, range
constraints, read-only enforcement, and ROS 2 parameter services (`~/get_parameters`,
`~/set_parameters`, etc.) gated by the `param-services` feature. Use this existing API
to make sentinel algorithm parameters configurable.

- [ ] 10.6a — Migrate existing sentinel constants to parameters
  - Replace hardcoded constants in each algorithm crate with parameter lookups
  - Use `ParameterBuilder` API with `.default()`, `.description()`, `.read_only()`
  - Parameters declared at startup with same values as current constants
  - Key constants per algorithm:

  | Algorithm | Constants to migrate |
  |-----------|---------------------|
  | stop_filter | `VX_THRESHOLD`, `WZ_THRESHOLD` |
  | shift_decider | `VEL_THRESHOLD`, `park_on_goal` |
  | emergency_stop_operator | `TARGET_ACCEL`, `TARGET_JERK` |
  | comfortable_stop_operator | `MIN_ACCEL`, jerk limits |
  | mrm_handler | heartbeat timeouts, `use_comfortable_stop` |
  | vehicle_cmd_gate | `VEL_LIM`, accel/jerk limits |
  | control_validator | distance/velocity/accel thresholds |
  | operation_mode_transition_manager | transition thresholds |

- [ ] 10.6b — Add new controller parameters
  - PID gains (Kp, Ki, Kd) and limits for longitudinal controller
  - MPC horizon, weights, vehicle model selection for lateral controller
  - Control period, delay compensation time
  - All declared as read-only via `ParameterBuilder::read_only()`

- [ ] 10.6c — Parameter equivalence audit (overlaps with Phase 9.4)
  - Extract Autoware default parameter values from launch YAML files
  - Compare against sentinel parameter defaults
  - Fix any mismatches
  - Sources:
    - `autoware-repo/src/universe/autoware_universe/control/autoware_trajectory_follower_node/`
    - `autoware-repo/src/universe/autoware_universe/control/autoware_pid_longitudinal_controller/`
    - `autoware-repo/src/universe/autoware_universe/control/autoware_mpc_lateral_controller/`

- [ ] 10.6d — Verify `ros2 param list` / `ros2 param get` works
  - Enable `param-services` feature in sentinel Linux binary
  - Verify all parameters visible via `ros2 param list /sentinel`
  - Verify `ros2 param get /sentinel <name>` returns correct values
  - Add integration test

### 10.7 — Clock synchronization for Zephyr sentinel

The ARM project initializes wall-clock time via SNTP at startup (`Clock::init_clock_via_sntp()`)
so that published ROS timestamps are meaningful. Our Zephyr sentinel currently uses monotonic
time only.

- [ ] 10.7a — Add SNTP client to Zephyr sentinel startup
  - Use Zephyr's `net_sntp` subsystem (`CONFIG_SNTP=y`)
  - Configure NTP server via Kconfig (default: configurable)
  - Set system clock once at boot, before executor starts
  - Fallback: if SNTP fails, log warning and continue with monotonic time

- [ ] 10.7b — ROS timestamp generation from wall clock
  - `Clock::now()` returns `builtin_interfaces::msg::Time` with sec + nanosec
  - Use Zephyr `k_uptime_get()` + SNTP offset for wall-clock conversion
  - Ensure nanosecond precision (Zephyr kernel tick → ns conversion)

- [ ] 10.7c — Verify timestamp compatibility
  - Publish a stamped message from sentinel, echo with `ros2 topic echo`
  - Confirm timestamps are within 1s of ROS 2 node timestamps
  - Add integration test

### 10.8 — Board overlay infrastructure for real hardware

The ARM project has clean board overlay files for FVP and NXP S32Z. Our Zephyr application
only targets `native_sim`.

- [ ] 10.8a — Survey target boards
  - Primary: STM32H7 (Cortex-M7, Ethernet, 1MB+ RAM) — best match for safety MCU
  - Secondary: NXP S32Z (Cortex-R52) — same target as ARM project, direct comparison
  - Tertiary: nRF5340 (Cortex-M33, BLE/Thread) — constrained IoT use case
  - Document minimum requirements: RAM, flash, networking, timer resolution

- [ ] 10.8b — Create board overlay for primary target
  - `src/autoware_sentinel/boards/<board>.overlay` — device tree
  - `src/autoware_sentinel/boards/<board>.conf` — Kconfig
  - Enable Ethernet, configure network stack sizes
  - Test: `west build -b <board> autoware-sentinel/src/autoware_sentinel`

- [ ] 10.8c — CI cross-compilation check
  - Add `just build-zephyr-<board>` recipe
  - Verify build succeeds in CI (no runtime test needed initially)

### 10.9 — Rosbag-based output comparison (overlaps with Phase 9.1)

The ARM project records DDS outputs and diffs against known-good Autoware baselines.

- [ ] 10.9a — Study ARM project's rosbag test format
  - Their `test/rosbag_test/` has YAML-serialized control commands
  - `test_control_cmd_outputs/` contains expected outputs for FVP, S32Z, and original Autoware
  - Adopt similar YAML-based expected output format for our comparison tooling

- [ ] 10.9b — Implement comparison in Phase 9 scripts
  - Integrate YAML baseline format into `scripts/compare_bags.py` (from Phase 9.1c)
  - Per-field tolerance configuration (float epsilon, timestamp tolerance)
  - Human-readable diff output

## Implementation Notes

### Priority ordering

1. **10.1** (foundation math) — prerequisite for both controllers
2. **10.2** (PID controller) — highest value, lowest risk, no C dependencies
3. **10.3** (base traits) — simple, needed by both controllers
4. **10.6** (parameters) — can be done in parallel with algorithm ports
5. **10.4** (MPC controller) — highest complexity, depends on QP solver decision
6. **10.5** (node wiring) — depends on both controllers
7. **10.7–10.9** (infra) — independent, can be done in parallel

### Porting approach

Following the established two-layer architecture:

1. **Algorithm library** (`src/autoware_*/src/lib.rs`): Pure `#![no_std]` logic. No nros
   dependency. Message types only. Fully testable with `cargo test`.
2. **Node binary** (in sentinel main): Uses nros `Executor`/`Node` to wire pub/sub/timer
   callbacks to algorithm instances.

### Eigen → Rust matrix strategy

| C++ (Eigen) | Rust equivalent | Notes |
|-------------|----------------|-------|
| `VectorXd` | `heapless::Vec<f64, N>` or `nalgebra::SVector<f64, N>` | Fixed-size |
| `MatrixXd` | `nalgebra::SMatrix<f64, R, C>` | Fixed rows/cols |
| `colPivHouseholderQr().solve()` | `nalgebra::SVD::solve()` or custom tridiag solver | For splines |
| `A * B` | `nalgebra` operator overloads | Same syntax |
| `A.transpose()` | `.transpose()` | Same API |

### OSQP decision

Start with unconstrained QP solver (Option A) for the initial port. This avoids the C
dependency entirely and is sufficient for a safety island where hard steering limits are
enforced by the vehicle_cmd_gate downstream. Add OSQP FFI behind a feature flag later
if constrained optimization is needed.

### Relationship to nano-ros changes

Transport tuning documentation is tracked in nano-ros Phase 64 (complete: `docs/guides/embedded-tuning.md`).
No new nano-ros features are needed — the existing `ParameterServer` API is sufficient.

## Acceptance Criteria

- [ ] Interpolation, motion utils, and vehicle info crates compile for `thumbv7em-none-eabihf`
- [ ] PID longitudinal controller passes unit tests and Kani verification
- [ ] MPC lateral controller compiles with unconstrained solver (constrained solver optional)
- [ ] Controller node publishes `Control` commands from trajectory + odometry inputs
- [ ] All existing sentinel constants migrated to `ParameterServer` declarations
- [ ] `ros2 param list /sentinel` shows all parameters (Linux binary with `param-services`)
- [ ] Sentinel publishes wall-clock timestamps (SNTP or fallback monotonic)
- [ ] At least one real hardware board overlay compiles
- [ ] Control command outputs validated against ARM project baselines
- [ ] All new crates are `#![no_std]` and pass `just cross-check`
- [ ] No regressions in existing tests

## References

- ARM actuation_porting: `external/actuation_porting/`
- ARM PID controller: `external/actuation_porting/actuation_module/src/autoware/autoware_pid_longitudinal_controller/`
- ARM MPC controller: `external/actuation_porting/actuation_module/src/autoware/autoware_mpc_lateral_controller/`
- ARM controller node: `external/actuation_porting/actuation_module/src/autoware/autoware_trajectory_follower_node/`
- ARM DDS config: `external/actuation_porting/actuation_module/include/common/dds/config.hpp`
- ARM clock: `external/actuation_porting/actuation_module/include/common/clock/clock.hpp`
- ARM rosbag tests: `external/actuation_porting/actuation_module/test/rosbag_test/`
- nano-ros parameter API: `~/repos/nano-ros/packages/core/nros-params/`
- nano-ros parameter services: `~/repos/nano-ros/packages/core/nros-node/src/parameter_services.rs`
- nano-ros embedded tuning guide: `~/repos/nano-ros/docs/guides/embedded-tuning.md`
- Autoware trajectory follower: `autoware-repo/src/universe/autoware_universe/control/`
