# Phase 1: Foundation

**Timeline:** Weeks 1–2
**Goal:** Generate Autoware message types for nano-ros and port three trivial packages to
validate the end-to-end pipeline (message codegen → Rust crate → pub/sub wiring).

## Description

Every Tier 1 porting candidate depends on Autoware-specific message types that do not yet
exist as nano-ros crates. This phase unblocks all subsequent work by generating those crates
via `cargo nano-ros generate`, then ports three small, stateless packages to prove the
pipeline works and to establish project conventions (directory layout, Cargo workspace
structure, CI, testing against a ROS 2 peer).

The three packages were chosen because they are tiny (50–163 LOC of core logic), have zero
external dependencies, use only pub/sub + timer patterns, and each exercises a distinct
concern: threshold filtering, format conversion, and state machine.

## Work Items

### 1.1 — Generate Autoware message crates

Create `src/messages/` workspace with generated crates for:

| ROS 2 Package | Key Types |
|----------------|-----------|
| `autoware_control_msgs` | `Control`, `Lateral`, `Longitudinal` |
| `tier4_system_msgs` | `OperateMrm` (srv), `MrmBehaviorStatus`, `OperationModeAvailability` |
| `autoware_vehicle_msgs` | `VelocityReport`, `GearCommand`, `ControlModeReport`, `HazardLightsCommand`, `TurnIndicatorsCommand`, `SteeringReport` |
| `autoware_adapi_v1_msgs` | `MrmState`, `OperationModeState`, `Heartbeat` |
| `nav_msgs` | `Odometry` |
| `geometry_msgs` | `TwistWithCovarianceStamped`, `TwistWithCovariance`, `Twist`, `Vector3` |
| `autoware_common_msgs` | `ResponseStatus` (dependency of `OperateMrm`) |

Transitive dependencies (`builtin_interfaces`, `std_msgs`) will be pulled in automatically.

### 1.2 — Port `autoware_stop_filter`

**Source:** `autoware-repo/src/core/autoware_core/localization/autoware_stop_filter/`
**Target:** `src/autoware_stop_filter/`

Dual-threshold velocity filter. Determines whether the vehicle is stopped and zeros all
twist components when it is.

| Interface | Direction | Type |
|-----------|-----------|------|
| `/input/odom` | Sub | `nav_msgs/Odometry` |
| `/output/odom` | Pub | `nav_msgs/Odometry` |

Core algorithm:
```
is_stopped = |vx| < vx_thresh AND |wz| < wz_thresh
if is_stopped: zero all twist components
```

### 1.3 — Port `autoware_vehicle_velocity_converter`

**Source:** `autoware-repo/src/core/autoware_core/sensing/autoware_vehicle_velocity_converter/`
**Target:** `src/autoware_vehicle_velocity_converter/`

Stateless format translation from vehicle CAN velocity report to standard twist message
with covariance matrix assignment.

| Interface | Direction | Type |
|-----------|-----------|------|
| `/vehicle/status/velocity_status` | Sub | `autoware_vehicle_msgs/VelocityReport` |
| `/vehicle/status/twist_with_covariance` | Pub | `geometry_msgs/TwistWithCovarianceStamped` |

### 1.4 — Port `autoware_shift_decider`

**Source:** `autoware-repo/src/universe/autoware_universe/control/autoware_shift_decider/`
**Target:** `src/autoware_shift_decider/`

State machine that decides gear (DRIVE/REVERSE/PARK/NEUTRAL) based on the velocity sign in
the current control command. Prevents gear-hunting.

| Interface | Direction | Type |
|-----------|-----------|------|
| `/control/command/control_cmd` | Sub | `autoware_control_msgs/Control` |
| `/control/command/gear_cmd` | Pub | `autoware_vehicle_msgs/GearCommand` |

## Acceptance Criteria

- [ ] All message crates compile with `no_std` (no heap allocation in message structs).
- [ ] Message crates pass CDR round-trip tests: serialize → deserialize → field equality.
- [ ] Each ported package compiles as a standalone `no_std` crate with the generated messages.
- [ ] Each ported package has unit tests covering the core algorithm (stop detection
      thresholds, covariance assignment, gear transitions).
- [ ] At least one package is integration-tested against a ROS 2 peer node via zenoh
      (publish from nano-ros, subscribe on ROS 2 side, verify message contents).
- [ ] Cargo workspace at project root includes all crates.
- [ ] CI runs `cargo build --target thumbv7em-none-eabihf` and `cargo test` (native).
