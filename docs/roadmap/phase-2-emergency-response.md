# Phase 2: Emergency Response

**Timeline:** Weeks 3–4
**Depends on:** Phase 1 (message crates)
**Goal:** Build the MRM (Minimum Risk Maneuver) chain on the safety island so it can
autonomously bring the vehicle to a safe stop with zero dependency on the main compute.

## Description

This phase ports the three components that form Autoware's emergency response pipeline:
the emergency stop operator (hard braking), the comfortable stop operator (gentle
deceleration), and the MRM handler that orchestrates between them. A heartbeat watchdog
is added to detect main compute failure.

After this phase, the safety island can:
1. Receive a heartbeat from the main Autoware stack via zenoh.
2. On heartbeat timeout (500 ms), trigger the MRM handler.
3. MRM handler activates comfortable stop first, then escalates to emergency stop.
4. Emergency stop operator publishes jerk-limited brake commands until velocity reaches zero.

This is the core value proposition of the safety island.

## Work Items

### 2.1 — Port `autoware_mrm_emergency_stop_operator`

**Source:** `autoware-repo/src/universe/autoware_universe/system/autoware_mrm_emergency_stop_operator/`
**Target:** `src/autoware_mrm_emergency_stop_operator/`

Service-triggered emergency braking with jerk-limited deceleration ramp.

| Interface | Direction | Type |
|-----------|-----------|------|
| `~/input/mrm/emergency_stop/operate` | Srv | `tier4_system_msgs/OperateMrm` |
| `~/input/control/control_cmd` | Sub | `autoware_control_msgs/Control` |
| `~/output/mrm/emergency_stop/status` | Pub | `tier4_system_msgs/MrmBehaviorStatus` |
| `~/output/mrm/emergency_stop/control_cmd` | Pub | `autoware_control_msgs/Control` |

Algorithm (30 Hz):
```
a(t+1) = max(a(t) + target_jerk * dt, target_acceleration)
v(t+1) = max(v(t) + a(t) * dt, 0.0)
```

Parameters: `target_acceleration = -2.5 m/s²`, `target_jerk = -1.5 m/s³`.

### 2.2 — Port `autoware_mrm_comfortable_stop_operator`

**Source:** `autoware-repo/src/universe/autoware_universe/system/autoware_mrm_comfortable_stop_operator/`
**Target:** `src/autoware_mrm_comfortable_stop_operator/`

Service-triggered gentle deceleration. On the safety island this publishes a control
command with ramped-down velocity rather than a velocity limit (no planner on the island).

| Interface | Direction | Type |
|-----------|-----------|------|
| `~/input/mrm/comfortable_stop/operate` | Srv | `tier4_system_msgs/OperateMrm` |
| `~/output/mrm/comfortable_stop/status` | Pub | `tier4_system_msgs/MrmBehaviorStatus` |
| `~/output/mrm/comfortable_stop/control_cmd` | Pub | `autoware_control_msgs/Control` |

Parameters: `min_acceleration = -1.0 m/s²`, `max_jerk = 0.3 m/s³`, `min_jerk = -0.3 m/s³`.

**Note:** The upstream Autoware version publishes a `VelocityLimit` to the planner. Since
the safety island has no planner, we adapt this to publish direct `Control` commands using
the same deceleration profile as the emergency stop operator but with gentler parameters.

### 2.3 — Port `autoware_mrm_handler`

**Source:** `autoware-repo/src/universe/autoware_universe/system/autoware_mrm_handler/`
**Target:** `src/autoware_mrm_handler/`

MRM orchestrator. Receives system health signals, decides which MRM behavior to activate,
and calls the corresponding operator service.

| Interface | Direction | Type |
|-----------|-----------|------|
| `~/input/operation_mode_availability` | Sub | `tier4_system_msgs/OperationModeAvailability` |
| `/localization/kinematic_state` | Sub | `nav_msgs/Odometry` |
| `~/output/mrm/state` | Pub | `autoware_adapi_v1_msgs/MrmState` |
| `~/output/hazard` | Pub | `autoware_vehicle_msgs/HazardLightsCommand` |
| `~/output/gear` | Pub | `autoware_vehicle_msgs/GearCommand` |
| `~/output/mrm/emergency_stop/operate` | Cli | `tier4_system_msgs/OperateMrm` |
| `~/output/mrm/comfortable_stop/operate` | Cli | `tier4_system_msgs/OperateMrm` |

State machine:
```
NORMAL ──[emergency detected]──→ MRM_OPERATING
  ↑                                    │
  │                              ┌─────┴─────┐
  │                              ▼           ▼
  └──[recovered]──── MRM_SUCCEEDED    MRM_FAILED
```

Behavior selection (priority):
1. Watchdog timeout → EMERGENCY_STOP
2. Comfortable stop available → COMFORTABLE_STOP
3. Else → EMERGENCY_STOP
4. On any service call failure → escalate to EMERGENCY_STOP

**nano-ros gap:** Requires service client support. If not yet available, implement as
direct function calls between co-located nodes on the same executor.

### 2.4 — Implement heartbeat watchdog

**Target:** `src/autoware_heartbeat_watchdog/`

Timer-based watchdog that monitors the main Autoware stack's heartbeat topic. On timeout,
publishes an `OperationModeAvailability` message with all modes set to `false`, which
triggers the MRM handler.

| Interface | Direction | Type |
|-----------|-----------|------|
| `/autoware/heartbeat` | Sub | `autoware_adapi_v1_msgs/Heartbeat` |
| `~/output/operation_mode_availability` | Pub | `tier4_system_msgs/OperationModeAvailability` |

Parameters: `timeout = 500 ms`.

## Acceptance Criteria

- [ ] Emergency stop operator decelerates from 20 m/s to 0 m/s following the jerk-limited
      ramp. Unit test verifies velocity profile over time.
- [ ] Comfortable stop operator decelerates with gentler profile (-1.0 m/s² vs -2.5 m/s²).
- [ ] MRM handler transitions through NORMAL → MRM_OPERATING → MRM_SUCCEEDED when velocity
      reaches zero. Unit test covers the full state machine.
- [ ] MRM handler escalates from COMFORTABLE_STOP to EMERGENCY_STOP on service call failure.
- [ ] Heartbeat watchdog triggers within 500 ms of last heartbeat. Unit test with simulated
      time verifies timeout detection.
- [ ] End-to-end test: stop heartbeat → watchdog fires → MRM handler activates →
      emergency stop operator publishes brake commands → velocity reaches zero.
- [ ] All crates compile `no_std` for `thumbv7em-none-eabihf`.
- [ ] Hazard lights are enabled and gear is set to PARK when MRM succeeds.
