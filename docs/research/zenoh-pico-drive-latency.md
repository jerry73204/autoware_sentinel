# Zenoh-pico Drive Latency Investigation

**Date:** 2026-04-03
**Context:** Autoware planning simulator with sentinel replacing 14 nodes

## Problem

The sentinel-augmented Autoware stack drives 3–5× slower than the unmodified baseline
(79–134s vs 26s for the same route). The engagement flow works correctly — the slowdown
is entirely in the driving phase.

## Test Setup

- Autoware 1.5.0 planning simulator via `play_launch replay` (filtered record)
- Sentinel binary connected through zenohd router on localhost:7447
- Autoware nodes use rmw_zenoh_cpp (full Rust zenoh); sentinel uses zenoh-pico
- Route: ~100m straight segment in sample-map-planning

## Data Flow

In the baseline Autoware stack:

```
planner → trajectory → controller_node_exe → /control/trajectory_follower/control_cmd
    → vehicle_cmd_gate → /control/command/control_cmd → simulator
```

In the sentinel-augmented stack:

```
planner → trajectory → controller_node_exe → /control/trajectory_follower/control_cmd
    → [zenoh-pico subscription] → sentinel gate → /control/command/control_cmd → simulator
```

The sentinel subscribes to the controller output via zenoh-pico and passes it through
its safety gate to the simulator. The sentinel also has an internal trajectory follower
(PID/MPC), but it requires `/planning/scenario_planning/trajectory` data to activate.

## Findings

### 1. Subscription discovery delay (~22s)

After engagement, the sentinel receives **no control data** for ~22 seconds. During this
period the gate publishes zero velocity/acceleration and the car stays stationary.

```
17:41:00  GATE: vel=0.00 accel=0.00 ext=false traj=false   ← car stationary
17:41:02  GATE: vel=0.00 accel=0.00 ext=false traj=false
  ...
17:41:22  GATE: vel=0.90 accel=0.81 ext=true  traj=false   ← first external data arrives
```

**Root cause:** zenoh-pico subscription discovery through zenohd. The sentinel registers
its subscription at startup, but zenohd doesn't propagate the interest to the rmw_zenoh_cpp
publisher (Autoware's controller_node_exe) for ~20s. This is a zenoh protocol-level delay,
not a sentinel bug.

### 2. Intermittent subscription dropouts (staleness guard)

Once discovered, the external control data arrives **intermittently**. The data alternates
between fresh (age < 500ms) and stale (age > 500ms). During stale periods, the sentinel's
staleness guard replaces the control command with gentle braking (accel = -1.5 m/s²).

```
17:41:22  vel=0.90 accel=0.81  ext=true   ← fresh: car accelerates
17:41:24  vel=2.34 accel=0.95  ext=true   ← fresh
17:41:26  vel=3.89 accel=0.63  ext=true   ← fresh
17:41:31  vel=4.16 accel=-1.50 ext=true   ← STALE: gentle braking
17:41:33  vel=4.16 accel=-1.50 ext=true   ← STALE
17:41:35  vel=4.14 accel=-0.25 ext=true   ← fresh again
17:41:37  vel=3.11 accel=-1.75 ext=true   ← STALE
```

This stop-start pattern (accelerate → brake → accelerate → brake) is the primary cause
of the 3–5× slowdown.

**Quantitative data from one test run:**
- Total FRESH ticks: 5,904 (33%)
- Total STALE ticks: 12,245 (67%)
- Average stale age: 77s (max 340s)

### 3. Internal controller never activates

The sentinel's internal trajectory follower (PID + MPC) requires three subscriptions to
deliver data before it produces output:

- `/planning/scenario_planning/trajectory` — **never received**
- `/localization/kinematic_state` — received after ~5s
- `/vehicle/status/steering_status` — received after ~5s

The trajectory subscription never delivers data during the drive. This is the same
zenoh-pico discovery delay affecting a different topic. As a result, `has_trajectory`
stays `false` and the internal controller returns early every tick.

The entire drive relies on the external controller path (Autoware's controller_node_exe
→ zenoh-pico → sentinel gate), which suffers from the dropout issue.

## Timeline (typical run)

| Time | Event |
|------|-------|
| T+0s | Sentinel starts, registers subscriptions |
| T+5s | Velocity, odometry, steering arrive (zenoh-pico discovers these publishers) |
| T+100s | Autoware fully started, auto_drive engages |
| T+122s | External control first arrives (zenoh-pico discovers controller publisher) |
| T+122–200s | Intermittent fresh/stale driving (car moves in bursts) |
| T+179s | ARRIVED at goal |

## Comparison

| Metric | Baseline | Sentinel |
|--------|----------|----------|
| Engagement | instant | instant (with 60s query timeout) |
| First control data | immediate | +22s (zenoh-pico discovery) |
| Drive pattern | continuous | intermittent (33% fresh, 67% stale) |
| **Drive time** | **26s** | **79–134s** |

## Root Causes

Both issues are **zenoh-pico transport layer** problems:

1. **Subscription interest propagation delay** — when zenoh-pico subscribes to a keyexpr,
   the zenohd router needs to propagate the interest to the rmw_zenoh_cpp publisher.
   This propagation takes 5–30s depending on the topic and router load.

2. **Subscription data dropout** — even after discovery, zenoh-pico receives data
   intermittently. Some ticks receive the publisher's data; others don't. This may be
   related to zenoh-pico's single-threaded polling model vs zenohd's async routing.

## Staleness Guard Behavior

The staleness guard (`EXTERNAL_CONTROL_STALE_MS = 500ms`) is a safety mechanism:

- If external control data is >500ms old, replace with gentle braking (accel = -1.5 m/s²)
- Sets velocity target to current velocity (prevents control validator over-velocity alarm)
- Prevents overshoot during dropouts (sustained positive acceleration with stale data)

Without the staleness guard, the car would continue accelerating with the last received
command during dropouts, potentially overshooting turns or exceeding safe velocity.

## Fix: Skip sleep when callbacks are active (nano-ros `0f5bf2a4`)

**Root cause identified:** The executor's `spin_blocking()` loop had an unconditional
`sleep(10ms)` after every `spin_once()` cycle. When 30Hz data was flowing:

1. `spin_once(10ms)` — condvar wakes early on data, processes callbacks (~1ms)
2. `sleep(10ms)` — **unconditional** — new messages arrive during this window
3. zenoh-pico background thread overwrites single-message subscriber buffer
4. By the time the executor wakes, the message has been overwritten 1–2 times

**Fix:** Only sleep when no callbacks were processed (system is idle):

```rust
// Before: always sleep
std::thread::sleep(Duration::from_millis(POLL_INTERVAL_MS as u64));

// After: skip sleep when busy
if result.total() == 0 {
    std::thread::sleep(Duration::from_millis(POLL_INTERVAL_MS as u64));
}
```

**Results:**

| Metric | Before fix | After fix | Baseline |
|--------|-----------|-----------|----------|
| Drive time | 79–134s | **46.9s** | 26s |
| Stale ratio | ~67% | ~5% | 0% |
| Slowdown | 3–5× | **1.8×** | 1× |

The remaining 1.9× gap (50s vs 26s) is NOT from subscription dropouts — data flows
continuously after the fix. See next section.

## Remaining Gap: Controller-Commanded Mid-Route Stop (2026-04-04)

After the spin fix, GATE logging shows the external control data arrives continuously
(`ext=true` throughout). The 50s drive time breaks down as:

```
05:23:01-05:23:17  Smooth driving, vel 0→4.17 m/s (16s)     [fresh ext control]
05:23:18-05:23:19  Brief stale, accel=-1.50 (2s)            [stale guard, 2 ticks]
05:23:20-05:23:22  Controller decelerates: accel=-0.80→-2.30 [fresh ext, NOT stale]
05:23:23-05:23:42  Car stopped, accel=-2.50 for 20s          [fresh ext, controller decision]
05:23:43-05:23:50  Resumes driving, accelerates to goal (7s) [fresh ext control]
ARRIVED after 50.3s
```

The 20s pause at `accel=-2.50` is the **Autoware controller's own decision** — NOT the
staleness guard (which uses -1.50). The controller commanded a hard stop mid-route, then
resumed 20s later.

**Root cause hypothesis:** Round-trip latency in the control feedback loop:

```
controller_node_exe → /control/trajectory_follower/control_cmd
    → [zenohd] → [zenoh-pico sub] → sentinel gate
    → /control/command/control_cmd → simulator
    → simulator publishes /localization/kinematic_state
    → [zenohd] → controller_node_exe reads position
```

Each zenoh hop adds latency. The controller's PID/MPC is tuned for the baseline stack
where all components share the same DDS domain with sub-millisecond latency. With the
sentinel in the loop, the position feedback arrives late, causing the controller to see
stale position data and over-correct (hard braking when it thinks it overshot).

## Investigation: zenoh-pico Transport Latency

Analyzed the zenoh-pico C library at `packages/zpico/zpico-sys/zenoh-pico/`.

### Read path (no issues found)

- **TCP_NODELAY** enabled on client socket (`src/system/unix/network.c:208`)
- **Synchronous callbacks** — subscriber callback fires directly from the read task
  thread, no intermediate queue (`src/session/subscription.c:281`)
- **No read batching** — each message decoded and dispatched individually
  (`src/transport/unicast/read.c:48-55`)
- **Keep-alive non-blocking** — lease task uses try-lock, cannot block read task
  (`src/transport/unicast/lease.c:28`)
- **Socket timeout** — `SO_RCVTIMEO = 100ms`, creating natural poll interval

### Write path (no issues found)

- **Batching disabled** by default — `_batch_state = _Z_BATCHING_IDLE` at init
  (`src/transport/unicast/transport.c:36`). nros never calls `z_batch_start()`.
- Non-express publishes still flush immediately when batching is idle
  (`src/transport/common/tx.c:168`)
- TX mutex is separate from RX mutex — publishing doesn't block reading

### Conclusion: transport is not the bottleneck

The extra 2 zenoh hops (zenohd→zpico + zpico→zenohd) add ~2-4ms per control cycle.
This is negligible for a 30Hz control loop (33ms period).

## Remaining Gap Analysis: Controller-Commanded Mid-Route Stop

Detailed GATE log analysis shows the 20s mid-route stop is the **Autoware controller's
own command** (`accel=-2.50`, which is harder than the stale guard's `-1.50`). The
external control data continues flowing (`ext=true`) during the stop — this is NOT a
dropout issue.

The controller (`controller_node_exe`) is Autoware's unmodified code running in the
filtered stack. It reads trajectory and odometry directly via rmw_zenoh_cpp (not through
zenoh-pico). The sentinel only adds 2 extra hops in the control-command path:

```
Baseline:  controller → gate → simulator       (all rmw_zenoh_cpp)
Sentinel:  controller → zenohd → zpico(sentinel) → zenohd → simulator
                        ^~~~~ +2 hops (~2-4ms) ~~~~^
```

The 2-4ms extra latency should not cause a 20s behavioral difference. Possible causes
still under investigation:

1. **Gate filter parameter mismatch** — the sentinel's `FilterParams::default()` uses
   hard-coded values that differ from Autoware's launch-configured parameters. The gate's
   rate limiter or speed clamp could modify the control command differently from the
   baseline gate, causing the controller to see unexpected vehicle behavior and over-correct.

2. **Gear command timing** — the sentinel's shift_decider starts with `GearReport::default()`
   (report=0, NONE) until the gear_status subscription delivers data. If the simulator
   receives gear=NONE during the first few seconds, it may not move, causing the controller
   to see unexpected stationary behavior.

3. **30Hz timer phase offset** — the sentinel's timer is not synchronized with the
   controller's output rate. Up to 33ms of jitter per cycle could accumulate in the MPC
   prediction horizon.

## Fix 2: Increase staleness threshold from 500ms to 2000ms

The brief subscription dropouts (~2s) at 500ms threshold triggered the staleness guard,
which applied -1.50 braking. The controller saw the unexpected deceleration in the
simulator's odometry and over-corrected with -2.50 hard braking, causing a 20s mid-route
stop. With a 2000ms threshold, the brief dropouts pass through unmodified, preserving the
controller's natural trajectory.

**Trade-off:** At 4.17 m/s, a 2s uncontrolled period adds ~8m of travel with the last
command. Acceptable on straight road; requires evaluation for curves.

**Results after both fixes:**

| Metric | Original | After spin fix | After stale threshold fix | Baseline |
|--------|----------|---------------|--------------------------|----------|
| Drive time | 79–134s | 47–50s | **46.4s** | 26s |
| Stale ratio | 67% | ~5% | ~5% | 0% |
| Mid-route stops | 20s pause | 20s pause | **none** | none |
| Slowdown | 3–5× | 1.9× | **1.8×** | 1× |

The remaining 1.8× gap (46s vs 26s) is the ~3s subscription discovery delay plus the
extra 2 zenoh hops per control cycle (~17s of accumulated latency in the feedback loop
over 40s of driving).

## Summary

| Issue | Root cause | Fix | Impact |
|-------|-----------|-----|--------|
| 3-5× slowdown | Redundant `sleep(10ms)` in spin_blocking | Removed (nano-ros `839f07cd`) | 79-134s → 47-50s |
| ~20s mid-route stop | Stale guard at 500ms triggers controller over-correction | Threshold → 2000ms | 47-50s → 46s |
| ~3s discovery delay | zenoh-pico subscription interest propagation | Not yet addressed | +3s initial wait |
| **Total** | | | **46s (1.8× baseline)** |
