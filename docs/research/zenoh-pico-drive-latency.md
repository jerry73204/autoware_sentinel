# Zenoh-pico Drive Latency Investigation

**Dates:** 2026-04-03 to 2026-04-08
**Context:** Autoware 1.5.0 planning simulator with sentinel replacing 14 nodes

## Problem

The sentinel-augmented Autoware stack drives 3–5× slower than the unmodified baseline
(79–134s vs 26s for the same ~100m route). The engagement flow works correctly — the
slowdown is entirely in the driving phase.

## Architecture

```
Baseline:
  controller → vehicle_cmd_gate → /control/command/control_cmd → simulator
  (all rmw_zenoh_cpp, intra-router, sub-ms latency)

Sentinel:
  controller → zenohd → zenoh-pico → sentinel gate → zenoh-pico → zenohd → simulator
  (+2 extra zenohd hops, ~2-4ms per cycle)
```

## Fixes Applied

### Fix 1: Remove redundant sleep from spin_blocking (nano-ros `839f07cd`)

`spin_blocking()` had an unconditional `sleep(10ms)` after each `spin_once(10ms)` cycle.
`spin_once(10ms)` already provides the wait semantics:
- Multi-threaded (POSIX): condvar wait, wakes early on data
- Single-threaded (Zephyr/bare-metal): `zp_read()` polling loop

The extra sleep doubled the cycle to 20ms, creating a window where the single-message
subscriber buffer was overwritten before the executor could process it.

| Metric | Before | After | Baseline |
|--------|--------|-------|----------|
| Drive time | 79–134s | 47–50s | 26s |
| Stale ratio | ~67% | ~5% | 0% |

### Fix 2: Increase staleness threshold from 500ms to 2000ms

Brief subscription dropouts (~2s) at the 500ms threshold triggered the staleness guard
(accel=-1.50). The Autoware controller saw the unexpected deceleration in the simulator's
odometry and over-corrected with hard braking (accel=-2.50), causing a 20s mid-route stop.

At 2000ms, brief dropouts pass through unmodified, preserving the controller's natural
trajectory. The controller no longer over-corrects.

**Trade-off:** At 4.17 m/s, 2s of uncontrolled travel ≈ 8m. Acceptable on straight road.

| Metric | Before | After | Baseline |
|--------|--------|-------|----------|
| Drive time | 47–50s | **46s** | 26s |
| Mid-route stops | 20s pause | none | none |

### Combined result: 1.8× baseline (46s vs 26s)

## Remaining Gap: zenohd Interest Routing Under Load

The remaining ~20s gap is caused by zenohd failing to propagate zenoh-pico subscription
interests to rmw_zenoh_cpp publishers when there are 200+ concurrent sessions.

### Evidence

**Comparative test** (Autoware fully running, 222 sessions on zenohd):

| Client | Time to first message |
|--------|----------------------|
| rmw_zenoh_cpp (full Rust zenoh) | **259ms** |
| zenoh-pico | **never** (>360s) |
| zenoh-pico with 1 publisher (no Autoware) | **instant** (<1s) |

**Debug tracing** (`fprintf` in `zpico.c`):
- All 9 `z_declare_subscriber()` calls succeed (return 0)
- Self-published data (sentinel→zenohd→sentinel loopback) arrives fine
- External data (rmw_zenoh_cpp→zenohd→zenoh-pico) never arrives with 222 sessions

### What was ruled out

| Hypothesis | Test | Result |
|-----------|------|--------|
| zenoh-pico wire format mismatch | Rebased to exact 1.7.2 tag | Same behavior (also broke connection) |
| Wildcard keyexpr matching | Used exact key instead of `.../*` | No change |
| Interest routing timeout | Set `routing.interests.timeout: 0` | No change |
| Session startup order | Sentinel first, then Autoware | No change |
| Peer mode connection | `ZENOH_MODE=peer` | ConnectionFailed |
| zenoh-pico transport latency | Analyzed read/write/keepalive paths | All correct: TCP_NODELAY on, sync callbacks, no batching |
| Gate filter parameters | Compared with Autoware source | Match exactly |

### zenoh-pico transport analysis

The zenoh-pico C library transport is correct:
- **TCP_NODELAY** enabled (no Nagle delay)
- **Synchronous callbacks** — subscriber callback fires directly from read task
- **No read batching** — each message decoded and dispatched individually
- **No write batching** — `_batch_state = _Z_BATCHING_IDLE` (nros never calls `z_batch_start`)
- **Non-blocking keep-alive** — lease task uses try-lock, cannot block read task
- **Separate TX/RX mutexes** — publishing doesn't block reading

### zenoh-pico version

- zenoh-pico: `1.7.2 + 18 commits` (main branch after tag)
- zenohd: `v1.7.2` (but built from main branch post-merge, not exact tag)
- rmw_zenoh_cpp (libzenohc): `v1.7.2`
- Connection works between zenoh-pico 1.7.2+18 and zenohd
- Connection FAILS between zenoh-pico exact 1.7.2 tag and zenohd (wire format evolved)

### Root cause assessment

The issue is in **zenohd's interest-based routing** when handling 200+ concurrent client
sessions. When a new zenoh-pico client declares a subscription, zenohd needs to:
1. Match the subscription keyexpr against all existing publisher declarations
2. Set up forwarding from matching publisher sessions to the new subscriber session

With 222 sessions (each declaring ~10 publishers), this is ~2200 publisher entries to
match against. The matching either takes very long, silently fails, or is rate-limited.

The non-deterministic behavior (sometimes works in `just launch-autoware-sentinel`, usually
doesn't in manual tests) suggests a race condition in zenohd's interest propagation logic
that depends on the timing of session connections.

### Successful runs

In `just launch-autoware-sentinel --drive` runs where the sentinel DID receive data and
drove the car (46s), the sentinel binary starts as part of a `parallel` job group alongside
zenohd and play_launch. The exact startup timing and session connection order differs from
manual testing. Data arrival was delayed by ~3s after engagement but then flowed continuously.

## Next Steps

1. **File upstream bug** on github.com/eclipse-zenoh/zenoh with reproduction steps:
   - zenohd + 200 rmw_zenoh_cpp sessions + 1 zenoh-pico client
   - zenoh-pico subscription declaration succeeds but data never forwarded
   - Same subscription from rmw_zenoh_cpp works in 259ms

2. **Enable zenohd debug logging** for interest routing module to capture the full
   interest propagation sequence when the zenoh-pico client connects

3. **Compare wire messages** between zenoh-pico and rmw_zenoh_cpp subscription
   declarations using Wireshark or zenohd trace logs

4. **Test with fewer sessions** to find the threshold where interest propagation breaks
   (e.g., 50, 100, 150, 200 sessions)

### Root cause found: missing DeclareKeyExpr (2026-04-09)

**tshark packet capture** with zenoh dissector reveals:

1. zenoh-pico sends **9 DeclareSubscriber** to the router — confirmed on wire ✓
2. zenohd sends **1318 Push messages** back to zenoh-pico — data IS forwarded ✓
3. Push messages use `WireExpr { scope: 118, suffix: "/TypeHashNotSupported", mapping: Receiver }`
4. **zenohd sent 0 DeclareKeyExpr** to the sentinel — scope 118 is never defined
5. zenoh-pico can't resolve the keyexpr → **silently drops all 1318 Push messages**

The `mapping: Receiver` flag means the scope ID must be resolved in the receiver's
keyexpr table. zenohd should send a `DeclareKeyExpr(id=118, keyexpr="0/vehicle/...")` 
before forwarding Pushes that reference scope 118. With 1 publisher this works (instant
data). With 200+ sessions, the `DeclareKeyExpr` is never sent to the new client.

**Detailed comparison** (1 publisher vs 200+ sessions):

In the working case (1 publisher):
- Sentinel declares keyexpr ID 116 = `0/vehicle/status/velocity_status/.../VelocityReport_`
- Push arrives with `scope: 116, suffix: "/TypeHashNotSupported", mapping: Receiver`
- zenoh-pico resolves: scope 116 (velocity_status) + "/TypeHashNotSupported" → matches subscriber's `/*` wildcard ✓

In the broken case (200+ sessions):
- Sentinel declares keyexpr ID 118 = `0/api/system/heartbeat/.../Heartbeat_`
- Push arrives with `scope: 118, suffix: "/TypeHashNotSupported", mapping: Receiver`
- zenoh-pico resolves: scope 118 (heartbeat) + "/TypeHashNotSupported" → does NOT match velocity subscriber ✗
- The velocity_status data was sent with the **wrong scope ID** — it used 118 (heartbeat) instead of the correct scope for velocity_status

**Verdict**: zenohd scope ID mapping bug under load. When forwarding Push data to a client
that joined after 200+ sessions were already established, the router maps the publisher's
keyexpr to the wrong scope ID in the receiver's keyexpr table. This causes the zenoh-pico
client to resolve the data to the wrong subscription (or no subscription) and silently drop it.

### Code-level root cause: `get_best_key` vs wildcard subscriptions

File: `external/zenoh/zenoh/src/net/routing/dispatcher/resource.rs:664`

When routing data to a subscriber, zenohd calls `expr.get_best_key(face_id)` to find the
best scope ID for the receiver. This function walks the resource tree from the data's exact
keyexpr resource, looking for the receiver's face ID in `session_ctxs`.

For wildcard subscriptions (e.g., `0/vehicle/.../VelocityReport_/*`), the session context is
registered on the **wildcard resource**, not on the exact data resource
(`0/vehicle/.../VelocityReport_/TypeHashNotSupported`). The `get_best_key` function walks
UP the tree via `get_best_parent_key`, looking for any ancestor that has the receiver's session
context. With 200+ sessions, many faces have session contexts on various tree nodes, and the
function finds the WRONG context first (e.g., a heartbeat resource instead of the velocity
resource), returning an incorrect scope ID.

With 1 session, there's only one session context to find, so the walk always succeeds. With
200+ sessions, the tree has many contexts at many levels, and the parent walk picks up an
unrelated context.

**Fix options:**
1. When routing wildcard-matched data, use the matched resource's keyexpr directly instead
   of calling `get_best_key` (which assumes exact-match tree structure)
2. Make `get_best_key` prefer contexts on resources that are actual prefixes of the data
   keyexpr, not just any ancestor in the resource tree
3. Report to eclipse-zenoh/zenoh as a bug with this analysis

## Upstream Fix: zenoh PR #2096 "Regions" (2026-04-08 update)

The zenoh project merged [PR #2096 "Regions: improved routing scalability"](https://github.com/eclipse-zenoh/zenoh/pull/2096)
to `main` on 2026-04-01. This is a major rewrite of the routing system that fixes
multiple interest propagation issues:

- [#2224](https://github.com/eclipse-zenoh/zenoh/issues/2224) — Regions (routing scalability)
- [#2233](https://github.com/eclipse-zenoh/zenoh/issues/2233) — Refuse interests without key expressions
- [#2278](https://github.com/eclipse-zenoh/zenoh/issues/2278) — Routers propagate pre-existing tokens to clients
- [#2283](https://github.com/eclipse-zenoh/zenoh/issues/2283) — Clients propagate pre-existing tokens to local sessions

These issues describe exactly our problem: interest propagation from late-joining clients
(zenoh-pico) to existing publishers through a router fails at scale.

**Status**: NOT in zenoh 1.8.0 (merged after release). Will be in a future release.
This is a **breaking change** with new region-based routing configuration.

Until the Regions PR lands in a release, our options are:
1. Build zenohd from zenoh `main` branch (post-Regions)
2. Accept the ~1.8× slowdown as a known limitation
3. Reduce Autoware session count (filter more nodes)

## Version Reference

| Component | Version | Source |
|-----------|---------|--------|
| zenohd | v1.7.2 (main post-merge) | `external/zenoh/` |
| zenoh-pico | 1.7.2 + 18 commits | `nano-ros/packages/zpico/zpico-sys/zenoh-pico/` |
| rmw_zenoh_cpp | v1.7.2 | `external/rmw_zenoh_ws/` |
| libzenohc | v1.7.2 | via rmw_zenoh_cpp vendor |
| nano-ros | local main branch | `~/repos/nano-ros/` |
