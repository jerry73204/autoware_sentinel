# Phase 5: Formal Verification

**Status:** Not Started
**Depends on:** Phase 1 (algorithm crates to verify), Phase 2 (MRM chain for convergence proofs)
**Goal:** Prove correctness, panic-freedom, and safety properties of all ported algorithms
using bounded model checking (Kani) and deductive proofs (Verus).

## Description

The safety island's value depends on confidence that its algorithms behave correctly under all
inputs, including corrupted sensor data (NaN, Inf), extreme values, and adversarial timing.
Testing covers expected paths; formal verification covers *all* paths.

This phase adds Kani harnesses inline in each algorithm crate and Verus proofs in a dedicated
verification crate, following the same patterns established in nano-ros (157 Kani harnesses
across 14 files, 67+ Verus proof functions across 9 files).

## Tool Selection

| Tool | Backend | Strengths | Use For |
|------|---------|-----------|---------|
| **Kani** | CBMC (bounded model checking) | Exhaustive over all IEEE 754 bit patterns; automatic — just assert properties | Per-call correctness, panic-freedom, NaN analysis, f32/f64 edge cases |
| **Verus** | Z3 (SMT solver) | Unbounded deductive proofs; reasons over arbitrary-length sequences | State machine invariants, convergence guarantees, ranking functions |

**Critical constraint:** Verus cannot reason about floating-point arithmetic. Therefore:
- Float-heavy properties → **Kani** (handles all IEEE 754 bit patterns)
- State machine / convergence properties → **Verus** (uses integer ghost models)
- Properties spanning both → **both tools** composed via the ghost type bridge

## Property Table

Properties ordered by automotive safety relevance:

| Rank | Property | Algorithm | Tool | Safety Relevance |
|------|----------|-----------|------|------------------|
| 1 | NaN input produces safe output (zero) | StopFilter | Kani | Sensor corruption → vehicle must stop |
| 2 | Emergency stop converges to v=0 | EmergencyStopOp | Verus | Core MRM guarantee |
| 3 | Output never NaN (all components) | StopFilter, VVC | Kani | NaN propagates through control chain |
| 4 | Gear output is valid constant | ShiftDecider | Kani | Invalid gear → transmission damage |
| 5 | MRM escalation monotonicity | MRM Handler | Verus | De-escalation during emergency = hazard |
| 6 | MRM Succeeded requires v=0 | MRM Handler | Verus | Premature "all clear" while moving |
| 7 | Anti-chattering dead zone | ShiftDecider | Kani | Gear hunting → drivetrain wear/jerking |
| 8 | Covariance is valid PSD diagonal | VVC | Kani | Invalid covariance → EKF divergence |
| 9 | f32→f64 sign preservation | VVC | Kani | Sign flip → wrong direction command |
| 10 | Panic freedom (all components) | All | Kani | Panic on MCU → system halt |
| 11 | Watchdog fires within timeout | Heartbeat | Kani | Late detection → delayed MRM |
| 12 | Rate limiter bounds respected | Cmd Gate | Kani | Excessive accel → passenger injury |

Three property categories: **correctness** (1–4, 7–9), **realtimeness** (2, 11, 12),
**fault tolerance** (1, 3, 5, 6, 10).

## Verification Crate Structure

```
src/
  autoware_stop_filter/src/lib.rs              # + #[cfg(kani)] mod verification
  autoware_vehicle_velocity_converter/src/...   # + #[cfg(kani)] mod verification
  autoware_shift_decider/src/lib.rs            # + #[cfg(kani)] mod verification
  verification/                                 # Verus crate (EXCLUDED from workspace)
    Cargo.toml                                  # edition = "2021", [package.metadata.verus]
    src/
      lib.rs                                    # verus! { } macro blocks, module declarations
      ghost_types.rs                            # Integer ghost models for float-heavy code
      shift_decider.rs                          # VelocityZone ghost + valid_gear invariant
      emergency_stop.rs                         # DecelState ghost + convergence proof
      mrm_handler.rs                            # GhostMrmHandler + state machine proofs
```

Design decisions:
- **Kani harnesses go inline** in each production crate (`#[cfg(kani)] mod verification`)
- **Verus proofs go in a separate crate** (`src/verification/`)
- The verification crate is **excluded** from workspace `members` (Verus has its own build)
- The verification crate uses **edition 2021** (Verus requirement) with
  `[package.metadata.verus] verify = true`
- Ghost types live in the verification crate

## Ghost Type Bridge Architecture

The ghost type system bridges production code (f32/f64) with Verus proofs (integers only)
through a three-layer design.

### Layer 1 — Production code (`lib.rs`, `#[cfg(test)]`)

Ghost contract functions map concrete float values to abstract domains:

```rust
pub enum VelocityZone { Forward, Reverse, DeadZone }

fn velocity_zone(v: f32) -> VelocityZone {
    if v > 0.01 { VelocityZone::Forward }
    else if v < -0.01 { VelocityZone::Reverse }
    else { VelocityZone::DeadZone }
}
```

If a production field is renamed or retyped, the ghost construction fails to compile
(structural binding ensures the abstraction stays in sync).

### Layer 2 — Kani verification (inline `#[cfg(kani)]`)

Exhaustively verifies the abstraction function for every f32 bit pattern, handling all
IEEE 754 edge cases (NaN, Inf, subnormals, signed zeros):

```rust
#[cfg(kani)]
#[kani::proof]
fn velocity_zone_covers_all_f32() {
    let v: f32 = kani::any();
    let zone = velocity_zone(v);
    // Zone is always well-defined (even for NaN/Inf)
}
```

### Layer 3 — Verus proofs (`src/verification/`)

Operates entirely on the abstract ghost model with integer arithmetic:

```rust
verus! {
    pub spec fn valid_gear(g: u8) -> bool {
        g == 2 || g == 20 || g == 22  // DRIVE, REVERSE, PARK
    }

    pub fn ghost_decide(
        sd: &mut GhostShiftDecider,
        vel_zone: VelocityZone,
        // ...
    ) -> (result: u8)
        ensures valid_gear(result),
    { /* ... */ }
}
```

### Soundness argument

1. **Kani** verifies the abstraction function is correct for all bit patterns
2. **Verus** proves properties over the abstract domain
3. **Composition:** Kani guarantees the abstraction preserves relevant structure; Verus
   guarantees properties hold in the abstract domain
4. The gap is floating-point rounding near zone boundaries — Kani covers this by checking
   all bit patterns

## Per-Algorithm Specifications

### StopFilter — Kani harnesses (5 + 1)

Harnesses go in `src/autoware_stop_filter/src/lib.rs`:

| Harness | Inputs | Assertion |
|---------|--------|-----------|
| `nan_velocity_is_not_stopped` | `linear.x = NaN` | `!result.was_stopped` (proves the bug) |
| `output_never_nan` | Symbolic with finite thresholds | All 6 output components are not NaN |
| `apply_never_panics` | All 8 inputs unconstrained | Function completes without panic |
| `stopped_means_all_zeros` | Symbolic with finite thresholds | `was_stopped → all components == 0.0` |
| `moving_means_exact_passthrough` | Symbolic with finite thresholds | `!was_stopped → output.to_bits() == input.to_bits()` |
| `threshold_symmetry` (post-fix) | Finite inputs | `is_stopped(vx, wz) == is_stopped(-vx, -wz)` |

The `output_never_nan` harness **fails** on the current code, proving the NaN safety gap.
After applying the inverted-comparison fix, it passes.

### VehicleVelocityConverter — Kani harnesses (4)

Harnesses go in `src/autoware_vehicle_velocity_converter/src/lib.rs`:

| Harness | Inputs | Assertion |
|---------|--------|-----------|
| `covariance_valid_diagonal` | Symbolic finite scale/stddev | Diagonal ≥ 0, off-diagonal == 0 |
| `f32_cast_preserves_sign` | Symbolic finite f32 | `sign(output_f64) == sign(input_f32)` |
| `convert_never_panics` | All inputs unconstrained | Function completes without panic |
| `convert_output_nan_check` | Report fields unconstrained | Output linear.x and angular.z not NaN |

The `convert_output_nan_check` harness exposes the same class of NaN propagation bug as
StopFilter: NaN f32 input → NaN f64 output → downstream gets NaN velocity.

### ShiftDecider — Kani harnesses (3) + Verus proofs

**Kani** harnesses go in `src/autoware_shift_decider/src/lib.rs`:

| Harness | Inputs | Assertion |
|---------|--------|-----------|
| `output_is_valid_gear` | Symbolic | Output is DRIVE (2), REVERSE (20), PARK (22), or passthrough |
| `dead_zone_holds_previous` | Two-step: establish then dead-zone | Second result equals first |
| `nan_velocity_holds_previous` | Establish DRIVE, then NaN | Result is DRIVE (safe by accident) |

**Verus** proofs go in `src/verification/src/shift_decider.rs`:

```rust
verus! {
    pub struct GhostShiftDecider {
        pub park_on_goal: bool,
        pub prev_command: u8,
    }

    pub fn ghost_decide(
        sd: &mut GhostShiftDecider,
        is_driving: bool,
        is_arrived_or_waiting: bool,
        vel_zone: VelocityZone,
        current_gear: u8,
    ) -> (result: u8)
        requires valid_gear(sd.prev_command),
                 is_arrived_or_waiting ==> valid_gear(current_gear),
                 !is_driving || !is_arrived_or_waiting,
        ensures valid_gear(result), valid_gear(sd.prev_command),
    { /* ... */ }
}
```

Plus `proof fn no_direct_drive_reverse_transition` for anti-chattering: when in DRIVE and
velocity is in DeadZone, the gear stays DRIVE.

### Emergency Stop Operator — Verus convergence proof

Proof goes in `src/verification/src/emergency_stop.rs`:

**Ghost model** (integer, scaled by 1000):

```rust
verus! {
    pub struct DecelState {
        pub velocity_mms: int,       // mm/s (non-negative)
        pub acceleration_mms2: int,  // mm/s^2 (negative during braking)
    }

    pub spec fn decel_step(s: DecelState) -> DecelState {
        let new_accel = if s.acceleration_mms2 + JERK_MMS3 * DT_MS > TARGET_ACCEL_MMS2 {
            s.acceleration_mms2 + JERK_MMS3 * DT_MS
        } else {
            TARGET_ACCEL_MMS2
        };
        let new_vel = if s.velocity_mms + new_accel * DT_MS > 0 {
            s.velocity_mms + new_accel * DT_MS
        } else {
            0
        };
        DecelState { velocity_mms: new_vel, acceleration_mms2: new_accel }
    }
}
```

Constants: `TARGET_ACCEL_MMS2 = -2500`, `JERK_MMS3 = -1500`, `DT_MS = 33`.

**Two-phase ranking function:**
1. **Ramp phase:** Ranking = `acceleration - target_accel` (positive, decreasing). ~51 steps.
2. **Decel phase:** Ranking = `velocity` (positive, decreasing by 82 mm/s/step). ~244 steps.

**Bounded convergence:** From 20 m/s → 0 in ≤ 300 steps (~10 seconds).

**Companion Kani harness** in the production crate: `#[kani::unwind(301)]` bounded check on
actual f32 implementation, asserting `velocity == 0.0 && step <= 300`.

### MRM Handler — Verus state machine proofs

Proofs go in `src/verification/src/mrm_handler.rs`:

```rust
verus! {
    pub enum MrmState { Normal, Operating, Succeeded, Failed }
    pub enum MrmBehavior { None, ComfortableStop, EmergencyStop }

    pub struct GhostMrmHandler {
        pub state: MrmState,
        pub current_behavior: MrmBehavior,
        pub velocity_is_zero: bool,
    }

    pub spec fn valid_mrm_state(h: GhostMrmHandler) -> bool {
        &&& (h.state == MrmState::Normal ==> h.current_behavior == MrmBehavior::None)
        &&& (h.state == MrmState::Operating ==> h.current_behavior != MrmBehavior::None)
        &&& (h.state == MrmState::Succeeded ==> h.velocity_is_zero)
    }
}
```

Three properties proved:

| Property | Proof technique |
|----------|-----------------|
| Terminal state stability | Case analysis: no transition rule applies when `Succeeded` or `Failed` |
| Escalation monotonicity | Ranking function `behavior_ord()`: `ord(post) >= ord(pre)` while Operating |
| Succeeded requires v=0 | Invariant `valid_mrm_state` preserved across all transitions |

## Build System Integration

### Cargo.toml lint for `cfg(kani)`

Each algorithm crate needs:

```toml
[lints.rust]
unexpected_cfgs = { level = "warn", check-cfg = ['cfg(kani)'] }
```

This tells `rustc` to expect the `cfg(kani)` attribute (injected by `cargo kani`) so that
normal `cargo check`/`cargo build` does not warn about the unknown cfg.

### Running verification

```bash
# Kani — per-crate
cargo kani -p autoware_stop_filter
cargo kani -p autoware_vehicle_velocity_converter
cargo kani -p autoware_shift_decider

# Kani — specific harness
cargo kani -p autoware_stop_filter --harness output_never_nan

# Kani — workspace (skips crates without harnesses)
cargo kani --workspace

# Verus
cd src/verification && cargo verus verify

# Combined (Justfile/Makefile)
# verify-kani: cargo kani --workspace
# verify-verus: cd src/verification && cargo verus verify
# verify: verify-kani verify-verus
```

Existing tests must still pass:

```bash
cargo test --workspace
cargo build --target thumbv7em-none-eabihf
```

## Implementation Sequencing

| Step | Work | Dependencies | Parallel |
|------|------|--------------|----------|
| 1 | Add `[lints.rust]` check-cfg to 3 crate Cargo.tomls | — | — |
| 2 | StopFilter: 5 Kani harnesses | Step 1 | Yes (with 3, 4) |
| 3 | VehicleVelocityConverter: 4 Kani harnesses | Step 1 | Yes (with 2, 4) |
| 4 | ShiftDecider: 3 Kani harnesses | Step 1 | Yes (with 2, 3) |
| 5 | Fix StopFilter NaN handling based on Kani results | Step 2 | — |
| 6 | Re-verify StopFilter — `output_never_nan` passes | Step 5 | — |
| 7 | Create `src/verification/` Verus crate skeleton | Can start with 2–4 | Yes |
| 8 | ShiftDecider Verus ghost model + invariant proof | Step 7 | — |
| 9 | Emergency stop convergence proof (Phase 2 prereq) | Steps 7 + Phase 2 | — |
| 10 | MRM handler state machine proofs (Phase 2 prereq) | Steps 7 + Phase 2 | — |

Critical path: Steps 1 → 2 → 5 → 6 (the NaN safety fix is the highest-value deliverable).

## Future Phase Verification

### Phase 3 — Vehicle Command Gate

- **Kani:** Rate limiter output bounds, jerk limiting bounded rate of change, source
  arbitration priority
- **Verus:** Monotonic source priority (unbounded), rate limit envelope invariant

### Phase 4 — Control Validator

- **Kani:** NaN-safe threshold comparisons, diagnostic severity correctness
- **Verus:** Overspeed detection completeness
