# Autoware Sentinel

An independent safety island for autonomous vehicles, porting selected Autoware packages to
[nano-ros](../nano-ros/), a ROS 2 runtime focused on formal verification and real-time safety
for embedded systems. Runs on a dedicated safety MCU to bring the vehicle to a safe stop
when the main compute fails.

**Repository:** https://github.com/jerry73204/autoware_sentinel.git

## Project Structure

Each algorithm crate is a standalone package (no root workspace). Message crates are
generated per-package into a local `generated/` directory.

```
autoware-nano-ros/
├── src/
│   ├── autoware_stop_filter/        # Phase 1 — velocity stop filter
│   │   ├── Cargo.toml
│   │   ├── .cargo/config.toml       # [patch.crates-io] — auto-generated
│   │   ├── package.xml              # ROS message dependencies
│   │   ├── justfile                 # generate, build, test, clean
│   │   ├── .gitignore
│   │   ├── generated/               # per-package generated message crates
│   │   └── src/lib.rs
│   ├── autoware_vehicle_velocity_converter/  # Phase 1 — VelocityReport→Twist
│   ├── autoware_shift_decider/      # Phase 1 — gear state machine
│   ├── autoware_mrm_emergency_stop_operator/  # Phase 2 — jerk-limited hard braking
│   ├── autoware_mrm_comfortable_stop_operator/ # Phase 2 — gentle deceleration
│   ├── autoware_heartbeat_watchdog/ # Phase 2 — main compute heartbeat monitor
│   ├── autoware_mrm_handler/        # Phase 2 — MRM orchestrator state machine
│   ├── autoware_vehicle_cmd_gate/   # Phase 3 — rate limiting + source arbitration
│   ├── autoware_twist2accel/        # Phase 4 — velocity→acceleration estimator
│   ├── autoware_control_validator/  # Phase 4 — command safety validation
│   ├── autoware_operation_mode_transition_manager/ # Phase 4 — mode transitions
│   ├── verification/                # Phase 5 — Verus formal proofs
│   ├── autoware_sentinel/           # Phase 6 — Zephyr application
│   └── autoware_sentinel_linux/     # Phase 7.1 — Linux native binary
├── tests/                           # Phase 7.2 — Integration tests (nextest)
│   ├── Cargo.toml                   # sentinel-tests crate
│   ├── src/
│   │   ├── lib.rs                   # TestError, helpers (count_pattern, wait_for_port)
│   │   ├── process.rs               # ManagedProcess, process group cleanup
│   │   ├── ros2.rs                  # Ros2Process (topic pub/echo with rmw_zenoh_cpp)
│   │   └── fixtures/                # rstest fixtures
│   │       ├── mod.rs
│   │       ├── zenohd_router.rs     # ZenohRouter (ephemeral port allocation)
│   │       └── sentinel.rs          # sentinel_binary(), start_sentinel()
│   └── tests/
│       └── transport_smoke.rs       # Sentinel ↔ ROS 2 transport tests (9 tests)
├── .config/
│   └── nextest.toml                 # nextest profiles, test group serialization
├── docs/
│   ├── roadmap/                     # Phase docs (1–7)
│   ├── guides/                      # Developer guides
│   │   └── zephyr-setup.md         # Zephyr workspace setup guide
│   └── research/                    # Architecture research
├── scripts/
│   ├── activate_autoware.sh         # Source Autoware ROS 2 environment
│   └── zephyr/
│       └── setup.sh                 # Zephyr workspace initialization
├── west.yml                         # Zephyr west manifest
├── justfile                         # Root convenience recipes
├── autoware-repo -> ~/repos/autoware/1.5.0-ws  # Autoware source (symlink)
└── CLAUDE.md                        # This file
```

## Build Commands

Per-package builds — run from each crate directory:

```bash
cd src/autoware_stop_filter
just generate        # generate message crates + .cargo/config.toml
just build           # cargo build
just test            # cargo test
```

Root justfile for convenience:

```bash
just build              # build all packages + Zephyr + Linux sentinel (with --tests)
just build-zephyr       # build Zephyr application only (native_sim)
just build-sentinel-linux # build Linux sentinel binary
just test               # test all packages (unit tests)
just test-integration   # run integration tests with nextest
just test-transport     # run transport smoke tests only
just cross-check        # cargo check --target thumbv7em-none-eabihf in each
just generate-bindings  # regenerate messages in all packages
just format             # cargo fmt on all packages
just ci                 # format-check + cross-check + test
just verify-kani        # run Kani verification on all harness crates
just verify-verus       # run Verus verification
just verify             # run all verification (Kani + Verus)
```

All algorithm crates must be `#![no_std]` and cross-compile to `thumbv7em-none-eabihf`.

## Message Generation

**Never hand-write message types.** Each package has a `package.xml` declaring message
dependencies and a `justfile` with a `generate` recipe.

```bash
cd src/autoware_stop_filter
just generate        # runs: cargo nano-ros generate --force
```

The generate command:
1. Reads `package.xml` for `<depend>` entries
2. Resolves transitive dependencies from the ROS 2 ament index
3. Generates Rust crates into `generated/`
4. Creates `.cargo/config.toml` with `[patch.crates-io]` entries

### Prerequisites

The Autoware environment (ROS 2 + Autoware message packages) must be available.
Edit `scripts/activate_autoware.sh` to source the correct setup script for your system.
The root `just generate-bindings` recipe sources this script automatically.

### Adding new message dependencies

1. Add `<depend>pkg_name</depend>` to the package's `package.xml`
2. Source the ROS 2 environment (and ament prefix for Autoware msgs)
3. Regenerate: `just generate` in the package directory

### Post-generation fix: `[f64; 36]` Default

Generated `geometry_msgs` has types with `[f64; 36]` covariance arrays that fail to compile
because `Default` is not implemented for `[T; N]` where N > 32. After generation, run:

```bash
python3 tmp/fix_covariance_default.py src/autoware_*/generated/geometry_msgs src/autoware_*/generated/geographic_msgs
```

Generated crates are `#![no_std]`, use `heapless::String<N>` / `heapless::Vec<T, N>`,
and implement `Serialize`, `Deserialize`, `RosMessage` traits.

## Porting Conventions

### Two-layer architecture

Each ported package follows two layers:

1. **Algorithm library** (`src/autoware_*/src/lib.rs`):
   - Pure `#![no_std]` logic operating on message types
   - No ROS/nros dependencies — only message crates
   - Fully testable with `cargo test` (no network, no executor)
   - Struct with `new()` constructor and methods like `apply()`, `convert()`, `decide()`

2. **Node binary** (future, when wiring is needed):
   - Uses `nros` high-level API (`Executor`, `Node`, publishers, subscribers)
   - Wires callbacks to algorithm instances
   - Depends on `nros = { ..., features = ["std", "rmw-zenoh", "platform-posix"] }`

### Algorithm crate Cargo.toml pattern

```toml
[package]
name = "autoware_stop_filter"
version = "0.1.0"
edition = "2024"
license = "Apache-2.0"
publish = false

[dependencies]
nav_msgs = { version = "*", default-features = false }
geometry_msgs = { version = "*", default-features = false }

[features]
default = []
std = ["nav_msgs/std", "geometry_msgs/std"]
```

Message deps use `version = "*"` — resolved via `[patch.crates-io]` in the auto-generated
`.cargo/config.toml` to point at local `generated/` crates.

### nros node wiring pattern

When creating ROS node binaries (Phase 2+), use the high-level API:

```rust
use nros::prelude::*;
use nav_msgs::msg::Odometry;

let config = ExecutorConfig::from_env().node_name("my_node");
let mut executor = Executor::<_, 8, 8192>::open(&config)?;
let mut node = executor.create_node("my_node")?;

// Create typed publisher
let publisher = node.create_publisher::<Odometry>("/output/odom")?;

// Register subscription callback on executor
executor.add_subscription::<Odometry, _>("/input/odom", move |msg| {
    // process msg, publish result
    publisher.publish(&filtered_msg).ok();
})?;

// Service server (callback-based)
executor.add_service::<OperateMrm, _>("/operate", |request| {
    OperateMrmResponse { /* ... */ }
})?;

// Service client (promise-based)
let mut client = node.create_client::<OperateMrm>("/operate")?;
let mut promise = client.call(&request)?;
let response = promise.wait(&mut executor, 5000)?;

// Timer (periodic)
executor.add_timer(33, move || { /* 30 Hz */ })?;

// Spin
executor.spin_blocking(SpinOptions::default())?;
```

**Do NOT use low-level RMW traits** (`Session`, `Publisher::publish_raw`, etc.) in
application code. Always use the `Executor`/`Node` layer.

## Known Issues

- **`[f64; 36]` Default**: `Default` is not implemented for `[T; N]` where N > 32. Types
  like `PoseWithCovariance` and `GeoPoseWithCovariance` need manual `Default` impls.
  Fix with `tmp/fix_covariance_default.py` (scans geometry_msgs and geographic_msgs).
- **f32→f64 precision**: `VelocityReport` fields are `f32`. In tests, compare against
  `value_f32 as f64`, not `value_f64`.
- **Generated crate edition**: Message crates use edition 2021; algorithm crates use 2024.
- **Constants in generated msgs**: Constants defined in `.msg` files are private in generated
  modules — define your own constants in application crates.
- **LLVM SIGSEGV on cross-compile**: `autoware_adapi_v1_msgs` triggers an LLVM crash in the
  ARM register scavenger (`scavengeFrameVirtualRegs`) when building at `opt-level=0` for
  `thumbv7em-none-eabihf`. Workaround: add `[profile.dev.package.autoware_adapi_v1_msgs]
  opt-level = 1` in any crate that depends on it. See [LLVM #64277](https://github.com/llvm/llvm-project/issues/64277).

## Autoware Source Reference

Autoware 1.5.0 workspace is symlinked at `autoware-repo` (points to
`~/repos/autoware/1.5.0-ws`). `autoware_common_msgs` lives in the 1.5.0-ws source,
not alongside `autoware_control_msgs`/`autoware_vehicle_msgs` in `autoware_msgs`.

## Formal Verification (Phase 5)

Kani harnesses live inline in production crates (`#[cfg(kani)] mod verification`).
Verus proofs live in a standalone crate (`src/verification/`).

### Kani harnesses (13 total)

- **stop_filter** (5): NaN→zero, output never NaN, panic-freedom, stopped⇒zeros, moving⇒passthrough
- **vehicle_velocity_converter** (4): covariance diagonal, f32→f64 sign, panic-freedom, output NaN
- **shift_decider** (3): valid gear, dead-zone hold, NaN velocity hold
- **emergency_stop_operator** (1): bounded convergence (`#[kani::unwind(301)]`)

Crates with Kani harnesses need `[lints.rust]` in `Cargo.toml`:

```toml
[lints.rust]
unexpected_cfgs = { level = "warn", check-cfg = ['cfg(kani)'] }
```

### Verus verification crate (`src/verification/`)

Standalone crate (not in workspace) using `vstd`. Integer-scaled ghost models avoid
floating-point reasoning. Modules: `ghost_types`, `shift_decider`, `emergency_stop`,
`mrm_handler`.

### NaN safety pattern

Use negated comparisons for NaN-safe thresholds:

```rust
// WRONG: NaN.abs() < threshold → false (NaN passes through)
linear.x.abs() < self.vx_threshold

// RIGHT: !(NaN.abs() >= threshold) → !(false) → true (NaN treated as stopped = safe)
!(linear.x.abs() >= self.vx_threshold)
```

## Development Phases

| Phase | Focus | Status |
|-------|-------|--------|
| 1 | Foundation (messages + 3 trivial ports) | Complete |
| 2 | Emergency Response (MRM chain) | Complete |
| 3 | Safety Gate (vehicle command gate) | Complete |
| 4 | Validation Layer (control validator, twist2accel) | Complete |
| 5 | Formal Verification (Kani + Verus) | Complete |
| 6 | Zephyr Application (single binary) | In progress |
| 7 | Integration Testing (Autoware planning simulator) | In progress (7.1–7.2 complete) |

See `docs/roadmap/` for detailed phase docs. Roadmap docs use `- [ ]` / `- [x]` checkboxes
on subphase headers and acceptance criteria to track completion progress.

## Temporary Scripts

Place temporary Python/shell scripts in `tmp/` at the repo root. This directory is
gitignored and used for one-off data extraction, analysis, or debugging scripts.

## Zephyr Build (Phase 6)

The safety island deploys as a single Zephyr application. Setup:

```bash
./scripts/zephyr/setup.sh                         # one-time workspace init
source ../autoware-sentinel-workspace/env.sh       # set ZEPHYR_BASE, etc.
cd ../autoware-sentinel-workspace
west build -b native_sim/native/64 autoware-sentinel/src/autoware_sentinel
```

Key files:
- `west.yml` — West manifest (Zephyr v3.7.0, zephyr-lang-rust)
- `scripts/zephyr/setup.sh` — Downloads SDK, creates workspace, symlinks nano-ros
- `src/autoware_sentinel/` — Zephyr application crate (Phase 6.2+)

nano-ros is registered as a Zephyr module via `ZEPHYR_EXTRA_MODULES` in `env.sh`.
See `docs/guides/zephyr-setup.md` for full instructions.

## Integration Testing (Phase 7)

Integration tests live in `tests/` as a standalone crate (`sentinel-tests`) run with
[nextest](https://nexte.st/). Tests verify bidirectional message flow between the sentinel
Linux binary and ROS 2 (`rmw_zenoh_cpp`) through a shared zenohd router.

### Running tests

```bash
just test-integration   # all integration tests
just test-transport     # transport smoke tests only
cd tests && cargo nextest run -E 'test(test_sentinel_starts)'  # single test
```

### Test infrastructure

- **`tests/src/process.rs`** — `ManagedProcess` with RAII process group cleanup
  (`setpgid` + `PR_SET_PDEATHSIG(SIGKILL)`). Never leaves orphan processes.
- **`tests/src/ros2.rs`** — `Ros2Process` wraps `ros2 topic pub/echo` with
  `rmw_zenoh_cpp` transport and Autoware message env.
- **`tests/src/fixtures/`** — rstest fixtures:
  - `zenohd_unique` — starts zenohd on an OS-assigned ephemeral port (parallel-safe)
  - `sentinel_binary` — builds the Linux sentinel once per test run (`OnceCell`)
  - `start_sentinel()` — spawns sentinel as `ManagedProcess`, waits for "Executor ready"

### nextest configuration (`.config/nextest.toml`)

Transport smoke tests run sequentially (`max-threads = 1`) to avoid port/resource contention.
Slow timeout is 60s (these tests involve multi-process coordination with sleep waits).

### Prerequisites

- zenohd built locally (`~/repos/nano-ros/build/zenohd/zenohd`)
- ROS 2 Humble with `rmw_zenoh_cpp` installed
- Autoware message packages (`/opt/autoware/1.5.0/`)

### Adding new integration tests

1. Create a new test file in `tests/tests/`
2. Register it as `[[test]]` in `tests/Cargo.toml`
3. Use rstest fixtures from `sentinel_tests::fixtures`
4. If tests need serialization, add to a test group in `.config/nextest.toml`

## nano-ros Reference

The nano-ros repo is at `~/repos/nano-ros/`. Key references:
- `packages/core/nros/` — unified facade crate (use this as dependency)
- `packages/core/nros-node/` — Executor, Node, callback infrastructure
- `packages/core/nros-core/` — foundation types, re-exports `heapless`
- `packages/core/nros-serdes/` — CDR serialization
- `examples/native/rust/zenoh/` — reference examples for node patterns
- `docs/guides/message-generation.md` — codegen guide
