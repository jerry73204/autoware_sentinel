# autoware-nano-ros

Port of selected Autoware packages to [nano-ros](../nano-ros/), a ROS 2 runtime focused on
formal verification and real-time safety for embedded systems. The goal is a "safety island"
that can independently bring an autonomous vehicle to a safe stop.

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
│   └── autoware_shift_decider/      # Phase 1 — gear state machine
├── docs/roadmap/                    # Phase docs (1–4)
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
just build             # build all packages
just test              # test all packages
just cross-check       # cargo build --target thumbv7em-none-eabihf in each
just generate-bindings # regenerate messages in all packages
just format            # cargo fmt on all packages
just ci                # format-check + cross-check + test
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
python3 tmp/fix_covariance_default.py src/autoware_*/generated/geometry_msgs
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
  like `PoseWithCovariance` need manual `Default` impls. Fix with `tmp/fix_covariance_default.py`.
- **f32→f64 precision**: `VelocityReport` fields are `f32`. In tests, compare against
  `value_f32 as f64`, not `value_f64`.
- **Generated crate edition**: Message crates use edition 2021; algorithm crates use 2024.
- **Constants in generated msgs**: Constants defined in `.msg` files are private in generated
  modules — define your own constants in application crates.

## Autoware Source Reference

Autoware 1.5.0 workspace is symlinked at `autoware-repo` (points to
`~/repos/autoware/1.5.0-ws`). `autoware_common_msgs` lives in the 1.5.0-ws source,
not alongside `autoware_control_msgs`/`autoware_vehicle_msgs` in `autoware_msgs`.

## Development Phases

| Phase | Focus | Status |
|-------|-------|--------|
| 1 | Foundation (messages + 3 trivial ports) | Complete |
| 2 | Emergency Response (MRM chain) | Not Started |
| 3 | Safety Gate (vehicle command gate) | Not Started |
| 4 | Validation Layer (control validator, twist2accel) | Not Started |

See `docs/roadmap/` for detailed phase docs.

## Temporary Scripts

Place temporary Python/shell scripts in `tmp/` at the repo root. This directory is
gitignored and used for one-off data extraction, analysis, or debugging scripts.

## nano-ros Reference

The nano-ros repo is at `~/repos/nano-ros/`. Key references:
- `packages/core/nros/` — unified facade crate (use this as dependency)
- `packages/core/nros-node/` — Executor, Node, callback infrastructure
- `packages/core/nros-core/` — foundation types, re-exports `heapless`
- `packages/core/nros-serdes/` — CDR serialization
- `examples/native/rust/zenoh/` — reference examples for node patterns
- `docs/guides/message-generation.md` — codegen guide
