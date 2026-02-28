# Zephyr Development Environment Setup

Setup procedure for building Autoware Sentinel as a Zephyr application with
`native_sim` testing and TAP networking.

## Overview

Autoware Sentinel uses a **sibling Zephyr workspace** alongside the repository.
The workspace contains Zephyr RTOS, the Rust language module, and HAL support.
nano-ros is registered as an extra Zephyr module via `ZEPHYR_EXTRA_MODULES`.

```
repos/
├── autoware-nano-ros/              # This repository
│   ├── west.yml                    # West manifest
│   ├── scripts/zephyr/
│   │   ├── setup.sh               # Initialize workspace
│   │   ├── downloads/             # SDK tarball cache (gitignored)
│   │   └── sdk/                   # Installed Zephyr SDK (gitignored)
│   └── src/
│       ├── autoware_sentinel/     # Zephyr application (Phase 6.2+)
│       ├── autoware_stop_filter/  # Algorithm crates
│       └── ...
│
├── nano-ros/                       # nano-ros repository (Zephyr module)
│   ├── zephyr/module.yml
│   ├── packages/core/nros/        # Executor, Node, pub/sub
│   └── ...
│
└── autoware-sentinel-workspace/    # Created by setup script
    ├── autoware-sentinel -> ../autoware-nano-ros  (symlink)
    ├── zephyr/                     # Zephyr RTOS v3.7.0
    └── modules/                    # zephyr-lang-rust, HALs
```

## Prerequisites

Install system packages (Ubuntu/Debian):
```bash
sudo apt install python3 python3-pip python3-venv cmake ninja-build aria2 git
```

Install Rust (if not already):
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Clone nano-ros as a sibling of this repository:
```bash
cd ~/repos
git clone <nano-ros-url> nano-ros
```

## Step 1: Initialize Workspace (One-Time)

```bash
./scripts/zephyr/setup.sh
```

This script automatically:
- Installs `west` and Python tools
- Downloads Zephyr SDK (~1.5 GB) with aria2c (parallel, resumable)
- Verifies download with SHA-256
- Installs SDK to `scripts/zephyr/sdk/`
- Creates sibling workspace `../autoware-sentinel-workspace/`
- Symlinks this repository into the workspace
- Fetches Zephyr RTOS and all modules
- Installs Rust embedded targets
- Creates `env.sh` with `ZEPHYR_EXTRA_MODULES` pointing to nano-ros

**Options:**
```bash
./scripts/zephyr/setup.sh --skip-sdk              # Skip SDK download/install
./scripts/zephyr/setup.sh --force                  # Recreate existing workspace
./scripts/zephyr/setup.sh --nros-path /path/to/nros  # Custom nano-ros location
```

Default nano-ros path is `../nano-ros` (sibling of this repository).

## Step 2: Source Environment

```bash
source ../autoware-sentinel-workspace/env.sh
```

This sets:
- `ZEPHYR_BASE` — Zephyr RTOS root
- `ZEPHYR_SDK_INSTALL_DIR` — Cross-compiler toolchains
- `ZEPHYR_TOOLCHAIN_VARIANT=zephyr`
- `ZEPHYR_EXTRA_MODULES` — nano-ros path (enables nros Kconfig and CMake)

## Step 3: Build

```bash
cd ../autoware-sentinel-workspace
west build -b native_sim/native/64 autoware-sentinel/src/autoware_sentinel
```

Run:
```bash
./build/zephyr/zephyr.exe
```

Cross-compile for Cortex-M7 (STM32H743):
```bash
west build -b nucleo_h743zi autoware-sentinel/src/autoware_sentinel
```

## Network Bridge Setup (For E2E Testing)

native_sim uses TAP interfaces for Ethernet. Create a bridge for Zephyr-to-host
communication:

```bash
# Create bridge + TAP (requires sudo, one-time)
sudo ip tuntap add dev zeth0 mode tap user $(whoami)
sudo ip link add name zeth-br type bridge
sudo ip link set zeth0 master zeth-br
sudo ip link set zeth0 up
sudo ip link set zeth-br up
sudo ip addr add 192.0.2.2/24 dev zeth-br
```

| Interface | IP Address | Role |
|-----------|------------|------|
| `zeth-br` (bridge) | 192.0.2.2 | Host (zenohd router) |
| `zeth0` (TAP) | — | Zephyr instance |
| Zephyr app | 192.0.2.1 | Safety island application |

Start a zenoh router on the host for Zephyr to connect to:
```bash
zenohd --listen tcp/0.0.0.0:7447
```

Teardown:
```bash
sudo ip link del zeth-br
sudo ip tuntap del dev zeth0 mode tap
```

## Kconfig Reference

All nros options are under `menuconfig NROS` (from nano-ros `zephyr/Kconfig`).

### Common Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_NROS` | bool | n | Enable nros module |
| `CONFIG_NROS_RUST_API` | bool | y | Use Rust API |
| `CONFIG_NROS_DOMAIN_ID` | int | 0 | ROS 2 domain ID |
| `CONFIG_NROS_INIT_DELAY_MS` | int | 2000 | Network init wait (ms) |

### Zenoh Options (default RMW backend)

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_NROS_ZENOH_LOCATOR` | string | `"tcp/192.0.2.2:7447"` | Router address |
| `CONFIG_NROS_MAX_PUBLISHERS` | int | 8 | Max concurrent publishers |
| `CONFIG_NROS_MAX_SUBSCRIBERS` | int | 8 | Max concurrent subscribers |
| `CONFIG_NROS_MAX_QUERYABLES` | int | 8 | Max concurrent queryables |
| `CONFIG_NROS_FRAG_MAX_SIZE` | int | 2048 | Max reassembled message size |
| `CONFIG_NROS_SUBSCRIBER_BUFFER_SIZE` | int | 1024 | Per-subscriber buffer |

The safety island application overrides these in `prj.conf` for its needs (16
publishers/subscribers to handle all topics).

## Updating the Workspace

Update Zephyr and modules to versions in `west.yml`:

```bash
cd ../autoware-sentinel-workspace
west update
```

Completely recreate:

```bash
./scripts/zephyr/setup.sh --force
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `west: command not found` | Run `pip3 install --user west` and add `~/.local/bin` to PATH |
| `nano-ros not found` | Clone nano-ros as sibling, or use `--nros-path` |
| `NROS Kconfig not found` | Source `env.sh` — it sets `ZEPHYR_EXTRA_MODULES` |
| `Build fails: no ZEPHYR_BASE` | Source `env.sh` before building |
| `Connection refused` on native_sim | Ensure zenohd listens on `tcp/0.0.0.0:7447` |
| `Permission denied on zeth` | TAP interface user mismatch, re-create with your user |
| Zenoh mutex exhaustion | Increase `CONFIG_MAX_PTHREAD_MUTEX_COUNT` (default 5 is too low) |
