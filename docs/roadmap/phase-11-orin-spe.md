# Phase 11: Orin SPE Deployment

**Status:** Not started
**Depends on:** Phase 7 (integration testing), Phase 10 (actuation porting)
**Goal:** Run the Autoware Sentinel safety island on the Jetson AGX Orin SPE (Cortex-R5F,
FreeRTOS), communicating with Autoware on the CCPLEX via IVC shared memory through a
zenohd router.

## Description

The sentinel currently runs on Linux (Phase 7) and Zephyr native_sim (Phase 6). This
phase targets the real safety island hardware: the Always-On (AON) Cortex-R5F core on
the Jetson AGX Orin SoC. The SPE runs NVIDIA's FreeRTOS V10.4.3 FSP with 256 KB BTCM.

The primary challenge is transport: the SPE has no Ethernet or dedicated serial port.
The only UART is the TCU (Tegra Combined UART), a shared debug multiplexer for all 8
SoC processors — unsuitable for data transport. **IVC (Inter-VM Communication)** is the
only viable SPE↔Linux transport: shared-memory ring buffers in a DRAM carveout mapped
into SPE address space via AST, with HSP (Hardware Synchronization Primitives) doorbell
signaling.

### Key constraints

- **256 KB BTCM** for code + data + heap + stacks (FreeRTOS ~40 KB, zenoh-pico ~60-80 KB,
  nros ~15 KB, algorithms ~30 KB, messages ~40 KB, stacks ~30 KB ≈ 215–235 KB)
- **IVC frame size**: 16 frames × 64 bytes per channel (configurable in `ivc-config.h`)
- **Float ABI mismatch**: BSP C code uses `-mfloat-abi=softfp`, Rust `eabihf` uses hard
  float — must align at link time
- **No trajectory follower**: MPC controller is too large for 256 KB; sentinel must use
  `has_external_control` mode, receiving `/control/command/control_cmd` from Autoware's
  controller running on CCPLEX
- **IVC on Orin**: L4T 36.4 BSP includes AGX Orin IVC infrastructure (echo channel demo),
  but NVIDIA's earlier forums noted IVC was "verified only on AGX Xavier." L4T 36.4 adds
  Orin support — must verify on actual hardware first

### Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  CCPLEX (Cortex-A78AE, Linux)                                │
│                                                              │
│  Autoware ──rmw_zenoh_cpp──► zenohd ◄──tcp──► IVC bridge    │
│                                 ▲               daemon       │
│                                 │                 │          │
│                              tcp:7447         /dev/aon_echo  │
│                                               (sysfs IVC)    │
└──────────────────────────────────────────────────────────────┘
                    │ DRAM carveout (shared memory) │
                    │     HSP doorbell signaling    │
┌──────────────────────────────────────────────────────────────┐
│  SPE (Cortex-R5F, FreeRTOS)                                  │
│                                                              │
│  nros Executor ──zenoh-pico──► IVC link backend              │
│    sentinel algorithms          (tegra_ivc_channel API)      │
│    (no_std, reduced set)                                     │
└──────────────────────────────────────────────────────────────┘
```

## Subphases

### - [ ] 11.1 — IVC Echo Verification on Hardware

Verify IVC communication works on the AGX Orin 64GB with L4T 36.4.4.

**Tasks:**
- [ ] Download and build SPE BSP with IVC echo enabled (`scripts/spe/download-bsp.sh`)
- [ ] Enable `aon_echo` in device tree (`status = "okay"` in DTB overlay)
- [ ] Flash SPE firmware with `ENABLE_IVC_ECHO := 1`
- [ ] Test bidirectional IVC echo from Linux:
  `echo "hello" > /sys/devices/platform/bus@0/bus@0:aon_echo/data_channel`
  and verify echo response
- [ ] Measure round-trip latency and maximum throughput (16×64B frames)
- [ ] Document device tree changes and any Orin-specific workarounds

**Acceptance criteria:**
- [ ] Bidirectional IVC echo works on AGX Orin with L4T 36.4.4
- [ ] Latency and throughput numbers recorded
- [ ] Device tree overlay documented

### - [ ] 11.2 — zenoh-pico IVC Link Backend

Add a custom IVC transport link to zenoh-pico in the nano-ros repo, enabling zenoh-pico
sessions over IVC shared memory instead of TCP/UDP/serial.

**Context:** zenoh-pico's transport is abstracted via 9 function pointers in `_z_link_t`
(open, listen, close, write, write_all, read, read_exact, read_socket, free). The serial
link (`src/link/unicast/serial.c`) is the closest reference. A custom IVC link maps these
to `tegra_ivc_channel_write()` / `tegra_ivc_channel_read()`.

**Tasks:**
- [ ] Create `src/link/unicast/ivc.c` in zenoh-pico with IVC link implementation
- [ ] Add `Z_FEATURE_LINK_IVC` feature flag (following `Z_FEATURE_LINK_SERIAL` pattern)
- [ ] Register IVC link type in `_z_endpoint_unicast_open()` / `_z_endpoint_unicast_listen()`
- [ ] Define IVC locator format: `ivc/<channel_id>` (e.g., `ivc/2`)
- [ ] Add IVC link config struct with channel ID, frame count, frame size
- [ ] Wire into zpico-sys `build.rs`: `build_zenoh_pico_freertos()` must compile `ivc.c`
  when `Z_FEATURE_LINK_IVC=1`
- [ ] Test with QEMU IVC mock or defer testing to 11.5

**Files to modify in nano-ros:**
1. `packages/zpico/zpico-sys/zenoh-pico/src/link/unicast/ivc.c` (new)
2. `packages/zpico/zpico-sys/zenoh-pico/include/zenoh-pico/link/config/ivc.h` (new)
3. `packages/zpico/zpico-sys/zenoh-pico/src/link/link.c` — register IVC in link table
4. `packages/zpico/zpico-sys/zenoh-pico/src/link/endpoint.c` — parse `ivc/` scheme
5. `packages/zpico/zpico-sys/zenoh-pico/include/zenoh-pico/link/link.h` — IVC feature flag
6. `packages/zpico/zpico-sys/zenoh-pico/src/link/unicast/transport.c` — open/listen
7. `packages/zpico/zpico-sys/zenoh-pico/CMakeLists.txt` — conditional compile
8. `packages/zpico/zpico-sys/build.rs` — add IVC source to FreeRTOS build

**Acceptance criteria:**
- [ ] IVC link compiles with `Z_FEATURE_LINK_IVC=1` for `armv7r-none-eabihf`
- [ ] Locator `ivc/2` parsed correctly
- [ ] Link open/close/write/read map to `tegra_ivc_channel_*` functions

### - [ ] 11.3 — nros-orin-spe Board Crate

Create the board support crate for nano-ros on the Orin SPE, following the existing
`nros-mps2-an385-freertos` pattern.

**Tasks:**
- [ ] Create `packages/boards/nros-orin-spe/` in nano-ros repo
- [ ] Implement `run()` function: allocates FreeRTOS task via `xTaskCreate`, calls user
  closure inside task context, returns immediately (scheduler already running)
- [ ] Implement `Config` with `zenoh_locator: "ivc/2"` default
- [ ] Implement `println!` macro via NVIDIA FSP `printf` (TCU debug output)
- [ ] Wire IVC init (`tegra_ivc_channel_get()`) before user closure runs
- [ ] Create `build.rs` to link against NVIDIA FSP static libraries
- [ ] Set target triple to `armv7r-none-eabihf` in `.cargo/config.toml`
- [ ] Resolve float ABI: either build BSP C code with `-mfloat-abi=hard` or use
  `armv7r-none-eabi` (soft float) for Rust and add shims

**Acceptance criteria:**
- [ ] Board crate compiles for `armv7r-none-eabihf`
- [ ] `run()` creates FreeRTOS task and returns
- [ ] A minimal pub-only app (no callbacks) compiles and links

### - [ ] 11.4 — Cross-compilation Infrastructure

Set up the full cross-compilation toolchain for building Rust + C SPE firmware.

**Tasks:**
- [ ] Add `rustup target add armv7r-none-eabihf` to setup instructions
- [ ] Create `scripts/spe/build.sh` — builds Rust board crate, then invokes NVIDIA Makefile
- [ ] Add `ENABLE_NROS_APP := 1` to `target_specific.mk` template
- [ ] Add `nros-app.c` C shim (calls `nros_app_rust_entry()`)
- [ ] Modify NVIDIA Makefile to link `libnros_orin_spe.a` when `ENABLE_NROS_APP=1`
- [ ] Add linker script awareness for BTCM 256 KB limit — fail build if `.text + .data +
  .bss` exceeds budget
- [ ] Create `just build-spe` recipe in root justfile
- [ ] Verify linked binary fits in 256 KB with `arm-none-eabi-size`

**Acceptance criteria:**
- [ ] `just build-spe` produces `spe.bin` with nano-ros linked in
- [ ] Binary size < 256 KB (with margin report)
- [ ] Float ABI consistent between Rust and C objects

### - [ ] 11.5 — Sentinel SPE Application

Port the sentinel to run on the SPE with a reduced feature set appropriate for the
256 KB memory constraint.

**Context:** The Linux sentinel has 32 publishers, 10 subscribers, 1 service, 1 timer,
56 parameters, and a 2.7 MB release binary. The SPE version must be drastically reduced.

**Tasks:**
- [ ] Determine minimum viable sentinel feature set for SPE:
  - Heartbeat watchdog (subscribe `/autoware/state`, publish MRM state)
  - MRM emergency stop operator (jerk-limited braking)
  - Vehicle command gate (pass-through or emergency override)
  - Drop: trajectory follower, debug topics, parameter services, control validator
- [ ] Create `src/autoware_sentinel_spe/` application crate (or in nano-ros boards)
- [ ] Wire reduced algorithm set with `Executor::<_, N, ARENA>` sized for SPE
- [ ] Use `has_external_control = true` (receive control commands from CCPLEX)
- [ ] Estimate and track memory usage: code, static data, heap, stacks
- [ ] If 256 KB is exceeded, evaluate:
  - LTO + `opt-level = "z"` for size optimization
  - Drop additional algorithms
  - Move zenoh-pico to DRAM (if accessible from SPE via AST)

**Acceptance criteria:**
- [ ] Sentinel SPE binary fits in 256 KB BTCM
- [ ] Heartbeat watchdog + emergency stop + gate functional
- [ ] Memory budget documented with per-component breakdown

### - [ ] 11.6 — Linux IVC Bridge Daemon

Create a Linux-side daemon that bridges IVC frames to zenohd via TCP, allowing the SPE
sentinel to communicate with the ROS 2 network.

**Tasks:**
- [ ] Create `src/ivc-bridge/` daemon (Rust, runs on CCPLEX Linux)
- [ ] Read/write IVC frames via sysfs:
  `/sys/devices/platform/bus@0/bus@0:aon_echo/data_channel` (or `/dev/tegra-ivc-*`)
- [ ] Open TCP connection to zenohd on `localhost:7447`
- [ ] Frame protocol: zenoh-pico sends zenoh wire frames over IVC, bridge forwards to
  zenohd TCP socket and vice versa
- [ ] Handle IVC frame fragmentation: zenoh messages may exceed 64-byte IVC frame size —
  implement simple length-prefixed reassembly
- [ ] Add systemd service file for auto-start
- [ ] Add `just run-ivc-bridge` recipe

**Acceptance criteria:**
- [ ] Bridge daemon forwards IVC↔TCP bidirectionally
- [ ] Fragmented zenoh messages reassembled correctly
- [ ] zenohd sees SPE sentinel as a connected client

### - [ ] 11.7 — Integration Test and Flash Deployment

End-to-end test: flash SPE firmware, start bridge daemon, verify sentinel participates
in Autoware planning simulator.

**Tasks:**
- [ ] Flash SPE firmware to AGX Orin (`flash.sh -k A_spe-fw`)
- [ ] Start IVC bridge daemon on Linux
- [ ] Start zenohd + Autoware planning simulator
- [ ] Verify SPE sentinel topics visible via `ros2 topic list`
- [ ] Verify heartbeat watchdog triggers MRM on simulated failure
- [ ] Measure end-to-end latency: Autoware pub → IVC → SPE → IVC → Autoware sub
- [ ] Create `just flash-spe` recipe
- [ ] Document full deployment procedure in `docs/guides/orin-spe-setup.md`

**Acceptance criteria:**
- [ ] SPE sentinel runs on real AGX Orin hardware
- [ ] Heartbeat watchdog triggers emergency stop within 3s of CCPLEX failure
- [ ] End-to-end topic latency < 5 ms (IVC shared memory)
- [ ] Deployment guide complete

## Dependencies

| Subphase | Depends on | Repository |
|----------|------------|------------|
| 11.1 | Hardware (AGX Orin 64GB) | autoware-nano-ros |
| 11.2 | 11.1 (IVC verified) | nano-ros |
| 11.3 | 11.2 (IVC link backend) | nano-ros |
| 11.4 | 11.3 (board crate) | autoware-nano-ros + nano-ros |
| 11.5 | 11.4 (cross-compilation) | autoware-nano-ros |
| 11.6 | 11.1 (IVC verified) | autoware-nano-ros |
| 11.7 | 11.5 + 11.6 | autoware-nano-ros |

Note: 11.2 and 11.6 can proceed in parallel once 11.1 confirms IVC works.

## Risk Assessment

1. **IVC not working on Orin**: L4T 36.4 adds Orin support but it's unconfirmed by us.
   Mitigation: 11.1 is the first subphase; if IVC fails, fall back to GPIO-only watchdog
   (SPE monitors heartbeat GPIO, asserts e-stop GPIO — no ROS transport needed).

2. **256 KB too small**: zenoh-pico alone may consume 60–80 KB. If the full sentinel
   doesn't fit, options: (a) reduce to watchdog-only, (b) map zenoh-pico to DRAM via AST,
   (c) use a custom minimal protocol instead of zenoh.

3. **Float ABI mismatch**: NVIDIA BSP uses softfp, Rust eabihf uses hard float. May cause
   ABI violations at FFI boundary. Mitigation: test with a minimal FFI example first; if
   incompatible, switch Rust to `armv7r-none-eabi` (softfp).

4. **IVC frame fragmentation**: zenoh messages may exceed the 64-byte IVC frame size.
   Mitigation: implement length-prefixed reassembly in both IVC link backend and bridge
   daemon (11.2 + 11.6).
