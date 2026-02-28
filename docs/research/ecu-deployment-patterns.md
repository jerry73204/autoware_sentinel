# ECU Deployment Patterns for Safety-Critical Automotive Software

Research into industrial practices for deploying safety algorithms on automotive ECUs,
with focus on the safety island / watchdog MCU use case.

## Executive Summary

The industry consensus for L2+ autonomous driving is a **three-tier topology**:

1. **Main compute** (Cortex-A SoC) — runs perception, planning, Autoware/ROS 2 under
   Linux or QNX. Not safety-certified (QM or ASIL B at most).
2. **Safety MCU** (Cortex-M/R, lockstep) — independent chip with separate power/clock.
   Runs MRM logic, watchdog, command gating. Targets ASIL D. **This is autoware-nano-ros.**
3. **Actuator ECUs** — AUTOSAR Classic on dedicated MCUs for steering, braking, powertrain.
   ASIL D certified, receive commands via CAN/FlexRay.

On the safety MCU, the universal pattern is a **single monolithic image** containing all
algorithms, scheduled by a static RTOS. There is no multi-process isolation — the entire
binary is developed and certified as one unit.

## 1. Single Image vs Multi-Process

### Safety MCUs: Always Single Image

Every production safety MCU deployment uses a single binary:

- **AUTOSAR Classic** — all Software Components (SWCs) are linked into one image with the
  RTE and Basic Software. Runnables are mapped to static OS tasks at build time. No heap,
  no dynamic loading, no virtual memory.
- **micro-ROS on MCUs** — single binary containing RTOS + middleware + application. Multiple
  ROS 2 nodes coexist in one process, scheduled by the rclc executor.
- **Zephyr** — single monolithic image. Application, kernel, and drivers are one binary.

This is not a limitation — it is a **deliberate design choice** for certifiability:
- Fully static analysis (WCET, stack usage, memory layout) at build time
- No dynamic allocation eliminates an entire class of failure modes
- Small trusted computing base — the entire system is one verified unit

### Main Compute: Multi-Process

Domain controllers (Cortex-A class) run multiple processes under POSIX:
- AUTOSAR Adaptive — each application is a separate process with MMU isolation
- Autoware on Linux/QNX — ROS 2 nodes as separate processes or composable components
- Apex.OS — safety-certified ROS 2 on QNX or INTEGRITY, multi-process

### Implication for autoware-nano-ros

All our algorithms should compose into a **single Zephyr application** — one `Executor`
running all callbacks (subscriptions, timers, services) in a single event loop. This matches
both AUTOSAR Classic and micro-ROS deployment patterns.

## 2. ISO 26262 and Freedom from Interference

### ASIL Decomposition

ISO 26262 allows an ASIL D requirement to be decomposed into two independent redundant
elements at lower ASIL levels (e.g., ASIL B(D) + ASIL B(D)). The key requirement is
**independence** — no common cause failures between the two elements.

This is the theoretical foundation for the safety island:
- Main compute (QM or ASIL B) handles normal operation
- Safety MCU (ASIL D, or ASIL B(D) as a decomposed element) provides redundant monitoring
- Physical separation (different chip, power, clock) satisfies the independence argument

### Three Types of Interference

When mixed-criticality components share hardware, ISO 26262-6 requires demonstrating freedom
from:

| Type                 | Threat                              | Mitigation                             |
|----------------------|-------------------------------------|----------------------------------------|
| Spatial              | Memory corruption across components | MPU regions, MMU, or separate chips    |
| Temporal             | CPU starvation / deadline misses    | Static scheduling, time partitioning   |
| Information exchange | Corrupted data between components   | CRC, sequence counters, E2E protection |

### Why Separate Hardware Wins

An on-chip safety island (e.g., Cortex-R52 inside NVIDIA Orin) shares the same die, package,
power delivery, and thermal environment as the main SoC. A single physical failure (solder
joint crack, voltage regulator fault, overheating) can take out both.

An **external safety MCU** provides true physical independence:
- Separate silicon die and package
- Independent power supply and voltage regulation
- Independent clock source (separate crystal)
- Direct CAN/Ethernet connection to actuators (bypasses main SoC entirely)

This makes the ASIL decomposition argument much stronger. The external safety MCU pattern
persists even as SoCs integrate internal safety islands.

## 3. Partitioning Techniques

### Within a Single MCU

| Technique | Spatial | Temporal | Used By |
|-----------|---------|----------|---------|
| MPU regions | Coarse (8-16 regions) | No | Zephyr userspace, AUTOSAR OS |
| Dual-core lockstep | N/A (redundancy) | N/A | AURIX, TMS570, S32K3 |
| Static task scheduling | No | Yes (fixed priority) | AUTOSAR Classic, Zephyr |

Zephyr's `CONFIG_USERSPACE` uses the MPU for intra-image isolation — threads can be
restricted to specific memory regions. However, the number of MPU regions is a hard
constraint (typically 8-16 on Cortex-M), limiting practical partitioning.

### Across Multiple Cores / SoCs

| Technique | Spatial | Temporal | Used By |
|-----------|---------|----------|---------|
| Hypervisor (Type 1) | Yes | Yes (time partitioning) | QNX, PikeOS, EB corbos |
| AMP (separate images) | Yes | Yes | NXP i.MX (A53 + M7) |
| Separate chips | Yes (strongest) | Yes | Safety MCU + main SoC |

### Hypervisor Players

- **QNX Hypervisor** — pre-certified ASIL D, runs QNX + Linux + AUTOSAR guests
- **PikeOS** — ASIL D / SIL 4 / DAL A certified separation kernel
- **EB corbos Hypervisor** — ASIL B certified, L4Re microkernel based
- **Green Hills INTEGRITY** — used with Apex.OS for production autonomous driving

## 4. AUTOSAR Classic Composition Model

AUTOSAR Classic is the reference for how safety MCUs compose software:

```
+--------------------------------------------------+
|                  Single ECU Image                  |
|                                                    |
|  +-----------+ +-----------+ +-----------+        |
|  |  SWC 1    | |  SWC 2    | |  SWC 3    |        |
|  | Watchdog  | | MRM Hdlr  | | Cmd Gate  |        |
|  +-----------+ +-----------+ +-----------+        |
|  |           Runtime Environment (RTE)    |        |
|  |  - routes data between SWCs            |        |
|  |  - maps runnables to OS tasks          |        |
|  |  - generated at build time             |        |
|  +----------------------------------------+        |
|  |  Basic Software (BSW)                  |        |
|  |  - OS (OSEK/VDX, static priorities)    |        |
|  |  - COM (CAN, Ethernet, SPI)            |        |
|  |  - Diagnostics (UDS, DTC)              |        |
|  +----------------------------------------+        |
+--------------------------------------------------+
```

Key properties:
- **Runnables** are the smallest schedulable unit (equivalent to our callback functions)
- Multiple runnables at the same period are packed into one OS task
- **InterRunnableVariables** handle data consistency between different-rate runnables
- Everything is statically configured — no dynamic allocation, no runtime service discovery

### Mapping to nros

| AUTOSAR Classic | nros Equivalent                                    |
|-----------------|----------------------------------------------------|
| SWC             | Algorithm crate (e.g., `autoware_mrm_handler`)     |
| Runnable        | Subscription/timer callback registered on Executor |
| RTE data port   | Publisher/Subscriber topic                         |
| OS Task         | Executor spin loop (single-threaded)               |
| BSW COM         | nros RMW layer (Zenoh or XRCE-DDS)                 |

## 5. micro-ROS Deployment Pattern

micro-ROS brings ROS 2 onto MCUs using a client-agent architecture:

- **Client** (on MCU) — single binary with RTOS + micro-ROS + application
- **Agent** (on companion computer) — bridges MCU to full DDS network

Within the client binary, **multiple nodes** coexist in one process. The rclc executor
manages callback dispatch with configurable execution order (supporting sense-plan-act
patterns).

nano-ros follows a similar pattern but without the agent dependency — it connects directly
to the ROS 2 network via Zenoh or XRCE-DDS.

## 6. Production Reference Architectures

### NVIDIA DRIVE Orin Topology

```
+------------------------------------------+
|        NVIDIA Orin SoC                    |
|  +------------------+  +------------+    |
|  | 12x Cortex-A78AE |  | 2048 CUDA  |    |
|  | (Autoware/Linux) |  | + 2x DLA   |    |
|  +------------------+  +------------+    |
|  +----------------------------------+    |
|  | On-chip Safety Island            |    |    SPI/CAN
|  | 4x Cortex-R52 (DCLS pairs)      |<--------+
|  | Separate clock + power rail      |    |    |
|  | 256KB ATCM + 128KB BTCM per R52 |    |    |
|  | Hardware Safety Manager (HSM)    |    |    |
|  +----------------------------------+    |    |
+------------------------------------------+    |
                                                |
+------------------------------------------+    |
|   External Safety MCU                     |----+
|   (NXP S32K3 / TI TMS570 / AURIX)       |
|   - Independent power + clock             |
|   - Lockstep cores (ASIL D)              |
|   - Heartbeat watchdog                    |
|   - MRM handler                           |
|   - Emergency stop operator               |
|   - Vehicle command gate                  |
|   - Direct CAN to actuators              |
+------------------------------------------+
         |
    CAN / FlexRay / Ethernet
         |
+------------------------------------------+
|   Actuator ECUs (AUTOSAR Classic)         |
|   Steering / Braking / Powertrain         |
+------------------------------------------+
```

### Apex.AI: Certified ROS 2

Apex.OS is the only ISO 26262 ASIL D certified ROS 2 stack:
- Replaced all dynamic allocation and blocking calls in ROS 2
- Runs on QNX, Green Hills INTEGRITY, PikeOS
- ~14 person-years of certification effort, 300 requirements, 100% MC/DC coverage
- Used for production autonomous driving stacks

### eSOL eMCOS: Autoware in Production

eSOL provides the Linux→production path for Autoware:
- eMCOS POSIX — certified ISO 26262 ASIL D, POSIX PSE 53 compliant
- Multi-kernel architecture provides freedom from interference at OS level
- Autoware applications port from Linux with minimal effort (same POSIX API)
- Replaces Linux under Autoware for deterministic real-time behavior

## 7. Recommended Architecture for autoware-nano-ros

Based on industry practice, the deployment architecture should be:

### Single Zephyr Application

All algorithm crates compose into one binary:

```rust
// src/autoware_safety_island/src/lib.rs

#![no_std]

use nros::prelude::*;

#[unsafe(no_mangle)]
extern "C" fn rust_main() {
    let config = ExecutorConfig::new("tcp/192.0.2.2:7447");
    let mut executor = Executor::<_, 16, 8192>::open(&config).unwrap();
    let mut node = executor.create_node("safety_island").unwrap();

    // --- Algorithm instances ---
    let stop_filter = StopFilter::new(0.1, 0.02);
    let velocity_converter = VehicleVelocityConverter::new(1.0, 0.2, 0.1);
    let mut shift_decider = ShiftDecider::new(true);
    let mut mrm_handler = MrmHandler::new(Params::default());
    let mut emergency_stop_op = EmergencyStopOperator::new(Params::default());
    // ... more algorithm instances ...

    // --- Subscriptions wire external inputs to algorithms ---
    // --- Timers drive periodic algorithm updates ---
    // --- Publishers emit commands to actuators ---

    executor.spin_blocking(SpinOptions::default()).unwrap();
}
```

### Scheduling Model

| Priority | Period       | Callbacks                                            |
|----------|--------------|------------------------------------------------------|
| Highest  | Event-driven | Heartbeat watchdog (timeout = immediate MRM trigger) |
| High     | 30 Hz        | MRM handler update, emergency stop update            |
| Medium   | 30 Hz        | Vehicle command gate, control validator              |
| Normal   | 30 Hz        | Stop filter, velocity converter, shift decider       |

In nros, all callbacks run in the single-threaded executor spin loop. Priority ordering
is achieved by registration order and timer scheduling, not OS task priorities.

### Communication Pattern

```
Main Compute (Orin)                    Safety MCU (S32K3)
  Autoware stack                        autoware-nano-ros
       |                                      |
       |-- /heartbeat ----------------------->| HeartbeatWatchdog
       |-- /vehicle/status ------------------->| VelocityConverter, StopFilter
       |-- /control/command ------------------>| ControlValidator, CmdGate
       |                                      |
       |<-- /vehicle/command ------------------| CmdGate (filtered output)
       |<-- /system/mrm_state ----------------| MrmHandler
```

Transport: Zenoh over Ethernet (preferred for deterministic QoS) or XRCE-DDS over UDP.

### Target Hardware

| Board        | CPU               | Flash/RAM   | Use Case                |
|--------------|-------------------|-------------|-------------------------|
| `native_sim` | x86_64            | Unlimited   | Development, CI testing |
| NXP S32K344  | Cortex-M7 (DCLS)  | 4MB / 512KB | Production safety MCU   |
| STM32H743    | Cortex-M7         | 2MB / 1MB   | Development prototype   |
| TI TMS570    | Cortex-R5F (DCLS) | 4MB / 512KB | Production (legacy)     |

The `thumbv7em-none-eabihf` target used by `just cross-check` maps to these Cortex-M7
boards. Cortex-R targets would need `thumbv7r-none-eabihf` (not yet used).

## References

- ISO 26262:2018 Part 6, Section 7.4.10 (Software partitioning for FFI)
- AUTOSAR Classic Platform Specification R22-11
- AUTOSAR Adaptive Platform Specification R22-11
- NVIDIA DRIVE OS 6.0 FSI Documentation
- Apex.AI ISO 26262 ASIL D Certification (AWF TSC, April 2021)
- eSOL eMCOS + Autoware Integration Guide
- Zephyr Project Safety Certification Status (2025)
- NXP S32K3 Safety Reference Manual
- Semi Engineering: "Why an External Safety MCU Remains a Cornerstone"
