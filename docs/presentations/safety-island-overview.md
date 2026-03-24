# Autoware Safety Island

## Building a Fail-Safe Companion with nano-ros

Hsiang-Jui (Jerry) Lin, National Taiwan University

---

## 1. Why a Safety Island?

The main compute runs hundreds of nodes on Linux — complex, non-deterministic, hard to certify.

**If it hangs, who stops the car?**

An independent MCU that monitors the main compute and brings the vehicle to a controlled stop.

|                   | Main Compute         | Safety Island     |
|-------------------|----------------------|-------------------|
| **Hardware**      | Cortex-A (Orin, x86) | Cortex-M/R (MCU)  |
| **OS**            | Linux                | RTOS / bare-metal |
| **Complexity**    | Millions of LOC      | Single binary     |
| **Certification** | QM–ASIL B            | ASIL C/D target   |
| **Failure mode**  | Detectable           | Must not fail     |

> Industry practice: NVIDIA DRIVE [1], Apex.AI [2], eSOL eMCOS [3] all use this pattern.

---

## 2. Autoware Sentinel — Our Reference Implementation

A single `#![no_std]` Rust binary replacing 7 Autoware safety nodes.
30 Hz deterministic loop. Zero heap allocation.

![Architecture](images/architecture.svg)

**Sense → Decide → Act:**

- Heartbeat watchdog detects main compute failure (500 ms timeout)
- MRM handler activates emergency or comfortable stop
- Vehicle command gate enforces rate/jerk limits
- Control validator checks safety bounds before actuator output

5 topics in (from Autoware), 30 topics out (to vehicle).
Connected via Zenoh (rmw_zenoh on both sides).

---

## 3. nano-ros: The Foundation

A `no_std` ROS 2 runtime for embedded real-time systems.

**API compatible** — same programming model across languages:

```rust
// Rust (rclrs-compatible)
let mut node = executor.create_node("safety")?;
let pub_ = node.create_publisher::<Control>("/cmd")?;
let _sub = node.create_subscription("/vel", |msg: VelocityReport| { ... })?;
```

```c
// C (rclc-compatible)
nros_node_t node;
nros_publisher_t pub;
nros_create_node(&node, &exec, "safety");
nros_create_publisher(&pub, &node, "/cmd", control_ts);
```

| | Support |
|---|---|
| **Languages** | Rust, C, freestanding C++14 |
| **Platforms** | POSIX, Zephyr, FreeRTOS, NuttX, ThreadX, bare-metal |
| **RMW** | zenoh-pico, DDS-XRCE |
| **Transport** | TCP, UDP, serial (UART), IVC |
| **Dev/test** | Full stack runs in QEMU — no hardware needed |

---

## 4. Why Rust for Safety-Critical Embedded?

**`no_std` + alloc-free = bare-metal native.**
No runtime, no GC, no hidden allocations. The entire binary is statically analyzable.

**`async/await` with vendored runtime** — the runtime is a library, not the language:

```rust
// RTIC: interrupt-driven tasks with async/await on Cortex-M
#[task(local = [executor], priority = 1)]
async fn net_poll(cx: net_poll::Context) {
    loop {
        cx.local.executor.spin_once(0);   // non-blocking I/O
        Mono::delay(10.millis()).await;    // yields to lower-priority work
    }
}

#[task(local = [publisher], priority = 2)]  // preempts net_poll
async fn publish(cx: publish::Context) {
    // This task runs at higher priority — guaranteed to meet deadline
    publisher.publish(&Control { velocity: 0.0 })?;
}
```

**Rich ecosystem:** 4,000+ crates on crates.io for `no_std`. Community-maintained HAL traits, drivers, and board support packages.

---

## 5. The Bare-Metal Software Stack

People often ask: *"How can a program run without an OS?"*  On a microcontroller, your
binary **is** the only software.  There is no kernel, no scheduler, no `main()` that
returns to a shell.  The hardware boots straight into your code.

### 5.1 Layers of the Stack

```
┌─────────────────────────────────────────────────────────┐
│                    Application                          │
│  Sentinel logic: watchdog, MRM handler, command gate    │
├─────────────────────────────────────────────────────────┤
│                    nano-ros                              │
│  ROS 2 API: Node, Publisher, Subscriber, Executor       │
├─────────────────────────────────────────────────────────┤
│                    RTIC framework                        │
│  Priority scheduler — maps tasks to HW interrupts       │
├─────────────────────────────────────────────────────────┤
│              Hardware Abstraction Layer (HAL)            │
│  Rust PAC/HAL crates: GPIO, UART, SPI, Ethernet, Timer  │
├─────────────────────────────────────────────────────────┤
│               Cortex-M / Cortex-R hardware              │
│  NVIC (interrupt controller), SysTick, MPU, DMA         │
└─────────────────────────────────────────────────────────┘
```

**Compare this with a Linux-based ROS 2 stack:**

```
┌─────────────────────────────┐   ┌──────────────────────────────┐
│  Linux + ROS 2 (rclcpp)     │   │  Bare-metal + nano-ros       │
├─────────────────────────────┤   ├──────────────────────────────┤
│  App (ROS nodes)            │   │  App (Sentinel algorithms)   │
│  rclcpp / rclrs             │   │  nano-ros (no_std)           │
│  rmw + DDS (millions LOC)   │   │  zenoh-pico (~30k LOC)       │
│  Linux kernel               │   │  RTIC (zero-cost scheduler)  │
│  Bootloader (U-Boot/GRUB)   │   │  HAL + startup code          │
├─────────────────────────────┤   ├──────────────────────────────┤
│  ~10M+ lines of code        │   │  ~50k lines of code          │
│  Non-deterministic           │   │  Deterministic               │
│  Hard to certify (QM–ASIL B)│   │  Certifiable (ASIL C/D)      │
└─────────────────────────────┘   └──────────────────────────────┘
```

Each layer is small and auditable.  No virtual memory, no process scheduler, no syscalls.
Every function call is a direct branch.  Every memory access is to a known static address.

### 5.2 How RTIC Replaces an RTOS

A traditional RTOS (FreeRTOS, Zephyr, ThreadX) provides:

1. **A scheduler** — decides which thread runs next
2. **Synchronization** — mutexes, semaphores, message queues
3. **Memory management** — heap allocator, stack allocation per thread

RTIC eliminates all three by leveraging **hardware interrupt priorities** directly:

| Concept | Traditional RTOS | RTIC |
|---|---|---|
| **Task** | OS thread (needs dedicated stack) | Interrupt handler (shares a single stack) |
| **Scheduling** | Software scheduler (tick-driven) | Hardware NVIC (zero-latency preemption) |
| **Priority** | Software priority queue | Hardware priority register |
| **Mutex** | Runtime lock (can deadlock) | Priority ceiling (deadlock-free by construction) |
| **Context switch** | Save/restore registers via kernel | Hardware interrupt entry/exit (~12 cycles on Cortex-M) |
| **Footprint** | Kernel + per-thread stacks (KB each) | No kernel, single stack (~1 KB total) |

**The key insight:** On Cortex-M, the NVIC (Nested Vectored Interrupt Controller) already
implements a hardware priority-based preemptive scheduler.  RTIC simply maps your tasks
onto interrupt vectors.  The compiler statically verifies that shared resources are accessed
without deadlocks using the Stack Resource Policy (SRP) [5].

### 5.3 Anatomy of a Bare-Metal Application

Here is what an RTIC application looks like in practice:

```rust
#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [SPI1, SPI2])]
mod app {
    use super::*;

    // ── Shared resources (accessed across tasks via priority ceiling) ──
    #[shared]
    struct Shared {
        executor: Executor,            // nano-ros executor (pub/sub engine)
        watchdog_ok: bool,             // heartbeat status
    }

    // ── Local resources (owned by exactly one task, no locking needed) ──
    #[local]
    struct Local {
        led: gpio::Pin<Output>,        // diagnostic LED
        timer: hal::timer::CounterHz,  // hardware timer peripheral
    }

    // ── init: runs once at boot, returns resources ──
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;            // raw peripheral access
        let clocks = dp.RCC.configure().sysclk(168.MHz()).freeze();
        let led = dp.GPIOA.pa5.into_push_pull_output();
        let timer = dp.TIM2.counter_hz(&clocks);

        let executor = Executor::open(&config).unwrap();
        // Set up subscriptions, publishers, timers on the executor...

        // Start the 30 Hz control loop
        control_loop::spawn().unwrap();

        (Shared { executor, watchdog_ok: true },
         Local { led, timer })
    }

    // ── Priority 1: network I/O (lowest priority — preemptible) ──
    #[task(shared = [executor], priority = 1)]
    async fn net_poll(mut cx: net_poll::Context) {
        loop {
            cx.shared.executor.lock(|exec| exec.spin_once(0));
            Mono::delay(1.millis()).await;   // yield to other tasks
        }
    }

    // ── Priority 2: 30 Hz control loop ──
    #[task(shared = [executor, watchdog_ok], local = [led], priority = 2)]
    async fn control_loop(mut cx: control_loop::Context) {
        loop {
            let ok = cx.shared.watchdog_ok.lock(|w| *w);
            if !ok {
                // MRM: bring vehicle to emergency stop
                cx.local.led.set_high();     // diagnostic indicator
            }
            cx.shared.executor.lock(|exec| exec.run_control_cycle());
            Mono::delay(33.millis()).await;   // 30 Hz
        }
    }

    // ── Priority 3: brake safety check (highest — never delayed) ──
    #[task(shared = [watchdog_ok], priority = 3)]
    async fn brake_check(mut cx: brake_check::Context) {
        // This task preempts everything else — guaranteed to run
        cx.shared.watchdog_ok.lock(|w| {
            *w = check_heartbeat_within_deadline();
        });
    }
}
```

**What happens at runtime:**

```
Time ──────────────────────────────────────────────────────────►

Priority 3  ·········█·····················█·················█···
             (brake)  ↑ preempts           ↑ preempts
Priority 2  ···██████·█████████████████████·███████████████··████
             (control loop, runs every 33ms)
Priority 1  ██·······························█████████████████···
             (net_poll, runs whenever higher priorities are idle)
```

- `brake_check` at priority 3 **always preempts** — it cannot be delayed by any other task
- `control_loop` at priority 2 preempts `net_poll` but yields to `brake_check`
- `net_poll` at priority 1 runs in the gaps, handling incoming/outgoing Zenoh messages
- All three tasks share **one stack** — no per-task stack allocation needed

### 5.4 Benefits of This Architecture

**For certification:**
- The entire binary is one compilation unit — whole-program analysis is possible
- No OS kernel to qualify (IEC 61508 / ISO 26262 tool qualification)
- Stack usage is statically bounded (single stack, analyzable by tools like `cargo-call-stack`)
- Interrupt latency is bounded by hardware — no kernel jitter

**For reliability:**
- No heap → no fragmentation, no out-of-memory at runtime
- No threads → no deadlocks, no priority inversion (proven by SRP [5])
- No dynamic dispatch → every code path is visible to the compiler and verifier

**For performance:**
- Context switch = hardware interrupt entry (~12 cycles on Cortex-M, ~0 on Cortex-R)
- No syscall overhead — peripheral access is a direct memory-mapped register write
- Entire binary fits in flash + SRAM (typically 64–256 KB)
- Boot to operational: <10 ms (no OS init, no service discovery wait)

**For development:**
- Same Rust `async/await` syntax as desktop — familiar to systems programmers
- Full test suite runs on the host (`cargo test`) — no hardware needed for logic testing
- QEMU emulation for integration testing before hardware is available

---

## 6. Formal Verification: Miri, Kani, Verus

Three tools, three scopes — all run on the actual Rust source code.

**Miri** — runtime UB detector (runs test suite under interpreter)
- Catches: use-after-free, uninitialized reads, unbounded loops, integer overflow

**Kani** — bounded model checking (exhaustive for all inputs up to a bound)

```rust
#[kani::proof]
#[kani::unwind(301)]
fn convergence_from_20_mps() {
    let mut op = EmergencyStopOperator::new(Params::default());
    op.set_initial_velocity(20.0);  // 72 km/h
    op.operate(true);

    let dt = 1.0 / 30.0;
    let mut step = 0u32;
    while op.velocity() > 0.0 && step < 300 {
        op.update(dt);
        step += 1;
    }
    assert!(op.velocity() == 0.0, "must reach zero");  // ← proved for ALL f32 paths
}
```

**Verus** — unbounded deductive proofs (mathematical guarantees, zero runtime cost)

```rust
proof fn ramp_phase_terminates(s: DecelState)
    requires s.acceleration_mms2 > TARGET_ACCEL_MMS2,
    ensures ({
        let next = decel_step(s);
        next.acceleration_mms2 < s.acceleration_mms2
            || next.acceleration_mms2 == TARGET_ACCEL_MMS2,
    })
```
Verus proofs live alongside production code and verify end-to-end properties:
reliability, latency bounds, state machine invariants.

---

## 7. Network Safety & Target Platforms

### TSN: Deterministic Network Guarantees

| Standard | What it does | Safety value |
|---|---|---|
| 802.1AS | Sub-microsecond time sync | Coordinated scheduling |
| 802.1Qbv | Scheduled traffic windows | Provable worst-case latency |
| 802.1Qci | Per-stream policing | Babbling idiot protection (FFI) |
| 802.1CB | Frame replication | Seamless redundancy |

ThreadX + NetX Duo [6] provides a complete TSN stack today.
Hardware required: TSN-capable Ethernet MAC (e.g., NXP S32K3 [7], i.MX RT1180).
IEEE 802.1DG-2025 defines the automotive TSN profile [8].
nano-ros adds E2E safety on top: CRC-32 + sequence tracking (AUTOSAR E2E compatible [9]).

### Target Platforms

| Platform | MCU | Transport | Status |
|---|---|---|---|
| QEMU MPS2-AN385 | Cortex-M3 | Ethernet / Serial | Working (dev/test) |
| Zephyr native_sim | x86_64 | POSIX sockets | Working (CI) |
| **NVIDIA Orin SPE** | Cortex-R5F | IVC (shared memory) | Planned |
| **NXP S32K3** | Cortex-M7 lockstep | TSN Ethernet | Planned (ASIL D) |

---

## 8. Discussion

**Open questions for the working group:**

- Which Autoware nodes belong on the safety island?
  - Current 7 system nodes? Add trajectory followers? Fewer?
- What ASIL level should the safety island target?
  - Drives tool qualification, hardware selection, V&V effort
- Preferred hardware platform?
  - Orin SPE (co-located) vs. external automotive MCU (NXP S32K3) vs. both?
- TSN adoption timeline?
  - Standard Ethernet is simpler; TSN adds determinism guarantees
- Integration with Autoware CI?
  - Sentinel tested against planning simulator today — how to make this continuous?

**Resources:**

- nano-ros: https://github.com/nicosio2/nano-ros
- Safety island architecture study: `docs/research/autoware-safety-island-architecture.md`
- TSN assessment: `docs/research/tsn-safety-island-assessment.md`
- ISO 26262 gap analysis: `docs/research/autosar-iso26262-gap-analysis.md`

**References:**

- [1] NVIDIA, "DRIVE OS 6.0 Safety Documentation," NVIDIA Developer, 2024. Orin SoC includes on-chip safety island (4x Cortex-R52 DCLS) with separate clock/power for ASIL D functions.
- [2] Apex.AI, "Apex.OS — ISO 26262 ASIL D Certified ROS 2," AWF TSC, April 2021. ~14 person-years certification effort, 300 requirements, 100% MC/DC coverage. Runs on QNX / INTEGRITY / PikeOS.
- [3] eSOL, "eMCOS POSIX — ISO 26262 ASIL D Certified RTOS," eSOL Co., Ltd. Multi-kernel POSIX PSE 53 architecture; Autoware ports from Linux with minimal effort.
- [4] RTIC Framework, https://rtic.rs/. Hardware-accelerated real-time task scheduler for Cortex-M using the NVIC as a priority-based preemptive dispatcher.
- [5] T. P. Baker, "A Stack-Based Resource Allocation Policy for Realtime Processes," *Proc. IEEE Real-Time Systems Symposium*, 1990. Proves deadlock-freedom and bounded priority inversion for single-processor systems.
- [6] Eclipse ThreadX, "NetX Duo — TSN APIs," https://github.com/eclipse-threadx/netxduo. Open-source (MIT) TCP/IP + TSN stack with 802.1AS, 802.1Qbv support.
- [7] NXP, "S32K3 Microcontrollers for General Purpose," https://www.nxp.com/products/processors-and-microcontrollers/s32-automotive-platform/s32k-general-purpose-mcus/s32k3-microcontrollers-for-general-purpose:S32K3. Dual Cortex-M7 lockstep, ASIL D, TSN-capable Ethernet.
- [8] IEEE, "802.1DG-2025 — TSN Profile for Automotive In-Vehicle Ethernet," IEEE Standards Association, 2025. Defines mandatory/optional TSN features for in-vehicle networks.
- [9] AUTOSAR, "E2E Protocol Specification R22-11," AUTOSAR Classic Platform. CRC + sequence counter + data ID for end-to-end communication protection.
