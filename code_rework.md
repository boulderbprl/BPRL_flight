# BPRL_flight Code Rework Plan

Status: planning document, drafted from an ArduPilot comparison deep-dive (2026-07-30).
Nothing in this file has been implemented yet.

## 1. Background — what ArduPilot actually does, and why it matters here

This section is the condensed technical grounding for the rework below. Full detail
lives in this session's conversation; this is the part that's load-bearing for the plan.

### 1.1 ArduPilot's rate controller

- Default main-loop / scheduler rate for Copter is **400 Hz**
  (`SCHEDULER_DEFAULT_LOOP_RATE`, `AP_Scheduler.cpp`).
- The rate controller (`AC_PID`/`rate_controller_run()`) executes once per main-loop
  iteration — **400 Hz**, independent of raw IMU sample rate. It reads
  `ahrs.get_gyro_latest()` directly — never routed through the EKF.
- Raw gyro *is* filtered (harmonic notch → LPF) at full native ODR (up to several kHz)
  inside the IMU backend, but only the single latest filtered sample is published per
  main-loop tick — oversampling only benefits the filter's anti-aliasing, not the rate
  controller's update rate.
- A separate, **opt-in, off-by-default** "fast rate thread" (`FSTRATE_ENABLE=0` by
  default) can tie the rate controller directly to individual gyro samples, decimated
  by a whole-number divisor of the true IMU ODR (never a fractional/independent rate).
  Most installs, including the comparison baseline used in this project, do not run
  this — the 400 Hz default is the relevant comparison point.

### 1.2 ArduPilot's EKF3

- Full state prediction + covariance propagation is **decimated to ~83 Hz**
  (`EKF_TARGET_DT = 0.012f`, i.e. 12 ms), via a **deterministic integer frame counter**
  (`framesSincePredict` vs `_framesPerPrediction`), not a separate thread or timer —
  purely a CPU-saving measure for boards much weaker than what this project targets.
- Critically, **`calcOutputStates()` (the "output predictor") runs every single tick**,
  unconditionally — a cheap complementary-filter correction that integrates the newest
  IMU delta-angle/delta-velocity onto the attitude estimate at the full 400 Hz rate,
  continuously nudged toward the slower full-EKF correction. So the angle controller
  always reads a smooth, current-tick attitude estimate — **not** a value held/stepped
  every 5 ticks.
- Net effect: **nothing in ArduPilot's control path ever consumes a value produced by
  an independently-clocked separate thread.** All decimation is either (a) a
  deterministic frame counter sharing the *same* clock as the consumer, or (b) an
  explicit integer divisor of a hardware sample rate with blocking hand-off (fast rate
  thread only). There is no scenario where two free-running timers at unrelated rates
  hand data to each other.

### 1.3 Where BPRL_flight currently differs

- `StateEstThread` (full 3-lane EKF + gyro filter chain) runs free-running at **625 Hz**;
  `ControlThread` runs free-running at **400 Hz**. Both use independent
  `chThdSleepUntilWindowed` timers with no synchronization between them beyond a mutex
  around the shared `g_state`/`g_euler` buffers.
- 625:400 reduces to 25:16 — **not an integer ratio**. The number of `StateEstThread`
  writes landing between two consecutive `ControlThread` reads alternates between 1 and
  2 in a repeating-but-irregular pattern (9-of-16 vs 7-of-16 over the 40 ms beat
  period), and real RTOS scheduling jitter (mutex contention, interrupt latency, thread
  priority preemption) smears this further into a broadband disturbance rather than a
  single clean tone.
- This is believed to be a plausible structural contributor to the reported high-
  frequency roll-axis jitter (present at hover, independent of stick input) and to the
  "responds to input then rings" behavior in logged rate-controller output — both
  consistent with a lightly-damped closed-loop mode being continuously excited by this
  self-inflicted timing noise, on top of whatever margin it costs outright as added
  loop latency.
- **Not yet confirmed against real flight data** — `LogThread` is fixed at 50 Hz and
  `DebugThread`/`$TEL` at 10 Hz, both well below what's needed to resolve this
  (Nyquist limit ~25 Hz for the SD log; the suspected vibration is plausibly at or
  above that). **A higher-rate diagnostic capture (≥400 Hz, raw + filtered gyro + final
  controller output) is a prerequisite before/alongside implementing Section 2**, to
  confirm this is the dominant mechanism rather than restructuring on theory alone.

---

## 2. Rework Item 1 + 2: Merge state estimation and control into one 400 Hz thread

**Decision:** merge `StateEstThread` and `ControlThread` into a single thread running
at 400 Hz, running the full 3-lane EKF (predict + covariance + fusion) every tick — no
decimation of the estimator.

### Rationale

- Matches ArduPilot's actual architecture: no cross-thread boundary in the
  estimator→controller path at all, which removes the jitter mechanism in Section 1.3
  by construction rather than by tuning around it.
- EKF compute cost was already assessed as negligible on this MCU even at higher rates
  (prior analysis: ~19.7M MACs/s for the 3-lane predict at 800 Hz on the STM32H7 —
  trivial headroom at 400 Hz). No CPU-driven reason to keep the concurrency.
- Running the full EKF at 400 Hz (rather than ArduPilot's decimated ~83 Hz core +
  400 Hz output-predictor split) is not a downgrade — it gives *finer* covariance
  updates than ArduPilot's default, at a CPU cost this board can afford. ArduPilot's
  split exists to save cycles on weaker boards, not because slower is more correct.
  Revisit only if real profiling later shows CPU pressure — implement the targeted
  covariance-decimation-with-fast-output-predictor pattern then, not a blanket slowdown.

### Implementation notes

- Fold `state_mgr.update(...)` directly into the top of the (single) control loop body,
  ahead of the `FlightStateMachine`/mixer stage — replacing the current
  `StateEstThread` loop entirely.
- `state_mtx` around `g_state`/`g_euler` can likely be removed entirely for this path
  (no longer a cross-thread handoff). Mutexes around the *inputs* to the estimator
  (`g_imu`, `g_can_imu`, `g_mocap`, `g_baro`, `g_rpm_gated`) stay — those are still fed
  by genuinely independent threads/interrupt contexts (`SPIThread`, `I2CThread`, CAN
  RX callback) and that boundary is legitimate, unlike the estimator→control one.
- Rate-dependent constants currently tuned for 625 Hz need rescaling to 400 Hz, same
  category of fix as the earlier 500→800 Hz rescaling pass — audit at minimum:
  `GRAV_VIBE_ALPHA` (EKF.hpp), `CAN_TIMEOUT_TICKS`, notch filter slew-rate-per-tick
  terms, and anywhere else a fixed tick count encodes a time constant.
- Delete the now-unused `StateEstThread` entry from `threads.cpp`'s thread-priority
  table and startup list.

### Verification

1. Get the ≥400 Hz diagnostic log (Section 1.3) **before** this change, to have a
   baseline.
2. After the merge, repeat the same capture and compare: the specific ~25 Hz-adjacent
   beat signature (or its smeared broadband equivalent) should be reduced or gone if
   this was a real contributor.
3. Confirm `ControlThread`'s new combined execution time (EKF + control + mixer) still
   fits comfortably inside the 2.5 ms / 400 Hz budget (use the existing
   `BPRL_TIMING` instrumentation, `TIM,status`).

---

## 3. Rework Item 3: Per-drone configuration system

### Goals

- Single source of truth per physical drone for everything that varies between them:
  flight-controller hardware, PID/controller gains, which controllers are available,
  RC channel mapping, motor mapping, sensors equipped, which log messages are enabled,
  and logging rate.
- Shared application code (controllers, state estimator, mixer, drivers, debug
  tooling) stays identical across drones — only the config differs.
- Build-time selection: `make DRONE=Drone1` (default to `Drone1` on a bare `make`),
  replacing today's `make BOARD=orange`/`BOARD=blue` as the primary entry point.
- Extensible: adding a new config field later shouldn't require touching every
  existing drone's file.

### Fleet context driving this

Each physical drone uses a different flight-controller board:

| Drone  | Board          | Status |
|--------|----------------|--------|
| Drone1 | CubeOrangePlus | existing |
| Drone2 | CubeBlueH7     | existing |
| Drone3 | Orqa FPV H7    | new — needs a `boards/OrqaFPVH7/` hardware port (separate work item; see 3.5) |

### 3.1 Two-level structure: DRONE selects BOARD

Keep "board" (MCU defines, HAL/peripheral wiring, which IMU driver classes get
instantiated — changes rarely, tied to physical silicon) and "drone" (gains, mixing,
sensors, logging, available controllers — changes per airframe) as logically separate
concerns in the codebase, even though the build exposes only one selector. A drone's
config *specifies* which board it's built for; the board-level mechanism (today's
`BOARD_FULL` / `BOARD_UDEFS` / `boards/<name>/board.mk` machinery in the top-level
`Makefile`) is otherwise unchanged.

This also leaves a natural home for eventually fixing the known
`SPI.cpp`-hardcodes-one-board's-wiring gap, since IMU/driver selection already has to
become board-conditional as part of adding the Orqa board regardless.

### 3.2 Build system

Replace/extend the current `BOARD` variable flow in the top-level `Makefile`:

```
DRONE ?= Drone1

include configs/$(DRONE)/config.mk      # sets BOARD_FULL/BOARD_UDEFS/etc. for this drone,
                                         # same mechanism as today's BOARD=blue/orange branch
```

`configs/<Drone>/config.mk` is a thin Makefile fragment that sets exactly the variables
the existing `Makefile` board-selection block already sets (`BOARD_FULL`,
`BOARD_UDEFS`, `BOARDDIR`) — i.e. a drone's config *chooses* one of the existing board
branches rather than duplicating the hardware wiring logic per drone. `make
DRONE=Drone1` becomes the standard invocation; existing `BOARD=` usage can be kept as a
lower-level fallback during the transition, or dropped once all three drones have
configs — worth deciding at implementation time rather than now.

Scope note: this plan targets the standalone ChibiOS `Makefile` build. Whether the
`waf`/ArduPilot build path also gets `DRONE=` support, or keeps its existing
`--board` flag as-is, is an open question to resolve during implementation — it isn't
blocking for the primary build path.

### 3.3 Per-drone parameters (C++ side)

Compile-time C++ config, not a runtime-parsed file — matches "compiled for that drone"
in the original ask, gives full compiler type-checking (typo → build error, not a
field failure), and costs nothing at runtime (no parser, no flash space for one).

`configs/<Drone>/drone_config.hpp`, included by the shared application code via an
`-I configs/$(DRONE)` include path (fixed filename, resolved per-build) or an
equivalent indirection — exact mechanism (designated initializers vs. a
defaults-struct-plus-override-function pattern) to be finalized against the project's
actual C++ standard/toolchain during implementation, but the shape is: a shared
`DroneConfig` struct with sensible defaults, and each drone's file supplying only the
fields that differ.

Fields to cover, per the original list:
- Board/flight-controller selection (feeds 3.2, plus anything C++-side that needs to
  know which board it's on beyond the existing `BPRL_BOARD_*` defines).
- Controller gains (PID, and per-controller for any future controller — see 3.4).
- RC channel mapping (channel-to-function assignment, currently hardcoded in
  `Radio.cpp`).
- Motor mapping / mixing geometry (currently hardcoded in `MotorMixer`/`Unmixer`).
- Sensors equipped (which IMUs/peripherals are actually populated on this airframe).
- Log message set + logging rate (which messages `LogThread` emits, and at what rate —
  also a natural place to make the current hardcoded 50 Hz `LogThread` period
  configurable, relevant to the Section 1.3/2 verification work too).

### 3.4 Controller list, not a single selection

Per your clarification: a drone's config declares a **list of available controllers**,
not one selected controller. PID is always present and is the default. Any additional
controller in the list (INDI today, more later) runs continuously in the background
alongside PID — this already matches the existing "shadow mode" behavior in
`FlightStateMachine::run_attitude()` (both `_pid` and `_indi` update every tick
regardless of which one is active), it just needs generalizing from a hardcoded pair
to an actual list:

- `FlightStateMachine` currently holds fixed `_pid`/`_indi` members and a single
  `_use_indi` bool. Rework to a small list/array of controller instances behind a
  common interface (each exposing an `update(...)` producing `out_cmds[3]`, matching
  what `AttitudePID`/`AttitudeINDI` already do), built from the drone config's
  controller list.
- RC-switch selection (currently `radio_use_indi()` / channel 7, binary) generalizes to
  an index into that list rather than a bool.
- A drone whose config only lists `PID` simply never offers the switch position(s) for
  anything else — naturally handled if the list itself drives what the switch can
  select, rather than the switch logic hardcoding "position 3 = INDI".

### 3.5 Orqa FPV H7 board (prerequisite/parallel work, not part of the config system itself)

Adding Drone3 needs a new `boards/OrqaFPVH7/` hardware port (MCU defines, board.c/
board.mk, linker script, IMU/peripheral driver wiring — same shape as the existing
`boards/CubeOrangePlus/` and `boards/CubeBlueH7/` directories) before Drone3's config
can point at it. This is independent effort from the config-system rework — the config
system should just be structured so that once the board port exists, adding Drone3 is
"add `configs/Drone3/` pointing at the new board," no core code changes required.

### Open items to resolve during implementation

- Exact override/defaults mechanism for `DroneConfig` (depends on confirming the C++
  standard the toolchain is built with).
- Whether `BOARD=` stays as a supported lower-level override or is fully retired in
  favor of `DRONE=`.
- Whether the waf/ArduPilot build gets `DRONE=` support in this pass or stays
  board-only for now.
- Exact `LogThread` rate configurability mechanism (compile-time constant from config
  vs. something more dynamic) — relevant both to 3.3 and to unblocking the Section 1.3
  diagnostic capture.

---

## 4. Suggested sequencing

1. **Diagnostic capture first** (Section 1.3) — confirm the thread-boundary jitter
   theory against real flight data before investing in the merge.
2. **Section 2** (thread merge) — smaller, self-contained, directly addresses the
   actively-diagnosed vibration bug, has a clear before/after verification path.
3. **Section 3** (per-drone config) — larger, touches the build system and several
   shared files (`Radio.cpp`, `MotorMixer`, `Unmixer`, `FlightStateMachine`,
   `LogThread`) that were untouched by Section 2. Cleaner to do once Section 2 has
   landed and the single-thread control loop is the stable baseline every drone's
   config will build on top of, rather than rebasing an in-flight config-system
   refactor across a threading change.
