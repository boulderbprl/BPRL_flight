# BPRL_flight

Standalone ChibiOS flight controller firmware for the [CubePilot](https://docs.cubepilot.org) CubeBlue H7 and CubeOrange+ autopilot hardware (STM32H753 / STM32H743 at 400 MHz). This project is loosely based on the open-source [Ardupilot](https://ardupilot.org/ardupilot/) project.

---

## TODO

- Add voltage feedback from the analog input on Power1 port (CubePilot Power Brick Mini).
- IMX5 yaw magnetometer / heading reference integration.
- Gain tuning on H7 hardware.


---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Controller](#2-controller)
3. [State Estimation (EKF)](#3-state-estimation-ekf)
4. [Math Utilities](#4-math-utilities)
5. [SD Card Logging](#5-sd-card-logging)
6. [Build and Upload](#6-build-and-upload)
7. [Comms Drivers](#7-comms-drivers)
8. [IMU Drivers](#8-imu-drivers)
9. [Building on Windows](#9-building-on-windows)

---

## 1. Project Overview

### What it does

The firmware runs RTOS threads on the STM32H7 that together perform the flight control for a quadcopter. Sensor data is read from three on-board SPI IMUs and one external CAN IMU (Inertial Sense IMX5), fused into a flight state estimate, fed through the controller, and mixed to motor PWM outputs. A low-priority logging thread records IMU and state data to an SD card in binary format.

### Directory layout

```
BPRL_flight/
├── main.cpp                  Entry point — hardware init, rate sequencer, threads_start()
├── Makefile
│
├── src/
│   ├── FlightState.hpp       Shared index enums: StateIdx, InputIdx
│   ├── threads.hpp           Shared state (g_state, g_imu, …), ThreadRates struct
│   ├── threads.cpp           All thread function bodies + global state definitions
│   │
│   ├── coms/                 Peripheral drivers
│   │   ├── SPI.hpp/.cpp      SPI bus init: 3× on-board IMU (ICM-45686 driver class) + MS5611 barometer
│   │   ├── IMUs/             ICM45686.hpp/.cpp (drives imu1/2/3 on this board), ICM42688.hpp/.cpp (supports the 1×45686+2×42688 CubeOrangePlus hardware variant, not instantiated here)
│   │   ├── Baro/             MS5611.hpp/.cpp — barometer state-machine driver (SPI1, CS=PD7)
│   │   ├── CAN.hpp/.cpp      FDCAN1 driver (register-level, interrupt-driven, self-healing — not ChibiOS's HAL_USE_CAN), IMX5 callback, device table
│   │   ├── I2C.hpp/.cpp      I2C2 driver (bus-recovery + reset), device table — strain-rate sensor fallback interface only
│   │   ├── PWM.hpp/.cpp      DShot600 / PWM motor output (MOTOR_PROTOCOL define)
│   │   ├── Radio.hpp/.cpp    Receiver input dispatch (CRSF default; SBUS.hpp/.cpp, CRSF.hpp/.cpp both compiled, selected via RADIO_PROTOCOL)
│   │   ├── MAVLink.hpp/.cpp  TELEM2 MAVLink parser — mocap ingestion (VISION_POSITION/SPEED_ESTIMATE → g_mocap)
│   │   └── CalFlash.hpp/.cpp Persistent IMU calibration bias storage (STM32H743 flash Bank2 sector 7)
│   │
│   ├── controllers/          Flight control algorithms (see src/controllers/README.md)
│   │   ├── PID.hpp/.cpp            PID base class with derivative filter + anti-windup
│   │   ├── FlightStateMachine.hpp/.cpp  Top-level mode/phase dispatcher (400 Hz)
│   │   ├── Attitude_PID.hpp/.cpp   Cascaded P+PID attitude controller
│   │   ├── Attitude_INDI.hpp/.cpp  Incremental NDI roll/pitch controller
│   │   ├── AltControl.hpp/.cpp     Altitude hold cascade (stick→climb rate→accel→thrust)
│   │   ├── PosControl.hpp/.cpp     Position hold cascade (pos→vel→lean angles)
│   │   ├── Unmixer.hpp/.cpp        RPM → physical torque (N·m) for INDI feedback
│   │   └── MotorMixer.hpp/.cpp     X-frame quadcopter mixer (torque+thrust → motor cmds)
│   │
│   ├── state_estimator/      EKF state estimation
│   │   ├── EKF.hpp/.cpp      16-state Extended Kalman Filter (one lane per IMU)
│   │   └── StateManager.hpp/.cpp  Assembles 19-state output
│   │
│   └── logging/              SD card logging
│       ├── LogMessages.hpp   Packed log structs + kLogDefs[] descriptor table
│       ├── Logger.hpp        Logger class declaration
│       └── Logger.cpp        Ring buffer + FatFS implementation
│
├── boards/
│   ├── CubeBlueH7/           STM32H753ZI board files (board.h, board.c, board.mk)
│   └── CubeOrangePlus/       STM32H743ZI board files
│
├── cfg/
│   ├── chconf.h              ChibiOS kernel configuration
│   ├── halconf.h             HAL peripheral enable switches
│   ├── mcuconf.h             STM32H7 clock tree, peripheral clock sources
│   ├── ffconf.h              FatFS configuration
│   └── bouncebuffer.h        Pass-through stub for ArduPilot-patched SDMMCv2
│
└── third_party/
    └── ChibiOS/              Pinned to ArduPilot Copter-4.6.3 (commit 88b84600)
```

### Thread priority table

| Thread | Priority | Rate | Role |
|---|---|---|---|
| SPIThread | NORMALPRIO+30 | 1 kHz | Read all three on-board IMUs + MS5611 barometer |
| CANThread | NORMALPRIO+28 | event-driven | Block on FDCAN1 RxFIFO, dispatch frames on arrival |
| StateEstThread | NORMALPRIO+25 | 800 Hz | Fuse sensors → g_state[] |
| I2CThread | NORMALPRIO+22 | 500 Hz | Poll I2C devices (strain rate sensor) |
| ControlThread | NORMALPRIO+20 | 400 Hz | FlightStateMachine → MotorMixer → motor output |
| RadioThread | NORMALPRIO+10 | 100 Hz | Read RC input → g_input[] |
| HeartbeatThread | NORMALPRIO-5 | 1 Hz | LED heartbeat |
| LogThread | NORMALPRIO-15 | 50 Hz | Snapshot all state → SD card (11 message types per tick) |
| DebugThread | NORMALPRIO-10 | 10 Hz | USB $TEL/$EKFL/$IMU streams (BPRL_DEBUG only) |

### Shared state

All inter-thread communication goes through mutex-protected globals defined in `src/threads.cpp` and declared in `src/threads.hpp`:

| Variable | Mutex | Description |
|---|---|---|
| `g_state[19]` | `state_mtx` | Fused 19-element flight state |
| `g_euler[3]` | `state_mtx` | [roll, pitch, yaw] in radians, derived from quaternion |
| `g_input[6]` | `state_mtx` | RC inputs (thrust, roll/pitch/yaw targets, flight mode switch, INDI/PID switch) |
| `g_output[4]` | `state_mtx` | Normalized motor commands 0–1000 [FR, RL, FL, RR] (0=disarm; protocol conversion in `motor_output_write()`) |
| `g_ctrl[4]` | `state_mtx` | PID torque outputs entering the mixer: [roll_tq, pitch_tq, yaw_tq, thrust] in [-1,1] |
| `g_armed` | `state_mtx` | Arm state |
| `g_imu[3]` | `imu_mtx` | Raw accel/gyro from each on-board IMU |
| `g_can_imu` | `can_imu_mtx` | Quaternion + rates from IMX5 over FDCAN1, each with an arrival timestamp for age-gating/forward-propagation in `StateManager` |
| `g_mocap` | `mocap_mtx` | NED position + velocity from motion capture radio |
| `g_baro` | `baro_mtx` | Pressure/temperature/altitude from the MS5611 barometer |
| `g_rpm_gated[4]` | `esc_mtx` | Fault-gated per-motor mechanical RPM (holds last good value below 100 RPM after having seen a real reading) — published by `ControlThread`, read by `LogThread`/`DebugThread`/`StateManager`'s notch filter |

---

## 2. Controller

> For a full breakdown of each controller, see [`src/controllers/README.md`](src/controllers/README.md).

### Architecture

The controller stack runs at 400 Hz. `FlightStateMachine` selects the active flight mode and dispatches to the attitude and throttle controllers, whose outputs feed `MotorMixer`.

```
RC input[6]  [thrust, roll_tgt, pitch_tgt, yaw_rate, flight_mode, indi_stk]
     │
     ▼
FlightStateMachine
     ├─ STABILIZE   — attitude PID/INDI + expo throttle passthrough
     ├─ ALT_HOLD    — attitude PID/INDI + altitude hold (stick → climb rate cascade)
     └─ POS_HOLD    — position hold → lean angles + altitude hold (D-axis rate)
     │
     ▼
MotorMixer  →  DShot600 (4 motors)
```

### Flight modes (3-position RC switch)

| `input[4]` value | Mode | Attitude target | Throttle |
|---|---|---|---|
| < −0.33 | STABILIZE | Pilot stick [rad] | Expo + tilt boost passthrough |
| −0.33 to +0.33 | ALT_HOLD | Pilot stick [rad] | Climb rate cascade → thrust |
| > +0.33 | POS_HOLD | PosControl lean angles [rad] | AltControl D-axis rate |

Mode changes reset all controller integrators.

### Attitude controller

Attitude is independent of flight mode, selected at runtime by `set_use_indi(bool)`:

| Controller | Description |
|---|---|
| `AttitudePID` (default) | Cascaded outer-P / inner-PID for roll, pitch, yaw |
| `AttitudeINDI` | Incremental NDI for roll/pitch using measured angular acceleration; yaw falls back to rate PID |

### AltControl — altitude hold

`AltControl` owns the throttle output for all modes. In STABILIZE it applies expo shaping and a tilt-boost (`1/min(cos φ, cos θ)`) to the raw stick. In ALT_HOLD and POS_HOLD it runs a two-loop cascade:

```
pilot stick [0,1]  ──►  stick_to_climb_rate  ──►  climb_rate_pid  ──►  accel_pid  ──►  thrust_out
```

In POS_HOLD the outer stick-to-rate step is skipped; the climb rate target comes directly from `PosControl`.

### PosControl — position hold

`PosControl` runs a two-stage cascade converting a 3-D position target to lean angles:

```
pos_error [m]  ──►  pos_P  ──►  vel_target [m/s]  ──►  vel_PID  ──►  accel_target [m/s²]
                                                                              │
                                                                    yaw rotation + atan2
                                                                              │
                                                                    roll/pitch targets [rad]
```

The NE axes include a per-axis pilot-blend state machine (PILOT → BRAKE → HOLD → RETURNING) so the drone brakes and holds automatically when the stick is released, and blends smoothly back to manual when the stick is pushed again.

### MotorMixer

Converts `[roll_tq, pitch_tq, yaw_tq, thrust]` (all normalised) to per-motor commands (0–1000) using an X-frame mixing matrix. All motors output 0 when disarmed or when attitude exceeds ~80°.

```
    FL [2]       FR [0]
         \       /
         [  body  ]
         /       \
    RL [1]       RR [3]
```

---

## 3. State Estimation (EKF)

### Architecture

State estimation runs in `StateEstThread` at 800 Hz — an exact 2:1 ratio with the 400 Hz control loop, chosen specifically to remove the undefined-phase aliasing a non-integer ratio (the previous 500 Hz was 1.25:1) introduces between the two loops. The core is a three-lane Extended Kalman Filter: one `EKF` instance per onboard IMU, orchestrated by `StateManager`. Each lane runs independently and `StateManager` selects the healthiest one (lowest smoothed innovation norm) as the primary output. All lanes share the same external sensor updates (IMX5 quaternion, mocap). `dt` is measured from actual elapsed time each tick (`chVTGetSystemTimeX()`), not a fixed nominal value, so EKF integration stays correct under scheduler jitter.

```
g_imu[0] ──► EKF lane 0 ──┐
g_imu[1] ──► EKF lane 1 ──┼──► StateManager ──► g_state[19]
g_imu[2] ──► EKF lane 2 ──┘         ▲
                                    │
              g_can_imu (IMX5) ─────┤
              g_mocap (mocap)  ─────┘
```

### EKF internal state (16 states per lane)

Each lane estimates:

| Indices | States | Description |
|---------|--------|-------------|
| 0–2 | X, Y, Z | NED position (m) |
| 3–5 | u, v, w | Body-frame velocity (m/s) |
| 6–9 | q0, q1, q2, q3 | Quaternion NED→Body, Hamilton [W,X,Y,Z] |
| 10–12 | ba_x, ba_y, ba_z | Accelerometer bias (m/s²) |
| 13–15 | bg_x, bg_y, bg_z | Gyroscope bias (rad/s) |

p/q/r, u_dot/v_dot/w_dot, and p_dot/q_dot/r_dot are **not** Kalman states, they are computed by `StateManager` and appended to the output vector.

### Predict step (800 Hz)

Each lane predicts forward using its own IMU after subtracting the estimated bias:

- **Position:** integrated from body velocity rotated to NED via the current quaternion
- **Velocity:** integrated from bias-corrected accel after removing gravity and the Coriolis term (ω × v)
- **Quaternion:** first-order integration of `dq/dt = 0.5 * q ⊗ {0, gyro_corr}`
- **Bias states:** random-walk model, only process noise Q drives them

The covariance is propagated as `P = F·P·Fᵀ + Q`. The Jacobian F includes off-diagonal blocks `∂v/∂ba = −I·dt` and `∂q/∂bg = −0.5·dt·Ξ(q)` that couple the bias states to the rest of the filter, making bias directly observable through measurement residuals.

### Measurement updates

Updates are applied in order each tick (earlier updates inform later ones):

| Step | Source | Rate | States updated |
|------|--------|------|----------------|
| 1.5 | Onboard accel (gravity vector) | 800 Hz, chi-squared gated | Quaternion (roll/pitch), Z accel bias |
| 2 | IMX5 quaternion over CAN | 200 Hz, async, age-gated (skipped if >50ms stale) and forward-propagated by its measured age before fusing | Full quaternion |
| 5 | Mocap NED position | Async | X, Y, Z |
| 5 | Mocap NED velocity → body frame | Async | u, v, w |
| 5.6 | MS5611 barometric altitude | ~100+ Hz, async, **suppressed while mocap is connected** | Z (and, via cross-covariance, w and Z accel bias) |

IMX5 angular rates (blended into StateManager's p/q/r output, not a formal EKF measurement update) are similarly age-gated — see [Sensor loss behaviour](#sensor-loss-behaviour) and the `STATEMGR_IMX5_RATE_WEIGHT` row below. Unlike the quaternion, the rate blend is a continuous sample-and-hold rather than event-gated: it re-blends whatever `g_can_imu.p/q/r` currently holds every 800 Hz tick (refreshed ~every 10ms), rather than only touching the blend on ticks where a new CAN rate frame actually arrived.

The gravity-vector update (`update_gravity`) hard-rejects samples where `|accel|` is outside `[0.1g, 3g]`, then applies a joint chi-squared gate (`GRAV_CHI2_GATE = 5σ`) over an adaptive measurement noise that grows with a lowpass-filtered vibration estimate (`GRAV_R_VIBE`) — so it's trusted less, not switched off outright, under vibration. Only the Z-axis accel bias is corrected here (X/Y residuals are ambiguous between bias and genuine horizontal acceleration; those are only resolved via mocap velocity fusion's cross-covariance with u/v). Yaw is not observable from gravity alone and requires the IMX5.

The barometer update (`update_altitude`) is a single-row, chi-squared-gated (`BARO_CHI2_GATE = 5σ`) fusion of altitude into Z — see the [MS5611 Barometer](#ms5611-barometer) section below for why it's disabled outright whenever mocap is connected rather than blended with it.

### StateManager output (19 states)

`StateManager::get_state()` assembles the shared `g_state[19]` vector:

| Indices | States | Source |
|---------|--------|--------|
| 0–2 | X, Y, Z | Primary EKF lane |
| 3–5 | u, v, w | Primary EKF lane |
| 6–8 | u_dot, v_dot, w_dot | Blended gravity+Coriolis-corrected accel, 20 Hz lowpass |
| 9–12 | q0, q1, q2, q3 | Primary EKF lane |
| 13–15 | p, q, r | Soft-blend of bias-corrected gyros across all valid lanes, optional 30% IMX5 mix, motor-vibration notch (RPM-tracked), then 20 Hz (roll/pitch) / 5 Hz (yaw) 2nd-order lowpass |
| 16–18 | p_dot, q_dot, r_dot | Finite-difference of blended rates, 20 Hz lowpass |

Quaternion uses hard lane selection (no blending). Angular rates use soft blending weighted by `1/innovation_norm`, itself low-passed at `STATEMGR_LP_BLENDW_HZ` (3 Hz) before renormalizing across lanes — the raw instantaneous weight is derived from a noisy quantity, so blending it in unsmoothed would make the blend ratio itself a fast-changing noise source.

### Tuning parameters

All EKF tuning lives in `src/state_estimator/EKF.hpp` (private `static constexpr` block). StateManager tuning lives in `src/state_estimator/StateManager.hpp`.

| Parameter | Location | Default | Effect |
|-----------|----------|---------|--------|
| `Q_BIAS_A` | EKF.hpp | 1e-7 | Accel bias random-walk rate. Increase if bias changes rapidly. |
| `Q_BIAS_G` | EKF.hpp | 1e-9 | Gyro bias random-walk rate. Typically slower than accel. |
| `P0_BIAS_A` | EKF.hpp | 0.01 | Initial accel bias uncertainty. Larger = faster startup convergence. |
| `P0_BIAS_G` | EKF.hpp | 1e-4 | Initial gyro bias uncertainty. |
| `GRAV_HARD_GATE` | EKF.hpp | 3g | Outer reject: skip gravity update outright if `\|accel\|` exceeds this multiple of g. |
| `GRAV_CHI2_GATE` | EKF.hpp | 5.0 | Joint chi-squared gate (σ) for the gravity-vector update. |
| `MOCAP_CHI2_GATE` | EKF.hpp | 5.0 | Joint chi-squared gate (σ) for mocap position/velocity updates. |
| `BARO_CHI2_GATE` | EKF.hpp | 5.0 | Chi-squared gate (σ) for the barometric altitude update. |
| `R_QUAT` | StateManager.hpp | 1e-3 | IMX5 quaternion noise. Lower = trust IMX5 more. |
| `R_GRAVITY` | StateManager.hpp | 0.5 | Accel gravity-vector noise (m/s²)². Lower = trust accel attitude more. |
| `R_MOCAP_POS` | StateManager.hpp | 1e-3 | Mocap position noise (m²). |
| `R_MOCAP_VEL` | StateManager.hpp | 1e-4 | Mocap velocity noise (m/s)². |
| `R_BARO_POS` | StateManager.hpp | 0.5 | Baro altitude noise (m²) — tune from bench log noise once flashed. |
| `STATEMGR_IMX5_RATE_WEIGHT` | StateManager.hpp | 0.3 | IMX5 share of blended p/q/r (0 = pure gyro, 1 = pure IMX5). |
| `STATEMGR_LP_UVWDOT_HZ` | StateManager.hpp | 20 | Lowpass cutoff for u_dot/v_dot/w_dot (Hz). |
| `STATEMGR_LP_PQ_HZ` / `STATEMGR_LP_R_HZ` | StateManager.hpp | 20 / 5 | Lowpass cutoff for blended roll/pitch vs. yaw rate fed to the rate PID (Hz). |
| `STATEMGR_LP_PQRDOT_HZ` | StateManager.hpp | 20 | Lowpass cutoff for p_dot/q_dot/r_dot (Hz). |
| `STATEMGR_LP_BLENDW_HZ` | StateManager.hpp | 3 | Lowpass cutoff for the lane-blend weight itself, before renormalizing (Hz). |
| `STATEMGR_NOTCH_BW_HZ` | StateManager.hpp | 10 | Motor-vibration notch bandwidth (sets Q = center/bandwidth). |
| `STATEMGR_NOTCH_MAX_SLEW_FRAC` | StateManager.hpp | 0.05 | Max fractional change in the notch's tracked center frequency per tick (matches ArduPilot's ±5%/update). |
| `STATEMGR_CAN_QUAT_STALE_US` / `STATEMGR_CAN_RATES_STALE_US` | StateManager.hpp | 50000 (both) | IMX5 CAN transport-delay staleness gate (µs) — a reading older than this is skipped rather than fused/blended as current. |
| `GRAV_VIBE_ALPHA` | EKF.hpp | 0.0125 | Fixed-rate IIR alpha for the vibration estimate driving adaptive gravity-update noise (~0.1s time constant at StateEstThread's 800 Hz — rescale if that rate changes, see the comment in EKF.hpp). |

### Sensor loss behaviour

**IMX5 disconnect:** `update_quaternion` calls stop. Gravity vector continues correcting roll/pitch. Yaw drifts at the gyro Z-axis bias rate (probably a few degrees per minute). Rates fall back to 100% onboard gyros. Bias states continue being estimated.

**Mocap disconnect:** `update_position` and `update_ned_vel` calls stop. Position and velocity states are no longer corrected and drift quickly (position drifts quadratically with time). Attitude and rates are unaffected. Barometric altitude fusion into Z automatically resumes the instant mocap goes invalid — see below.

**Mocap vs. barometer priority:** mocap always wins for absolute position/altitude when connected — `StateManager::update()` gates barometer fusion on `!mocap.valid`, never fusing both simultaneously. This isn't just about mocap being more accurate: the two aren't anchored to the same origin (the barometer zeroes to whatever pressure it read during its own boot-time warm-up; mocap's Z origin is whatever the motion-capture system's world frame defines), so fusing both into the same `Z` state at once would fight between two different "zero points" rather than average two views of the same truth. Barometer fusion is purely a mocap-unavailable fallback.

### Quaternion convention

Scalar-first Hamilton [W, X, Y, Z], representing rotation from NED frame to Body frame. ROS/ROS2 uses scalar-last — take care when interfacing with those libraries.

---

## 4. Math Utilities

All math helpers live in `src/math/math.hpp` / `src/math/math.cpp`. The quaternion convention throughout is Hamilton scalar-first **[W, X, Y, Z]**, representing the rotation from NED frame to Body frame (`q_NED→Body`). The kinematic propagation equation is `dq/dt = 0.5 * q ⊗ ω_pure` where `ω_pure = {0, p, q, r}`.

### Scalar helpers

| Function | Signature | Description |
|----------|-----------|-------------|
| `constrain_float` | `float constrain_float(float v, float lo, float hi)` | Clamps `v` to `[lo, hi]`. |

### Signal processing

| Function | Signature | Description |
|----------|-----------|-------------|
| `lowpass_alpha` | `float lowpass_alpha(float fc_hz, float dt_s)` | Computes first-order IIR coefficient: `α = dt / (dt + 1/(2π·fc))`. Call once when `fc` or `dt` changes. |
| `lowpass` | `float lowpass(float input, float prev_out, float alpha)` | Applies one IIR tick: `y_k = α·x_k + (1−α)·y_{k−1}`. Caller owns `prev_out`. |
| `lowpass2p` | `float lowpass2p(float input, Biquad2pState& state, float fc_hz, float dt_s)` | Second-order (2-pole) Butterworth IIR lowpass — ~-40 dB/decade vs. `lowpass`'s ~-20 dB/decade. Coefficients recomputed from `fc_hz`/`dt_s` every call. `fc_hz <= 0` disables filtering (passthrough). Caller owns the `Biquad2pState` (2 delay elements). |
| `notch` | `float notch(float input, Biquad2pState& state, float center_hz, float bandwidth_hz, float dt_s)` | Second-order notch (RBJ biquad form), reusing `Biquad2pState` for delay memory — same direct-form-II structure as `lowpass2p`. `bandwidth_hz` sets notch width (`Q = center_hz/bandwidth_hz`). Caller drives frequency tracking (and should slew-limit `center_hz` between calls). `center_hz <= 0` or `bandwidth_hz <= 0` disables filtering. |
| `derivative` | `float derivative(float current, float prev, float dt_s)` | Backward-difference numerical derivative: `(current − prev) / dt`. Caller owns `prev`. |
| `integrate` | `float integrate(float value, float dt_s)` | Rectangular (Euler) integration step: `value · dt`. Caller owns the accumulator. |
| `rpm_gate` | `uint32_t rpm_gate(RpmGateState& state, uint32_t raw_rpm)` | RPM plausibility gate: once a motor has reported >100 RPM, a subsequent reading below that threshold is treated as a missed telemetry frame (motors don't legitimately idle that slowly while armed) and the last good value is held instead. Returns `raw_rpm` unchanged before the first >100 RPM reading. |

`Biquad2pState` (2 delay elements) backs both `lowpass2p` and `notch`; `RpmGateState` (`last_good`, `seen_valid`) backs `rpm_gate` — both are plain structs, matching this file's convention of small per-channel filter state rather than a class.

### 3-vector helpers

| Function | Signature | Description |
|----------|-----------|-------------|
| `cross3` | `void cross3(const float a[3], const float b[3], float out[3])` | Cross product `out = a × b`.|

### Quaternion

The `Quat` struct holds `{float w, x, y, z}`.

| Function | Signature | Description |
|----------|-----------|-------------|
| `quat_mul` | `Quat quat_mul(const Quat& a, const Quat& b)` | Hamilton product `a ⊗ b`. Non-commutative. |
| `quat_conj` | `Quat quat_conj(const Quat& q)` | Conjugate `q* = {w, −x, −y, −z}`. For a unit quaternion this is also the inverse. |
| `quat_norm` | `Quat quat_norm(const Quat& q)` | Re-normalises to unit length. Returns identity `{1,0,0,0}` if the input norm is below `1e-10`. |
| `quat_dot` | `float quat_dot(const Quat& a, const Quat& b)` | 4-element dot product. Used for antipodal sign checks (`dot < 0 → flip sign before SLERP`). |
| `quat_to_euler` | `void quat_to_euler(const Quat& q, float& roll, float& pitch, float& yaw)` | Extracts ZYX Euler angles (rad) from `q_NED→Body`. Pitch is clamped to `[−1, 1]` before `asin` to guard against numerical overflow. |
| `quat_to_rot_body2ned` | `void quat_to_rot_body2ned(const Quat& q, float R[3][3])` | Builds the 3×3 DCM that maps body-frame vectors to NED: `v_NED = R · v_body`. For `q_NED→Body` this equals `R(q)ᵀ = R(q*)`. `R` is stored row-major. |

### N×N matrix operations

Three header-only templates parameterized on the square dimension `Dim`. Because they are templates the full definitions live in the header — no `.cpp` entry.

| Function | Signature | Description |
|----------|-----------|-------------|
| `mat_mul` | `void mat_mul<Dim>(const float A[Dim][Dim], const float B[Dim][Dim], float C[Dim][Dim])` | Matrix multiply `C = A · B`. `C` must not alias `A` or `B`. Uses an early-out on zero entries to skip work on sparse matrices like the EKF Jacobian. |
| `mat_add` | `void mat_add<Dim>(const float A[Dim][Dim], const float B[Dim][Dim], float C[Dim][Dim])` | Element-wise addition `C = A + B`. `C` may alias `A` or `B`. |
| `mat_trans` | `void mat_trans<Dim>(const float A[Dim][Dim], float AT[Dim][Dim])` | Transpose `AT = Aᵀ`. `AT` must not alias `A`. |

**Usage** — supply the dimension as the template argument:
```cpp
float A[4][4], B[4][4], C[4][4];
mat_mul<4>(A, B, C);     // 4×4 multiply
mat_trans<4>(A, C);      // 4×4 transpose
mat_add<4>(C, A, C);     // C += A  (aliasing is safe for mat_add)
```

Each unique `Dim` value used in the firmware produces a separate compiled instantiation, so avoid calling these with many different sizes.

---

## 5. SD Card Logging

Binary log format is compatible with the [ArduPilot DataFlash standard](https://ardupilot.org/dev/docs/logmessages.html). Log files can be opened directly in [UAV Log Viewer](https://plot.ardupilot.org) for interactive plotting.

### How it works

`LogThread` runs at 50 Hz. Each tick it snapshots all shared state under their respective mutexes, builds eleven packed records, and pushes them into a 32 KB in-RAM ring buffer. A low-priority flush call drains the buffer to the SD card via FatFS without ever blocking flight-critical threads.

**Log file location:** `/LOGS/LOG0001.BIN`, `LOG0002.BIN`, … auto-incremented on each boot.

### Logged message types (50 Hz each)

| Name | ID | Fields |
|---|---|---|
| `ATT` | 0x09 | TimeUS, Roll, Pitch, Yaw (rad), P, Q, R (rad/s), Pdot, Qdot, Rdot (rad/s²) |
| `LIN` | 0x0A | TimeUS, X, Y, Z (m NED), U, V, W (m/s body), Udot, Vdot, Wdot (m/s² body) |
| `RCIN` | 0x05 | TimeUS, RollStk, PitchStk, YawStk, ThrStk (normalized), FlightMode (raw switch value), IndiStk (raw switch value, >0.33=INDI), Armed |
| `OUTP` | 0x06 | TimeUS, RollTq, PitchTq, YawTq (normalized torque [-1,1] into mixer), Thr |
| `RPMS` | 0x07 | TimeUS, RPM0–RPM3 (mechanical RPM via DShot GCR telemetry) |
| `STRN` | 0x08 | TimeUS, S0–S3 (int16 strain-rate, CAN 0x69), Valid |
| `IMU1`/`IMU2`/`IMU3` | 0x0B/0x0C/0x0D | TimeUS, AccX, AccY, AccZ (m/s²), GyrX, GyrY, GyrZ (rad/s), Valid — one series per on-board IMU |
| `INDI` | 0x0E | TimeUS, UnmixR, UnmixP (N·m measured), DeltaR, DeltaP (N·m INDI correction), CmdR, CmdP (normalized), AccR, AccP (rad/s² INDI-commanded accel) |
| `BARO` | 0x0F | TimeUS, Press (Pa), Temp (°C), Alt (m, positive up), Valid |

TimeUS is a uint64 microsecond timestamp, always first. There is no per-record rate field — every message here logs at the fixed 50 Hz `LogThread` period, so it would only ever record a constant.

### Decoding log files

Use `tools/logs.py` — no firmware source needed:

```bash
# Decode a downloaded .bin file to CSV (one file per message type)
python3 tools/logs.py logs decode LOG0042.BIN

# Download latest completed log and decode immediately
python3 tools/logs.py logs download --decode
```

Output files: `LOG0042_att.csv`, `LOG0042_lin.csv`, `LOG0042_rcin.csv`, `LOG0042_outp.csv`, `LOG0042_rpms.csv`, `LOG0042_strn.csv`, `LOG0042_imu1.csv`, `LOG0042_imu2.csv`, `LOG0042_imu3.csv`, `LOG0042_indi.csv`, `LOG0042_baro.csv`.

Or open the `.bin` file directly in [UAV Log Viewer](https://plot.ardupilot.org) — all eleven message types appear in the message list; use `ATT.Roll` vs `TimeUS` for attitude plots.

### Adding a new log message type

Three files, three steps (example below adds a hypothetical rangefinder):

**Step 1 — Define the struct in `src/logging/LogMessages.hpp`**

```cpp
constexpr uint8_t LOG_MSG_RNGF = 0x10U;   // next unused ID

struct __attribute__((packed)) LogMsgRNGF {
    uint64_t time_us;    // always first — required by UAV Log Viewer
    float    range_m;
    uint8_t  valid;
};
```

Rules: `__attribute__((packed))`, fixed-size types only, `time_us` first. There is no rate field — every message logs at whatever rate `LogThread` calls `logger.write()` for it (50 Hz, unless you add your own divisor counter).

**Step 2 — Add a row to `kLogDefs[]` in the same file**

```cpp
{ LOG_MSG_RNGF, "RNGF", "QfB", "TimeUS,Range,Valid", sizeof(LogMsgRNGF) },
```

Format codes: `Q`=uint64, `f`=float32, `i`=int32, `h`=int16, `B`=uint8. Name must be exactly 4 chars (space-pad). `fmt` must be ≤15 chars and `labels` ≤63 chars — these are the actual usable lengths of the FMT record's fixed `format[16]`/`labels[64]` fields once `Logger::write_schema_header()`'s `strncpy` reserves a byte for the null terminator. A `static_assert` right after `kLogDefs[]` in `LogMessages.hpp` checks every entry against these limits at build time, so a string that's too long fails the build instead of being silently truncated by `strncpy` (which happened once before this check existed — UAV Log Viewer showed a message with fields missing past the cut, with no error anywhere to explain why).

**Step 3 — Snapshot and write in `LogThread` (`src/threads.cpp`)**

```cpp
{ LogMsgRNGF msg = {}; msg.time_us = t_us;
  msg.range_m = rangefinder_distance(); msg.valid = rangefinder_valid();
  logger.write(LOG_MSG_RNGF, msg); }
```

If you need a different rate than `LogThread`'s 50 Hz, add a divisor counter around the write call.

### Binary format reference

**Data record** (every record after the header):
```
[0xA3][0x95][msg_id][...packed struct body...]
```

**FMT record** (one per message type, written at file open, 89 bytes):
```
[0xA3][0x95][0x80][type_u8][length_u8][name_4b][format_16b][labels_64b]
```
`length` = 3 + body_size (total record size including the 3-byte header). Files are self-describing: the decoder reads the schema entirely from the FMT records at the start of the file.

**Write rate:** 50 Hz × ~375 B/tick ≈ 18.8 KB/s (sum of all 11 records' body+header sizes; see `LogMessages.hpp`'s per-struct size comments). The 32 KB ring buffer holds several seconds of write-stall tolerance. `f_sync()` is called every 100 flushes (~1 Hz) to limit data loss on unexpected power loss.

**D-cache coherency:** FatFS structures (`s_fs`, `s_file`) and the flush staging buffer (`s_flush_buf`) live in the `.nocache` linker section (SRAM3, 0x30040000). `STM32_NOCACHE_ENABLE TRUE` in `cfg/mcuconf.h` marks that region non-cacheable at boot, so the SDMMC IDMA always sees coherent data.

**SD card retry:** If no card is present at boot, `logger.init()` retries every 5 seconds. The rest of the firmware is unaffected.

---

## 6. Build and Upload

### Prerequisites

- `arm-none-eabi-gcc` toolchain (tested with 10.2.1-2020q4)
- `python3` for the flash upload script
- SD card formatted FAT32 in the Cube microSD slot (required only for logging)

### Build

```bash
# Default board (CubeBlue H7)
make

# Explicitly select board
make BOARD=CubeBlueH7
make BOARD=CubeOrangePlus

# Enable debug USB streams ($TEL/$EKFL/$IMU at 10 Hz over USB CDC)
make BOARD=CubeBlueH7 UDEFS_EXTRA=-DBPRL_DEBUG

# Clean build directory
make clean
```

Build artefacts are written to `build/BPRL.bin` and `build/BPRL.hex`.

### Upload

**Via Cube USB bootloader:**
```bash
make flash BOARD=CubeBlueH7 PORT=/dev/ttyACM0
```
`tools/flash_upload.py` handles the protocol.

**Via ST-Link / OpenOCD:**
```bash
make flash-stlink BOARD=CubeBlueH7
```
Requires OpenOCD with `interface/stlink.cfg` and `target/stm32h7x.cfg`.

### Debug USB

With `-DBPRL_DEBUG`, `DebugThread` emits three CSV streams at 10 Hz over the **USB CDC** port (`/dev/ttyACM0`):

| Prefix | Content |
|---|---|
| `$TEL` | time_ms, roll°, pitch°, yaw°, p, q, r, thr, rc_roll, rc_pitch, rc_yaw, armed, rpm×4, imu_valid×3, can_valid, can_quat_hz, can_rate_hz, flight_mode, use_indi |
| `$EKFL` | time_ms, primary_lane, then 4×{roll°, pitch°, yaw°, p, q, r} (lanes 0–2 + IMX5 INS) |
| `$IMU` | time_ms, then 3×{ax, ay, az, gx, gy, gz, valid} + can_p, can_q, can_r, can_valid |

Without `-DBPRL_DEBUG` the USB port still accepts commands from the ground tools — only the continuous stream is suppressed. Remove `-DBPRL_DEBUG` before flight to eliminate scheduling jitter from the print thread.

---

## 7. Comms Drivers

All drivers live in `src/coms/`. See [`src/coms/README.md`](src/coms/README.md) for full protocol details.

### Channel summary

| Channel | Driver | Device(s) | Status |
|---|---|---|---|
| SPI1 | `SPI.hpp/.cpp` | imu1 (ICM-45686, CS=PG1), baro1 (MS5611, CS=PD7) | Working |
| SPI4 | `SPI.hpp/.cpp` | imu2 (ICM-45686, CS=PC15), imu3 (ICM-45686, CS=PC13) | Working — this board's SPI4 slots are ICM-45686; other CubeOrangePlus revisions populate ICM-42688 there instead (see IMU Drivers below) |
| FDCAN1 | `CAN.hpp/.cpp` | IMX5 INS (0x01–0x04), strain rate sensor (0x69, default interface) | Working |
| TIM1/TIM4 | `PWM.hpp/.cpp` | DShot600 bidirectional (4 motors) | Working |
| UART (TELEM1) | `Radio.hpp/.cpp` | CRSF receiver (default; SBUS also compiled, `RADIO_PROTOCOL` selects) | Working |
| UART (TELEM2) | `MAVLink.hpp/.cpp` | MAVLink — mocap ingestion (`VISION_POSITION/SPEED_ESTIMATE` → `g_mocap`) | Working |
| I2C2 | `I2C.hpp/.cpp` | Strain rate sensor (fallback interface only — CAN is default) | Working |

---

## 8. IMU Drivers

The firmware reads three on-board IMUs plus one external IMU/AHRS over CAN, plus a barometer. Index assignments are fixed:

| Index | Variable | Sensor | Bus | DOF |
|---|---|---|---|---|
| 0 | `g_imu[0]` | ICM-45686 | SPI1, CS=PG1 | 6 (accel + gyro) |
| 1 | `g_imu[1]` | ICM-45686 | SPI4, CS=PC15 | 6 (accel + gyro) |
| 2 | `g_imu[2]` | ICM-45686 | SPI4, CS=PC13 | 6 (accel + gyro) |
| — | `g_can_imu` | IMX5 (INS) | FDCAN1 | attitude + rates |
| — | `g_baro` | MS5611 | SPI1, CS=PD7 | pressure + temperature |

### ICM-45686 (`src/coms/IMUs/ICM45686.hpp/.cpp`)

InvenSense 6-DOF MEMS (accelerometer + gyroscope), FIFO-based output. **One driver class serves all three on-board IMUs on this board** — all three slots (`imu1/2/3`) are confirmed populated with real ICM-45686 parts (verified by each successfully passing its WHOAMI check on init). Other CubeOrangePlus hardware revisions instead populate the SPI4 slots (`imu2`/`imu3`) with ICM-42688 rather than ICM-45686 — `ICM42688.hpp/.cpp` exists in the tree to support that variant, not because it's dead code, it just isn't instantiated on this board's `SPI.cpp`. `ICM20948.hpp/.cpp`/`ICM20602.hpp/.cpp` are genuinely unused leftovers from an earlier (CubeBlueH7-class) hardware revision.

- **Configured ranges:** ±16 g accelerometer, ±2000 °/s gyroscope
- **Outputs:** accel in m/s², gyro in rad/s
- **Read rate:** 1 kHz from SPIThread; internal ODR ~3.2 kHz fast-sampling (matches ArduPilot's own default for this chip over SPI). `read()` drains and averages every FIFO packet queued since the last call (capped at 8 packets), so the 1 kHz caller gets a decimated/averaged sample rather than picking one aliased sample out of several — this is a deliberate oversample-then-filter design, not just a rate bump.
- **SPI speeds:** ~781 kHz for init, 6.25–12.5 MHz for burst reads (per-instance clock divider)
- **Axis rotation** (`SPIThread`, `src/threads.cpp`) to body-frame NED z-down, per IMU: imu1 `ROTATION_ROLL_180_YAW_135`, imu2 `ROTATION_YAW_90`, imu3 `ROTATION_PITCH_180_YAW_90`

### MS5611 Barometer

`src/coms/Baro/MS5611.hpp/.cpp`, SPI1, CS=PD7, shares the bus with imu1. Unlike the IMU FIFO reads, a full pressure+temperature sample needs multi-millisecond ADC conversions, so `read()` is a small state machine called once per SPIThread tick (1 kHz): reset → read 6 PROM calibration words → alternate D1 (pressure) / D2 (temperature) conversions, returning a compensated pair roughly every 6 ticks.

- Altitude is computed from the standard barometric formula and **zeroed to a boot-time reference** (averaged over the first 8 completed samples) — absolute accuracy doesn't matter here, only consistent relative height.
- Fused into the EKF via `EKF::update_altitude()` — a single-row, chi-squared-gated (`BARO_CHI2_GATE`) measurement of the Z position state. Existing pos↔vel and vel↔accel-bias cross-covariance in the process model (`_build_F()`) automatically propagates the correction into vertical velocity and accelerometer bias — the same mechanism ArduPilot's EKF3 uses for baro fusion.
- **Suppressed whenever mocap is connected** (`StateManager::update()` gates on `!mocap.valid`) — mocap measures absolute position far more accurately, and the two aren't anchored to a common origin, so fusing both would fight rather than agree. See [Sensor loss behaviour](#sensor-loss-behaviour) above.
- Logged as `BARO` (pressure, temperature, altitude, valid) — see [SD Card Logging](#5-sd-card-logging).

### Inertial Sense IMX5 (FDCAN1)

External INS/AHRS module transmitting fused attitude and body rates over FDCAN1 at 1 Mbit/s.

**Frame protocol (standard 11-bit IDs):**

| CAN ID | Content | Encoding | Rate |
|---|---|---|---|
| `0x01` | Quaternion NED→Body [W, X, Y, Z] | 4 × int16 ÷ 10000 | 200 Hz |
| `0x02` | p rate + x accel | 2 × int16; rates ÷ 1000 → rad/s, accel ÷ 1000 → m/s² | 100 Hz |
| `0x03` | q rate + y accel | same encoding | 100 Hz |
| `0x04` | r rate + z accel | same encoding | 100 Hz |

When the IMX5 is connected, its quaternion is fused into all three EKF lanes via `update_quaternion()` at 200 Hz — each CAN frame is timestamped on arrival, and `StateManager` skips fusing it if it's more than `STATEMGR_CAN_QUAT_STALE_US` (50ms) stale by the time it's processed, otherwise forward-propagating it by its measured age (using each lane's own gyro) before fusing, rather than treating a few-ms-old sample as if it were instantaneous. Angular rates are optionally blended into the StateManager p/q/r output (30% IMX5, 70% onboard gyros by default — see `STATEMGR_IMX5_RATE_WEIGHT`); this blend is a continuous sample-and-hold rather than event-gated like the quaternion — it re-blends whatever the last-received `g_can_imu.p/q/r` holds on every 800 Hz tick (refreshed ~every 10ms), gated only on the same 50ms staleness check (`STATEMGR_CAN_RATES_STALE_US`), not on whether a new frame arrived this specific tick. The on-board IMUs continue to run and are logged regardless of IMX5 state.

---

## 9. Building on Windows

The build system uses **GNU Make** and **`arm-none-eabi-gcc`**. Neither runs natively on Windows without a compatibility layer; WSL2 is the recommended path.

### Setup (WSL2)

1. **Install WSL2** — run in PowerShell as Administrator, then reboot:
   ```powershell
   wsl --install
   ```
   This installs Ubuntu by default.

2. **Inside Ubuntu (WSL2), install the toolchain:**
   ```bash
   sudo apt update
   sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi make python3
   ```

3. **Clone the repo and build** — the Makefile works unchanged:
   ```bash
   make BOARD=CubeBlueH7
   ```

### Flashing from WSL2

WSL2 does not expose USB devices by default. Two options:

**Option A — `usbipd-win` (USB bootloader or ST-Link over WSL2):**
```powershell
# In PowerShell (install usbipd-win first from https://github.com/dorssel/usbipd-win)
usbipd list                       # find the Cube or ST-Link bus ID
usbipd attach --wsl --busid <ID>
```
Then in WSL2:
```bash
make flash BOARD=CubeBlueH7 PORT=/dev/ttyACM0
# or
make flash-stlink BOARD=CubeBlueH7
```

**Option B — STM32CubeProgrammer (no usbipd needed):**
Build in WSL2, then flash `build/BPRL.bin` using [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) on the Windows side. Connect the Cube in DFU mode (hold BOOT, apply power), select USB DFU, and write the `.bin` at address `0x08000000` (CubeBlueH7) or `0x08020000` (CubeOrangePlus).

