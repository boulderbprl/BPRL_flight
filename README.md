# BPRL_flight

Standalone ChibiOS flight controller firmware for the [CubePilot](https://docs.cubepilot.org) CubeBlue H7 and CubeOrange+ autopilot hardware (STM32H753 / STM32H743 at 400 MHz). This project is loosely based on the open-source [Ardupilot](https://ardupilot.org/ardupilot/) project.

---

## TODO

- Add position hold controller.
- Add voltage feedback from the analog input on Power1 port (CubePilot Power Brick Mini).
- Complete arming logic (dedicated RC switch channel).
- IMX5 yaw magnetometer / heading reference integration.


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
│   │   ├── SPI.hpp/.cpp      SPI bus init, ICM-20948/20602 instantiation
│   │   ├── CAN.hpp/.cpp      FDCAN1 driver, IMX5 callback, device table
│   │   ├── I2C.hpp/.cpp      I2C device table (stub — planned for strain gauges)
│   │   ├── PWM.hpp/.cpp      DShot600 / PWM motor output (MOTOR_PROTOCOL define)
│   │   ├── Radio.hpp/.cpp    CRSF receiver input
│   │   ├── ICM20948.hpp/.cpp InvenSense ICM-20948 9-DOF driver
│   │   └── ICM20602.hpp/.cpp InvenSense ICM-20602 6-DOF driver
│   │
│   ├── controllers/          Flight control algorithms
│   │   ├── PID.hpp/.cpp      Cascade PID with derivative filter + anti-windup
│   │   ├── AttitudeController.hpp/.cpp  Outer (attitude) + inner (rate) PIDs
│   │   └── MotorMixer.hpp/.cpp          X-frame quadcopter mixer
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
| SPIThread | NORMALPRIO+30 | 1 kHz | Read all three on-board IMUs |
| CANThread | NORMALPRIO+28 | event-driven | Block on FDCAN1 RxFIFO, dispatch frames on arrival |
| StateEstThread | NORMALPRIO+25 | 400 Hz | Fuse sensors → g_state[] |
| I2CThread | NORMALPRIO+22 | 100 Hz | Poll I2C devices (TODO: strain) |
| ControlThread | NORMALPRIO+20 | 400 Hz | Cascade PID → MotorMixer → motor output |
| RadioThread | NORMALPRIO+10 | 50 Hz | Read RC input → g_input[] |
| HouseThread | NORMALPRIO-5 | 5 Hz | LED heartbeat |
| LogThread | NORMALPRIO-15 | 50 Hz | Snapshot all state → SD card (6 message types per tick) |
| DebugThread | NORMALPRIO-10 | 10 Hz | UART status print (BPRL_DEBUG only) |

### Shared state

All inter-thread communication goes through mutex-protected globals defined in `src/threads.cpp` and declared in `src/threads.hpp`:

| Variable | Mutex | Description |
|---|---|---|
| `g_state[19]` | `state_mtx` | Fused 19-element flight state |
| `g_euler[3]` | `state_mtx` | [roll, pitch, yaw] in radians, derived from quaternion |
| `g_input[4]` | `state_mtx` | RC inputs (thrust, roll/pitch/yaw targets) |
| `g_output[4]` | `state_mtx` | Normalized motor commands 0–1000 [FR, RL, FL, RR] (0=disarm; protocol conversion in `motor_output_write()`) |
| `g_ctrl[4]` | `state_mtx` | PID torque outputs entering the mixer: [roll_tq, pitch_tq, yaw_tq, thrust] in [-1,1] |
| `g_armed` | `state_mtx` | Arm state |
| `g_imu[3]` | `imu_mtx` | Raw accel/gyro from each on-board IMU |
| `g_can_imu` | `can_imu_mtx` | Quaternion + rates from IMX5 over FDCAN1 |
| `g_mocap` | `mocap_mtx` | NED position + velocity from motion capture radio |

---

## 2. Controller

### Architecture

The flight controller uses a two-loop cascade structure, standard for quadcopters:

```
RC input
  │
  ├─ Thrust ──────────────────────────────────────────────── MotorMixer
  │
  ├─ Roll target  ┐                                               │
  ├─ Pitch target ├─ Outer PID (attitude) → inner PID (rate) ────┘
  └─ Yaw rate     ┘
```

**Outer loop** (`_roll_att`, `_pitch_att` in `AttitudeController`): converts angle error (rad) to a body-rate target (rad/s). P-only by default.

**Inner loop** (`_roll_rate`, `_pitch_rate`, `_yaw_rate`): converts rate error (rad/s) to a normalised torque output in [-1, 1]. PID with derivative low-pass filter (30 Hz cutoff) and integrator anti-windup clamping.

**Throttle shaping** (`compute_throttle`): applies an exponential curve around mid-throttle and an angle boost to hold altitude during maneuvers.

**MotorMixer** converts `[roll_cmd, pitch_cmd, yaw_cmd, thrust]` to four normalized motor commands (0–1000) using an X-frame mixing matrix. All motors are set to 0 when disarmed or when |roll| or |pitch| exceeds ~80°. `motor_output_write()` in `src/coms/PWM.hpp` translates these values to DShot or PWM pulses depending on the `MOTOR_PROTOCOL` compile-time define.

Motor channel mapping (top view):

```
    FL [2]       FR [0]
         \       /
         [  body  ]
         /       \
    RL [1]       RR [3]
```

### Current gain values

Gains live in `src/controllers/AttitudeController.cpp`:

| Loop | Kp | Ki | Kd | Imax |
|---|---|---|---|---|
| Roll attitude | 4.50 | 0 | 0 | 0.5 |
| Pitch attitude | 4.50 | 0 | 0 | 0.5 |
| Roll rate | 0.11 | 0.09 | 0.003 | 0.5 |
| Pitch rate | 0.11 | 0.09 | 0.003 | 0.5 |
| Yaw rate | 0.10 | 0.02 | 0 | 0.5 |

### TODOs

- **Arming logic** — `radio_armed()` returns `false` unconditionally. Needs a dedicated switch channel decoded from the radio.

- **Gain tuning** — gains were ported from the Tiva platform and have not been flight-tested on the H7 hardware.

- **Yaw position hold** — currently only yaw *rate* is commanded. An outer yaw angle loop would require a heading reference from the magnetometer or IMX5.

- **INDI** — add an INDI controller around angular accelerations. 
---

## 3. State Estimation (EKF)

### Architecture

State estimation runs in `StateEstThread` at 500 Hz. The core is a three-lane Extended Kalman Filter: one `EKF` instance per onboard IMU, orchestrated by `StateManager`. Each lane runs independently and `StateManager` selects the healthiest one (lowest smoothed innovation norm) as the primary output. All lanes share the same external sensor updates (IMX5 quaternion, mocap).

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

### Predict step (500 Hz)

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
| 1.5 | Onboard accel (gravity vector) | 500 Hz, gated on \|a\| ≈ g | Quaternion (roll/pitch), accel bias |
| 2 | IMX5 quaternion over CAN | 200 Hz, async | Full quaternion |
| 5 | Mocap NED position | Async | X, Y, Z |
| 5 | Mocap NED velocity → body frame | Async | u, v, w |

The gravity-vector update (`update_gravity`) is gated, it is skipped whenever `|accel| − g > 1.0 m/s²` to suppress corrupted attitude corrections during aggressive maneuvers. Yaw is not observable from gravity alone and requires the IMX5.

### StateManager output (19 states)

`StateManager::get_state()` assembles the shared `g_state[19]` vector:

| Indices | States | Source |
|---------|--------|--------|
| 0–2 | X, Y, Z | Primary EKF lane |
| 3–5 | u, v, w | Primary EKF lane |
| 6–8 | u_dot, v_dot, w_dot | Blended gravity+Coriolis-corrected accel, 50 Hz lowpass |
| 9–12 | q0, q1, q2, q3 | Primary EKF lane |
| 13–15 | p, q, r | Soft-blend of bias-corrected gyros across all valid lanes, optional 30% IMX5 mix |
| 16–18 | p_dot, q_dot, r_dot | Finite-difference of blended rates, 50 Hz lowpass |

Quaternion uses hard lane selection (no blending). Angular rates use soft blending weighted by `1/innovation_norm` to improve noise reduction.

### Tuning parameters

All EKF tuning lives in `src/state_estimator/EKF.hpp` (private `static constexpr` block). StateManager tuning lives in `src/state_estimator/StateManager.hpp`.

| Parameter | Location | Default | Effect |
|-----------|----------|---------|--------|
| `Q_BIAS_A` | EKF.hpp | 1e-6 | Accel bias random-walk rate. Increase if bias changes rapidly. |
| `Q_BIAS_G` | EKF.hpp | 1e-7 | Gyro bias random-walk rate. Typically slower than accel. |
| `P0_BIAS_A` | EKF.hpp | 0.1 | Initial accel bias uncertainty. Larger = faster startup convergence. |
| `P0_BIAS_G` | EKF.hpp | 0.01 | Initial gyro bias uncertainty. |
| `GRAV_GATE_MS2` | EKF.hpp | 1.0 | Gate width (m/s²) for gravity update. Smaller value reduces gyro drift but drops updates during maneuvers.|
| `R_QUAT` | StateManager.hpp | 1e-4 | IMX5 quaternion noise. Lower = trust IMX5 more. |
| `R_GRAVITY` | StateManager.hpp | 0.5 | Accel gravity-vector noise (m/s²)². Lower = trust accel attitude more. |
| `R_MOCAP_POS` | StateManager.hpp | 1e-3 | Mocap position noise (m²). |
| `R_MOCAP_VEL` | StateManager.hpp | 1e-2 | Mocap velocity noise (m/s)². |
| `STATEMGR_IMX5_RATE_WEIGHT` | StateManager.hpp | 0.3 | IMX5 share of blended p/q/r (0 = pure gyro, 1 = pure IMX5). |
| `STATEMGR_LP_UVWDOT_HZ` | StateManager.hpp | 50 | Lowpass cutoff for u_dot/v_dot/w_dot (Hz). |
| `STATEMGR_LP_PQRDOT_HZ` | StateManager.hpp | 50 | Lowpass cutoff for p_dot/q_dot/r_dot (Hz). |

### Sensor loss behaviour

**IMX5 disconnect:** `update_quaternion` calls stop. Gravity vector continues correcting roll/pitch. Yaw drifts at the gyro Z-axis bias rate (probably a few degrees per minute). Rates fall back to 100% onboard gyros. Bias states continue being estimated.

**Mocap disconnect:** `update_position` and `update_ned_vel` calls stop. Position and velocity states are no longer corrected and drift quickly ( position drifts quadratically with time ). Attitude and rates are unaffected.

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
| `derivative` | `float derivative(float current, float prev, float dt_s)` | Backward-difference numerical derivative: `(current − prev) / dt`. Caller owns `prev`. |
| `integrate` | `float integrate(float value, float dt_s)` | Rectangular (Euler) integration step: `value · dt`. Caller owns the accumulator. |

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

`LogThread` runs at 50 Hz. Each tick it snapshots all shared state under their respective mutexes, builds six packed records, and pushes them into a 32 KB in-RAM ring buffer. A low-priority flush call drains the buffer to the SD card via FatFS without ever blocking flight-critical threads.

**Log file location:** `/LOGS/LOG0001.BIN`, `LOG0002.BIN`, … auto-incremented on each boot.

### Logged message types (50 Hz each)

| Name | ID | Fields |
|---|---|---|
| `ATT` | 0x03 | TimeUS, Rate, Roll, Pitch, Yaw (rad), P, Q, R (rad/s), Pdot, Qdot, Rdot (rad/s²) |
| `LIN` | 0x04 | TimeUS, Rate, X, Y, Z (m NED), U, V, W (m/s body), Udot, Vdot, Wdot (m/s² body) |
| `RCIN` | 0x05 | TimeUS, Rate, RollStk, PitchStk, YawStk, ThrStk (normalized), Armed |
| `OUTP` | 0x06 | TimeUS, Rate, RollTq, PitchTq, YawTq (normalized torque [-1,1] into mixer), Thr |
| `RPMS` | 0x07 | TimeUS, Rate, RPM0–RPM3 (mechanical RPM via DShot GCR telemetry) |
| `STRN` | 0x08 | TimeUS, Rate, S0–S3 (int16 strain-rate, CAN 0x69), Valid |

TimeUS is a uint64 microsecond timestamp. Rate is the log rate in Hz (always 50).

### Decoding log files

Use `tools/logs.py` — no firmware source needed:

```bash
# Decode a downloaded .bin file to CSV (one file per message type)
python3 tools/logs.py logs decode LOG0042.BIN

# Download latest completed log and decode immediately
python3 tools/logs.py logs download --decode
```

Output files: `LOG0042_att.csv`, `LOG0042_lin.csv`, `LOG0042_rcin.csv`, `LOG0042_outp.csv`, `LOG0042_rpms.csv`, `LOG0042_strn.csv`.

Or open the `.bin` file directly in [UAV Log Viewer](https://plot.ardupilot.org) — all six message types appear in the message list; use `ATT.Roll` vs `TimeUS` for attitude plots.

### Adding a new log message type

Three files, four steps:

**Step 1 — Define the struct in `src/logging/LogMessages.hpp`**

```cpp
constexpr uint8_t LOG_MSG_BARO = 0x09U;   // next unused ID

struct __attribute__((packed)) LogMsgBaro {
    uint64_t time_us;    // always first — required by UAV Log Viewer
    uint16_t rate_hz;    // always second
    float    pressure_pa;
    float    temp_c;
    float    altitude_m;
};
```

Rules: `__attribute__((packed))`, fixed-size types only, `time_us` first, `rate_hz` second.

**Step 2 — Add a row to `kLogDefs[]` in the same file**

```cpp
{ LOG_MSG_BARO, "BARO", "QHfff", "TimeUS,Rate,Pressure,Temp,Alt", sizeof(LogMsgBaro) },
```

Format codes: `Q`=uint64, `H`=uint16, `f`=float32, `i`=int32, `h`=int16, `B`=uint8. Name must be exactly 4 chars (space-pad).

**Step 3 — Snapshot and write in `LogThread` (`src/threads.cpp`)**

```cpp
{ LogMsgBaro msg = {}; msg.time_us = t_us; msg.rate_hz = 50U;
  msg.pressure_pa = baro_pressure(); msg.temp_c = baro_temp(); msg.altitude_m = baro_alt();
  logger.write(LOG_MSG_BARO, msg); }
```

**Step 4 — No rate change needed.** All messages share the 50 Hz `LogThread` period (`TIME_MS2I(20)` in `main.cpp`). If you need a different rate, add a divisor counter in `LogThread`.

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

**Write rate:** 50 Hz × 208 B/tick = ~10.4 KB/s. The 32 KB ring buffer holds ~3 s of write-stall tolerance. `f_sync()` is called every 100 flushes (~1 Hz) to limit data loss on unexpected power loss.

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
| `$TEL` | time_ms, roll°, pitch°, yaw°, p, q, r, thr, rc_roll, rc_pitch, rc_yaw, armed, rpm×4, imu_valid×3, can_valid, can_quat_hz, can_rate_hz |
| `$EKFL` | time_ms, primary_lane, then 4×{roll°, pitch°, yaw°, p, q, r} (lanes 0–2 + IMX5 INS) |
| `$IMU` | time_ms, then 3×{ax, ay, az, gx, gy, gz, valid} + can_p, can_q, can_r, can_valid |

Without `-DBPRL_DEBUG` the USB port still accepts commands from the ground tools — only the continuous stream is suppressed. Remove `-DBPRL_DEBUG` before flight to eliminate scheduling jitter from the print thread.

---

## 7. Comms Drivers

All drivers live in `src/coms/`. See [`src/coms/README.md`](src/coms/README.md) for full protocol details.

### Channel summary

| Channel | Driver | Device(s) | Status |
|---|---|---|---|
| SPI1 | `SPI.hpp/.cpp` | ICM-20948 (primary IMU) | Working |
| SPI4 | `SPI.hpp/.cpp` | ICM-20948 (external IMU), ICM-20602 (backup IMU) | Working |
| FDCAN1 | `CAN.hpp/.cpp` | IMX5 INS (0x01–0x04), strain rate sensor (0x69) | Working |
| TIM1/TIM4 | `PWM.hpp/.cpp` | DShot600 bidirectional (4 motors) | Working |
| UART | `Radio.hpp/.cpp` | CRSF receiver | Working |
| I2C1 | `I2C.hpp/.cpp` | (planned: strain gauge amplifiers) | Stub |

---

## 8. IMU Drivers

The firmware reads four IMU sources. Index assignments are fixed:

| Index | Variable | Sensor | Bus | DOF |
|---|---|---|---|---|
| 0 | `g_imu[0]` | ICM-20948 | SPI1 | 6 (accel + gyro) |
| 1 | `g_imu[1]` | ICM-20948 | SPI4 | 6 (accel + gyro) |
| 2 | `g_imu[2]` | ICM-20602 | SPI4 | 6 (accel + gyro) |
| — | `g_can_imu` | IMX5 (INS) | FDCAN1 | attitude + rates |

### ICM-20948 (`src/coms/ICM20948.hpp/.cpp`)

InvenSense 9-DOF MEMS (accelerometer, gyroscope, magnetometer). Two instances on FMUv5x hardware.

- **Configured ranges:** ±16 g accelerometer, ±2000 °/s gyroscope
- **Outputs:** accel in m/s², gyro in rad/s
- **Read rate:** 1 kHz from SPIThread; internal ODR set to 1.125 kHz
- **SPI speeds:** 1 MHz for init, 8 MHz for burst reads
- **Magnetometer:** on-chip AK09916 is **not currently initialised** — TODO in state estimator

### ICM-20602 (`src/coms/ICM20602.hpp/.cpp`)

InvenSense 6-DOF MEMS (accelerometer + gyroscope only). One instance on FMUv5x hardware, sharing SPI4 with imu2.

- **Configured ranges:** ±16 g accelerometer, ±2000 °/s gyroscope
- **Outputs:** accel in m/s², gyro in rad/s
- **Read rate:** 1 kHz from SPIThread; internal ODR set to 1 kHz
- **SPI speeds:** 1 MHz for init, 8 MHz for burst reads

Both ICM drivers use 32-byte aligned DMA buffers and apply `cacheBufferFlush` before TX / `cacheBufferInvalidate` after RX to maintain H7 D-cache coherency.

### Inertial Sense IMX5 (FDCAN1)

External INS/AHRS module transmitting fused attitude and body rates over FDCAN1 at 1 Mbit/s.

**Frame protocol (standard 11-bit IDs):**

| CAN ID | Content | Encoding | Rate |
|---|---|---|---|
| `0x01` | Quaternion NED→Body [W, X, Y, Z] | 4 × int16 ÷ 10000 | 200 Hz |
| `0x02` | p rate + x accel | 2 × int16; rates ÷ 1000 → rad/s, accel ÷ 1000 → m/s² | 100 Hz |
| `0x03` | q rate + y accel | same encoding | 100 Hz |
| `0x04` | r rate + z accel | same encoding | 100 Hz |

When the IMX5 is connected, its quaternion is fused into all three EKF lanes via `update_quaternion()` at 200 Hz. Angular rates are optionally blended into the StateManager p/q/r output (30% IMX5, 70% onboard gyros by default — see `STATEMGR_IMX5_RATE_WEIGHT`). The on-board IMUs continue to run and are logged regardless of IMX5 state.

