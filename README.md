# BPRL_flight

Standalone ChibiOS flight controller firmware for the [CubePilot CubeBlue H7](https://docs.cubepilot.org) and CubeOrange+ autopilot hardware (STM32H753 / STM32H743 at 400 MHz). This project is loosely based on the open source Ardupilot project.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Controller](#2-controller)
3. [State Estimation (EKF)](#3-state-estimation-ekf)
4. [SD Card Logging](#4-sd-card-logging)
5. [Build and Upload](#5-build-and-upload)
6. [Comms Drivers](#6-comms-drivers)
7. [IMU Drivers](#7-imu-drivers)

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
│   │   ├── I2C.hpp/.cpp      I2C device table (TODO — strain future)
│   │   ├── PWM.hpp/.cpp      Motor PWM output (TODO — TIM1 future)
│   │   ├── Radio.hpp/.cpp    RC radio input (TODO — ICU/SBUS future)
│   │   ├── ICM20948.hpp/.cpp InvenSense ICM-20948 9-DOF driver
│   │   └── ICM20602.hpp/.cpp InvenSense ICM-20602 6-DOF driver
│   │
│   ├── controllers/          Flight control algorithms
│   │   ├── PID.hpp/.cpp      Cascade PID with derivative filter + anti-windup
│   │   ├── AttitudeController.hpp/.cpp  Outer (attitude) + inner (rate) PIDs
│   │   └── MotorMixer.hpp/.cpp          X-frame quadcopter mixer
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
| CANThread | NORMALPRIO+28 | 1 kHz | Drain FDCAN1 RxFIFO, dispatch frames |
| StateEstThread | NORMALPRIO+25 | 500 Hz | Fuse sensors → g_state[] |
| I2CThread | NORMALPRIO+22 | 100 Hz | Poll I2C devices ( TODO:strain) |
| ControlThread | NORMALPRIO+20 | 500 Hz | Cascade PID → MotorMixer → PWM out |
| RadioThread | NORMALPRIO+10 | 50 Hz | Read RC input → g_input[] |
| HouseThread | NORMALPRIO-5 | 5 Hz | LED heartbeat |
| LogThread | NORMALPRIO-15 | 100/50 Hz | Snapshot sensors + state → SD card |
| DebugThread | NORMALPRIO-10 | 10 Hz | UART status print (BPRL_DEBUG only) |

### Shared state

All inter-thread communication goes through mutex-protected globals defined in `src/threads.cpp` and declared in `src/threads.hpp`:

| Variable | Mutex | Description |
|---|---|---|
| `g_state[9]` | `state_mtx` | Fused flight state (roll…z_accel) |
| `g_input[4]` | `state_mtx` | RC inputs (thrust, roll/pitch/yaw targets) |
| `g_output[4]` | `state_mtx` | Motor PWM values [µs] |
| `g_armed` | `state_mtx` | Arm state |
| `g_imu[3]` | `imu_mtx` | Raw accel/gyro from each on-board IMU |
| `g_can_imu` | `can_imu_mtx` | Attitude + rates from CAN IMU (IMX5) |

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

**MotorMixer** converts `[roll_cmd, pitch_cmd, yaw_cmd, thrust]` to four motor PWM values using an X-frame mixing matrix. All motors are set to `PWM_IDLE` (1000 µs) when disarmed or when |roll| or |pitch| exceeds ~80°.

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

- **Motor output wiring** — `motor_output_init()` and `motor_output_write()` in `src/coms/PWM.cpp` are TODOs. Needs TIM1 PWM configuration (FMU CH1-4) in `halconf.h` and `mcuconf.h`. To switch to DShot, only `motor_output_write()` needs to change; everything upstream produces microsecond PWM values.

- **Radio input wiring** — `radio_input_update()` in `src/coms/Radio.cpp` is a TODO. 
  - **PWM capture:** enable `HAL_USE_ICU`, configure TIM8 input capture on `LINE_RC_INPUT`
  - **SBUS:** configure USART6 at 100000 baud 8E2 inverted, decode 25-byte frame, set RadioThread period to `TIME_MS2I(14)` in `main.cpp`

- **Arming logic** — `radio_armed()` returns `false` unconditionally. Needs a dedicated switch channel decoded from the radio.

- **Gain tuning** — gains were ported from the Tiva platform and have not been flight-tested on the H7 hardware.

- **Yaw position hold** — currently only yaw *rate* is commanded. An outer yaw angle loop would require a heading reference from the magnetometer or IMX5.

- **INDI** — add an INDI controller around angular accelerations.
    - **Update State Vector:** The state vector needs to be updated with accelerations making an 18 state, state vector. This is a TODO in the EKF.
---

## 3. State Estimation (EKF)

### Current status

`StateEstThread` (`src/threads.cpp`) is a **pass-through placeholder**: if the CAN IMU (IMX5) reports a valid frame, its attitude and rates are used directly as `g_state[]`. Otherwise, raw gyro rates from IMU 0 are passed through with no angle integration.

This means:
- Attitude angles are only valid when the IMX5 is connected and running.
- There is no accelerometer tilt correction or drift compensation from on-board IMUs alone.
- Altitude (`z_pos`, `z_vel`, `z_accel`) in `g_state[]` is always zero.


### TODO:

**Planned implementation**: Replace the pass-through in `StateEstThread` with a real EKF.

**State Vector**

The system state is represented by a 19-dimensional vector:
 
$$\mathbf{x} = \begin{bmatrix} X,\ Y,\ Z,\ u,\ v,\ w,\ \dot{u},\ \dot{v},\ \dot{w},\ q_0,\ q_1,\ q_2,\ q_3,\ p,\ q,\ r,\ \dot{p},\ \dot{q},\ \dot{r} \end{bmatrix}^T$$
 
| Index | State   | Symbol        | Description                             | Units   |
|-------|---------|---------------|-----------------------------------------|---------|
| 1     | `X`     | $X$           | Position — inertial frame x-axis        | m       |
| 2     | `Y`     | $Y$           | Position — inertial frame y-axis        | m       |
| 3     | `Z`     | $Z$           | Position — inertial frame z-axis        | m       |
| 4     | `u`     | $u$           | Translational velocity — body frame x   | m/s     |
| 5     | `v`     | $v$           | Translational velocity — body frame y   | m/s     |
| 6     | `w`     | $w$           | Translational velocity — body frame z   | m/s     |
| 7     | `u_dot` | $\dot{u}$     | Translational acceleration — body x     | m/s²    |
| 8     | `v_dot` | $\dot{v}$     | Translational acceleration — body y     | m/s²    |
| 9     | `w_dot` | $\dot{w}$     | Translational acceleration — body z     | m/s²    |
| 10    | `q0`    | $q_0$         | Quaternion — scalar part                | —       |
| 11    | `q1`    | $q_1$         | Quaternion — vector part x              | —       |
| 12    | `q2`    | $q_2$         | Quaternion — vector part y              | —       |
| 13    | `q3`    | $q_3$         | Quaternion — vector part z              | —       |
| 14    | `p`     | $p$           | Angular velocity — roll rate            | rad/s   |
| 15    | `q`     | $q$           | Angular velocity — pitch rate           | rad/s   |
| 16    | `r`     | $r$           | Angular velocity — yaw rate             | rad/s   |
| 17    | `p_dot` | $\dot{p}$     | Angular acceleration — roll             | rad/s²  |
| 18    | `q_dot` | $\dot{q}$     | Angular acceleration — pitch            | rad/s²  |
| 19    | `r_dot` | $\dot{r}$     | Angular acceleration — yaw              | rad/s²  |
 
> **Note — quaternion convention:** Scalar-first ordering $\mathbf{q} = q_0 + q_1 i + q_2 j + q_3 k$, where $q_0$ is the scalar (real) part. This matches the Hamilton convention used in aerospace. ROS/ROS2 uses scalar-last $(q_1, q_2, q_3, q_0)$ — take care when interfacing with those libraries.
 

---

## 4. SD Card Logging

Follows the logging standard in Ardupilot

### How it works

`LogThread` snapshots sensor and state data each tick, serialises it into a 32 KB in-RAM ring buffer, and drains that buffer to the SD card via FatFS at low priority. The ring buffer absorbs SD write stalls without ever blocking flight-critical threads.

**Log file location:** `/LOGS/LOG0001.BIN`, `LOG0002.BIN`, … auto-incremented on each boot.

### Adding a new log set

This is a four-step process. No changes outside the three files listed below are ever needed.

---

**Step 1 — Define the message struct in `src/logging/LogMessages.hpp`**

```cpp
constexpr uint8_t LOG_MSG_BARO = 0x03U;   // pick the next unused ID

struct __attribute__((packed)) LogMsgBaro {
    uint32_t time_ms;       // always first
    float    pressure_pa;
    float    temp_c;
    float    altitude_m;
};
```

Rules:
- Always use `__attribute__((packed))`.
- Always start with `uint32_t time_ms`.
- Use only fixed-size types (`float`, `int32_t`, `uint8_t`, etc.) — no pointers, no padding.

---

**Step 2 — Add a row to `kLogDefs[]` in the same file**

```cpp
constexpr LogDef kLogDefs[] = {
    { LOG_MSG_IMU,   "IMU ", "TimeMS,...", sizeof(LogMsgIMU)   },
    { LOG_MSG_STATE, "STAT", "TimeMS,...", sizeof(LogMsgState) },
    // new entry example:
    { LOG_MSG_strain,  "STRN", "TimeMS,rAccel,pAccel,zAccel", sizeof(LogMsgStrain) },
};
```

- `name` must be exactly 4 characters (space-pad if shorter).
- `labels` is a comma-separated list of field names in the same order as the struct members. Used by the decoder script to produce column headers.

---

**Step 3 — Snapshot and write in `LogThread` (`src/threads.cpp`)**

Inside the `while(true)` loop, at the rate you want:

```cpp
{
    LogMsgStrain msg = {};
    msg.time_ms  = t_ms;
    msg.rAccel   = roll_acceleration();
    msg.pAccel   = pitch_acceleration();
    msg.zAccel   = zAxis_acceleration();
    logger.write(LOG_MSG_strain, msg);
}
```

Use the same `tick % state_div` divisor pattern as the state log if you want a rate slower than the IMU log rate.

---

**Step 4 — Set the log rate in `main.cpp`**

If you want an independent rate, add a field to `LogRates` in `src/threads.hpp`:

```cpp
struct LogRates {
    sysinterval_t imu;     // 100 Hz
    sysinterval_t state;   // 50 Hz
    sysinterval_t strain;  // add: 25 Hz
};
```

Then initialise it in the sequencer block in `main.cpp`:

```cpp
/* .log = */ { TIME_US2I(10000),   // IMU:   100 Hz
               TIME_US2I(20000),   // state:  50 Hz
               TIME_MS2I(40) },    // strain:   25 Hz
```

And compute the divisor at LogThread startup just as `state_div` is computed.

---

### Decoding log files

A minimal Python script to parse the binary format (claude generated and untested):

```python
import struct, sys

FMT_HDR_SIZE = 74   # 1+1+1+1+2+4+64

with open(sys.argv[1], "rb") as f:
    data = f.read()

# Step 1: read FMT schema records from the file header
schema = {}   # msg_id -> (name, body_size, [field_names])
i = 0
while i + FMT_HDR_SIZE <= len(data):
    if data[i] == 0xA3 and data[i+1] == 0x95 and data[i+2] == 0x80:
        type_id = data[i+3]
        length  = struct.unpack_from("<H", data, i+4)[0]
        name    = data[i+6:i+10].decode("ascii").strip()
        labels  = data[i+10:i+74].decode("ascii").rstrip("\x00")
        schema[type_id] = (name, length, labels.split(","))
        i += FMT_HDR_SIZE
    else:
        break   # FMT header section ended

# Step 2: decode data records
records = {k: [] for k in schema}
while i + 3 <= len(data):
    if data[i] != 0xA3 or data[i+1] != 0x95:
        i += 1
        continue
    msg_id = data[i+2]
    if msg_id not in schema:
        i += 1
        continue
    name, length, fields = schema[msg_id]
    body = data[i+3 : i+3+length]
    # Each field is 4 bytes (float or uint32) except uint8 trailing flags.
    # Adjust the format string to match your specific struct layout.
    records[msg_id].append(body)
    i += 3 + length

# Example: print IMU record count
for msg_id, (name, _, fields) in schema.items():
    print(f"{name}: {len(records[msg_id])} records  fields={fields}")
```

### Backend Details 

**Record format — every record in the file:**
```
[0xA3][0x95][msg_id][...packed struct body...]
```

**File header — written once at file open, one FMT record per log type:**
```
[0xA3][0x95][0x80][type_id][body_size_u16_LE][name_4b][labels_64b]
```
This makes files self-describing: a decoder can read the schema from the file itself without access to the firmware source.

**Write rate (for current log set):** ~13 KB/s (100 Hz IMU at ~113 bytes/record + 50 Hz state at ~51 bytes/record). The 32 KB ring buffer provides ~2.5 s of write-stall tolerance. `f_sync()` is called every 100 flushes (~1 Hz) to limit data loss on unexpected power loss.

**D-cache coherency:** FatFS structures (`s_fs`, `s_file`) and the flush staging buffer (`s_flush_buf`) live in the `.nocache` linker section (SRAM3, 0x30040000). `STM32_NOCACHE_ENABLE TRUE` in `cfg/mcuconf.h` configures the MPU to mark that region non-cacheable at boot, ensuring the SDMMC IDMA always reads coherent data.

**SD card retry:** If no card is present at boot, `logger.init()` retries every 5 seconds. The rest of the firmware is unaffected.

---

## 5. Build and Upload

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

# Enable debug UART (USART3 @ 115200 on Telem1 — adds 10 Hz print thread)
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

### Debug UART

With `-DBPRL_DEBUG`, `DebugThread` prints one status line per 100 ms over **USART3** (PD8 TX / PD9 RX, 115200 baud 8N1). On the Cube this is the **Telem1** connector. Example output:

```
armed=0 r=0.00 p=0.01 y=-0.02 thr=0.00 m=[1000,1000,1000,1000]
```

Remove `-DBPRL_DEBUG` before flight — the print thread adds ~1 KB of stack and non-trivial scheduling jitter at 10 Hz.

---

## 6. Comms Drivers

All drivers live in `src/coms/`. Register devices in `main.cpp` before calling `threads_start()`.

### SPI — `SPI.hpp/.cpp`

Two SPI buses drive all three on-board IMUs. Each IMU has its own chip-select pin.

| Bus | Peripheral | CS pin | Device |
|---|---|---|---|
| SPI1 | SPID1 | PC2 | ICM-20948 (imu1, primary) |
| SPI4 | SPID4 | PE4 | ICM-20948 (imu2, external) |
| SPI4 | SPID4 | PC13 | ICM-20602 (imu3) |

`spi_drv_init()` initialises all three devices and must be called from inside `SPIThread` because the ICM power-on reset sequences use `chThdSleepMilliseconds`. `imu2` and `imu3` share SPI4 and use `spiAcquireBus`/`spiReleaseBus` for mutual exclusion.

### FDCAN — `CAN.hpp/.cpp`

FDCAN1 at **500 kbps** using HSE (24 MHz) as the clock source. Standard 11-bit IDs only.

A table of up to 8 ID→callback pairs is maintained. `CANThread` polls the RxFIFO at 1 kHz and calls `can_dispatch()`, which routes each frame to its registered handler in O(n) time.

**Currently supported device:** Inertial Sense IMX5 — four frame IDs pre-registered by `can_drv_init()`. See `CAN.hpp` for the byte-level protocol.

**Adding a CAN device:**
```cpp
// in main.cpp before threads_start():
bprl_can_register(0x10, my_callback, nullptr);
```

### I2C — `I2C.hpp/.cpp`

**Status: stub.** The registration table and `i2c_poll_all()` dispatch loop are implemented, but the I2C peripheral is not started.

**TODO:** Enable `HAL_USE_I2C TRUE` in `cfg/halconf.h`, configure `I2CD1` in `cfg/mcuconf.h`, call `i2cStart()` inside `i2c_drv_init()`. Planned devices: strain sensors

### PWM — `PWM.hpp/.cpp`

**Status: stub.** The interface accepts four PWM values in microseconds (1000–1950 µs) but does not drive any timer.

**TODO:** Enable `HAL_USE_PWM TRUE` in `cfg/halconf.h`, configure TIM1 CH1-4 for FMU outputs. To switch to DShot, only `motor_output_write()` needs changing — the MotorMixer and ControlThread produce protocol-agnostic microsecond values.

### Radio — `Radio.hpp/.cpp`

**Status: stub.** All getters (`radio_thr()`, `radio_roll()`, etc.) return safe defaults (0.0 / false).

**TODO — choose radio:**
- **PWM capture:** Enable `HAL_USE_ICU`, configure TIM8 on `LINE_RC_INPUT`.
- **SBUS:** Configure USART6 at 100000 baud, 8E2, inverted RX line; decode the 25-byte SBUS frame in `radio_input_update()`. Change `kRates.radio` in `main.cpp` to `TIME_MS2I(14)`.

---

## 7. IMU Drivers

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

External INS/AHRS module transmitting fused attitude and body rates over FDCAN1 at 500 kbps.

**Frame protocol (standard 11-bit IDs):**

| CAN ID | Bytes 0–1 | Bytes 2–3 | Bytes 4–5 | Scale |
|---|---|---|---|---|
| `0x01` | roll (int16) | pitch (int16) | yaw (int16) | ÷ 10000 → rad |
| `0x02` | p rate (int16) | x accel (int16) | — | rates ÷ 1000 → rad/s |
| `0x03` | q rate (int16) | y accel (int16) | — | |
| `0x04` | r rate (int16) | z accel (int16) | — | |

When the IMX5 is connected and sending valid frames, `StateEstThread` uses its attitude and rates directly as the primary state estimate. The on-board IMU data continues to be collected and logged regardless.

**TODO:** Configure the IMX5 to output quaternions. Fuse the IMX5 output with the other IMUs in the EKF. posiblly switch the angular rates from CAN (limited to 100Hz) to UART (raw readings at 1kHz) 
