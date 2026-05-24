# BPRL Flight Controller — Status

## Hardware

| Component | Detail |
|-----------|--------|
| Flight controller | CubeOrange+ (STM32H743ZI) on CubePilot standard carrier |
| IMU 0 (primary) | ICM-45686 on SPI1/PG1 — `ROTATION_ROLL_180_YAW_135` |
| IMU 1 | ICM-45686 on SPI4/PC15 — `ROTATION_YAW_90` |
| IMU 2 | ICM-45686 on SPI4/PC13 — `ROTATION_PITCH_180_YAW_90` |
| INS | Pixhawk IMX5 on CAN1 (PH13=TX, PH14=RX) — **active next** |
| RC radio | CRSF receiver on TELEM1 (USART2, PD5=TX, PD6=RX) — disabled pending testing |
| Motors | AUX 2–5 via TIM1/TIM4 DShot |
| USB | OTG_FS (PA11=DM, PA12=DP) — active, enumerates as `0483:5740 BPRL Debug USB` |
| SD card | SDMMC1 (PC8–PC12, PD2) — LogThread currently disabled |

---

## Changes Made to Bring the Firmware Up on Hardware

### Bringup Fixes

**1. Flash load address** (`boards/CubeOrangePlus/STM32H743xI_app.ld`, `Makefile`)
BL5 bootloader occupies 0x08000000–0x0801FFFF. App flash origin corrected to
`0x08020000`. Without this the reset vector pointed into bootloader space and the
app never ran. The linker script also reserves Bank2 sector 7 (`cal` region at
`0x081E0000`) for persistent IMU calibration.

**2. USB CDC hang after bootloader jump** (`src/usb_serial.cpp`)
BL5 leaves D+ asserted. `usbStart()` → `otg_core_reset()` loops on AHBIDL forever
when SOF frames are arriving. Fix: call `usbDisconnectBus()` before `usbStart()` to
de-assert D+ and let the host stop, then reconnect. Same pattern as ArduPilot.

**3. DShot wrong pins** (`src/coms/DShot.cpp`)
Original code drove TIM1 on PA8/PA9/PA10/PE14 — none of those reach AUX OUT on the
CubePilot carrier. Corrected mapping (from ArduPilot hwdef):

| Motor | AUX OUT | Pin  | Timer |
|-------|---------|------|-------|
| 0 FR  | AUX 3   | PE11 | TIM1_CH2 |
| 1 RL  | AUX 4   | PE9  | TIM1_CH1 |
| 2 FL  | AUX 5   | PD13 | TIM4_CH2 |
| 3 RR  | AUX 2   | PE13 | TIM1_CH3 |

Motors 0–2 use a TIM1 3-channel DMA burst; motor 3 uses TIM4 single-channel DMA.

**4. DMAMUX source IDs** (`src/coms/DShot.cpp`)
`TIM1_UP = 15` (was wrongly set to 11 = TIM1_CH1); `TIM4_UP = 32`. DMA never
triggered with the wrong request source.

**5. RC input wiring** (`src/coms/CRSF.cpp`, `boards/*/board.c`, `src/coms/Radio.hpp`)
USART6 (PC6/PC7) is the internal FMU↔IOMCU bridge on the CubePilot carrier — it
cannot receive external RC. Switched RC to CRSF on TELEM1 (USART2 / SD2).
USART6 init removed from both board.c files.

**6. DebugThread USB blocking** (`src/threads.cpp`)
`chprintf` on SDU1 blocks indefinitely when the output queue is full. Fixed by
formatting into a stack `MemoryStream` buffer then sending with a 50 ms timeout
(`chnWriteTimeout`). Added `s_usb_write_mtx` mutex to prevent byte-interleaving
between DebugThread and USBCmdThread.

**7. CHPRINTF_USE_FLOAT** (`Makefile`)
Without `-DCHPRINTF_USE_FLOAT=1`, `%.2f` emits nothing and bprl.py float parsing
fails silently on all telemetry fields.

**8. DebugThread compile guard** (`src/threads.cpp`)
`waDebug` / `DebugThread` working area and launch were not inside `#ifdef BPRL_DEBUG`
guards, causing undeclared-identifier errors on release builds. Fixed.

### Sensor Alignment & Calibration

**9. IMU rotation corrections** (`src/threads.cpp` — SPIThread)
Cross-referenced ArduPilot hwdef for CubeOrangePlus. NED z-down body frame (x=forward,
y=right, z=down) matching ArduPilot convention. Rotations applied:

| Lane | CS pin | ArduPilot rotation | NED z-down formula |
|------|--------|--------------------|--------------------|
| g_imu[0] | PG1 | `ROTATION_ROLL_180_YAW_135` | `[(y−x)/√2, (y+x)/√2, −z]` |
| g_imu[1] | PC15 | `ROTATION_YAW_90` | `[−y, +x, +z]` |
| g_imu[2] | PC13 | `ROTATION_PITCH_180_YAW_90` | `[−y, −x, −z]` |

All three axes confirmed correct on hardware: roll/pitch/yaw and p/q/r all have
correct sign and direction on all three lanes simultaneously.

**10. NED z-down body frame** (`src/state_estimator/EKF.cpp`, `src/state_estimator/StateManager.cpp`)
Converted entire codebase from z-up to NED z-down (ArduPilot convention).
Changes:
- `EKF::predict()`: `a_true = accel + g_body` (was `− g_body`; sensor reads −g at hover in z-down)
- `EKF::update_gravity()`: predicted measurement is `−g_body`; H matrix quaternion entries negated
- `StateManager`: removed all z-up→z-down output sign corrections (`−_blended_p/r`, `−roll/yaw`)
- `StateManager` IMX5 rate rotation: plain `R_b2n^T × ω_NED` (removed ROLL_180 correction)

**11. IMX5 quaternion conjugation** (`src/state_estimator/StateManager.cpp`)
IMX5 CAN ID 0x01 outputs q_NED→body. EKF stores q_body→NED. Conjugation
`{q0, −q1, −q2, −q3}` applied in the `has_new_quat` branch. Convention-agnostic —
unchanged by the z-down conversion.

### EKF Architecture & Tuning

**12. EKF improvements** (`src/state_estimator/EKF.cpp`)
Implemented ArduPilot EKF3-equivalent robustness features:
- **Joseph form** covariance update: `P = (I−KH)P(I−KH)^T + KRK^T` — prevents covariance
  collapse under sustained noise
- **Chi-squared innovation gate** on gravity update: 5σ joint gate mirrors ArduPilot
  `velTestRatio` pattern — rejects bad IMU samples without fully blocking updates
- **Adaptive measurement noise**: `R_eff = R_base + R_VIBE × vibe_rms²` — IIR vibration
  filter increases gravity update R under vibration, matching ArduPilot's accel nav magnitude scaling
- **Gyro bias Q** matched to ArduPilot EKF3 formula (`Q_BIAS_G = 1e-9`, `Q_BIAS_A = 1e-7`)

### Calibration Infrastructure

**13. CalFlash driver** (`src/coms/CalFlash.cpp/.hpp`)
New persistent IMU bias calibration stored in STM32H743 Bank 2 sector 7
(0x081E0000, 128 KB). Direct register access (HAL_USE_EFL = FALSE):
unlock KEYR2 → erase SNB=7 → program 4×32-byte flash words → lock.
CRC-32/ISO-HDLC covers bytes 0–79 of the 128-byte `CalibData` struct.

**14. Calibration applied in SPIThread** (`src/threads.cpp`)
`g_cal` loaded from flash at boot. Body-frame biases subtracted from each IMU's
corrected accel/gyro readings before storing to `g_imu[]`.

**15. USB CAL commands** (`src/threads.cpp` — `usb_cmd_dispatch()`)
`CAL,set,<i>,gx,gy,gz,ax,ay,az` — stage biases for IMU i  
`CAL,commit` — write staged buffer to flash  
`CAL,clear` — erase flash sector  
`CAL,query` — read back stored values  

**16. bprl.py calibrate subcommand** (`tools/bprl.py`)
Collects 30 s of `$IMU` data at rest, computes mean gyro biases and accel biases
(z-axis corrected for gravity), sends CAL,set/commit, verifies round-trip with
CAL,query. Includes live Rich progress bar and motion warning.

**17. IMU labels** (`tools/bprl.py`)
Changed from chip-specific strings to generic `IMU0 (pri)`, `IMU1 (ext)`, `IMU2`.

---

## Active Threads

| Thread | Priority | Rate | Stack | Status |
|--------|----------|------|-------|--------|
| **HeartbeatThread** | NORMALPRIO−5 | 2 Hz LED blink | 1 KB | **Running** |
| **SPIThread** | NORMALPRIO+30 | 1 kHz | 2 KB | **Running** |
| **StateEstThread** | NORMALPRIO+25 | 500 Hz | 6 KB | **Running** |
| **USBCmdThread** | NORMALPRIO−20 | event-driven | 4 KB | **Running** |
| **DebugThread** | NORMALPRIO−10 | 10 Hz `$TEL` | 2 KB | **Running** (BPRL_DEBUG build only) |
| CANThread | NORMALPRIO+28 | event-driven | 2 KB | **Next — IMX5 CAN INS** |
| I2CThread | NORMALPRIO+22 | 100 Hz | 1 KB | Disabled |
| ControlThread | NORMALPRIO+20 | 500 Hz | 2 KB | Disabled — PID untested |
| RadioThread | NORMALPRIO+10 | 50 Hz | 1 KB | Disabled — CRSF not yet tested |
| LogThread | NORMALPRIO−15 | 100 Hz | 8 KB | Disabled — SD logging not yet tested |

**Hardware inits also disabled in main.cpp:** `motor_output_init()`, `radio_input_init()`,
`i2c_drv_init()`.

---

## Known Open Issues

| # | Issue | Status |
|---|-------|--------|
| 1 | ~~P/Q/R and roll/pitch/yaw sign errors~~ | **Resolved** — NED z-down conversion + correct per-IMU rotations |
| 2 | ~~EKF diverges under vibration~~ | **Resolved** — Joseph form, chi-sq gate, adaptive R, ArduPilot-matched noise tuning |
| 3 | CAN / IMX5 fusion untested | **Active** — enabling CANThread next |

---

## TODO — Testing Roadmap

### ✅ 1. EKF Accuracy (Complete)

All three lanes confirmed correct roll/pitch/yaw/p/q/r signs on hardware.
EKF robustness improvements (Joseph form, chi-sq gate, adaptive R, ArduPilot noise
parameters) implemented and build-verified.

---

### 2. IMU Calibration

**Goal:** Gyro rates < 0.005 rad/s at rest on all three IMUs after calibration.

```bash
# Place drone on flat, level surface — do not move
python3 tools/bprl.py calibrate --duration 30
# Follow prompts; writes CAL,set/commit over USB
# Power-cycle and verify
python3 tools/bprl.py telemetry
# Body rates should be near-zero at rest
```

Verify persistence by power-cycling and running `CAL,query` from the USB terminal:
```bash
python3 tools/bprl.py
# In the REPL or terminal: send CAL,query
```

---

### ▶ 3. CAN Bus / IMX5 INS Fusion (Active)

**Goal:** IMX5 quaternion and rate data fused correctly; stationary body rates
near-zero when blending with IMX5.

Steps:
1. Enable `can_drv_init()` in `main.cpp`.
2. Uncomment `CANThread` in `threads_start()`.
3. Connect IMX5 to CAN1 (PH13=TX, PH14=RX).
4. Flash and open telemetry: verify `CAN INS (IMX5)` shows as valid in the IMU
   Status panel and `can_quat_hz` / `can_rate_hz` show ~200/200.
5. Hold drone still — verify blended body rates remain near-zero.
6. Rotate slowly — verify quaternion update doesn't cause EKF divergence (watch
   innovation norm; large sustained values indicate a sign error).

Frame conventions already implemented and build-verified:
- IMX5 quaternion: q_NED→body, conjugated to q_body→NED before EKF update
- IMX5 rates: NED z-down world frame, rotated to body frame via `R_b2n^T × ω_NED`

---

### 4. SD Card Logging

**Goal:** LogThread writes IMU + state frames to SD; bprl.py downloads and decodes
the file correctly.

Steps:
1. Insert SD card formatted FAT32.
2. Uncomment `LogThread` in `threads_start()`.
3. Enable `sdmmc_lld_init()` / FatFS mount in the thread (verify `Logger::init()`
   succeeds — check for `LOG,ERR,no_sd` on USB output).
4. After a brief flight or bench run, download the log:
   ```bash
   python3 tools/bprl.py logs list
   python3 tools/bprl.py logs download <filename>
   python3 tools/bprl.py logs decode <filename>.bin
   ```
5. Open the decoded CSV and verify IMU columns, state columns, and timestamps are
   sane (no NaN, timestamps monotonically increasing, accel z ≈ −9.8 m/s² at rest in NED z-down).

---

### 5. Motor Test

**Goal:** All four motors spin at commanded throttle; ESC telemetry (RPM) returns
valid data.

Steps:
1. Remove propellers.
2. Enable `motor_output_init()` in `main.cpp`.
3. Power ESCs from battery (USB power is not sufficient to spin motors).
4. Run motor test:
   ```bash
   python3 tools/bprl.py motor-test
   ```
5. Test each motor individually at low throttle (~5–10%). ESC should play the
   "connected" chime on first DShot packet and spin briefly.
6. Verify RPM telemetry appears in the Motor RPM panel of `bprl.py telemetry`.
7. Confirm motor 0=FR, 1=RL, 2=FL, 3=RR spin in correct rotation direction for
   X-quad (motors 0,1 CW; motors 2,3 CCW — or per the mixer convention).

---

### 6. CRSF Radio Receiver

**Goal:** RC channels appear in the telemetry stream; arming/disarming works.

Steps:
1. Wire CRSF receiver to TELEM1: FC TX (PD5) → receiver RX, FC RX (PD6) → receiver TX.
2. Enable `radio_input_init()` in `main.cpp`.
3. Uncomment `RadioThread` in `threads_start()`.
4. Flash and open telemetry:
   ```bash
   python3 tools/bprl.py telemetry
   ```
5. Move sticks — verify `RC INPUTS` panel (Thr, Roll, Pitch, Yaw) responds correctly.
6. Test the arming sequence (if implemented) and confirm `ARMED` indicator changes state.
7. Verify stick ranges match the expected ±1.0 normalised range; trim
   `CRSF_MID` / `CRSF_RANGE` constants in `src/coms/CRSF.cpp` if needed.

---

## Build Commands Reference

```bash
# Release build
make BOARD=CubeOrangePlus

# Debug build (enables DebugThread and $TEL telemetry stream)
make BOARD=CubeOrangePlus UDEFS_EXTRA=-DBPRL_DEBUG

# Flash via CubePilot BL5 bootloader (USB)
make flash BOARD=CubeOrangePlus PORT=/dev/ttyACM0

# Flash via ST-Link / OpenOCD
make flash-stlink BOARD=CubeOrangePlus

# Live telemetry dashboard
python3 tools/bprl.py telemetry

# Per-lane EKF status
python3 tools/bprl.py ekf-status

# IMU calibration
python3 tools/bprl.py calibrate --duration 30
```
