# BPRL Flight Controller — Status

## Hardware

| Component | Detail |
|-----------|--------|
| Flight controller | CubeOrange+ (STM32H743ZI) on CubePilot standard carrier |
| IMU 0 (primary) | ICM-45686 on SPI1/PG1 — `ROTATION_ROLL_180_YAW_135` |
| IMU 1 | ICM-45686 on SPI4/PC15 — `ROTATION_YAW_90` |
| IMU 2 | ICM-45686 on SPI4/PC13 — `ROTATION_PITCH_180_YAW_90` |
| INS | Pixhawk IMX5 on CAN1 (PH13=TX, PH14=RX) |
| RC radio | CRSF receiver on TELEM1 (USART2, PD5=TX, PD6=RX) — disabled pending testing |
| Motors | AUX 2–5 via TIM1/TIM4 DShot600 bidirectional |
| USB | OTG_FS (PA11=DM, PA12=DP) — active, enumerates as `0483:5740 BPRL Debug USB` |
| SD card | SDMMC1 (PC8–PC12, PD2) — LogThread currently disabled |

---

## Build Commands Reference

```bash
# Debug build (HeartbeatThread $DSHOT diagnostics + DebugThread $TEL)
make BOARD=CubeOrangePlus UDEFS_EXTRA=-DBPRL_DEBUG

# Flash via CubePilot BL5 bootloader (USB)
make flash BOARD=CubeOrangePlus PORT=/dev/ttyACM0

# Clean → build → flash in one shot
make clean && make BOARD=CubeOrangePlus UDEFS_EXTRA=-DBPRL_DEBUG && make flash BOARD=CubeOrangePlus

# Live telemetry dashboard (requires BPRL_DEBUG build)
python3 tools/bprl.py telemetry
```

---

## Active Threads (motor-test configuration)

| Thread | Priority | Rate | Stack | Status |
|--------|----------|------|-------|--------|
| **ControlThread** | NORMALPRIO+20 | 500 Hz | 2 KB | **Running** — sends DShot 0 directly (motor test bypass via `MT,` USB commands) |
| **HeartbeatThread** | NORMALPRIO−5 | 5 Hz tick / 2 s print | 1 KB | **Running** — LED blink + `$DSHOT` diagnostic auto-print |
| **USBCmdThread** | NORMALPRIO−20 | event-driven | 4 KB | **Running** — `MT,*`, `DSHOT,diag`, `BOOT`, `CAL,*` |
| **DebugThread** | NORMALPRIO−10 | 10 Hz `$TEL` | 2 KB | **Running** (BPRL_DEBUG build only) |
| SPIThread | NORMALPRIO+30 | 1 kHz | 2 KB | Disabled — causes SPI DMA ISR contention with DShot edge capture |
| CANThread | NORMALPRIO+28 | event-driven | 2 KB | Disabled |
| StateEstThread | NORMALPRIO+25 | 500 Hz | 6 KB | Disabled |
| I2CThread | NORMALPRIO+22 | 100 Hz | 1 KB | Disabled |
| RadioThread | NORMALPRIO+10 | 50 Hz | 1 KB | Disabled |
| LogThread | NORMALPRIO−15 | 100 Hz | 8 KB | Disabled |

**Hardware inits disabled in main.cpp:** `can_drv_init()`, `radio_input_init()`, `i2c_drv_init()`.

---

## DShot Diagnostic Output

HeartbeatThread auto-prints every 2 s (no command needed):

```
$DSHOT,<ms>,tc=<tim1_dma_tc>/<tim4_dma_tc>,cc=<tim1_cc>/<tim4_cc>,edges=<m0>/<m1>/<m2>/<m3>
```

| Value | Meaning |
|-------|---------|
| `tc` counting up at ~500/s | DShot IS transmitting — firmware side confirmed working |
| `tc=0/0` | DShot NOT transmitting — dshot_init() failed; watch for LED error flash |
| `edges` > 0 | ESC responding with GCR telemetry — signal physically reaches ESC |
| `edges=0/0/0/0` | ESC NOT responding — signal not reaching ESC, or ESC not in bdshot mode |

**LED error codes if DMA alloc fails in dshot_init():**
- 20 rapid flashes loop = TIM1 DMA alloc failed
- 30 rapid flashes loop = TIM4 DMA alloc failed

**Current hardware test result:**
```
$DSHOT,36203,tc=16000/16000,cc=16000/16000,edges=0/0/0/0
```
Firmware is sending DShot at 500 Hz ✓. ESC is not responding — see Issue #1 below.

---

## Changes Made

### Original Bringup Fixes (committed, working)

**1. Flash load address** (`boards/CubeOrangePlus/STM32H743xI_app.ld`, `Makefile`)
BL5 bootloader at 0x08000000–0x0801FFFF. App start corrected to `0x08020000`.
Bank2 sector 7 (`0x081E0000`) reserved for persistent IMU calibration.

**2. USB CDC hang** (`src/usb_serial.cpp`)
BL5 leaves D+ asserted. Fixed: call `usbDisconnectBus()` before `usbStart()`.

**3. DShot pin mapping** (`src/coms/DShot.cpp`)
Original code used wrong pins (PA8/PA9/PA10/PE14 — not routed to AUX OUT).

| Motor | AUX | Pin  | Timer     | AF  |
|-------|-----|------|-----------|-----|
| 0 FR  | 3   | PE11 | TIM1_CH2  | AF1 |
| 1 RL  | 4   | PE9  | TIM1_CH1  | AF1 |
| 2 FL  | 5   | PD13 | TIM4_CH2  | AF2 |
| 3 RR  | 2   | PE13 | TIM1_CH3  | AF1 |

**4. DMAMUX source IDs** (`src/coms/DShot.cpp`)
`DMAMUX_TIM1_UP = 15` (was 11); `DMAMUX_TIM4_UP = 32`. DMA never triggered
with the wrong request source.

**5. RC input wiring** (`src/coms/CRSF.cpp`, `boards/*/board.c`)
USART6 is FMU↔IOMCU bridge, not usable for external RC. Moved to USART2/TELEM1.

**6. DebugThread USB blocking** (`src/threads.cpp`)
`chprintf` on full USB queue blocks indefinitely. Fixed: format into `MemoryStream`
then send with `chnWriteTimeout`. Added `s_usb_write_mtx` mutex.

**7. IMU rotation corrections** (`src/threads.cpp` SPIThread)
NED z-down body frame per ArduPilot hwdef for all three IMU lanes.

**8. NED z-down conversion** (`src/state_estimator/EKF.cpp`, `StateManager.cpp`)
EKF predict, gravity update, and output sign corrections all converted from z-up.

**9. IMX5 quaternion conjugation** (`src/state_estimator/StateManager.cpp`)
IMX5 outputs q_NED→body; EKF stores q_body→NED. Conjugation applied.

**10. EKF robustness** (`src/state_estimator/EKF.cpp`)
Joseph form covariance, chi-squared innovation gate, adaptive R, ArduPilot-matched
noise parameters. All three lanes confirmed correct roll/pitch/yaw on hardware.

**11. CalFlash driver** (`src/coms/CalFlash.cpp/.hpp`)
Persistent IMU bias calibration in Bank2 sector 7.

---

### DShot / Motor Test Fixes (this session)

**12. DShot telemetry bit — root cause of ESC not arming** (`src/coms/DShot.cpp` `make_frame()`)

`make_frame()` was setting the DShot telemetry bit to 1 unconditionally:
```cpp
uint16_t val = (uint16_t)((thr << 1) | 1U);  // WRONG — telem=1 always
```

In the DShot protocol, **values 0–47 with telem=1 are special commands**, not
throttle packets. Value=0, telem=1 = **DShot Command 0 = MOTOR STOP**.

BPRL was sending "MOTOR STOP" at 500 Hz. The ESC received this as a continuous
stream of stop commands and never completed its arming sequence.

ArduPilot sets telem=0 for all throttle packets:
```cpp
uint16_t packet = (value << 1);   // telem bit = 0
if (telem_request) { packet |= 1; } // only set once per 32 packets for serial telemetry
```

Fix — remove `| 1U`:
```cpp
uint16_t val = (uint16_t)(thr << 1);  // CORRECT — telem=0
```

Frame comparison (throttle=0, bidirectional):
| | Hex | Throttle | Telem | CRC | ESC interpretation |
|--|-----|----------|-------|-----|-------------------|
| ArduPilot | `0x000F` | 0 | **0** | 0xF | Zero throttle → arms ✓ |
| BPRL (buggy) | `0x001E` | 0 | **1** | 0xE | MOTOR STOP command → never arms ✗ |
| BPRL (fixed) | `0x000F` | 0 | **0** | 0xF | Zero throttle → arms ✓ |

The inverted CRC (bidirectional protocol signal) is independent of the telem bit and is unaffected by this fix.

**12. AUX output power and voltage select** (`boards/CubeOrangePlus/board.c`, `boards/CubeBlueH7/board.c`, both `board.h` files)

Two pins were missing from `boardInit()` on both boards. Without them the
level-shifter ICs between the STM32 timer pins and the physical AUX connector
are unpowered or output-disabled — no DShot signal ever leaves the board.

| Pin | Name | Value | Why |
|-----|------|-------|-----|
| PA8 | `nVDD_5V_PERIPH_EN` | OUTPUT LOW | Enables 5 V peripheral rail (powers level-shifters). PCB pull-up holds HIGH at reset → rail stays OFF by default. |
| PB4 | `PWM_VOLT_SEL` | OUTPUT HIGH | Selects 3.3 V output mode; releases level-shifter OE. |

Source: `libraries/AP_HAL_ChibiOS/hwdef/CubeOrange/hwdef.inc` lines
`PA8 nVDD_5V_PERIPH_EN OUTPUT LOW` and `PB4 PWM_VOLT_SEL OUTPUT HIGH`.
Applied to both boards (identical PCB design per board.h comments).

**13. DShot IRQ priority raised** (`src/coms/DShot.cpp`)

Changed `nvicEnableVector` from `CORTEX_MINIMUM_PRIORITY` (15) to
`STM32_IRQ_TIM1_CC_PRIORITY` / `STM32_IRQ_TIM4_PRIORITY` (both 7, matching
other timer IRQs in `mcuconf.h`).

At priority 15, any ISR at 6–14 (SPI DMA, USB, CAN ~12) preempted the
TIM1_CC / TIM4 edge-capture handler mid-GCR-frame. At 750 kHz GCR each bit is
1.33 µs; a 5 µs preemption overwrites CCR before it is read. These ISRs never
call `ch*` functions so priority 7 is safe.

**14. DIER cleared before switch_to_output** (`src/coms/DShot.cpp`)

Added `TIM1->DIER = 0; TIM4->DIER = 0; TIM1->SR = 0; TIM4->SR = 0;` at the
start of every `dshot_write()` call, before `switch_to_output()`.

`switch_to_output()` calls `EGR=UG`. With `UIE` still in DIER from the previous
IC session, that UEV queues a spurious TIM1_UP IRQ that fires and decodes stale
edge data.

**15. Thread and hardware simplification** (`src/threads.cpp`, `main.cpp`)

- Removed SPIThread, CANThread, StateEstThread, I2CThread, RadioThread, LogThread.
  SPIThread's 1 kHz SPI DMA ISRs (priority ~12) preempted TIM1_CC edge-capture (was priority 15) mid-GCR-frame.
- Removed `can_drv_init()` from `main.cpp` — no CANThread to consume frames; FDCAN RxFIFO overflow ISRs added noise.
- ControlThread simplified to `dshot_write({0,0,0,0})` directly every 2 ms,
  bypassing AttitudeController/MotorMixer (which depend on zero-initialised state from disabled threads).
- HeartbeatThread restored with `$DSHOT` diagnostic output instead of `$IMU` (SPIThread disabled).
- DebugThread re-enabled for `BPRL_DEBUG` builds.

**16. Debug assertions enabled** (`cfg/chconf.h`)

`CH_DBG_ENABLE_ASSERTS = TRUE` for `BPRL_DEBUG` builds (was unconditionally
`FALSE`). With assertions off, a failed `dmaStreamAlloc()` returned null silently;
the next call `dmaStreamSetPeripheral(nullptr, ...)` caused a silent hard fault
→ IWDG reset loop with no visible symptom.

**17. DMA alloc failure detection** (`src/coms/DShot.cpp`)

Explicit null-checks after both `dmaStreamAlloc()` calls. On failure the firmware
enters a visible LED-flash loop (20 or 30 flashes) rather than crashing silently.

---

## Known Open Issues

| # | Issue | Status |
|---|-------|--------|
| 1 | **ESC not connecting** | **Active** — DShot transmitting (`tc` counts up), ESC not responding (`edges=0`). Either signal not reaching AUX connector despite PA8/PB4 fix, or ESC not in bidirectional DShot mode. |
| 2 | CAN / IMX5 fusion | Deferred |
| 3 | CRSF radio | Deferred |

### Issue 1 Next Steps

`tc=16000` confirms the STM32 is generating correct DShot600 frames at 500 Hz.
`edges=0` means the ESC's GCR telemetry is not being received.

**Step A — Verify ESC firmware mode:**
Open BLHeli Suite / AM32 configurator, connect to each ESC, confirm
"Bidirectional DSHOT" is enabled. BPRL sends inverted signal + inverted CRC
(bidirectional format); an ESC in standard DShot mode will not recognise it
and will not beep.

**Step B — Verify signal at AUX connector:**
Probe AUX 2 (PE13), AUX 3 (PE11), AUX 4 (PE9), AUX 5 (PD13) with an
oscilloscope or logic analyser. Expected: 500 Hz stream of LOW pulses
(625 ns or 1250 ns) on a HIGH baseline. If the line stays static, the
level-shifter is still not passing the signal — recheck PA8/PB4 in a scope.

**Step C — Try non-bidirectional DShot as a fallback test:**
Temporarily change `switch_to_output()` to use PWM mode 1 (active-HIGH) and
`make_frame()` to use non-inverted CRC. If the ESC connects in standard DShot
mode, the hardware path is confirmed working and the issue is ESC firmware
configuration only.

---

## Testing Roadmap

### ▶ 1. Motor Test (Active — blocked on ESC connectivity)

See Issue 1 above for diagnosis steps. Once connected:

```
MT,0,5    ← spin motor 0 (FR) at 5%
MT,1,5    ← spin motor 1 (RL) at 5%
MT,2,5    ← spin motor 2 (FL) at 5%
MT,3,5    ← spin motor 3 (RR) at 5%
MT,stop   ← stop all
```

Verify RPM telemetry: `edges` in `$DSHOT` output should be non-zero when motors
are spinning. Use `DSHOT,diag` for full per-motor edge timestamps.

### 2. IMU Calibration

```bash
python3 tools/bprl.py calibrate --duration 30
```

### 3. CAN Bus / IMX5 Fusion

1. Re-enable `can_drv_init()` in `main.cpp`
2. Uncomment `CANThread` in `threads_start()`
3. Connect IMX5 to CAN1 (PH13=TX, PH14=RX)
4. Verify `can_quat_hz` / `can_rate_hz` ≈ 200 in `$TEL` output

### 4. Full Flight Stack

Once motors work: re-enable SPIThread, StateEstThread, RadioThread, LogThread,
and restore ControlThread to full PID/mixer path. Test complete attitude loop.
