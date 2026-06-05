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
| **ControlThread** | NORMALPRIO+20 | **400 Hz** | 2 KB | **Running** — sends DShot 0 idle normally; motor-test bypass via `MT,` USB commands |
| **HeartbeatThread** | NORMALPRIO−5 | 5 Hz tick / 2 s print | 1 KB | **Running** — LED blink + `$DSHOT` diagnostic auto-print |
| **USBCmdThread** | NORMALPRIO−20 | event-driven | 4 KB | **Running** — `MT,*`, `DSHOT,diag`, `BOOT`, `CAL,*`, `LOG,*`, `CAN,status` |
| **DebugThread** | NORMALPRIO−10 | 10 Hz `$TEL` + `$EKFL` | 2 KB | **Running** (BPRL_DEBUG build only) |
| SPIThread | NORMALPRIO+30 | 1 kHz | 2 KB | Disabled — SPI DMA ISR contention with DShot edge capture |
| CANThread | NORMALPRIO+28 | event-driven | 2 KB | Disabled |
| StateEstThread | NORMALPRIO+25 | 500 Hz | 6 KB | Disabled |
| I2CThread | NORMALPRIO+22 | 100 Hz | 1 KB | Disabled |
| RadioThread | NORMALPRIO+10 | 50 Hz | 1 KB | Disabled |
| LogThread | NORMALPRIO−15 | 100 Hz | 8 KB | Disabled |

**Hardware inits disabled in main.cpp:** `can_drv_init()`, `radio_input_init()`, `i2c_drv_init()`.

---

## Motor Pin Mapping

| Motor | Role | AUX | Pin  | Timer    | Channel | AF  |
|-------|------|-----|------|----------|---------|-----|
| 0 | FR | 3 | PE11 | TIM1 | CH2 | AF1 |
| 1 | RL | 4 | PE9  | TIM1 | CH1 | AF1 |
| 2 | FL | 5 | PD13 | TIM4 | CH2 | AF2 |
| 3 | RR | 2 | PE13 | TIM1 | CH3 | AF1 |

---

## DShot600 Bidirectional Architecture

### TX Phase

Each `dshot_write()` call:

1. Stops both timers and disables all DMA streams.
2. Calls `switch_to_output()`: all TIM1/TIM4 channels set to **PWM mode 2** (active-LOW, bidir idle = HIGH), preload enabled, CCR=0, EGR=UG to flush shadow registers.
3. Reloads **three separate TX DMA streams** for TIM1, all mapped to `DMAMUX=TIM1_UP(15)`:

   | Stream | Writes to | Motor | Buffer |
   |--------|-----------|-------|--------|
   | `s_dma_tx_m0` | `&TIM1->CCR2` | M0 PE11 | `s_tx_buf_m0[18]` |
   | `s_dma_tx_m1` | `&TIM1->CCR1` | M1 PE9  | `s_tx_buf_m1[18]` |
   | `s_dma_tx_m3` | `&TIM1->CCR3` | M3 PE13 | `s_tx_buf_m3[18]` |

   All three DMAMUX channels receive each TIM1_UP event simultaneously — CCR1, CCR2, and CCR3 are written on the same UEV with **zero inter-channel offset**. This is the critical fix: the old single-stream burst approach wrote CCR1→CCR2→CCR3 on successive UEVs (1.67 µs / 3.33 µs offset), causing motors 0 and 3 to receive misaligned DShot frames.

4. Reloads one TX stream for TIM4: `s_dma_tim4` → `&TIM4->DMAR` (DCR: DBA=CCR2, DBL=0).
5. Sets phase=TX for both timers, then starts TIM1 and TIM4.

Each buffer is 18 words: 16 data bits (CCR value = DS_T1H=250 or DS_T0H=125 timer counts) + 2 trailing zeros (output high = idle). The 16-bit DShot frame format is `(throttle << 1) | telem_bit`, then a 4-bit inverted CRC appended at bits [3:0]: `frame = (val << 4) | ((~(val ^ val>>4 ^ val>>8)) & 0xF)`.

### TX → IC Transition

When any of the three TX streams completes (all 18 words written = 18 UEVs), its TC callback fires at priority 7. The **phase check** ensures only the first TC to fire acts:

```
dshot_dma_tc_tx_tim1():
  if phase != TX → return          // second/third TC from sibling streams
  s_dma_tc_count[0]++
  TIM1->CR1 &= ~CEN                // stop timer
  phase = IC
  start_ic_tim1()
```

TIM4 uses a single callback (`dshot_dma_tc_tim4`) that handles both TX→IC and IC-complete transitions by checking phase.

### IC Phase — TIM1

`start_ic_tim1()` configures the dedicated `s_dma_ic_tim1` stream for the current rotation slot and restarts TIM1 in free-run mode (ARR=20000 = 100 µs timeout):

| Slot | Motor | Pin | CCMR setting | DMAMUX | DCR DBA |
|------|-------|-----|--------------|--------|---------|
| 0 | M0 | PE11 | CC2S=TI2 (direct) | TIM1_CH2 (12) | CCR2 (14) |
| 1 | M1 | PE9  | CC2S=TI1 (cross)  | TIM1_CH2 (12) | CCR2 (14) |
| 2 | M3 | PE13 | CC3S=TI3 (direct) | TIM1_CH3 (13) | CCR3 (15) |

The cross-capture trick (slot 1): CC2S=TI1 routes PE9 (CH1's pin) into the CC2 capture path. This lets both M0 and M1 use the same DMA stream (TIM1_CH2 request → reads DMAR → DCR maps to CCR2), avoiding any need for a CC1DE path. CCER is configured with both polarity bits (CC2P + CC2NP) set — this enables capture on both rising and falling edges (both-edge mode, needed for GCR).

IC runs until either:
- `s_dma_ic_tim1` TC fires (all 22 buffer slots filled → `finish_ic_tim1(22)`)
- TIM1_UP ISR fires (UIE timeout → `finish_ic_tim1(captured)` with partial count)

Slot advances 0→1→2→0 after each IC session.

### IC Phase — TIM4

`start_ic_tim4()` reconfigures `s_dma_tim4` P→M for TIM4_CH2 (DMAMUX=30). TIM4 DCR (DBA=CCR2, DBL=0) is the same for TX and IC — no change needed. Captures M2/PD13 every frame at 400 Hz.

### GCR Decoder

Input: array of up to 22 CCR timestamps from the IC buffer.

1. **Bit reconstruction** (transition-marking, matches ArduPilot/BetaFlight):
   For each inter-edge interval of `n` GCR bit-widths:
   ```cpp
   bits = (bits << n) | (1U << (n - 1U));
   ```
   The last edge fills remaining bits to 21.

2. **GCR decode**: Split 21-bit word into four 5-bit codes (bits [20:16], [15:11], [10:6], [5:1]); look up each in `kGcrDecode[32]` → four 4-bit nibbles → 16-bit frame.

3. **CRC check**: `(frame ^ frame>>4 ^ frame>>8 ^ frame>>12) & 0xF == 0xF` (inverted nibble sum).

4. **eRPM**: `frame[15:4]` = 12-bit value; bits [11:9] = 3-bit exponent, bits [8:0] = 9-bit mantissa. `period_us = mantissa << exponent`. `eRPM = 60,000,000 / period_us`. Mechanical RPM = `eRPM / (pole_pairs)` = `eRPM / 7` for a 14-pole motor.

### DMA stream summary

| Stream | Timer | Phase | DMAMUX | Direction | Destination | Size |
|--------|-------|-------|--------|-----------|-------------|------|
| `s_dma_tx_m0` | TIM1 | TX | TIM1_UP (15) | M→P | `&TIM1->CCR2` | 18 words |
| `s_dma_tx_m1` | TIM1 | TX | TIM1_UP (15) | M→P | `&TIM1->CCR1` | 18 words |
| `s_dma_tx_m3` | TIM1 | TX | TIM1_UP (15) | M→P | `&TIM1->CCR3` | 18 words |
| `s_dma_ic_tim1` | TIM1 | IC | TIM1_CH2 or CH3 | P→M | `&TIM1->DMAR` | 22 words |
| `s_dma_tim4` | TIM4 | TX+IC | TIM4_UP / TIM4_CH2 | M→P / P→M | `&TIM4->DMAR` | 18 / 22 words |

---

## DShot Diagnostic Output

HeartbeatThread auto-prints every 2 s (no command needed):

```
$DSHOT,<ms>,tc=<tim1_dma_tc>/<tim4_dma_tc>,cc=<tim1_cc>/<tim4_cc>,edges=<m0>/<m1>/<m2>/<m3>
```

| Value | Meaning |
|-------|---------|
| `tc` counting up at ~400/s | DShot IS transmitting |
| `tc=0/0` | DShot NOT transmitting — `dshot_init()` failed; watch for LED error flash |
| `edges` > 0 | ESC responding with GCR telemetry — signal physically reaches ESC |
| `edges=0/0/0/0` | ESC NOT responding — not armed, not spinning, or wiring issue |

**LED error codes if DMA alloc fails in `dshot_init()`:**
- 20 rapid flashes loop = any TIM1 DMA alloc failed
- 30 rapid flashes loop = TIM4 DMA alloc failed

**Full diagnostic** (per-motor edge timestamps): send `DSHOT,diag` over USB.

---

## USB Command Interface

USBCmdThread (NORMALPRIO-20) reads newline-terminated commands from the USB CDC port. All responses are newline-terminated ASCII.

| Command | Response | Description |
|---------|----------|-------------|
| `MT,<m>,<pct>` | `MT,OK,<m>,<pct>` | Spin motor `m` (0–3) at `pct`% (0=stop, 1–100). Blocked if `g_armed=true`. `pct=0` → DShot value 0 (disarm). `pct>0` → `48 + pct*1999/100`. |
| `MT,stop` | `MT,OK,stopped` | Stop all motor test commands, resume idle DShot output. |
| `DSHOT,diag` | `DSHOT,DIAG,...` | Full per-motor edge timestamps + DMA/CC counters. |
| `BOOT` | `BOOT,OK` (then reset) | Reboot into CubePilot BL5 bootloader for firmware upload. |
| `CAL,set,<i>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>` | `CAL,SET,<i>,OK` | Stage IMU bias for lane `i` (0–2). Does not write to flash. |
| `CAL,commit` | `CAL,OK` | Write staged calibration to Bank 2 sector 7 flash. Applies immediately. |
| `CAL,clear` | `CAL,OK` | Erase calibration sector. |
| `CAL,query` | `CAL,DATA,...` | Read and print current flash calibration data. |
| `LOG,list` | `LOG,FILE,<name>,<size>` × N, then `LOG,LIST,END` | List files in `/LOGS/` on SD card. |
| `LOG,get,<name>` | `LOG,SIZE,<n>` then binary | Download log file as binary stream. |
| `LOG,erase` | `LOG,ERASED,<n>` | Delete all log files except the currently-open one. |
| `CAN,status` | `CAN,STATUS,psr=…` | Read FDCAN1 PSR/ECR/RXF0S/CCCR registers (requires no CANThread). |

---

## DebugThread Telemetry ($TEL, $EKFL)

Available in `BPRL_DEBUG` builds at 10 Hz.

**`$TEL` format:**
```
$TEL,<ms>,<roll°>,<pitch°>,<yaw°>,<p>,<q>,<r>,<thr>,<rc_roll>,<rc_pitch>,<rc_yaw>,<armed>,<rpm0>,<rpm1>,<rpm2>,<rpm3>,<imu0_v>,<imu1_v>,<imu2_v>,<can_v>,<can_quat_hz>,<can_rate_hz>
```
RPM values: `eRPM / 7` (mechanical RPM for a 14-pole motor). Zero if ESC telemetry not valid.

**`$EKFL` format:**
```
$EKFL,<ms>,<primary_lane>,<roll0°>,<pitch0°>,<yaw0°>,<p0>,<q0>,<r0>,[lane1...],[lane2...],[INS placeholder 0s]
```
Per-lane EKF state; useful for verifying lane agreement before enabling any sensor fusion.

---

## USB CDC Initialisation

`usb_serial_init()` in [src/usb_serial.cpp](src/usb_serial.cpp):

The CubePilot BL5 bootloader leaves D+ asserted (DCTL_SDIS=0). Calling `usbStart()` while the host is actively sending SOF frames causes `otg_core_reset` to spin forever waiting for AHBIDL. Fix: `usbDisconnectBus()` → 5 ms delay → `usbStart()` → `usbConnectBus()`. Host sees a clean disconnect/reconnect and enumerates fresh.

Additionally, `USB_EVENT_WAKEUP` restores `SDU_READY` state when Linux autosuspend causes SUSPEND→WAKEUP without a new SET_CONFIGURATION (so `USB_EVENT_CONFIGURED` never re-fires and SDU stays stuck in SDU_STOP).

---

## Persistent IMU Calibration — CalFlash

`cal_save()` / `cal_load()` / `cal_clear()` in [src/coms/CalFlash.cpp](src/coms/CalFlash.cpp):

Stores a `CalibData` struct (gyro/accel biases for all 3 IMU lanes) in **Bank 2 Sector 7** (`0x081E0000`). STM32H743 Bank 2 registers are `FLASH->KEYR2 / CR2 / SR2 / CCR2`. Programming unit is 256 bits (32 bytes); `CalibData` at 128 bytes requires 4 programming cycles. CRC-32/ISO-HDLC over the data header detects corruption. The `magic` field (`0x42525943` = `BPRC`) and `version` (currently 1) reject stale or incompatible blobs.

SPIThread loads calibration at startup via `cal_load(g_cal)`. `CAL,commit` both writes to flash and applies the biases to the live `g_cal` without requiring a reboot.

---

## State Estimation — EKF Architecture

Three parallel EKF lanes in [src/state_estimator/EKF.cpp](src/state_estimator/EKF.cpp), managed by [src/state_estimator/StateManager.cpp](src/state_estimator/StateManager.cpp):

- **19-state vector**: quaternion (q0–q3), angular rates (p, q, r), velocity (u, v, w, w_dot), position (x, y, z, z_pos), gyro biases (bpx, bpy, bpz), accel bias (baz).
- Each lane receives one on-board IMU as its primary sensor. The IMX5 (CAN INS) quaternion is fused into all three lanes as a measurement update when available.
- IMX5 sends `q_NED→body`; EKF stores `q_body→NED` — conjugation is applied in `StateManager`.
- Covariance uses Joseph form; chi-squared innovation gate; adaptive R; ArduPilot-matched noise parameters.
- Primary lane selection and lane-health monitoring handled by `StateManager`.

**Currently disabled** — StateEstThread not started in the motor-test configuration.

---

## CAN Driver — IMX5

`can_drv_init()` in [src/coms/CAN.cpp](src/coms/CAN.cpp):

FDCAN1 at 1 Mbit/s (BRP=1, TSEG1=18, TSEG2=5, 79% sample point, HSE=24 MHz). IMX5 sends four CAN frames on IDs 0x01–0x04:

| ID | Content |
|----|---------|
| 0x01 | `CID_INS_QUATN2B` — quaternion [W,X,Y,Z] as int16 × 10000 |
| 0x02 | p (rad/s × 1000), ax (m/s² × 100) |
| 0x03 | q (rad/s × 1000), ay (m/s² × 100) |
| 0x04 | r (rad/s × 1000), az (m/s² × 100) |

**Currently disabled** — `can_drv_init()` not called; CANThread not started.

---

## Main Startup Sequence

`main()` in [main.cpp](main.cpp):

1. `halInit()` — hardware abstraction layer; `boardInit()` runs in here.
2. IWDG1 started with ~32 s timeout (`/256`, RLR=0xFFF) — main thread kicks it every second.
3. LED diagnostic blinks: 3 fast (halInit done) + 5 slow (chSysInit done).
4. `chSysInit()` — RTOS kernel starts.
5. `usb_serial_init()` — USB CDC enumerated as `0483:5740`.
6. 1500 ms delay for host USB enumeration.
7. `motor_output_init()` → calls `dshot_init()`.
8. `threads_start(kRates)` — creates ControlThread, USBCmdThread, HeartbeatThread (+ DebugThread if BPRL_DEBUG).
9. Main thread loops kicking IWDG every second.

`boardInit()` in [boards/CubeOrangePlus/board.c](boards/CubeOrangePlus/board.c): Configures PA8 LOW (`nVDD_5V_PERIPH_EN` — enables 5 V peripheral rail powering the AUX level-shifter ICs) and PB4 HIGH (`PWM_VOLT_SEL` — 3.3 V output mode). Without these two pins the AUX connector outputs no signal. Also configures sensor power rail (PE3), SPI1/SPI4 buses, USART2 (CRSF), USART3 (TELEM2), USB OTG_FS, FDCAN1, SDMMC1, and status LED (PB0).

---

## Changes History

### Original Bringup Fixes

**1. Flash load address** (`boards/CubeOrangePlus/STM32H743xI_app.ld`, `Makefile`)
BL5 at `0x08000000–0x0801FFFF`. App start corrected to `0x08020000`. Bank 2 sector 7 (`0x081E0000`) reserved for IMU calibration.

**2. USB CDC hang** (`src/usb_serial.cpp`)
BL5 leaves D+ asserted. Fixed: `usbDisconnectBus()` before `usbStart()`.

**3. DShot pin mapping** (`src/coms/DShot.cpp`)
Original code used wrong pins (PA8/PA9/PA10/PE14, not routed to AUX OUT). Corrected to PE9/PE11/PE13/PD13.

**4. DMAMUX source IDs** (`src/coms/DShot.cpp`)
`DMAMUX_TIM1_UP = 15` (was 11); `DMAMUX_TIM4_UP = 32`. DMA never triggered with the wrong IDs.

**5. RC input wiring** (`src/coms/CRSF.cpp`, `boards/*/board.c`)
USART6 is FMU↔IOMCU bridge. Moved RC input to USART2/TELEM1.

**6. DebugThread USB blocking** (`src/threads.cpp`)
`chprintf` on a full USB queue blocks indefinitely. Fixed: format into `MemoryStream` then send with `chnWriteTimeout`. Added `s_usb_write_mtx` mutex for all USB writes.

**7. IMU rotation corrections** (`src/threads.cpp` SPIThread)
NED z-down body frame per ArduPilot hwdef for all three IMU lanes.

**8. NED z-down conversion** (`src/state_estimator/EKF.cpp`, `StateManager.cpp`)
EKF predict, gravity update, and output sign corrections all converted from z-up.

**9. IMX5 quaternion conjugation** (`src/state_estimator/StateManager.cpp`)
IMX5 outputs `q_NED→body`; EKF stores `q_body→NED`. Conjugation applied.

**10. EKF robustness** (`src/state_estimator/EKF.cpp`)
Joseph form covariance, chi-squared innovation gate, adaptive R, ArduPilot-matched noise parameters. All three lanes verified correct roll/pitch/yaw on hardware.

**11. CalFlash driver** (`src/coms/CalFlash.cpp/.hpp`)
Persistent IMU bias calibration in Bank 2 sector 7. CRC-32 + magic + version validation.

---

### DShot / Motor Test Fixes

**12. DShot telemetry bit — ESC not arming** (`src/coms/DShot.cpp` `make_frame()`)

`make_frame()` set telem bit unconditionally: `(thr << 1) | 1U`. DShot values 0–47 with telem=1 are special commands; value 0, telem=1 = MOTOR STOP. The ESC received a continuous stream of MOTOR STOP commands and never completed arming.

Fix: `val = (uint16_t)(thr << 1)` — telem bit stays 0 for all throttle packets (matches ArduPilot). Correct frame for throttle=0 bidir: `0x000F`.

**12b. AUX output power and voltage select** (`boards/CubeOrangePlus/board.c`)

`boardInit()` was missing two pins required to power the AUX level-shifter ICs:
- PA8 → OUTPUT LOW (`nVDD_5V_PERIPH_EN` — enables 5 V peripheral rail)
- PB4 → OUTPUT HIGH (`PWM_VOLT_SEL` — 3.3 V output mode, releases OE)

Without these, no DShot signal ever reaches the motors regardless of timer/DMA config.

**13. DShot IRQ priority raised** (`src/coms/DShot.cpp`)

TIM1_CC and TIM4 IRQs raised from `CORTEX_MINIMUM_PRIORITY` (15) to priority 7 (matching `STM32_IRQ_TIM1_CC_PRIORITY` / `STM32_IRQ_TIM4_PRIORITY`). At priority 15, USB/SPI/CAN ISRs preempted edge-capture handlers mid-GCR-frame, overwriting CCR before it was read.

**14. DIER cleared before `switch_to_output`** (`src/coms/DShot.cpp`)

Added `TIM1->DIER = 0; TIM4->DIER = 0; TIM1->SR = 0; TIM4->SR = 0` at the start of every `dshot_write()`. With UIE still set from a previous IC session, `EGR=UG` in `switch_to_output()` would queue a spurious TIM1_UP IRQ that decoded stale edge data.

**15. Thread and hardware simplification** (`src/threads.cpp`, `main.cpp`)

Removed SPIThread, CANThread, StateEstThread, I2CThread, RadioThread, LogThread from `threads_start()`. SPIThread's 1 kHz SPI DMA ISRs (priority ~12) preempted TIM1_CC edge-capture (then at priority 15) mid-GCR-frame. Removed `can_drv_init()` — FDCAN RxFIFO overflow ISRs added noise when no CANThread consumed frames.

**16. Debug assertions enabled** (`cfg/chconf.h`)

`CH_DBG_ENABLE_ASSERTS = TRUE` for BPRL_DEBUG builds. With assertions off, a failed `dmaStreamAlloc()` returned null silently; the next `dmaStreamSetPeripheral(nullptr, ...)` caused a silent hard fault → IWDG reset loop.

**17. DMA alloc failure detection** (`src/coms/DShot.cpp`)

Explicit null-checks after every `dmaStreamAlloc()`. On failure: 20-flash LED loop (TIM1) or 30-flash loop (TIM4) rather than a silent crash.

---

### DMA-Driven Input Capture Rewrite

**18. Root cause of ESC not arming — stale TX DMA TC callback**

TX DMA TC callback ran at priority 15. In the worst case, the TC from frame N fired inside the TX window of frame N+1 (delayed by higher-priority ISRs), calling `TIM1->CR1 &= ~CEN` and `switch_tim1_to_ic()` mid-TX. That frame was silently dropped. After ~1 s of dropped frames, the ESC watchdog reset and no arming chime occurred.

Fix: Raise DMA TC callback priority from 15 to 7 (`STM32_IRQ_TIM1_CC_PRIORITY`). TC fires within nanoseconds of the last DMA transfer at priority 7.

**19. DMA-based input capture** (`src/coms/DShot.cpp`)

Replaced ISR-based edge capture (CCxIE) with DMA-based capture (CCxDE + DMAR). ISR approach at 750 kHz GCR (1.33 µs/bit) missed edges when two edges arrived within one ISR service time. DMA approach: each CC event triggers hardware DMA reads CCR via DMAR into IC buffer with no CPU involvement.

**20. Phase state machine** (`src/coms/DShot.cpp`)

`DshotPhase { IDLE, TX, IC }` per timer. Set to IDLE at the start of every `dshot_write()` so any stale DMA TC callback sees IDLE and returns without action. Set to TX just before `TIM1->CR1 |= CEN`.

**21. GCR decoder rewrite** (`src/coms/DShot.cpp`)

Prior decoder used level-filling (fill n copies of current level). ArduPilot/BetaFlight use transition-marking: `bits = (bits << n) | (1 << (n-1))`. These produce completely different 21-bit patterns — the level-filling approach produced invalid GCR codes for every real ESC packet. CRC check was also inverted (always false). Both bugs fixed together.

**22. `$DSHOT` diagnostic field meanings (updated)**

| Field | Meaning |
|-------|---------|
| `tc` | TX DMA TC count — increments once per `dshot_write()` when TX completes |
| `cc` | IC complete count — increments when IC decode fires (DMA TC or UIE timeout) |
| `edges` | GCR edges captured in last IC session per motor |

**23. Rate aligned to ArduPilot** (`main.cpp`)

`TIME_US2I(2000)` → `TIME_US2I(2500)`. ControlThread at **400 Hz** matches ArduPilot ArduCopter default.

**24. Concurrent IC/DCR conflict** (`src/coms/DShot.cpp`)

An attempt to run M3 IC concurrently with TX by arming a second IC stream before TX started introduced two fatal bugs: (1) `start_ic_m3()` changed `TIM1->DCR` mid-TX burst, so all TX burst words went to CCR3 only — M0/M1/M3 all sent garbage. (2) Setting CC3S=input before TX started switched CH3 to input-capture mode, disabling the PE13 output driver. Fix: removed `s_dma_ic_tim1` concurrent approach; reverted to clean sequential TX→IC rotation.

**25. ArduPilot DMA channel scheme** (`src/coms/DShot.cpp`)

TIM1 IC scheme now matches ArduPilot's cross-capture approach: M0/M1 rotation uses CC2 only (CC2S=TI2 for M0, CC2S=TI1 cross for M1), both reading from CCR2 via DBA=14. M3 uses CC3/CCR3. ArduPilot never uses CC1DE; BPRL now matches.

**26. Two GCR decoder bugs — always returned false** (`src/coms/DShot.cpp`)

**Bug A:** Wrong bit reconstruction (level-filling vs transition-marking) — produced invalid GCR codes for all real ESC packets.

**Bug B:** CRC check was `CRC_computed == frame[3:0]` using only 12-bit value; correct is `(n0 ^ n1 ^ n2 ^ n3) & 0xF == 0xF` over all four nibbles. The BPRL check reduced to `0 == 0xF` — always false.

**27. eRPM decoder: period stored instead of RPM** (`src/coms/DShot.cpp` `decode_gcr()`)

BDShot response encodes electrical revolution **period** in µs. Decoder was storing the raw period as `erpm` — higher throttle produced smaller values. Fix: `erpm = (period > 0) ? (60000000 / period) : 0`.

**28. Pole count divisor halved** (`src/threads.cpp` DebugThread)

`$TEL` RPM display divided eRPM by 14 (pole count) instead of 7 (pole pairs). Changed `/14U` → `/7U`.

**29. Python GCR test script bugs** (`tools/dshot_decode_test.py`)

- Frame format backwards: CRC at bits [15:12] instead of [3:0].
- Wrong timestamp generation: edge timestamps from 20-bit raw GCR word; decoder requires timestamps from 21-bit word `(1<<20)|gcr20`.
- Tests expect exact eRPM recovery; mantissa/exponent encoding is lossy. Updated to compare against lossy-encoded value.

10/15 tests pass after fixes; 5 still failing (low priority — see Known Issues).

**30. Root cause of motors 0, 2, 3 not working** (`src/coms/DShot.cpp`)

The single-stream burst TX approach (one DMA stream → `TIM1->DMAR`, DCR: DBA=CCR1, DBL=2) wrote CCR1, CCR2, CCR3 on successive UEVs, offset by one timer period (1.67 µs) each. Motor 1 (CCR1) received its value one UEV before motors 0 and 3 — the DShot bit boundaries on M0/M3 were misaligned by 1.67 µs / 3.33 µs, causing those ESCs to reject every frame.

**31. Per-channel TX DMA attempt — broke all motors** (`src/coms/DShot.cpp`)

Attempted to use three separate DMA streams all mapped to DMAMUX=TIM1_UP, expecting the DMAMUX to fan out each UEV to all three streams simultaneously. In practice on this hardware the DMAMUX appears to deliver each TIM1_UP request to only one stream at a time (round-robin), so each stream received every 3rd UEV → each bit period became 5 µs = DShot200. All ESCs rejected the out-of-spec frames. **Reverted — see change #32.**

**32. Burst DMAR with correctly interleaved buffer — correct fix** (`src/coms/DShot.cpp`)

Reverted to a single-stream DMAR burst approach matching ArduPilot exactly. One DMA stream writes to `TIM1->DMAR`. DCR: DBA=CCR1, DBL=3 (4 registers: CCR1/M1, CCR2/M0, CCR3/M3, CCR4/dummy). FIFO full threshold (4 words): one TIM1_UP event drains all 4 FIFO words to DMAR in one burst, updating CCR1/CCR2/CCR3 simultaneously within the same UEV (~5 ns apart).

Buffer: `s_tx_buf_tim1[72]` — interleaved stride-4: `[M1_bit, M0_bit, M3_bit, 0] × 18 rows`.

| Field | Value |
|-------|-------|
| DCR | DBA=CCR1(13), DBL=3 |
| Buffer | 72 words (18 UEVs × 4 words/UEV) |
| FIFO | full threshold (4 words per TIM_UP drain) |
| IC stream | `s_dma_ic_tim1` unchanged |

---

## Known Open Issues

| # | Issue | Status |
|---|-------|--------|
| 1 | **All 4 motors working** | Code fix complete (change #32). **Pending hardware verification** — flash and test `MT,0,10` / `MT,1,10` / `MT,2,10` / `MT,3,10`. |
| 2 | Python GCR test script | 5/15 tests still failing. Low priority — not needed once all 4 motors verified on hardware. |
| 3 | CAN / IMX5 fusion | Deferred |
| 4 | CRSF radio | Deferred |

---

## Planned Work

**Priority 1 — Verify all 4 motors on hardware**

Flash current build and test:
```
MT,0,10    ← AUX 3, PE11, Motor 0 (FR)
MT,1,10    ← AUX 4, PE9,  Motor 1 (RL) — was working before
MT,2,10    ← AUX 5, PD13, Motor 2 (FL)
MT,3,10    ← AUX 2, PE13, Motor 3 (RR)
MT,stop
```
Verify ESC connection chime on power-up for all 4. Verify `edges` non-zero in `$DSHOT` output when spinning. Verify RPM increases with throttle %.

**Priority 2 — Re-enable full flight stack**

Once all 4 motors work:
1. Re-enable SPIThread (verify no ISR contention with DShot at the new priority 7 — may need to adjust SPIThread DMA ISR priority or use the BPRL_DEBUG build's `$DSHOT` output to confirm `edges` remain non-zero).
2. Re-enable StateEstThread, RadioThread.
3. Restore ControlThread to full PID/mixer path.
4. Re-enable CAN (`can_drv_init()` + CANThread) and verify IMX5 quaternion arriving at ~200 Hz.
5. Test complete attitude loop.

**Priority 3 — Fix Python GCR test (5 remaining failures)**

`tools/dshot_decode_test.py` — corrupted-edge rejection test and CRC inversion test still failing. Likely the inline CRC-test code block at the bottom of the script which duplicates the encode logic without the frame-format fix.

---

## Testing Roadmap

### ▶ 1. Motor Test (Active — 1 of 4 motors verified, fix applied for 0/2/3)

```
MT,0,5    ← spin motor 0 (FR) at 5%
MT,1,5    ← spin motor 1 (RL) at 5%
MT,2,5    ← spin motor 2 (FL) at 5%
MT,3,5    ← spin motor 3 (RR) at 5%
MT,stop   ← stop all
```

Verify RPM telemetry: `edges` in `$DSHOT` output should be non-zero when motors are spinning. Use `DSHOT,diag` for full per-motor edge timestamps.

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

Once motors work: re-enable SPIThread, StateEstThread, RadioThread, LogThread, and restore ControlThread to full PID/mixer path. Test complete attitude loop.

