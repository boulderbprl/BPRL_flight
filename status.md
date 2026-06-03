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
| **ControlThread** | NORMALPRIO+20 | **400 Hz** | 2 KB | **Running** — sends DShot 0 directly (motor test bypass via `MT,` USB commands) |
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

**Current hardware test result (ISR-based build, pre-DMA-IC fix):**
```
$DSHOT,36203,tc=16000/16000,cc=16000/16000,edges=0/0/0/0
```
Firmware was sending DShot at 500 Hz ✓. ESC was not arming (no connection chime) — root cause identified and fixed in change #18 below.

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

### DMA-Driven Input Capture Rewrite (this session — fixes ESC not arming)

**18. Root cause of ESC never arming (no connection chime)** (`src/coms/DShot.cpp`)

Two compounding bugs prevented the ESC from receiving a valid DShot signal:

**Bug A — DMA TC callback at CORTEX_MINIMUM_PRIORITY (15).**

The TX DMA TC callback (`dshot_dma_tc_tim1`) ran at priority 15 — the lowest
possible on Cortex-M.  The OS tick (priority ~2) and any thread wakeup can
preempt and delay it.  In the worst case:

1. ControlThread calls `dshot_write()` (2 ms rate).
2. TX DMA fires 18 UEVs × 1.667 µs ≈ 30 µs later; TC IRQ becomes pending.
3. A higher-priority ISR or OS event delays the TC callback.
4. `dshot_write()` for frame N+1 runs: DMA is re-enabled, TIM1 starts in TX mode.
5. The stale TC callback from frame N now fires inside the new TX window.
6. Callback does `TIM1->CR1 &= ~CEN` (stops mid-TX) then `switch_tim1_to_ic()`
   which rewrites CCMR to IC mode and starts TIM1 in IC mode.
7. TIM1 is now in IC mode, but DIER = CCxDE|UIE (not UDE) → no DMA TX requests
   → the DShot frame is silently dropped.

Enough dropped frames → ESC never accumulates the ~1 second of valid frames
needed to complete arming → no connection chime, no motor response.

**Fix:** Raise DMA TC callback priority from `CORTEX_MINIMUM_PRIORITY` (15) to
`STM32_IRQ_TIM1_CC_PRIORITY` (7).  At priority 7 the TC fires within
nanoseconds of the last DMA transfer, before any OS activity can intervene.

**Bug B — ISR-based edge capture (CCxIE) at 750 kHz GCR rate.**

The prior IC used `TIM_DIER_CC1IE|CC2IE|CC3IE` — a CPU ISR per edge.  At
750 kHz GCR, each bit is 1.33 µs.  Even at priority 7, three simultaneous
channels share one `TIM1_CC_HANDLER` ISR.  If two edges arrive within one ISR
service time, the earlier CCR is overwritten before it is read → edge missed.
Result: `edges` counter bounces or stays 0, RPM decode fails.

**19. DMA-based input capture — ArduPilot approach** (`src/coms/DShot.cpp`)

The same DMA stream that drives TX is reconfigured for IC after the TX TC fires.

| Phase | DMAMUX | Direction | DCR | DIER |
|-------|--------|-----------|-----|------|
| TX | TIM1_UP (15) | M→P | DBA=CCR1, DBL=2 (burst 3) | UDE |
| IC | TIM1_CCx (11/12/13) | P→M | DBA=CCRx, DBL=0 (1 reg) | CCxDE + UIE |

Each CC capture event → hardware DMA reads DMAR (= selected CCR) → appended
to IC buffer with no CPU involvement.  GCR edges at 750 kHz are captured
perfectly regardless of IRQ load.

TIM1 rotates through one channel per frame (3 channels → each motor gets
telemetry at ~167 Hz instead of 500 Hz, which is adequate):

| Slot | Channel | CCR | Motor | Pin |
|------|---------|-----|-------|-----|
| 0 | CH2 | CCR2 | M0 FR | PE11 |
| 1 | CH1 | CCR1 | M1 RL | PE9 |
| 2 | CH3 | CCR3 | M3 RR | PE13 |

TIM4 captures CH2/M2/PD13 every frame (500 Hz).

**20. Phase state machine** (`src/coms/DShot.cpp`)

`DshotPhase { IDLE, TX, IC }` per timer.  Set to `IDLE` at the start of every
`dshot_write()` so any stale DMA TC callback sees `IDLE` and returns without
action.  Set to `TX` just before `TIM1->CR1 |= CEN`.

**21. GCR decoder rewrite** (`src/coms/DShot.cpp`)

Prior decoder used edges[0] absolute timestamp (which includes the ~30 µs
guard time, = ~22 GCR bit-widths, causing the 21-bit frame to overflow) and
had incorrect level logic.

New decoder:
- Starts from the first inter-edge interval (`buf[1] - buf[0]`), not `buf[0]`.
- Tracks signal level starting at LOW (first edge is always falling
  = ESC pulls line low to start GCR).
- Fills any remaining bits with the current level when edges are exhausted.
- CRC check uses **non-inverted** CRC (the GCR telemetry reply from the ESC
  uses a standard CRC; only the DShot command CRC is inverted for bidir).

**23. Match ArduPilot rate and DMA channel scheme** (`main.cpp`, `src/coms/DShot.cpp`)

**Rate:** `TIME_US2I(2000)` → `TIME_US2I(2500)` in main.cpp.
ControlThread now runs at **400 Hz** (2.5 ms), matching ArduPilot ArduCopter default.

**DMA channels — why BPRL was different:**
BPRL was written from scratch.  The prior design used one DMA stream for TIM1 TX
and rotated all three TIM1 channels (M0/M1/M3) through it in IC mode — a 3-way
rotation giving each motor ~167 Hz telemetry.  ArduPilot uses a dedicated IC
stream for the CH3/CH4 group (M3 only) and a shared TX→IC stream for the CH1/CH2
group (M0/M1 rotation), plus the ArduPilot cross-capture trick: CC2 captures PE9
(TI1 cross-wired) for M1 instead of CC1 capturing TI1 directly.

**New DMA scheme (matches ArduPilot):**

| DMA stream | TX use | IC use | Motors | Rate |
|-----------|--------|--------|--------|------|
| `s_dma_tx_tim1` | TIM1_UP burst | CC2DE (M0/M1 rotation, CC2S selects pin) | M0, M1 | 200 Hz each |
| `s_dma_ic_tim1` | — (IC only) | CC3DE (M3 always) | M3 | 400 Hz |
| `s_dma_tim4` | TIM4_UP burst | CC2DE (M2 always) | M2 | 400 Hz |

**ArduPilot cross-capture trick for M1:**
- M0 turn: `CC2S=TI2` (CC2 reads PE11 directly) → reads CCR2
- M1 turn: `CC2S=TI1` (CC2 reads PE9 via TI1 cross-input) → reads CCR2
- DCR.DBA always points to CCR2 for both; DMAMUX always TIM1_CH2 (12).
- ArduPilot never uses CC1DE; neither does BPRL now.

**M3 concurrent IC:** `s_dma_ic_tim1` is armed at the start of every
`dshot_write()` call, before the TX timers start.  CC3DE fires on TIM1_CH3 edges.
During TX, PE13 is driven by the timer (output mode) so no edges appear.
The first capture will be the ESC's GCR response after the DShot frame ends.

**24. Concurrent M3 IC during TX — critical DCR conflict** (`src/coms/DShot.cpp`)

An attempt to capture M3 telemetry at full 400 Hz by arming a second IC DMA stream
before the TX timers started introduced two fatal bugs:

**Bug 1 — DCR overwrite:** `start_ic_m3()` changed `TIM1->DCR` from the TX burst
config (`DBA=CCR1, DBL=2`) to the IC single-register config (`DBA=CCR3, DBL=0`).
The TX DMA then started and wrote all 54 burst words through CCR3 only — CCR1 and
CCR2 were never updated. M0, M1, and M3 all transmitted garbage pulse widths (or
no output at all on some slots).

**Bug 2 — CH3 in IC mode during TX:** `start_ic_m3()` set `CC3S=TI3` before the
timer started, switching CH3 to input-capture mode. The timer output driver for
CH3 was disabled (input mode = high-Z), so PE13/M3 produced no DShot signal
during the TX phase.

**Result:** All three TIM1 motors (M0, M1, M3) sent invalid or absent DShot.
The ESC watchdog fired after ~30 seconds of no valid signal → ESC rebooted →
power-up chime repeated every ~30 s. TIM4/M2 was unaffected (separate timer).

**Fix:** Removed `s_dma_ic_tim1` and `start_ic_m3()` entirely. Reverted to clean
3-way rotation (M0→M1→M3→M0) using the single TX DMA stream after TX completes.
TX and IC are fully sequential — TX fully finishes and the timer stops before IC
is configured. This matches ArduPilot's architecture exactly.

**Why concurrent IC on the same timer is impossible:** TX burst uses `DCR.DBA=CCR1,
DBL=2` (3-register burst per UEV). IC single-capture needs `DCR.DBA=CCRx, DBL=0`
(1-register read per CC event). There is one DCR per timer — the two modes fight
over it. ArduPilot never overlaps TX and IC on the same timer for the same reason.

**25. Rate and DMA channel alignment with ArduPilot** (`main.cpp`, `src/coms/DShot.cpp`)

`TIME_US2I(2000)` → `TIME_US2I(2500)`: ControlThread now runs at 400 Hz.

IC channel scheme now matches ArduPilot's cross-capture approach exactly:
- M0/M1 rotation uses CC2 only (`CC2S=TI2` for M0, `CC2S=TI1` cross for M1).
  Both read from CCR2 via DBA=14. ArduPilot never uses CC1DE; BPRL now matches.
- M3 uses CC3/CCR3 (slot 2 of rotation). M2/TIM4 uses CC2 every frame.

**Telemetry rates:** M0=~133 Hz, M1=~133 Hz, M3=~133 Hz, M2=400 Hz.
This is identical to ArduPilot with 3 active TIM1 channels and 1 TIM4 channel.

**26. Two critical GCR decoder bugs — always returned false** (`src/coms/DShot.cpp`)

Full pipeline comparison against ArduPilot's `bdshot_decode_telemetry_packet` (via
automated agent analysis) revealed that the GCR decoder was broken in two
independent ways, guaranteeing it would return `false` on every valid ESC packet.

**Bug A — Wrong bit reconstruction algorithm (level-filling vs transition-marking)**

BPRL used *level-filling*: for a run of n bit-periods, fill n copies of the
current level (0 or 1) into the bit buffer, then toggle level.

ArduPilot/BetaFlight use *transition-marking*: for a run of n bit-periods, place
a '1' at the MSB followed by (n-1) '0's, regardless of level:
```cpp
bits = (bits << n) | (1U << (n - 1U));
```

These produce completely different 21-bit patterns. For any real ESC packet, the
level-filling approach produces invalid GCR codes that fail the decode table
lookup (`0xFF` = invalid nibble). A real-world ERPM value like 100 RPM produces
GCR code `0b01000` = 8 → decode table entry `0xFF` → returns false at the first
nibble. The decoder was unreachable for any valid signal.

**Bug B — CRC check was inverted (always false for valid packets)**

BPRL checked: `CRC_computed == frame[3:0]` using only the 12-bit value field.
ArduPilot checks: `(n0 ^ n1 ^ n2 ^ n3) & 0xF == 0xF` over all four nibbles of
the decoded 16-bit frame (which is the inverted nibble sum).

For any valid ESC GCR packet the inverted nibble sum IS 0xF (same CRC convention
as the bidir TX frame). The BPRL check was effectively `CRC == CRC ^ 0xF` which
reduces to `0 == 0xF` — always false.

**Combined effect:** Even if edges were being captured correctly, the decoder
returned false 100% of the time. `s_telem[motor].valid` was never set true.
RPM telemetry was completely non-functional.

**Fix:** Replace level-filling with transition-marking; replace the CRC check with
the inverted nibble sum check matching ArduPilot. Both bugs fixed together.

**Note on arming:** The TX pipeline was verified correct by the pipeline comparison
(all 14 TX register checks passed). The power-up chime repeating every ~30 s was
caused by Bug 24 (concurrent IC/DCR conflict) which silenced M0/M1/M3. The GCR
decoder bugs (Bug 26) would not prevent arming in themselves but would prevent RPM
telemetry from ever showing valid data.

**22. `$DSHOT` diagnostic field meanings (updated)**

| Field | Meaning |
|-------|---------|
| `tc` | TX DMA TC count — increments once per `dshot_write()` if TX completes |
| `cc` | IC complete count — increments when IC decode fires (DMA TC or UIE timeout) |
| `edges` | Number of GCR edges captured in last IC session per motor |

`tc` ≈ `cc` ≈ 500/s → both TX and IC are cycling.
`edges` 0/0/0/0 → ESC not sending GCR (not armed, not spinning, or hardware issue).
`edges` > 0 on any motor → GCR signal arriving; RPM decoding active.

---

### Hardware Test Results — Motor 1 Working (session 3)

**Hardware confirmed working:**
- Motor 1 (AUX 4, PE9, TIM1_CH1) arms, spins, and reports correct RPM
- RPM increases with throttle %, decreases with load — physically correct
- ESC connection chime confirmed on power-up

**Hardware confirmed broken:**
- Motors 0, 2, 3 (AUX 3, 5, 2) do not respond to MT commands
- Root cause identified — see change #27 below

---

### Session 3 Fixes (motor test partially working)

**27. eRPM decoder: period stored instead of RPM** (`src/coms/DShot.cpp` `decode_gcr()`)

The BDShot response encodes the electrical revolution **period** in µs, not RPM.
The decoder was storing the raw period value as `erpm`:
```cpp
*erpm_out = (uint32_t)mantissa << exponent;  // WRONG — this is the period
```

The ESC sends shorter periods as the motor spins faster, so higher throttle
produced *smaller* `erpm` values — RPM appeared backwards (e.g. ~4000 at stop,
~150 at 10% throttle). Fix:
```cpp
uint32_t period = (uint32_t)mantissa << exponent;
*erpm_out = (period > 0U) ? (60000000U / period) : 0U;
```
ArduPilot uses the same `60,000,000 / period_us` conversion.

**28. Pole count divisor halved** (`src/threads.cpp` DebugThread)

`$TEL` RPM display was dividing eRPM by 14 (pole count) instead of 7 (pole
pairs). For a 14-pole motor: `mechanical_RPM = eRPM / (poles/2) = eRPM / 7`.
Changed `/14U` → `/7U`.

**29. Python GCR test script — encoder bug** (`tools/dshot_decode_test.py`)

`gcr_encode_packet()` had two bugs:
- **Frame format backwards:** CRC was placed at bits [15:12] instead of [3:0].
  The BDShot telemetry frame (and the DShot TX frame) both use `(val<<4)|crc`
  format — period at bits [15:4], CRC at bits [3:0]. All decoded values were wrong.
- **Wrong timestamp generation:** The edge timestamps were computed from runs of
  the raw 20-bit GCR word. The decoder's transition-marking algorithm requires
  timestamps derived from the 21-bit word `(1<<20)|gcr20` — each `1`-bit in that
  word is one captured edge position.
- **Round-trip precision:** Tests expected exact eRPM recovery; the
  mantissa/exponent encoding is lossy (e.g. 5000 → 4992). Tests updated to compare
  against the lossy-encoded value.

Status: 10/15 tests pass after these fixes. **5 tests still failing** — the
Python test script is not yet complete (see planned work below).

**30. Root cause of motors 0, 2, 3 not working** (`src/coms/DShot.cpp`)

Confirmed by ArduPilot hwdef (CubeOrangePlus-bdshot):
```
AUX 2 = PE13 = TIM1_CH3 → Motor 3 in code
AUX 3 = PE11 = TIM1_CH2 → Motor 0 in code
AUX 4 = PE9  = TIM1_CH1 → Motor 1 in code  ← WORKS
AUX 5 = PD13 = TIM4_CH2 → Motor 2 in code
```
Pin mapping is correct. Old standard-DShot firmware works on all 4 motors.

**Root cause:** The TX DMA uses a single stream with burst mode (DCR: DBA=CCR1,
DBL=2). In burst mode with single-transfer-per-DMA-request, each update event
(UEV) writes ONE word via DMAR — the timer's burst controller advances the CCR
pointer by one register per write. This means:
- UEV 1 → CCR1 updated (motor 1 — AUX4)
- UEV 2 → CCR2 updated (motor 0 — AUX3), offset by 1 period (1.67 µs)
- UEV 3 → CCR3 updated (motor 3 — AUX2), offset by 2 periods (3.33 µs)

Motor 1 (CCR1) always has its shadow loaded one UEV before the others.
The 1–2 period offset on motors 0 and 3 causes their DShot frame bit boundaries
to be misaligned relative to what the ESC expects. Motor 2 (TIM4_CH2) is
similarly affected — TIM4 uses the same burst approach with DBL=0, but other
timing issues may apply.

ArduPilot avoids this by using **one DMA stream per channel**, all triggered by
the same TIM_UP event. All CCRs update on the same UEV with zero offset.

**Fix in progress** (partially applied, code not yet compilable):

Replace the single interleaved `s_tx_buf_tim1[18*3]` + burst DMAR approach
with three separate per-channel DMA streams writing directly to CCR1, CCR2,
CCR3 (no burst mode, no DMAR):

| Old | New |
|-----|-----|
| 1 stream (`s_dma_tim1`), 54-word interleaved buf, writes to `TIM1->DMAR` | 3 streams (`s_dma_tx_m0/m1/m3`), 18-word per-channel bufs, write to `CCR1/CCR2/CCR3` directly |
| DCR: DBA=CCR1, DBL=2 (burst) | DCR: not used for TX |
| 1 separate IC stream | 1 dedicated IC stream (`s_dma_ic_tim1`, separate from TX) |

All 3 TX streams triggered by TIM1_UP simultaneously → all CCRs update on the
same UEV → zero timing offset between channels.

---

## Known Open Issues

| # | Issue | Status |
|---|-------|--------|
| 1 | **Motors 0, 2, 3 not responding** | **Root cause found** (burst DMA CCR timing offset). **Fix in progress** — DShot.cpp TX DMA refactor to 3 separate streams. **Code partially applied, not yet compilable.** |
| 2 | Python GCR test script | 5/15 tests still failing after frame-format fix. Encoder needs final debugging. |
| 3 | CAN / IMX5 fusion | Deferred |
| 4 | CRSF radio | Deferred |

---

### Planned Work (next session)

**Priority 1 — Complete DShot.cpp TX DMA refactor**

Files to change: `src/coms/DShot.cpp`

Remaining work (code partially applied from this session):
1. `dshot_init()`: allocate 4 TIM1 DMA streams (`s_dma_tx_m0`, `s_dma_tx_m1`,
   `s_dma_tx_m3`, `s_dma_ic_tim1`) instead of 1. Set each TX stream peripheral
   to its CCR (`&TIM1->CCR2`, `&TIM1->CCR1`, `&TIM1->CCR3`). Remove DCR burst setup.
2. `dshot_write()`: reload 3 separate TX streams (18 words each), disable old
   single-stream code. TC callback uses `s_dma_tx_m1` only (others fire silently).
3. TIM1_UP ISR (`finish_ic_tim1` path): already updated to use `s_dma_ic_tim1`.
4. Verify it compiles and motor 1 still works, then test motors 0, 2, 3.

**Priority 2 — Verify all 4 motors**

Once TX DMA refactor is complete:
```
MT,0,10    ← AUX 3, PE11, Motor 0 (FR)
MT,1,10    ← AUX 4, PE9,  Motor 1 (RL) — currently working
MT,2,10    ← AUX 5, PD13, Motor 2 (FL)
MT,3,10    ← AUX 2, PE13, Motor 3 (RR)
MT,stop
```

**Priority 3 — Fix Python GCR test (5 remaining failures)**

`tools/dshot_decode_test.py` — the corrupted-edge rejection test and CRC
inversion test are still failing. Likely caused by the same frame-format bug
appearing in the inline CRC-test code block at the bottom of the script (which
duplicates the encode logic without the fix).

not super important as long as all 4 motors are working with RPM feedback the eval script is not nessicary 



---

## Testing Roadmap

### ▶ 1. Motor Test (Active — 1 of 4 motors working)

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
