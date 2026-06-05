# BPRL Flight Controller — Handoff Document
**Date:** 2026-06-05  
**Author/Recipient:** Ian Mcconachie (iamc9683@colorado.edu)  
**Branch:** `destructive_testing`  
**Current hardware target:** CubeOrange+ (STM32H743ZI)

---

## 1. Project Overview

BPRL_flight is a standalone ChibiOS/STM32H7 quadcopter flight controller living at `/data/Documents/BPRL_flight`. It is **not** inside the ArduPilot tree — ArduPilot at `/home/bprl-dev/Documents/ardupilot` is reference only. The Makefile builds directly against ChibiOS.

The goal is a complete flight controller that matches ArduPilot sensor/driver behavior but runs a custom control stack.

---

## 2. Hardware Configuration

| Component | Detail |
|-----------|--------|
| MCU | STM32H743ZI (CubeOrange+), 200 MHz |
| AUX connector | Level-shifted via 5 V rail (PA8 = nVDD_5V_PERIPH_EN, PB4 = PWM_VOLT_SEL) |
| Motors | 4× AUX 2–5, DShot600 bidirectional |

### Motor Pin Mapping

| Motor | Role | AUX | STM32 Pin | Timer | Channel |
|-------|------|-----|-----------|-------|---------|
| 0 | Front-Right | 3 | PE11 | TIM1 | CH2 |
| 1 | Rear-Left | 4 | PE9 | TIM1 | CH1 |
| 2 | Front-Left | 5 | PD13 | TIM4 | CH2 |
| 3 | Rear-Right | 2 | PE13 | TIM1 | CH3 |

---

## 3. Build & Flash Commands

```bash
# Standard build
make BOARD=CubeOrangePlus

# Debug build (enables BPRL_DEBUG — DebugThread + $DSHOT auto-print)
make BOARD=CubeOrangePlus UDEFS_EXTRA=-DBPRL_DEBUG

# Flash via BL5 USB bootloader
make flash BOARD=CubeOrangePlus PORT=/dev/ttyACM0

# Clean + build + flash in one shot
make clean && make BOARD=CubeOrangePlus UDEFS_EXTRA=-DBPRL_DEBUG && make flash BOARD=CubeOrangePlus

# Live telemetry dashboard
python3 tools/bprl.py telemetry
```

---

## 4. Active Threads (motor-test configuration)

All threads except the four below are **disabled** to eliminate ISR contention with DShot edge capture.

| Thread | Priority | Rate | Purpose |
|--------|----------|------|---------|
| ControlThread | NORMALPRIO+20 | 400 Hz | Sends DShot throttle; motor-test bypass via `MT,` USB commands |
| HeartbeatThread | NORMALPRIO−5 | 5 Hz / 2 s print | LED blink + `$DSHOT` auto-diagnostic |
| USBCmdThread | NORMALPRIO−20 | event-driven | USB CLI (`MT,`, `DSHOT,diag`, `BOOT`, `CAL,*`, etc.) |
| DebugThread | NORMALPRIO−10 | 10 Hz | `$TEL` + `$EKFL` telemetry (BPRL_DEBUG only) |

Disabled: SPIThread, CANThread, StateEstThread, I2CThread, RadioThread, LogThread.

---

## 5. DShot600 Bidirectional Implementation

### Architecture Summary

**TX Phase** — single burst DMA stream → `TIM1->DMAR`

```
DCR: DBA=CCR1(13), DBL=3  →  burst of 4: CCR1(M1), CCR2(M0), CCR3(M3), CCR4(dummy)
Buffer: s_tx_buf_tim1[72]  — interleaved stride-4 rows
        [CCR1_b15, CCR2_b15, CCR3_b15, 0,   ← bit 15 row
         CCR1_b14, CCR2_b14, CCR3_b14, 0,
         ...
         0, 0, 0, 0,  ← trailing row 1
         0, 0, 0, 0]  ← trailing row 2

One TIM1_UP event → FIFO drains 4 words → CCR1/CCR2/CCR3 updated simultaneously (~5 ns apart)
72 words / 4 per UEV = 18 UEVs = 16 data bits + 2 trailing idle
```

TIM4 (M2/PD13): separate direct-mode DMA, `s_tx_buf_tim4[18]`, 18 words.

**TX → IC Transition** — DMA TC callback (priority 7):
1. `dshot_dma_tc_tx_tim1` fires, stops TIM1, sets phase=IC, calls `start_ic_tim1()`
2. IC rotates through 3 slots per cycle (M0 → M1 → M3), each at ~133 Hz
3. TIM4 TC callback handles M2/PD13 every frame at 400 Hz

**IC Phase** — DMA-based input capture (no ISR polling):
```
Slot 0 (M0/PE11): CC2S=TI2 (direct), DMAMUX=TIM1_CH2, DBA=CCR2
Slot 1 (M1/PE9):  CC2S=TI1 (cross-capture), DMAMUX=TIM1_CH2, DBA=CCR2
Slot 2 (M3/PE13): CC3S=TI3 (direct), DMAMUX=TIM1_CH3, DBA=CCR3
```

IC timeout: hardware UIE at ARR=20000 (100 µs). Partial captures decoded on timeout.

**GCR Decode** — transition-marking algorithm (matches ArduPilot/BetaFlight):
```cpp
bits = (bits << n) | (1U << (n - 1U));   // n = run length in GCR bit widths
```
Decode 21 bits → four 5-bit GCR codes → four nibbles → 16-bit frame. CRC: all four nibbles XOR must equal 0xF. eRPM: `60,000,000 / (mantissa << exponent)`. Mechanical RPM = eRPM / 7 (14-pole motor).

### Key DMA Streams

| Stream | Timer | Phase | DMAMUX | Direction | Target |
|--------|-------|-------|--------|-----------|--------|
| `s_dma_tx_tim1` | TIM1 | TX | TIM1_UP (15) | M→P | `TIM1->DMAR` (burst 4) |
| `s_dma_ic_tim1` | TIM1 | IC | TIM1_CH2/CH3 | P→M | `TIM1->DMAR` (single) |
| `s_dma_tim4` | TIM4 | TX+IC | TIM4_UP/CH2 | M↔P | `TIM4->DMAR` |

### Frame Format

```
make_frame(thr):
  val = thr << 1          // telem bit = 0 (always; telem=1 reserved for special commands)
  crc = (~(val ^ val>>4 ^ val>>8)) & 0xF   // inverted nibble CRC for bidirectional
  return (val << 4) | crc
```

Idle frame: `0x000F` (throttle=0, telem=0, CRC=0xF) — this is the "arm" signal.

---

## 6. Diagnostic Interface

### Auto-print (every 2 s, no command needed)
```
$DSHOT,<ms>,tc=<tim1_tc>/<tim4_tc>,cc=<tim1_cc>/<tim4_cc>,edges=<m0>/<m1>/<m2>/<m3>
```

| Field | Healthy value | Fault |
|-------|--------------|-------|
| `tc` | incrementing ~400/s | = 0: DShot not transmitting (`dshot_init()` failed) |
| `cc` | incrementing | = 0: IC never completing |
| `edges` | > 0 when spinning | = 0: ESC not responding (not armed/spinning, wiring fault) |

### USB Commands

| Command | Effect |
|---------|--------|
| `MT,<m>,<pct>` | Spin motor m (0–3) at pct% (0=stop) |
| `MT,stop` | Stop all motor tests |
| `DSHOT,diag` | Full per-motor edge timestamps + DMA counters |
| `BOOT` | Reboot into BL5 bootloader for USB flash |
| `CAL,*` | IMU calibration commands |

### LED Error Codes (on `dshot_init()` DMA alloc failure)
- 20 rapid flashes loop = TIM1 DMA alloc failed
- 30 rapid flashes loop = TIM4 DMA alloc failed

---

## 7. History of All Bugs Fixed

| # | File | Bug | Fix |
|---|------|-----|-----|
| 1 | Linker script / Makefile | Wrong flash base address (BL5 at 0x08000000; app must start at 0x08020000) | Set flash origin to 0x08020000 |
| 2 | `src/usb_serial.cpp` | BL5 leaves USB D+ asserted → `otg_core_reset` spin-loop | `usbDisconnectBus()` → 5 ms → `usbStart()` → `usbConnectBus()` |
| 3 | `src/coms/DShot.cpp` | Wrong GPIO pins (PA8/PA9/PA10/PE14, not on AUX) | Changed to PE9/PE11/PE13/PD13 |
| 4 | `src/coms/DShot.cpp` | Wrong DMAMUX IDs (TIM1_UP was 11, should be 15; TIM4_UP was wrong) | `DMAMUX_TIM1_UP=15`, `DMAMUX_TIM4_UP=32` |
| 5 | `boards/CubeOrangePlus/board.c` | AUX level-shifter not powered — PA8/PB4 not initialized | Added PA8=OUTPUT_LOW (5V rail), PB4=OUTPUT_HIGH (3.3V sel) |
| 6 | `src/coms/DShot.cpp` `make_frame()` | Telem bit = 1 unconditionally → ESC received MOTOR STOP command continuously | Changed to `val = thr << 1` (telem=0) |
| 7 | `src/coms/DShot.cpp` | ISR priority = 15 (minimum) → USB/SPI ISRs preempted edge capture mid-GCR | Raised to priority 7 (`STM32_IRQ_TIM1_CC_PRIORITY`) |
| 8 | `src/coms/DShot.cpp` | `EGR=UG` in `switch_to_output()` queued spurious TIM1_UP ISR (UIE still set from prior IC) | Added `TIM1->DIER=0; TIM1->SR=0` at start of every `dshot_write()` |
| 9 | `src/threads.cpp` | SPIThread 1 kHz SPI DMA ISRs (priority ~12) preempted TIM1_CC edge capture (priority 15) mid-GCR | Disabled SPIThread (and CAN/StateEst/I2C/Radio/Log threads) |
| 10 | `src/coms/DShot.cpp` | ISR-based edge capture missed edges at 750 kHz GCR — ISR latency too high | Rewrote to DMA-based capture (CCxDE + DMAR), CPU not involved |
| 11 | `src/coms/DShot.cpp` | GCR decoder used level-filling instead of transition-marking → wrong 21-bit patterns for all real ESC packets | Rewrote: `bits = (bits << n) \| (1U << (n-1))` |
| 12 | `src/coms/DShot.cpp` | CRC check used only 12-bit value (never passed for valid frames) | Correct: `(frame ^ frame>>4 ^ frame>>8 ^ frame>>12) & 0xF == 0xF` |
| 13 | `src/coms/DShot.cpp` | eRPM stored raw period instead of RPM | Added `erpm = 60,000,000 / period` |
| 14 | `src/threads.cpp` DebugThread | RPM divided by 14 (poles) instead of 7 (pole pairs) | Changed `/14` → `/7` |
| 15 | `src/coms/DShot.cpp` | Concurrent IC/DCR conflict: `start_ic_m3()` changed `TIM1->DCR` mid-TX burst, corrupting CCR writes | Removed concurrent approach; clean sequential TX→IC rotation |
| 16 | `src/coms/DShot.cpp` | 3-stream DMAMUX fan-out (change #31) — DMAMUX round-robins TIM1_UP between streams, not broadcast → each stream got every 3rd UEV → 5 µs/bit = DShot200 | Reverted to single-stream DMAR burst matching ArduPilot (#32) |
| 17 | `cfg/chconf.h` | Debug asserts disabled → failed `dmaStreamAlloc()` returned null silently → hard fault → IWDG loop | Enabled `CH_DBG_ENABLE_ASSERTS=TRUE` for BPRL_DEBUG |

---

## 8. Current State (as of 2026-06-05)

### What is working
- Motor 1 (AUX4, PE9): connects, spins, returns RPM telemetry — **verified on hardware**
- USB CDC debug interface: enumerate, commands, diagnostics
- `$DSHOT` auto-diagnostic output
- `$TEL` telemetry output (BPRL_DEBUG build)

### What needs hardware verification (code complete, not yet tested)
- **Motors 0, 2, 3** — change #32 (burst DMAR with interleaved stride-4 buffer) is the fix for all three. This is the top priority on the next bench session.

### Test sequence for all 4 motors
```bash
# Flash current build
make BOARD=CubeOrangePlus UDEFS_EXTRA=-BPRL_DEBUG && make flash BOARD=CubeOrangePlus

# Connect bprl.py
python3 tools/bprl.py telemetry

# In another terminal, send USB commands:
MT,0,10    # Motor 0 (AUX3, PE11, Front-Right) at 10%
MT,1,10    # Motor 1 (AUX4, PE9,  Rear-Left)   at 10%  ← was working
MT,2,10    # Motor 2 (AUX5, PD13, Front-Left)  at 10%
MT,3,10    # Motor 3 (AUX2, PE13, Rear-Right)  at 10%
MT,stop
```

Pass criteria:
1. ESC power-up chime on all 4 at boot
2. `$DSHOT` shows `tc` incrementing and `edges` non-zero for all 4 motors when spinning
3. RPM values in `$TEL` increase with throttle %

---

## 9. Known Open Issues

| # | Issue | Priority | Notes |
|---|-------|----------|-------|
| 1 | Motors 0, 2, 3 — hardware verification | **P0** | Code fixed (change #32). Bench test needed. |
| 2 | Python GCR test: 5/15 failing | Low | `tools/dshot_decode_test.py` — corrupted-edge and CRC-inversion test cases still wrong. Not blocking flight. |
| 3 | CAN / IMX5 fusion | Deferred | `can_drv_init()` commented out in `main.cpp`; CANThread disabled |
| 4 | CRSF radio input | Deferred | RadioThread disabled; USART2/TELEM1 configured but not running |
| 5 | EDT telemetry (temp/voltage/current) | Add before sustained flight | ESC sends EDT packets; BPRL silently returns false (safe, but wastes GCR slots) |
| 6 | SPIThread re-enable | After all 4 motors work | May need ISR priority audit once DShot at priority 7 |

---

## 10. Re-enabling the Full Flight Stack (after motors verified)

Order of operations:
1. **Re-enable SPIThread** — verify no `edges=0` regression in `$DSHOT` output (SPI DMA ISR should not preempt DShot at priority 7)
2. **Re-enable StateEstThread** and RadioThread
3. **Restore ControlThread** to full PID/mixer path (AttitudeController, MotorMixer)
4. **Re-enable CAN** (`can_drv_init()` + CANThread) — verify IMX5 quaternion at ~200 Hz in `$TEL`
5. **Test complete attitude loop**

---

## 11. Key Source Files

| File | Purpose |
|------|---------|
| [src/coms/DShot.cpp](src/coms/DShot.cpp) | DShot600 bidirectional — the core of current work |
| [src/coms/DShot.hpp](src/coms/DShot.hpp) | Public API: `dshot_init()`, `dshot_write()`, `dshot_get_telemetry()`, `dshot_get_diag()` |
| [src/threads.cpp](src/threads.cpp) | All thread definitions; ControlThread sends DShot at 400 Hz |
| [main.cpp](main.cpp) | Startup sequence; `motor_output_init()` → `dshot_init()`; IWDG kick loop |
| [boards/CubeOrangePlus/board.c](boards/CubeOrangePlus/board.c) | `boardInit()`: PA8/PB4 critical pin init for AUX power |
| [status.md](status.md) | Living status doc — architecture, diagnostic reference, change history |
| [ardupilot_comparison.md](ardupilot_comparison.md) | Side-by-side BPRL vs ArduPilot implementation diff |
| [tools/bprl.py](tools/bprl.py) | USB debug tool — telemetry dashboard, motor commands, log download |

---

## 12. ArduPilot Comparison Summary

BPRL's TX DMA now matches ArduPilot's `send_pulses_DMAR` exactly:
- Single burst stream → `TIM->DMAR`, DCR: DBA=CCR1, DBL=3, interleaved stride-4 buffer
- All CCRs updated on the same UEV via FIFO full-threshold burst

Notable intentional differences from ArduPilot:
- IC timeout: hardware UIE (ARR=20000) instead of ChibiOS virtual timer — simpler, still correct
- IC timer: no PSC (PSC=0, 267 ticks/GCR bit); ArduPilot uses PSC=15 (16 ticks/GCR bit)
- Cache: `.nocache` MPU section instead of explicit flush/invalidate per transaction
- GCR decode: in-place in ISR context instead of copy + decode in thread
- eRPM: stored as full `uint32_t` instead of eRPM/100 in `uint16_t`
- No EDT (temperature/voltage/current telemetry) — safe but won't surface ESC warnings

Full detail: [ardupilot_comparison.md](ardupilot_comparison.md)



I need you to debug a bi-directional DShot600 motor control issue in my codebase by comparing it to ArduPilot's implementation. 

### Objective
Identify the bugs in my PWM/DShot frame generation logic that prevent Motors 2, 3, and 4 from initializing and receiving RPM feedback, and fix the frame-interval timing.

### Files to Analyze
* `status.md`
* `ardupilot_comparison.md`
* The PWM and DShot source code files in this repository.

### Context & Working Baseline
* **Hardware:** 4 motors, identical wiring/ESC firmware, CubeOrangePlus.
* **ArduPilot Baseline (Working):** All 4 motors connect, spin, and return RPM feedback. 
  * *Parameters:* `SERVO_BLH_BDMASK = 7680`, `SERVO_BLH_MASK = 7680`, `MOT_PWM_TYPE = 6`
* **Current State (My Code):** Only Motor 1 (AUX4) successfully connects, spins, and returns RPM feedback. Motors 0,2,3 fail.

### Logic Analyzer Findings & Debugging Tasks

1. **Investigate the Faulty Pre-Frame Low Signal (Priority 1)**
   * *Symptom:* A 2µs low signal appears 1.7µs *before* the actual frame starts. 
   * *Pattern:* It occurs every frame on AUX5, every 2nd frame on AUX2, and every 3rd frame on AUX3. It is absent on the working AUX4. ArduPilot does not do this.
   * *Task:* Trace the frame setup and DMA/timer interrupts. This cyclic pattern suggests a faulty buffer address calculation, stride error, or memory corruption during frame initialization.

2. **Fix Signal Glitches on AUX5**
   * *Symptom:* AUX5 on the rising edge spikes high for 6ns, drops for 49ns, and goes high again when other channels trigger. This disrupts timing, cutting the frame period to 1.61µs (vs. the standard 1.67µs).
   * *Task:* Look for pin multiplexing conflicts, timer channel synchronization issues, or race conditions specific to the AUX5 GPIO/timer configuration.

3. **Optimize the Inter-Frame Interval**
   * *Symptom:* My code spaces frames ~2.47ms apart. ArduPilot sends frames much faster (repeating pattern of 975µs, 975µs, ~450µs).
   * *Task:* Trace the scheduler or timer update loop trigger. Identify why my code is bottlenecked at 2.5ms and how to match ArduPilot’s throughput.

4. **Review Frame Phase Offsets**
   * *Observation:* ArduPilot offsets the AUX5 frame by ~9µs behind other channels. My code does not.
   * *Task:* Determine if this offset is required for DMA/timer stability on this hardware and if its absence in my code causes the AUX5 glitches.

Please trace the path of DShot frame generation in my code vs. ArduPilot, isolate the root causes for these 4 anomalies, and propose specific code fixes.
