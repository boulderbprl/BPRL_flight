# BPRL Debug Session — Issues & Findings

## Hardware Configuration

- **Flight controller:** CubeOrange+ (STM32H743ZI) on CubePilot standard carrier board
- **IMU/INS:** IMX5 on CAN1 (PH13=TX, PH14=RX → AF9)
- **RC radio:** CRSF receiver on TELEM1 (USART2, PD5=TX, PD6=RX)
- **Motors:** AUX OUT 2, 3, 4, 5 (ESC signal lines)
- **USB debug:** Main USB port on the Cube (PA11=DM, PA12=DP → AF10, OTG_FS)

---

## Root Cause #1: PA11 USB/DShot Conflict — FIXED

**What happened:** The original DShot code mapped Motor 4 to PA11 (TIM1_CH4). PA11 is the
USB OTG_FS D- line. `switch_to_output()` overwrote PA11's AF with TIM1 AF1, killing USB.

**Fix applied:** Motor 4 was moved off PA11 entirely. The entire DShot pin mapping was later
superseded by Root Cause #2 fix.

---

## Root Cause #2: DShot Targeting Wrong Pins — FIXED

**What happened:** The DShot code drove TIM1 on PA8/PA9/PA10/PE14. These pins are **not
connected to any AUX OUT header** on the CubePilot standard carrier. The CubePilot carrier
uses a dual-MCU architecture:

- **FMU (STM32H743)** — runs our firmware, drives AUX OUT via PE9/PE11/PE13/PE14 (TIM1)
  and PD13/PD14 (TIM4).
- **IOMCU (STM32F100)** — separate co-processor that handles MAIN OUT 1-8 and RC input.
  The FMU communicates with the IOMCU over USART6 (PC6/PC7). This is an internal bridge,
  not available for SBUS or any other external device.

**Correct AUX OUT pin mapping (from ArduPilot hwdef):**

| AUX OUT | STM32H743 Pin | Timer Channel | AF |
|---------|--------------|---------------|----|
| AUX 1   | PE14         | TIM1_CH4      | AF1 |
| AUX 2   | PE13         | TIM1_CH3      | AF1 |
| AUX 3   | PE11         | TIM1_CH2      | AF1 |
| AUX 4   | PE9          | TIM1_CH1      | AF1 |
| AUX 5   | PD13         | TIM4_CH2      | AF2 |
| AUX 6   | PD14         | TIM4_CH3      | AF2 |

**Fix applied (`src/coms/DShot.cpp` — complete rewrite):**

| Motor | AUX OUT | Pin  | Timer Channel |
|-------|---------|------|---------------|
| 0 (FR)| AUX 2   | PE13 | TIM1_CH3      |
| 1 (RL)| AUX 3   | PE11 | TIM1_CH2      |
| 2 (FL)| AUX 4   | PE9  | TIM1_CH1      |
| 3 (RR)| AUX 5   | PD13 | TIM4_CH2      |

Motors 0–2 use TIM1 with a 3-channel DMA burst (DBL=2, CCR1/CCR2/CCR3).
Motor 3 uses TIM4 with a 1-channel DMA burst (DBL=0, CCR2 only).
Both timers are started back-to-back to minimise inter-frame skew.

---

## Root Cause #3: Wrong DMAMUX Source ID — FIXED

**What happened:** The original code called `dmaSetRequestSource(stream, 11U)` claiming
request 11 = TIM1_UP. Per `third_party/ChibiOS/os/hal/ports/STM32/STM32H7xx/stm32_dmamux.h`,
request 11 is actually **TIM1_CH1** (a capture-compare event), not the update event.
With `TIM_DIER_UDE` set on TIM1, the timer fires TIM1_UP (request 15) but the DMAMUX
was wired to TIM1_CH1 (request 11) — a mismatch that caused DMA to never trigger.

**Fix applied:**
- `DMAMUX_TIM1_UP = 15` (STM32_DMAMUX1_TIM1_UP)
- `DMAMUX_TIM4_UP = 32` (STM32_DMAMUX1_TIM4_UP)

---

## Root Cause #4: SBUS Wired to FMU↔IOMCU Bridge — FIXED

**What happened:** The firmware configured PC7/USART6 as SBUS RX. On the CubePilot carrier,
PC6/PC7 is the **internal USART6 bridge between the FMU and the IOMCU co-processor**. The
RCIN connector on the carrier routes RC input to the IOMCU (not the FMU). Attempting to
receive raw SBUS on PC7 from an external receiver would conflict with the IOMCU bridge.

**Fix applied:**
- USART6 (PC6/PC7) configuration **removed** from both `boards/CubeOrangePlus/board.c`
  and `boards/CubeBlueH7/board.c`.
- RC radio switched to **CRSF on TELEM1 (USART2, PD5=TX, PD6=RX, AF7)**.
- `src/coms/CRSF.cpp` updated from `SD3` → `SD2`.
- `src/coms/Radio.hpp` default changed to `RADIO_PROTO_CRSF`.
- TELEM1 comment in both `board.c` files corrected (was mislabelled as USART3).

---

## Other Fixes Made

### `src/threads.cpp` — DebugThread MemoryStream
`chprintf` directly on SDU1 uses `TIME_INFINITE` on the output queue. If the queue fills,
DebugThread blocks forever. Switched to: format `$TEL` line into a local buffer via
`MemoryStream` (non-blocking), then send with `chnWriteTimeout(..., 50 ms)`.

### `Makefile` — `CHPRINTF_USE_FLOAT=1`
Without this flag, `%.2f` format specifiers in `$TEL` emit nothing, producing empty fields
that crash the Python float parser in `tools/bprl.py`.

### `tools/bprl.py` — Diagnostic improvements
- `lines_rx` counter in telemetry panel: shows total lines received.
- Three-state USB status: "no USB data", "USB ok but no $TEL", "stale contact".
- `TelState.last_rx` initialised to `time.monotonic()` (was 0.0 → always showed stale).

### `src/threads.cpp` — `s_usb_write_mtx`
`chprintf` is not atomic. `DebugThread` and `USBCmdThread` were interleaving bytes on SDU1,
producing corrupt lines. Mutex serialises all USB CDC writes.

### `main.cpp` — Heartbeat LED rate
HeartbeatThread period changed from `TIME_MS2I(200)` → `TIME_MS2I(250)` (2 Hz visible blink,
toggle every 250 ms). Lets you visually confirm the firmware is running after a flash.

---

## Current Port Assignment (Complete Picture)

| Peripheral        | Port / Protocol | STM32 Pins      | ChibiOS Driver |
|-------------------|-----------------|-----------------|----------------|
| USB debug/cmd     | OTG_FS          | PA11=DM, PA12=DP| SDU1           |
| IMX5 INS          | CAN1 / FDCAN1   | PH13=TX, PH14=RX| CAND1          |
| CRSF radio        | TELEM1 / USART2 | PD5=TX, PD6=RX  | SD2            |
| Future sensor     | TELEM2 / USART3 | PD8=TX, PD9=RX  | SD3            |
| Motor 0 (AUX 2)   | TIM1_CH3        | PE13            | DShot.cpp      |
| Motor 1 (AUX 3)   | TIM1_CH2        | PE11            | DShot.cpp      |
| Motor 2 (AUX 4)   | TIM1_CH1        | PE9             | DShot.cpp      |
| Motor 3 (AUX 5)   | TIM4_CH2        | PD13            | DShot.cpp      |
| SD card logging   | SDMMC1          | PC8-PC12, PD2   | FatFS          |
| IMU SPI bus 1     | SPI1            | PA5/PA6/PA7     | SPIDriver      |
| IMU SPI bus 2     | SPI4            | PE2/PE5/PE6     | SPIDriver      |
| Sensor power rail | GPIO output     | PE3             | boardInit      |

**No pin conflicts exist** between any of these peripherals.

---

## Immediate Next Steps

1. **Flash current build** and verify heartbeat LED blinks at 2 Hz (firmware alive check):
   ```
   make flash BOARD=CubeOrangePlus UDEFS_EXTRA=-DBPRL_DEBUG
   ```

2. **Verify USB telemetry** with Python tool:
   ```
   python3 tools/bprl.py telemetry
   # lines_rx should increment; roll/pitch/yaw fields should show floats
   ```

3. **Connect CRSF receiver** to TELEM1 connector (TX→receiver RX, RX→receiver TX).
   Verify `armed` and channel values appear in the telemetry stream.

4. **Test motor outputs** — with props off, use motor test command:
   ```
   python3 tools/bprl.py motor_test 0 10   # Motor 0 at 10%
   ```
   ESC should play the "connected to FC" chime on first DShot packet, then spin briefly.

5. **Add future TELEM2 sensor** — use `sdStart(&SD3, ...)` in its driver init.
   USART3 (PD8=TX, PD9=RX) is already configured in `board.c`.
