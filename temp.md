# BPRL Debug Session — Issues & Findings

## Root Cause #1: PA11 USB/DShot Conflict (FIXED)

**What happened:** `dshot_init()` → `switch_to_output()` directly writes to GPIOA MODER/AFRH
registers for pins 8–11 (TIM1_CH1–4). PA11 is also the USB OTG_FS D- line (set to AF10 in
`board.c`). The DShot init overwrote it with TIM1 AF1 the moment `motor_output_init()` ran,
killing USB hardware. The port node `/dev/ttyACM0` stayed alive on the host but no data flowed.

**Symptoms:**
- `python3 tools/bprl.py telemetry` → "no USB data — check connection", `lines_rx=0`
- Motor test commands sent but no response
- `logs list` timeout
- ESCs power-up chime plays (hardware works), but NO ESC-connected-to-FC chime (no DShot reaching ESCs)

**Fix applied (`src/coms/DShot.cpp`):**
- Motor 4 moved from PA11 → PE14 (the other TIM1_CH4 mapping on STM32H7)
- GPIOE clock enable added (`RCC_AHB4ENR_GPIOEEN`)
- PA11 is no longer touched by DShot code

**Status:** Built, not yet flashed/verified.

---

## Root Cause #2: Wrong DShot Timer for Standard CubePilot Carrier (UNRESOLVED)

**Finding:** The current DShot code uses TIM1 (PA8–PA10, PE14). The standard CubePilot carrier
board routes its AUX OUT pins to **TIM8** and **TIM4** — not TIM1.

### Standard carrier AUX OUT pin mapping (STM32H7):

| AUX OUT | STM32H7 Pin | Timer Channel | Status in firmware |
|---------|-------------|---------------|--------------------|
| AUX 1   | PC6         | TIM8_CH1 (AF3) | Free — but known DShot issue per user |
| AUX 2   | PC7         | TIM8_CH2 (AF3) | **CONFLICT: USART6 RX (SBUS input)** |
| AUX 3   | PC8         | TIM8_CH3 (AF3) | **CONFLICT: SDMMC1 D0** |
| AUX 4   | PC9         | TIM8_CH4 (AF3) | **CONFLICT: SDMMC1 D1** |
| AUX 5   | PD14        | TIM4_CH3 (AF2) | **CLEAN — no conflict** |
| AUX 6   | PD15        | TIM4_CH4 (AF2) | **CLEAN — no conflict** |

The current DShot code (TIM1 on PA8–PA10/PE14) outputs to pins that are **not connected to
any AUX OUT on the standard carrier**. This explains why no ESC connection chime plays.

### Conflicts in detail:

**AUX 2 / PC7:** `board.c` configures PC7 as USART6 RX (SBUS RC input, AF8).
**AUX 3-4 / PC8-PC9:** `board.c` configures PC8–PC11 as SDMMC1 D0–D3 (AF12). The SD card
slot on the standard carrier is physically wired to SDMMC1 (PC8–PC12). These pins cannot be
moved in software — the hardware trace is fixed.

---

## Potential Solutions

### Option A: Switch radio + accept no SD logging (cleanest short-term)
- Switch from SBUS → CRSF (USART3/TELEM1, already wired PD8/PD9). This frees PC7 for AUX 2.
- Disable SDMMC1 in firmware temporarily. This frees PC8–PC9 for AUX 3–4.
- Result: Motors on AUX 2–5 using TIM8_CH2–4 + TIM4_CH3, no SD logging.
- Radio change: `RADIO_PROTOCOL=CRSF` in `src/coms/Radio.hpp` (already supported).

### Option B: Use AUX 5–6 for 2 motors + find 2 more clean channels
- AUX 5 (PD14/TIM4_CH3) and AUX 6 (PD15/TIM4_CH4) are both clean.
- Only 2 clean AUX channels available → can't drive 4 motors this way alone.
- Would need to wire remaining 2 motors to custom pads/pins not on the AUX rail.

### Option C: Remap DShot to TIM8+TIM4 (full fix, requires carrier pin understanding)
Full port to TIM8 (PC6–PC9) + TIM4 (PD14–PD15) for all 6 AUX outputs.
Requires resolving SBUS and SDMMC conflicts first. DShot code would need significant
refactor since TIM8 and TIM4 are separate timers (no 4-channel single DMA burst).

### Option D: Alternative carrier board
A custom or different carrier that routes motor outputs to pins not shared with SD/SBUS.

---

## Other Fixes Made This Session

### `src/threads.cpp` — DebugThread MemoryStream (fixes USB data flow)
`chprintf` directly on SDU1 uses `TIME_INFINITE` on the output queue. If the queue fills,
DebugThread blocks forever. Switched to: format `$TEL` line into a local buffer via
`MemoryStream` (non-blocking), then send with `chnWriteTimeout(..., 50 ms)`. No more blocking.

### `Makefile` — `CHPRINTF_USE_FLOAT=1`
`chprintf` defaults to no floating-point support. Without this flag, `%.2f` etc. in `$TEL`
emit nothing, producing empty fields that crash the Python float parser.

### `tools/bprl.py` — Diagnostic improvements
- `lines_rx` counter in telemetry panel: shows total lines received, immediate USB health check.
- Three-state status: "no USB data", "USB ok but no $TEL", "stale contact".
- `TelState.last_rx` initialised to `time.monotonic()` (was 0.0 → always showed stale).

### `src/threads.cpp` — `s_usb_write_mtx`
`chprintf` is not atomic. `DebugThread` and `USBCmdThread` were interleaving bytes on SDU1,
producing corrupt lines. Mutex serialises all USB CDC writes.

---

## Immediate Next Steps

1. **Flash current build** and verify USB is restored (PA11 fix):
   ```
   make flash BOARD=CubeOrangePlus UDEFS_EXTRA=-DBPRL_DEBUG
   python3 tools/bprl.py telemetry   # lines_rx should increment
   ```

2. **Decide on radio protocol** (SBUS vs CRSF) — this determines whether AUX 2 (PC7) is
   available for motors.

3. **Decide on SD logging** during motor testing — if SD card is removed, PC8–PC9 become
   free for AUX 3–4.

4. **Rewrite DShot for TIM8+TIM4** targeting AUX 2–5 actual pins once pin conflicts resolved.



## Other things to check.

Im using the cube pilot standard carrier board for both the Cube Blue and Cube Orange+. 
the IMX5 is connected to CAN1
the SBUS radio is connected to RCIN.
the motors are connected to AUX OUT 2,3,4,5
Im connecting the programming/DEBUG usb over the main USB port on teh cube not on the carrier board.

check to make sure the code is set up to use these ports on the cube standard carrier board. 


