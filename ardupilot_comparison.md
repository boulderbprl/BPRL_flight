# BPRL vs ArduPilot — Bidirectional DShot Pipeline Comparison

ArduPilot reference: `libraries/AP_HAL_ChibiOS/RCOutput_bdshot.cpp` and `RCOutput.cpp`.
BPRL implementation: `src/coms/DShot.cpp`.

Both target STM32H743 (CubeOrange+) at 200 MHz, DShot600 bidirectional, ChibiOS RTOS.

---

## 1. TX DMA Architecture

### ArduPilot (`send_pulses_DMAR`)

One DMA stream per timer group writes to `TIM->DMAR` using DCR burst mode:

```
DCR: DBA = offsetof(CCR)/4,  DBL = 3  (burst of 4 registers: CCR1–CCR4)
Buffer: stride-4 interleaved — [ch0b0, ch1b0, ch2b0, ch3b0, ch0b1, ch1b1, ...]
Total:  19 rows × 4 channels = 76 words per timer group
```

Each TIM_UP event fires one DMA request, which drains 4 words from the FIFO to DMAR.
The timer routes these DMAR writes to CCR1→CCR2→CCR3→CCR4, all within ~5–10 ns.
DMA uses FIFO mode with full-threshold (4-word) buffering to ensure the burst is atomic.

### BPRL (`dshot_write`)

Identical architecture to ArduPilot for TIM1 (3 motors) and TIM4 (1 motor):

```
TIM1 — s_dma_tx_tim1 → &TIM1->DMAR, DCR: DBA=CCR1(13), DBL=3
  Buffer: s_tx_buf_tim1[76], interleaved stride-4 [CCR1=M1, CCR2=M0, CCR3=M3, CCR4=0]
  19 rows × 4 words = 76 words  (CCR4 column is always 0 — dummy padding)
  FIFO mode: DMA_SxFCR_DMDIS | DMA_SxFCR_FTH, MBURST=INCR4, PBURST=INCR4

TIM4 — s_dma_tim4 → &TIM4->DMAR, DCR: DBA=CCR2(14), DBL=0
  Buffer: s_tx_buf_tim4[19], single-channel linear
  19 words  (CCR2 only — one motor)
  Direct mode (FCR=0, no burst)
```

The only structural difference: BPRL uses DBL=3 with 3 active channels (CCR1/M1, CCR2/M0,
CCR3/M3) and pads CCR4 with zeros, while ArduPilot targets 4-channel groups and all four
CCRs carry data. The DMA mechanism and timing are otherwise identical.

TIM4 uses direct mode because it drives a single channel — no atomic burst needed.
ArduPilot's FIFO-mode note applies only to the multi-channel burst case.

---

## 2. TX Buffer Format and Preamble

| | ArduPilot | BPRL |
|-|-----------|------|
| Words before data | 1 (`dshot_pre=1`, CCR=0) | 1 (row 0 = zeros) |
| Data bits | 16 (MSB first) | 16 (MSB first) |
| Words after data | 2 (`dshot_post=2`, CCR=0) | 2 (rows 17–18 = zeros) |
| **Total per channel** | **19** | **19** |
| Layout | Interleaved stride-4 | Interleaved stride-4 (TIM1) / linear (TIM4) |

BPRL and ArduPilot have identical buffer sizes and identical leading/trailing zero structure.

**Why `dshot_pre=1` is necessary:**
BPRL sets OC preload enable (OC1PE, OC2PE, OC3PE) in CCMR. With preload active, each
DMA write to DMAR goes to the CCR shadow register and takes effect only on the *next* UEV.
The pre-word row absorbs this one-UEV delay: the DMA write for bit 15 happens on UEV2 but
activates on UEV3 — the correct time for the first DShot pulse.

ArduPilot uses `dshot_pre=1` for the same reason, plus it also handles the case where DMA
is armed mid-period on a running timer (not applicable to BPRL which restarts the timer).

ArduPilot `fill_DMA_buffer_dshot()`:
```cpp
for (i = 0; i < dshot_pre; i++)              buffer[i * stride] = 0;        // pre
for (i = dshot_pre; i < 16 + dshot_pre; i++) buffer[i * stride] = bitval;   // data
for (; i < dshot_bit_length; i++)             buffer[i * stride] = 0;        // post
```

BPRL `build_tx_buf()`:
```cpp
s_tx_buf_tim1[0..3] = 0;                       // row 0: pre-word (dshot_pre=1)
for (b = 15; b >= 0; b--) {
    int row = (16 - b) * 4;                    // rows 1–16: data bits 15..0
    s_tx_buf_tim1[row+0] = (frame[1]>>b)&1 ? DS_T1H : DS_T0H;  // M1 / CCR1
    s_tx_buf_tim1[row+1] = (frame[0]>>b)&1 ? DS_T1H : DS_T0H;  // M0 / CCR2
    s_tx_buf_tim1[row+2] = (frame[3]>>b)&1 ? DS_T1H : DS_T0H;  // M3 / CCR3
    s_tx_buf_tim1[row+3] = 0;                                    // CCR4 dummy
}
for (i = 68; i < 76; i++) s_tx_buf_tim1[i] = 0;  // rows 17–18: trailing zeros
```

---

## 3. Frame Encoding

### ArduPilot (`create_dshot_packet`)

```cpp
uint16_t packet = (value << 1);
if (telem_request) packet |= 1;          // telem bit set for special packets
uint16_t csum = 0, csum_data = packet;
for (uint8_t i = 0; i < 3; i++) { csum ^= csum_data; csum_data >>= 4; }
if (bidir_telem) csum = ~csum;           // inverted CRC for bidir
csum &= 0xf;
packet = (packet << 4) | csum;
```

### BPRL (`make_frame`)

```cpp
uint16_t val = (uint16_t)(thr << 1);                        // telem bit = 0 always
uint16_t crc = (~(val ^ (val >> 4) ^ (val >> 8))) & 0x0FU; // inverted CRC directly
return (uint16_t)((val << 4) | crc);
```

BPRL always forces `telem=0` (correct for throttle packets; telem=1 is only used for DShot
special command packets, which BPRL does not send). Both produce identical wire frames for
the common case. Wire example: throttle=0, bidir → both emit `0x000F`.

---

## 4. Bit Timing Values

**ArduPilot** uses relative tick counts scaled by `bit_width_mul`:

```
DSHOT_BIT_WIDTH_TICKS_DEFAULT = 8
DSHOT_BIT_0_TICKS_DEFAULT     = 3   (37.5% duty)
DSHOT_BIT_1_TICKS_DEFAULT     = 6   (75.0% duty)
```
For DShot600 at 200 MHz: `bit_width_mul ≈ 42`, giving period≈336, T0H≈126, T1H≈252.

**BPRL** uses absolute timer counts (PSC=0, 200 MHz):

```
DS_ARR  = 333   (period = 334 ticks = 1.67 µs = 600 kHz)
DS_T0H  = 125   (37.4% duty)
DS_T1H  = 250   (74.9% duty)
```

Both produce compliant DShot600 timing. Values differ by ~1% due to rounding of
200 MHz / 600 kHz = 333.33.

---

## 5. GPIO Speed and Pin Configuration

### ArduPilot (`bdshot_setup_group_ic_DMA`)

```cpp
palSetLineMode(group.pal_lines[i],
    PAL_MODE_ALTERNATE(group.alt_functions[i])
    | PAL_STM32_OTYPE_PUSHPULL
    | PAL_STM32_PUPDR_PULLUP
    | PAL_STM32_OSPEED_MID1);   // medium speed (~50 MHz)
```

ArduPilot code comment: *"bi-directional dshot requires less than MID2 speed and PUSHPULL
in order to avoid noise on the line when switching from output to input"*

### BPRL (`dshot_init`)

```cpp
GPIOE->OSPEEDR = ... | (0x1U<<18) | (0x1U<<22) | (0x1U<<26);  // PE9/11/13: medium
GPIOD->OSPEEDR = ... | (0x1U<<26);                              // PD13: medium
// PUPDR = pull-up (0x1) for all pins
// OTYPE = push-pull (default, OTYPER not written = 0 = push-pull)
```

Both use medium speed (OSPEEDR=0x1 ≈ 50 MHz) and pull-up. Medium speed is required for
bidirectional DShot: the slower output slew rate damps ringing during the IC→OC output
reconnect transition. Very high speed (0x3) would cause overshoot/undershoot on that
transition that appears as a spurious LOW bit to the ESC.

---

## 6. TX Timer Startup and IC→OC Transition

### ArduPilot

TX is started by ChibiOS `pwmStart()` (which configures CCMR, CCER, etc.) followed by
`send_pulses_DMAR()` which arms DMA on an already-running timer. The transition from IC
back to OC is handled by `bdshot_reset_pwm()`:

```cpp
void RCOutput::bdshot_reset_pwm(pwm_group& group, uint8_t telem_channel) {
    pwmStop(group.pwm_drv);    // stops timer, clears all CCMR/CCER/DIER
    pwmStart(group.pwm_drv, &group.pwm_cfg);  // full re-init from pwm_cfg
}
```

`pwmStart()` configures OC mode (PWM mode from `pwm_cfg.channels[i].mode`), enables CCER,
and leaves the timer running.

### BPRL (`switch_to_output` + `dshot_write`)

BPRL implements the IC→OC transition manually:

```cpp
// switch_to_output():
TIM1->CCER  = 0U;               // disable outputs before touching CCxS
TIM1->CCMR1 = (7U<<4)|OC1PE | (7U<<12)|OC2PE;  // OC mode 2, preload
TIM1->CCMR2 = (7U<<4)|OC3PE;
TIM1->ARR   = DS_ARR;
TIM1->CCR1  = TIM1->CCR2 = TIM1->CCR3 = 0U;  // idle CCR
TIM1->EGR   = TIM_EGR_UG;      // flush shadows, reset CNT to 0
TIM1->SR    = 0U;               // clear UIF from EGR
TIM1->CCER  = CC1E|CC2E|CC3E;  // re-enable outputs (pin → HIGH, CCR=0 → always active)

// dshot_write():
TIM1->DCR  = (3U<<DBL_Pos) | (DBA_CCR1<<DBA_Pos);  // burst: CCR1–CCR4
TIM1->DIER = TIM_DIER_UDE;
// ... configure DMA ...
TIM1->CR1 |= TIM_CR1_CEN;      // start timer from CNT=0
```

Key behaviour: `EGR=UG` resets CNT to 0 and flushes all CCR shadows to 0. When CEN starts,
the first period (CNT 0→ARR) has CCR=0 (all pins HIGH/idle). UEV1 fires → DMA writes
pre-word row to shadow → still HIGH. UEV2 → DMA writes bit-15 row to shadow → UEV3 → first
DShot LOW pulse. This matches ArduPilot's timing with `dshot_pre=1`.

**TIM4 phase offset** (BPRL only):
TIM4 is started with `TIM4->CNT = DS_ARR/2` (=166) so its UEVs fire halfway between TIM1's,
preventing simultaneous DMA requests from both timers on the AHB bus.

---

## 7. IC Timeout Mechanism

### ArduPilot — ChibiOS virtual timer

```cpp
// in bdshot_receive_pulses_DMAR():
chVTSetI(&group->dma_timeout,
    chTimeUS2I(group->dshot_pulse_send_time_us + 30U + 10U),
    bdshot_finish_dshot_gcr_transaction, group);

TIM->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_UDIS | TIM_CR1_CEN;
//         ^preload       ^overflow only  ^no UEV fired
```

Timeout calibrated to `TX_duration + 30 µs + 10 µs` — just after the GCR response window.
`UDIS=1` suppresses all hardware update events (safe because the timeout is handled by the
OS timer, not by UIE). `URS=1` ensures `EGR=UG` (used to flush PSC shadow during IC setup)
does not generate a spurious UIF.

### BPRL — Hardware UIE

```cpp
// in start_ic_tim1():
TIM1->ARR = 20000U;             // 100 µs at 200 MHz (conservative)
TIM1->CNT = 0U;
TIM1->SR  = 0U;
// DIER has CC2DE/CC3DE | UIE — UIE fires when CNT overflows ARR
dmaStreamEnable(s_dma_ic_tim1);
TIM1->CR1 = TIM_CR1_URS | TIM_CR1_CEN;
//           ^overflow only — UIE fires, EGR=UG would not

// TIM1_UP ISR:
uint8_t captured = (uint8_t)(22U - dmaStreamGetTransactionSize(s_dma_ic_tim1));
finish_ic_tim1(captured);
```

BPRL uses the counter-overflow UIE as a 100 µs hardware timeout. `UDIS` is **not** set
because BPRL needs the overflow to generate UIF (for UIE). `ARPE` is **not** set because
`ARR=20000` takes effect immediately without buffering; adding ARPE without flushing via
`EGR=UG` would leave ARR shadow at DS_ARR=333 from the prior TX phase, causing the IC
timer to wrap every 1.67 µs and break GCR timestamp decoding.

**ArduPilot's ARPE+EGR=UG sequence:**
```cpp
TIM->ARR = 0xFFFF;          // preload  (buffered behind ARPE)
TIM->EGR |= EGR_UG;         // flush preload → shadow (ARR becomes 0xFFFF immediately)
TIM->SR   = 0;              // clear UIF from EGR
TIM->CR1  = ARPE|URS|UDIS|CEN;
```

BPRL avoids needing this sequence by not using ARPE. The 100 µs (20000-tick) timeout is
conservative but always correct at 400 Hz (2.5 ms frame period).

---

## 8. IC Timer Register Setup

### ArduPilot

```cpp
TIM->CR1  = 0;
TIM->SR   = 0;
TIM->CCER = TIM->CCMR1 = TIM->CCMR2 = TIM->DIER = TIM->CR2 = 0;
TIM->PSC  = telempsc;         // 15 → 16 ticks/GCR bit at 200 MHz
TIM->ARR  = 0xFFFF;           // count forever
TIM->CNT  = 0;
bdshot_config_icu_dshot(TIM, curr_ch, telem_tim_ch[curr_ch]);
TIM->DCR  = DBA(ccr_ofs) | DBL(0);
TIM->EGR |= EGR_UG;           // flush PSC shadow (required with ARPE)
TIM->SR   = 0;
TIM->CR1  = ARPE | URS | UDIS | CEN;
dmaStreamEnable(ic_dma);
```

### BPRL

```cpp
// (timer already stopped by TX TC callback; CCMR/CCER cleared in start_ic_tim1)
TIM1->CCER  = TIM1->CCMR1 = TIM1->CCMR2 = 0;
TIM1->DCR   = DBL(0) | DBA(kIcDba[slot]);
// configure CCMR (CCxS, ICxF), CCER (CCxE|P|NP), DIER (CCxDE|UIE)
TIM1->ARR   = 20000U;         // 100 µs timeout (immediate, no ARPE)
TIM1->CNT   = 0U;
TIM1->SR    = 0U;
dmaStreamEnable(s_dma_ic_tim1);
TIM1->CR1   = TIM_CR1_URS | TIM_CR1_CEN;
```

Key differences:
| | ArduPilot | BPRL |
|-|-----------|------|
| PSC | 15 (16 ticks/GCR bit) | 0 (267 ticks/GCR bit) |
| ARR | 0xFFFF (count forever) | 20000 (100 µs hardware timeout) |
| ARPE | Set (flushes PSC shadow) | Not set (ARR is immediate) |
| URS | Set (EGR=UG won't generate UIF) | Set (same, even though EGR not called) |
| UDIS | Set (no UEV — timeout via OS timer) | **Not set** (UIE needed for timeout) |
| EGR=UG | Called (flush PSC shadow) | Not called (no PSC change) |
| Timeout | OS virtual timer | Hardware UIE |

---

## 9. IC Bit Timing and Buffer Values

**ArduPilot** (`TELEM_IC_SAMPLE = 16`, PSC=15):
```
One IC tick = 16 / 200 MHz = 80 ns
GCR bit width = 1/750 kHz = 1333 ns = 16.7 IC ticks ≈ 16
Buffer: uint16_t values ~16 per GCR bit; max ≈ 16×22 = 352 → fits uint16_t
```

**BPRL** (`GCR_BIT_TICKS = 267`, PSC=0):
```
One IC tick = 1 / 200 MHz = 5 ns
GCR bit width = 1333 ns = 266.7 ticks ≈ 267
Buffer: uint32_t values ~267 per GCR bit; max ≈ 267×21 = 5607 → needs uint32_t
```

Both decode using `n = (diff + half_bit) / bit_width`. ArduPilot's PSC saves memory at the
cost of a PSC register write during IC setup; BPRL's full-clock approach avoids that write.

---

## 10. IC Channel Selection — Cross-Capture Trick

Both use the same ArduPilot cross-capture trick for TIM1 CH1 (Motor 1 / PE9):

```
CC2S = TI2 (0b01) → CH2 captures its own pin PE11 → Motor 0
CC2S = TI1 (0b10) → CH2 captures CH1's pin PE9 via cross-input → Motor 1
CC3S = TI3 (0b01) → CH3 captures its own pin PE13 → Motor 3
```

M0 and M1 both use CC2/CCR2 and DMAMUX=TIM1_CH2; only CC2S differs between slots.
BPRL implements this explicitly for slots 0–1:

```cpp
uint32_t cc2s = (slot == 0U) ? kCC2S_TI2 : kCC2S_TI1;
TIM1->CCMR1 = cc2s | kIC2F;
```

ArduPilot implements the same logic via `bdshot_config_icu_dshot()` which `switch(ccr_ch)`
handles all four possible CC channels generically.

---

## 11. GCR Decode Algorithm

Both use **transition-marking** (from BetaFlight):
```
bits = (bits << n) | (1U << (n - 1U))
```
where `n` = inter-edge interval in GCR bit-widths.

**ArduPilot** loop:
```cpp
dmar_uint_t oldValue = buffer[0];
for (uint32_t i = 1; i <= count; i++) {
    len = (i < count) ? round_to_gcr_bits(buffer[i] - oldValue) : (21 - bits);
    value <<= len; value |= 1U << (len-1);
    oldValue = buffer[i];
    bits += len;
}
```

**BPRL** loop:
```cpp
for (uint8_t i = 0; i < n_edges && bits_filled < 21U; i++) {
    n = (i+1 < n_edges) ? round_to_gcr_bits(buf[i+1] - buf[i]) : (21 - bits_filled);
    bits = (bits << n) | (1U << (n-1));
    bits_filled += n;
}
```

Mathematically identical. The first real interval is `buffer[1]-buffer[0]` in both cases.

---

## 12. GCR Decode Table — BPRL Is Stricter

**ArduPilot** — invalid codes silently return 0:
```cpp
static const uint32_t decode[32] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
    0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };
// No validity check — invalid code → nibble=0 → decode continues
```

**BPRL** — invalid codes immediately abort:
```cpp
static constexpr uint8_t kGcrDecode[32] = {
    0xFF, 0xFF, ... 0x09, 0x0A, ... };  // 0xFF = invalid
if (nibble == 0xFF) return false;
```

BPRL rejects any packet containing an invalid 5-bit GCR code. ArduPilot's decode-to-zero
approach could theoretically allow a corrupt packet to pass the CRC check if errors cancel.

---

## 13. CRC Check

Both use the inverted nibble sum: XOR of all four 4-bit nibbles of the 16-bit decoded
frame must equal 0xF.

**ArduPilot:**
```cpp
csum = csum ^ (csum >> 8U); csum = csum ^ (csum >> 4U);
if ((csum & 0xfU) != 0xfU) return INVALID_ERPM;
```

**BPRL:**
```cpp
uint16_t csum = frame ^ (frame>>4) ^ (frame>>8) ^ (frame>>12);
if ((csum & 0xF) != 0xF) return false;
```

Same algorithm, different notation.

---

## 14. eRPM Decoding and Conversion

The ESC response encodes the electrical period: `period_µs = mantissa << exponent`,
then `eRPM = 60,000,000 / period_µs`.

**ArduPilot** stores eRPM/100 in uint16_t:
```cpp
erpm = (1000000U * 60U / 100U + erpm/2U) / erpm;  // = 600000 / period_µs = eRPM/100
update_rpm(chan, erpm * 200U / motor_poles);          // eRPM/100 × 200/poles = RPM
```

**BPRL** stores full eRPM in uint32_t:
```cpp
*erpm_out = (period > 0U) ? (60000000U / period) : 0U;
rpm = erpm / 7U;  // eRPM / pole_pairs (14-pole motor, 7 pole pairs)
```

Equivalent formula; ArduPilot scales down to avoid uint16_t overflow at high RPM.

---

## 15. Cache Coherency

**ArduPilot** — explicit per-transaction cache operations:
```cpp
stm32_cacheBufferFlush(group.dma_buffer, buffer_length);      // before TX DMA
stm32_cacheBufferInvalidate(group.dma_buffer, dma_tx_size);   // after IC DMA
```

**BPRL** — `.nocache` MPU section (no per-transaction operations):
```cpp
static uint32_t s_tx_buf_tim1[76] __attribute__((aligned(32), section(".nocache")));
```

Simpler at the cost of slightly slower CPU access to these small, infrequently-read buffers.

---

## 16. IC Double-Buffering

**ArduPilot** — copies IC data in ISR, decodes in thread context:
```cpp
// ISR: memcpy dma_buffer → dma_buffer_copy, signal dshot_waiter thread
// Thread: bdshot_decode_dshot_telemetry() on dma_buffer_copy
```

**BPRL** — decodes in-place within the DMA TC callback or UIE ISR:
```cpp
// In finish_ic_tim1() (called from DMA TC callback or TIM1_UP ISR):
if (decode_gcr(s_ic_buf_tim1, n_edges, &erpm)) { ... }
```

ArduPilot's double-buffer allows the next TX to start while decoding the previous frame.
BPRL decodes synchronously (~5–10 µs of ISR time at most) — acceptable at 400 Hz.

---

## 17. State Machine Complexity

**ArduPilot** — 6-state, managed across ISR and dedicated timer thread:
```
IDLE → SEND_START → SEND_COMPLETE → RECV_START → RECV_COMPLETE → RECV_FAILED → IDLE
```
Supports: multi-group, serial ESC passthrough, DShot command queue, EDT v1/v2 telemetry.

**BPRL** — 3-state, ISR-only:
```
IDLE → TX → IC → IDLE
```
Supports: 4-motor throttle, eRPM feedback. Simple, deterministic, zero OS overhead.

---

## 18. Extended DShot Telemetry (EDT)

**ArduPilot** decodes EDT v1/v2 alongside eRPM based on the `expo` field:

| expo | meaning |
|------|---------|
| 001 | Temperature (°C) |
| 010 | Voltage (×0.25 V) |
| 011 | Current (A) |
| 100–101 | Debug |
| 110 | Stress level (EDT v2) |
| 111 | Status flags (EDT v2) |

**BPRL** only decodes eRPM. EDT packets return `false` from `decode_gcr()` — safe but ESC
temperature/voltage warnings are not visible. Add EDT decode before sustained flight.

---

## Summary Table

| Aspect | ArduPilot | BPRL |
|--------|-----------|------|
| TX DMA streams per timer | 1 (burst DMAR, FIFO mode) | 1 TIM1 (burst DMAR, FIFO) + 1 TIM4 (direct) |
| TX buffer rows per channel | 19 (pre=1, data=16, post=2) | 19 (pre=1, data=16, post=2) |
| TX buffer layout | Interleaved stride-4 | Interleaved stride-4 (TIM1) / linear (TIM4) |
| TX CCR4 padding | Active channel | Zero (dummy) |
| TX FIFO mode | Enabled, full threshold | TIM1: enabled; TIM4: direct mode |
| OC preload (OC1PE) | Enabled | Enabled |
| TIM4 phase offset | N/A (single timer) | CNT=DS_ARR/2 → interleaved UEVs |
| GPIO speed | Medium (PAL_STM32_OSPEED_MID1) | Medium (OSPEEDR=0x1) |
| GPIO pull | Pull-up | Pull-up |
| IC→OC transition | pwmStop()/pwmStart() | switch_to_output() (manual) |
| IC timeout | ChibiOS virtual timer | Hardware UIE (ARR=20000 = 100 µs) |
| IC timer PSC | 15 (16 ticks/GCR bit) | 0 (267 ticks/GCR bit) |
| IC timer CR1 | ARPE + URS + UDIS + CEN | URS + CEN |
| IC ARR | 0xFFFF (count forever) | 20000 (hardware timeout) |
| IC FIFO mode | Enabled | Direct mode |
| IC double-buffering | Yes (copy in ISR) | No (decode in-place) |
| GCR invalid code handling | Silently decode as 0 | Return false immediately |
| eRPM units stored | eRPM/100 in uint16_t | Full eRPM in uint32_t |
| Cache management | Explicit flush/invalidate | `.nocache` MPU section |
| State machine | 6-state + thread | 3-state ISR-only |
| ESC command queue | Yes | No |
| Extended DShot Telemetry | Full EDT v1/v2 | eRPM only |

---

## Open Items

| # | Item | Priority | Notes |
|---|------|----------|-------|
| 1 | Motors 0, 2, 3 — hardware verification | **P0** | Code complete; awaiting bench test after recent GPIO/buffer/IC fixes. |
| 2 | EDT telemetry | Add before sustained flight | ESC sends EDT; BPRL safely returns false. |
| 3 | DShot command queue | Low | Cannot send ESC commands (arm/direction/save). Not needed for basic test. |
| 4 | GCR decode in ISR | Low | ~5–10 µs extra ISR time. Fine at 400 Hz; revisit if SPIThread re-enable creates contention. |
| 5 | SPIThread re-enable | After all 4 motors verified | Needs ISR priority audit at DShot priority 7. |
