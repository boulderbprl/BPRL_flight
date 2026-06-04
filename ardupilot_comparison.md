# BPRL vs ArduPilot — Bidirectional DShot Pipeline Comparison

ArduPilot reference: `libraries/AP_HAL_ChibiOS/RCOutput_bdshot.cpp` and `RCOutput.cpp`.
BPRL implementation: `src/coms/DShot.cpp`.

Both target STM32H743 (CubeOrange+) at 200 MHz, DShot600 bidirectional, ChibiOS RTOS.

---

## 1. TX DMA Architecture — Biggest Structural Difference

### ArduPilot (`send_pulses_DMAR`)

One DMA stream per timer group writes to `TIM->DMAR` using burst mode:

```
DCR: DBA = offset_of(CCR)/4,  DBL = 3  (burst of 4 registers)
Buffer: [ch0b0][ch1b0][ch2b0][ch3b0][ch0b1][ch1b1][ch2b1][ch3b1]...
Stride: 4 (one word per channel per bit position)
Total:  19 words × 4 channels = 76 words per timer group
```

Each TIM_UP event fires one DMA request, which triggers a burst of 4 consecutive transfers:
CCR1 ← buf[N×4+0], CCR2 ← buf[N×4+1], CCR3 ← buf[N×4+2], CCR4 ← buf[N×4+3].
All four CCRs update within the same UEV, in rapid intra-burst succession (~5–10 ns apart).

### BPRL (`dshot_write`)

Three separate DMA streams for TIM1, all mapped to `DMAMUX=TIM1_UP`:

```
s_dma_tx_m0 → &TIM1->CCR2  (Motor 0 / PE11)   s_tx_buf_m0[18]
s_dma_tx_m1 → &TIM1->CCR1  (Motor 1 / PE9)    s_tx_buf_m1[18]
s_dma_tx_m3 → &TIM1->CCR3  (Motor 3 / PE13)   s_tx_buf_m3[18]
```

Each TIM1_UP event is routed by DMAMUX to all three streams simultaneously. Each stream
independently transfers one word to its own CCR. All three CCRs update on the **exact same
UEV** with zero inter-channel offset. No DCR or DMAR register is used for TX.

TIM4 (Motor 2 / PD13) uses one stream → `&TIM4->DMAR` (DCR: DBA=CCR2, DBL=0) — a
single-channel timer has no ordering issue, so the DMAR approach is fine there.

### Why BPRL differs from ArduPilot

With ArduPilot's burst approach and a full 4-channel group, CCR intra-burst ordering is
~5–10 ns — negligible at DShot600 bit widths (1.67 µs). But BPRL uses only 3 of TIM1's 4
channels. When the old BPRL code attempted the burst approach with DBL=2 (burst of 3), the
buffer arrangement caused all three CCRs to be written on *sequential* UEVs (1.67 µs apart
each), giving motors 0 and 3 misaligned frames — they never received a valid DShot packet.
The per-channel stream approach eliminates any ordering dependency entirely.

---

## 2. TX Buffer Format and Preamble

| | ArduPilot | BPRL |
|-|-----------|------|
| Words before data | 1 (`dshot_pre = 1`, CCR=0) | 0 explicit; 1 implicit (see below) |
| Data bits | 16 (MSB first) | 16 (MSB first) |
| Words after data | 2 (`dshot_post = 2`, CCR=0) | 2 trailing zeros |
| **Total per channel** | **19** | **18** |
| Layout | Interleaved stride-4 | Separate per-channel array |

BPRL's implicit preamble: `switch_to_output()` sets all CCRs to 0 and calls `EGR=UG`,
so the first timer period (UEV0, before the DMA writes `buf[0]`) outputs high (CCR=0 with
PWM mode 2 = always high). This is functionally identical to ArduPilot's `dshot_pre=1`.

`fill_DMA_buffer_dshot()` in ArduPilot (packet MSB first):
```cpp
for (; i < dshot_pre; i++)             buffer[i * stride] = 0;        // leading zero
for (; i < 16 + dshot_pre; i++)        buffer[i * stride] = (packet & 0x8000) ? T1H : T0H;
for (; i < dshot_bit_length; i++)      buffer[i * stride] = 0;        // trailing zeros
```

`build_tx_buf()` in BPRL (packet MSB first):
```cpp
for (int b = 15; b >= 0; b--) {
    int s = 15 - b;
    s_tx_buf_m1[s] = (frame[1] & (1U << b)) ? DS_T1H : DS_T0H;
    ...
}
s_tx_buf_m1[16] = 0U; s_tx_buf_m1[17] = 0U;   // trailing zeros
```

---

## 3. Frame Encoding

### ArduPilot (`create_dshot_packet`)

```cpp
uint16_t packet = (value << 1);
if (telem_request) packet |= 1;          // telem bit (set once per 32 packets for serial telem)
uint16_t csum = 0, csum_data = packet;
for (uint8_t i = 0; i < 3; i++) { csum ^= csum_data; csum_data >>= 4; }
if (bidir_telem) csum = ~csum;           // invert CRC for bidirectional mode
csum &= 0xf;
packet = (packet << 4) | csum;           // CRC at bits [3:0]
```

ArduPilot conditionally inverts the CRC based on whether bidirectional mode is active.
The telem bit can be non-zero for special ESC command packets.

### BPRL (`make_frame`)

```cpp
uint16_t val = (uint16_t)(thr << 1);                        // telem bit always 0
uint16_t crc = (~(val ^ (val >> 4) ^ (val >> 8))) & 0x0FU; // inverted CRC directly
return (uint16_t)((val << 4) | crc);
```

BPRL always forces `telem=0` (correct for all throttle packets; the telem bit is only
meaningful for DShot special command packets, which BPRL does not send). BPRL computes the
inverted CRC in one pass rather than computing normal then inverting. Both produce identical
wire frames for telem=0 bidirectional throttle packets.

Wire comparison for `throttle=0`, bidirectional:

| | Frame (hex) | Throttle | Telem | CRC | ESC interpretation |
|-|-------------|----------|-------|-----|-------------------|
| ArduPilot | `0x000F` | 0 | 0 | 0xF | Zero throttle → arms |
| BPRL | `0x000F` | 0 | 0 | 0xF | Zero throttle → arms |

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

Both produce compliant DShot600 timing. The values differ by ~1% due to integer rounding
of 200MHz/600kHz = 333.33.

---

## 5. TX DMA FIFO Mode

| | ArduPilot | BPRL |
|-|-----------|------|
| TX | FIFO enabled, full threshold | Direct mode (FCR=0) |
| IC | FIFO enabled, full threshold | Direct mode (FCR=0) |

ArduPilot enables FIFO for burst-DMAR TX to ensure burst writes complete atomically.
BPRL writes single 32-bit words per transfer (one CCR per UEV) — direct mode is correct
and simpler. For IC (P→M, single register per CC event), direct mode is equally correct.

---

## 6. IC Timeout Mechanism

### ArduPilot — Software virtual timer

```cpp
// in bdshot_receive_pulses_DMAR():
chVTSetI(&group->dma_timeout,
    chTimeUS2I(group->dshot_pulse_send_time_us + 30U + 10U),
    bdshot_finish_dshot_gcr_transaction, group);

// IC timer CR1:
TIM->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_UDIS | TIM_CR1_CEN;
//                                        ^^^^ Update disabled — no UEV fires
```

The virtual timer fires at precisely `TX_time + 30 + 10` µs (calibrated to just after the
GCR response window). On fire: disables IC DMA, copies buffer, signals the dshot_waiter
thread. Timer runs with `UDIS=1` so no hardware UEV fires during IC; `URS=1` so EGR=UG
(called for PSC loading) doesn't generate a spurious UEV.

### BPRL — Hardware UIE

```cpp
// in start_ic_tim1():
TIM1->ARR = 20000U;   // 100 µs at 200 MHz
TIM1->CR1 |= TIM_CR1_CEN;

// TIM1_UP ISR fires when CNT overflows:
uint8_t captured = (uint8_t)(22U - dmaStreamGetTransactionSize(s_dma_ic_tim1));
finish_ic_tim1(captured);
```

BPRL uses the timer's own overflow interrupt (UIE) as a hardware timeout. This is why BPRL
does NOT set UDIS — it needs UEV to fire for the timeout. ARPE and URS are not needed since
BPRL does not use a PSC for IC and does not call EGR=UG during IC setup.

**Implications:** ArduPilot's vTimer is tighter (calibrated to the GCR window). BPRL's
100 µs hardware timeout is conservative but always correct at 400 Hz (2.5 ms frame period).
Both approaches are functionally correct.

---

## 7. IC Timer Setup

### ArduPilot

```cpp
TIM->CR1 = 0;
TIM->SR  = 0;
TIM->CCER = TIM->CCMR1 = TIM->CCMR2 = TIM->DIER = TIM->CR2 = 0;
TIM->PSC = telempsc;          // prescaler: 16 ticks per GCR bit
TIM->ARR = 0xFFFF;            // count forever (timeout is vTimer)
TIM->CNT = 0;
bdshot_config_icu_dshot(TIM, curr_ch, telem_tim_ch[curr_ch]);
TIM->DCR = STM32_TIM_DCR_DBA(ccr_ofs) | STM32_TIM_DCR_DBL(0);
TIM->EGR |= STM32_TIM_EGR_UG;   // flush PSC shadow
TIM->SR  = 0;
TIM->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_UDIS | TIM_CR1_CEN;
dmaStreamEnable(ic_dma);
```

### BPRL

```cpp
TIM1->CCER = TIM1->CCMR1 = TIM1->CCMR2 = 0;
// configure CCxS, CCxP, CCxNP, DIER for the rotation slot
TIM1->DCR  = (0U << TIM_DCR_DBL_Pos) | (kIcDba[slot] << TIM_DCR_DBA_Pos);
TIM1->ARR  = 20000U;          // 100 µs hardware timeout
TIM1->CNT  = 0U;
TIM1->SR   = 0U;
dmaStreamEnable(s_dma_ic_tim1);
TIM1->CR1 |= TIM_CR1_CEN;
```

Key differences:
- ArduPilot sets `PSC=telempsc`, BPRL uses `PSC=0` (unchanged from TX).
- ArduPilot sets `ARPE` (not needed without PSC shadow in BPRL).
- ArduPilot sets `UDIS` (suppresses UEV — not wanted in BPRL which uses UIE as timeout).
- ArduPilot calls `EGR=UG` to flush PSC shadow (not needed in BPRL — no PSC change).
- ArduPilot sets `ARR=0xFFFF` (count forever); BPRL sets `ARR=20000` (hardware timeout).

---

## 8. IC Bit Timing and Buffer Values

**ArduPilot** (`TELEM_IC_SAMPLE = 16`):
```
PSC = 15 → one IC tick = 16/200MHz = 80 ns
One GCR bit = 1/750kHz = 1333 ns = 16.7 IC ticks ≈ 16
Buffer values: ~16 per bit, max ~16×22 = 352 per edge → fits uint16_t
```

**BPRL** (`GCR_BIT_TICKS = (333+1)*4/5 = 267`):
```
PSC = 0 → one IC tick = 1/200MHz = 5 ns
One GCR bit = 1333 ns = 266.7 ticks ≈ 267
Buffer values: ~267 per bit, max ~267×21 = 5607 per edge → stored as uint32_t
```

Both correctly round inter-edge intervals to the nearest GCR bit count using
`n = (diff + half_bit) / bit_width`. ArduPilot's prescaler saves memory at the cost of
a PSC register write during IC setup. BPRL's full-clock approach avoids that write.

---

## 9. IC Channel Selection — Cross-Capture Trick

Both use the same ArduPilot cross-capture trick for TIM1 CH1 (Motor 1 / PE9):

```
CC2S = TI2 (0b01) → CH2 captures its own pin PE11 → Motor 0
CC2S = TI1 (0b10) → CH2 captures CH1's pin PE9 via cross-input → Motor 1
```

Both M0 and M1 use CC2 and DBA=CCR2 — the same DMA request (`TIM1_CH2`) and DMAR
address, with only CC2S toggled between slots. ArduPilot's `bdshot_config_icu_dshot()`
does this via a `switch(ccr_ch)` that handles all four possible CC channels.

BPRL implements the same logic explicitly for slots 0 and 1:
```cpp
uint32_t cc2s = (slot == 0U) ? kCC2S_TI2 : kCC2S_TI1;
TIM1->CCMR1 = cc2s | kIC2F;
```

---

## 10. GCR Decode Algorithm

Both use **transition-marking** (matching BetaFlight):
```
bits = (bits << n) | (1U << (n - 1U))
```
where `n` = number of GCR bit-widths in the inter-edge interval.

### Loop structure comparison

**ArduPilot:**
```cpp
dmar_uint_t oldValue = buffer[0];            // edge[0] = reference
for (uint32_t i = 1; i <= count; i++) {     // i = 1..count
    if (i < count) diff = buffer[i] - oldValue;
    else           len = 21 - bits;          // last interval: fill to 21
    value <<= len; value |= 1U << (len-1U);
    oldValue = buffer[i];
    bits += len;
}
```

**BPRL:**
```cpp
for (uint8_t i = 0; i < n_edges && bits_filled < 21U; i++) {
    if (i + 1U < n_edges) diff = buf[i+1] - buf[i];
    else                   n = 21U - bits_filled;  // last edge: fill to 21
    bits = (bits << n) | (1U << (n-1U));
    bits_filled += n;
}
```

These are mathematically equivalent. The first real inter-edge interval in both cases is
`buffer[1] - buffer[0]`. The number of real intervals processed is `count - 1` (ArduPilot)
and `n_edges - 1` (BPRL) — identical since `count == n_edges`.

---

## 11. GCR Decode Table — BPRL Is More Robust

**ArduPilot** — invalid codes silently return 0:
```cpp
static const uint32_t decode[32] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
    0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8,  1, 0,  4, 12,  0
};
// No validity check — invalid code → nibble=0 → decode continues
```

**BPRL** — invalid codes immediately abort:
```cpp
static constexpr uint8_t kGcrDecode[32] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07,
    0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF,
};
if (nibble == 0xFF) return false;   // invalid GCR code → reject packet
```

BPRL is stricter: any invalid 5-bit GCR code immediately fails the packet. ArduPilot's
decode-to-zero approach could, in theory, allow a corrupt packet to pass the CRC check if
the errors cancel — unlikely but not impossible.

---

## 12. CRC Check

Both use the **inverted nibble sum**: the XOR of all four 4-bit nibbles of the decoded
16-bit frame must equal 0xF.

**ArduPilot:**
```cpp
uint32_t csum = decodedValue;
csum = csum ^ (csum >> 8U);   // xor bytes
csum = csum ^ (csum >> 4U);   // xor nibbles
if ((csum & 0xfU) != 0xfU) return INVALID_ERPM;
```

**BPRL:**
```cpp
uint16_t csum = frame ^ (frame >> 4U) ^ (frame >> 8U) ^ (frame >> 12U);
if ((csum & 0x0FU) != 0x0FU) return false;
```

Identical algorithm, different notation. Both correctly check the 4-nibble XOR against 0xF.

---

## 13. eRPM Decoding and Conversion

The BDShot telemetry packet encodes the electrical revolution **period** in µs:
`period_µs = mantissa << exponent` (9-bit mantissa, 3-bit exponent).
`eRPM = 60,000,000 / period_µs`.

### ArduPilot (`bdshot_decode_telemetry_from_erpm`)

Stores eRPM in units of **eRPM/100** to fit in `uint16_t`:
```cpp
uint8_t expo   = (encodederpm >> 9) & 0x7;
uint16_t value = encodederpm & 0x1FF;
uint16_t erpm  = value << expo;                       // period in µs
erpm = (1000000U * 60U / 100U + erpm/2U) / erpm;    // = 600000 / period_µs = eRPM/100
_bdshot.erpm[chan] = erpm;                            // stored as eRPM/100
// Converted to mechanical RPM for ESC telemetry:
update_rpm(chan, erpm * 200U / motor_poles);          // eRPM/100 * 200/poles = eRPM*2/poles
```

### BPRL (`decode_gcr`)

Stores actual eRPM in `uint32_t`:
```cpp
uint32_t period   = (uint32_t)mantissa << exponent;   // period in µs
*erpm_out = (period > 0U) ? (60000000U / period) : 0U; // actual eRPM
// Displayed as mechanical RPM:
rpm[i] = telem[i].erpm / 7U;                          // eRPM / pole_pairs (14-pole motor)
```

Both implement `mechanical_RPM = 60,000,000 / (period_µs × pole_pairs)` — the same
formula, different scaling. ArduPilot divides by 100 to keep values in uint16_t (useful
for high-RPM motors that exceed 65535 eRPM). BPRL uses uint32_t and stores full eRPM.

Example for a 5000 eRPM motor (period = 12000 µs):
- ArduPilot stores: 600000/12000 = 50 (units: eRPM/100), mechanical RPM = 50×200/14 ≈ 714
- BPRL stores: 60000000/12000 = 5000 (actual eRPM), mechanical RPM = 5000/7 ≈ 714 ✓

---

## 14. Cache Coherency

**ArduPilot** — explicit cache operations per transaction:
```cpp
// Before TX DMA:
stm32_cacheBufferFlush(group.dma_buffer, buffer_length);
// After IC DMA:
stm32_cacheBufferInvalidate(group.dma_buffer, dma_tx_size);
```

**BPRL** — buffers placed in non-cacheable MPU region:
```cpp
static uint32_t s_tx_buf_m0[18] __attribute__((aligned(32), section(".nocache")));
static uint32_t s_ic_buf_tim1[22] __attribute__((aligned(32), section(".nocache")));
// etc.
```

BPRL's `.nocache` section is simpler — no per-transaction flush/invalidate required. The
tradeoff is slightly slower CPU access to DMA buffers (no cache benefit), but these buffers
are small and infrequently accessed by the CPU, so the cost is negligible.

---

## 15. IC Double-Buffering

**ArduPilot** — copies IC data in ISR, decodes in thread context:
```cpp
// In bdshot_finish_dshot_gcr_transaction() (ISR):
memcpy(group->bdshot.dma_buffer_copy, group->dma_buffer,
       sizeof(dmar_uint_t) * group->bdshot.dma_tx_size);
chEvtSignalI(group->dshot_waiter, group->dshot_event_mask);
// In dshot_send() (thread), next iteration:
bdshot_decode_dshot_telemetry(group, prev_telem_chan);  // processes dma_buffer_copy
```

**BPRL** — decodes in-place within ISR/callback context:
```cpp
// In finish_ic_tim1() (called from DMA TC callback or TIM1_UP ISR):
if (decode_gcr(s_ic_buf_tim1, n_edges, &erpm)) {
    s_telem[motor].erpm  = erpm;
    s_telem[motor].valid = true;
}
```

ArduPilot's double-buffer approach allows the main thread to decode GCR while the next TX
is already in flight. BPRL decodes synchronously in ISR context — simpler but adds a few
microseconds of ISR time. At 400 Hz with 100 µs IC timeout budget, this is not a concern.

---

## 16. State Machine Complexity

**ArduPilot** — 6-state machine managed across ISR and dedicated timer thread:

```
IDLE → SEND_START → SEND_COMPLETE → RECV_START → RECV_COMPLETE → RECV_FAILED → IDLE
```

Supports: concurrent multi-group operation, serial ESC pass-through, DShot command queue
(arm, save settings, direction change, etc.), ESC active detection, safety switch, per-ESC
enable/disable masks, EDT v1/v2 telemetry (temperature, voltage, current, stress).

**BPRL** — 3-state machine, ISR-only, no thread:

```
IDLE → TX → IC → IDLE
```

Supports: 4-motor throttle output, eRPM feedback. Simple, deterministic, zero OS overhead.

---

## 17. Extended DShot Telemetry (EDT)

**ArduPilot** (`bdshot_decode_telemetry_from_erpm`):

Decodes EDT v1 and v2 alongside eRPM based on the `expo` field of the 12-bit period value:

| expo | EDT v1/v2 meaning |
|------|-------------------|
| `0b001` | Temperature (°C) |
| `0b010` | Voltage (×0.25 V) |
| `0b011` | Current (A) |
| `0b100–0b101` | Debug |
| `0b110` | Stress level (EDT v2) |
| `0b111` | Status flags (EDT v2) |
| `0b000` or high bit set | eRPM (standard period encoding) |

**BPRL:**

Only decodes eRPM. The GCR decoder assumes all packets contain the period encoding. EDT
packets (where bit 8 of the 9-bit value is 0 and expo is 001–111) would be misinterpreted
as corrupt eRPM data and return false — which is the safe behavior (no crash, no wrong RPM).

EDT support is not required for motor testing. It would be needed to surface ESC temperature
warnings during sustained flight.

---

## Summary Table

| Aspect | ArduPilot | BPRL |
|--------|-----------|------|
| TX DMA streams per timer | 1 (burst DMAR, all channels) | 3 TX + 1 IC (direct CCR) |
| TX buffer layout | Interleaved stride-4 | Separate per-channel |
| TX FIFO mode | Enabled (full threshold) | Direct mode |
| Preamble bits | 1 explicit (`dshot_pre=1`) | 1 implicit (CCR=0 init period) |
| IC timeout | ChibiOS virtual timer | Hardware UIE (ARR=20000) |
| IC timer PSC | 15 (16 ticks/GCR bit) | 0 (267 ticks/GCR bit) |
| IC timer CR1 flags | ARPE + URS + UDIS + CEN | CEN only |
| IC FIFO mode | Enabled (full threshold) | Direct mode |
| IC ARR | 0xFFFF (count forever) | 20000 (100 µs timeout) |
| GCR invalid code handling | Silently decode as 0 | Return false immediately |
| eRPM units stored | eRPM/100 in uint16_t | Full eRPM in uint32_t |
| eRPM formula | 600,000 / period_µs | 60,000,000 / period_µs |
| Cache management | Explicit flush/invalidate | `.nocache` MPU section |
| IC double-buffering | Yes (copy in ISR) | No (decode in-place) |
| State machine | 6-state + thread | 3-state ISR-only |
| ESC command queue | Yes | No |
| Extended DShot Telemetry | Full EDT v1/v2 | eRPM only |

---

## Potential Issues to Address

### Low risk — understood differences

| # | Item | Risk | Notes |
|---|------|------|-------|
| 1 | IC timer: no ARPE | None | ARPE only needed when ARR has a shadow register; BPRL writes ARR before CEN. |
| 2 | IC timer: no URS | None | BPRL does not call EGR=UG during IC setup, so URS has no effect. |
| 3 | IC timer: no UDIS | Intentional | BPRL relies on UIE for timeout; setting UDIS would prevent the timeout ISR from firing. |
| 4 | FIFO vs direct mode | None | Direct mode is correct for single 32-bit word transfers (TX) and P→M single-register IC. |
| 5 | eRPM stored at full scale | None | uint32_t avoids overflow; division by 7 for display is correct. |

### Medium priority — add when ready

| # | Item | Impact | Notes |
|---|------|--------|-------|
| 6 | No EDT (temp/voltage/current) | Low during testing | ESC will occasionally send EDT packets; BPRL returns false for these (safe). Add EDT decode before sustained flight to catch overtemperature. |
| 7 | No DShot command queue | Low during testing | Cannot send ESC commands (arm sequence, motor direction, save settings). Not required for basic motor test or autonomous flight if ESC is pre-configured. |
| 8 | GCR decode in ISR | Low | A few µs of extra ISR time. Fine at 400 Hz. If re-enabling SPIThread creates ISR contention, move GCR decode to a deferred callback. |
