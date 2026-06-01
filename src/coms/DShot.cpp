#include "src/coms/DShot.hpp"
#include "ch.h"
#include "hal.h"
#include <cstring>

/*
 * DShot600 bidirectional implementation for STM32H7 (CubeBlue H7 / CubeOrange+).
 *
 * Motor pin mapping (CubePilot standard carrier, AUX OUT rail):
 *   Motor 0 (FR) → PE11 = TIM1_CH2  (AUX 3, AF1)
 *   Motor 1 (RL) → PE9  = TIM1_CH1  (AUX 4, AF1)
 *   Motor 2 (FL) → PD13 = TIM4_CH2  (AUX 5, AF2)
 *   Motor 3 (RR) → PE13 = TIM1_CH3  (AUX 2, AF1)
 *
 * TIM1 (Motors 0, 1, 3): APB2 timer clock @ 200 MHz.
 *   DMA burst over CCR1/CCR2/CCR3 (DBL=2, 3 regs).
 *   DMA buffer: 17 slots × 3 words = 51 uint32_t.
 *   DMAMUX1 source 15 = STM32_DMAMUX1_TIM1_UP.
 *
 * TIM4 (Motor 2): APB1 timer clock @ 200 MHz (APB1 prescaler != 1 → ×2).
 *   DMA burst over CCR2 only (DBL=0, 1 reg).
 *   DMA buffer: 17 slots × 1 word = 17 uint32_t.
 *   DMAMUX1 source 32 = STM32_DMAMUX1_TIM4_UP.
 *
 * DMA burst interleave for TIM1 (CCR1 is first in address order):
 *   slot b → [CCR1 = Motor1(RL) bit b, CCR2 = Motor0(FR) bit b, CCR3 = Motor3(RR) bit b]
 *
 * After each 16-bit frame (+ inter-frame gap slot), the DMA TC ISR stops the
 * timer and switches the pins to floating input capture.  The timer's CC ISR
 * then records 21 GCR edges and decodes eRPM.  Pins are restored to AF output
 * at the start of the next dshot_write() call.
 */

/* ── DShot600 timing (200 MHz timer clock, PSC = 0) ─────────────────────── */
static constexpr uint32_t DS_ARR = 333U; // 334 ticks = 1.667 µs bit period
static constexpr uint32_t DS_T0H = 125U; // '0' pulse ≈ 625 ns
static constexpr uint32_t DS_T1H = 250U; // '1' pulse ≈ 1250 ns

/* ── DMAMUX1 request IDs (stm32_dmamux.h, STM32H7xx) ────────────────────── */
static constexpr uint32_t DMAMUX_TIM1_UP = 15U; // STM32_DMAMUX1_TIM1_UP
static constexpr uint32_t DMAMUX_TIM4_UP = 32U; // STM32_DMAMUX1_TIM4_UP

/* ── GCR decode table: 5-bit → 4-bit nibble (0xFF = invalid) ────────────── */
static constexpr uint8_t kGcrDecode[32] = {
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0x09, 0x0A, 0x0B,
    0xFF, 0x0D, 0x0E, 0x0F,
    0xFF, 0xFF, 0x02, 0x03,
    0xFF, 0x05, 0x06, 0x07,
    0xFF, 0x00, 0x08, 0x01,
    0xFF, 0x04, 0x0C, 0xFF,
};

/* ── DMA transmit buffers ────────────────────────────────────────────────── */
/* Placed in .nocache (AHB SRAM3, 0x30040000 — D2 domain, outside D-cache
 * coverage on STM32H743).  Regular AXI SRAM (.bss) is D-cached; DMA reads
 * bypass the cache and would see stale data without this placement. */
/* 18 slots = 16 data bits + 2 gap slots.  The extra gap ensures DMA TC fires
 * at UEV #18 (end of bit-0 period) rather than UEV #17 (start), so the last
 * DShot bit completes fully before the timer is stopped and we switch to input
 * capture.  Without it, bit 0 is cut to ~500 ns and ESC CRC check fails. */
static uint32_t s_dma_buf_tim1[18 * 3] __attribute__((aligned(32), section(".nocache")));
static uint32_t s_dma_buf_tim4[18]     __attribute__((aligned(32), section(".nocache")));

/* ── Telemetry state ─────────────────────────────────────────────────────── */
static ESCTelemetry s_telem[4] = {};
/* No mutex — decode_* run inside OSAL ISRs (I-locked), so they write
 * directly.  dshot_get_telemetry() uses chSysLock/Unlock for atomic reads. */

/* ── Input capture edge buffers (one array per motor) ───────────────────── */
static uint32_t s_edges[4][21];
static uint8_t  s_edge_idx[4];

/* ── Diagnostics — written from ISR, read by dshot_get_diag() ───────────── */
static volatile uint32_t s_dma_tc_count[2] = {};   // [0]=TIM1 TC fires, [1]=TIM4 TC fires
static volatile uint32_t s_cc_isr_count[2] = {};   // [0]=TIM1 CC ISR fires, [1]=TIM4 ISR fires
static uint8_t  s_last_edge_cnt[4]  = {};           // edge count from previous frame
static uint32_t s_last_edges[4][5]  = {};           // first 5 edge timestamps from previous frame

/* ── ChibiOS DMA stream handles ─────────────────────────────────────────── */
static const stm32_dma_stream_t *s_dma_tim1 = nullptr;
static const stm32_dma_stream_t *s_dma_tim4 = nullptr;

/* ── Forward declarations ────────────────────────────────────────────────── */
static uint16_t make_frame(uint16_t thr);
static void     build_dma_buf(const uint16_t throttle[4]);
static void     switch_to_output(void);
static void     switch_tim1_to_input_capture(void);
static void     switch_tim4_to_input_capture(void);
static void     decode_tim1_channels(void);
static void     decode_tim4_channel(void);
static bool     decode_gcr(const uint32_t edges[], uint8_t edge_count, uint32_t *erpm_out);

/* ═══════════════════════════════════════════════════════════════════════════
 * DMA TC callbacks — stop timer, hand off to input capture.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void dshot_dma_tc_tim1(void *param, uint32_t flags)
{
    (void)param; (void)flags;
    s_dma_tc_count[0]++;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    switch_tim1_to_input_capture();
}

static void dshot_dma_tc_tim4(void *param, uint32_t flags)
{
    (void)param; (void)flags;
    s_dma_tc_count[1]++;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    switch_tim4_to_input_capture();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TIM1 CC ISR — edge capture for motors 0 (FR), 1 (RL), 3 (RR).
 *   Motor 3 (RR) → TIM1_CH3/CCR3 → PE13 → s_edges[0]
 *   Motor 0 (FR) → TIM1_CH2/CCR2 → PE11 → s_edges[1]
 *   Motor 1 (RL) → TIM1_CH1/CCR1 → PE9  → s_edges[2]
 * ═══════════════════════════════════════════════════════════════════════════ */
OSAL_IRQ_HANDLER(STM32_TIM1_CC_HANDLER)
{
    OSAL_IRQ_PROLOGUE();

    uint32_t sr = TIM1->SR;
    TIM1->SR = 0;

    if ((sr & TIM_SR_CC3IF) && s_edge_idx[0] < 21)
        s_edges[0][s_edge_idx[0]++] = TIM1->CCR3;

    if ((sr & TIM_SR_CC2IF) && s_edge_idx[1] < 21)
        s_edges[1][s_edge_idx[1]++] = TIM1->CCR2;

    if ((sr & TIM_SR_CC1IF) && s_edge_idx[2] < 21)
        s_edges[2][s_edge_idx[2]++] = TIM1->CCR1;

    bool all_done = (s_edge_idx[0] >= 21) &&
                    (s_edge_idx[1] >= 21) &&
                    (s_edge_idx[2] >= 21);
    if (all_done || (sr & TIM_SR_UIF)) {
        s_cc_isr_count[0]++;
        TIM1->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC2IE |
                        TIM_DIER_CC3IE | TIM_DIER_UIE);
        TIM1->CR1  &= ~TIM_CR1_CEN;
        /* Save diagnostics snapshot before decode resets edge data. */
        for (int c = 0; c < 3; c++) {
            s_last_edge_cnt[c] = s_edge_idx[c];
            uint8_t n = s_edge_idx[c] < 5 ? s_edge_idx[c] : 5;
            for (uint8_t k = 0; k < n; k++) s_last_edges[c][k] = s_edges[c][k];
        }
        decode_tim1_channels();
    }

    OSAL_IRQ_EPILOGUE();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TIM4 ISR — edge capture for motor 2 (FL).
 *   Motor 2 (FL) → TIM4_CH2/CCR2 → PD13 → s_edges[3]
 * ═══════════════════════════════════════════════════════════════════════════ */
OSAL_IRQ_HANDLER(STM32_TIM4_HANDLER)
{
    OSAL_IRQ_PROLOGUE();

    uint32_t sr = TIM4->SR;
    TIM4->SR = 0;

    if ((sr & TIM_SR_CC2IF) && s_edge_idx[3] < 21)
        s_edges[3][s_edge_idx[3]++] = TIM4->CCR2;

    bool done = (s_edge_idx[3] >= 21) || (sr & TIM_SR_UIF);
    if (done) {
        s_cc_isr_count[1]++;
        TIM4->DIER &= ~(TIM_DIER_CC2IE | TIM_DIER_UIE);
        TIM4->CR1  &= ~TIM_CR1_CEN;
        s_last_edge_cnt[3] = s_edge_idx[3];
        uint8_t n = s_edge_idx[3] < 5 ? s_edge_idx[3] : 5;
        for (uint8_t k = 0; k < n; k++) s_last_edges[3][k] = s_edges[3][k];
        decode_tim4_channel();
    }

    OSAL_IRQ_EPILOGUE();
}

/* ── make_frame ──────────────────────────────────────────────────────────── */
static uint16_t make_frame(uint16_t thr)
{
    uint16_t val = (uint16_t)((thr << 1) | 1U); // set telemetry bit
    uint16_t crc = (~(val ^ (val >> 4) ^ (val >> 8))) & 0x0FU; // inverted CRC required for bidir
    return (uint16_t)((val << 4) | crc);
}

/* ── build_dma_buf ───────────────────────────────────────────────────────── */
static void build_dma_buf(const uint16_t throttle[4])
{
    uint16_t frame[4];
    for (int c = 0; c < 4; c++)
        frame[c] = make_frame(throttle[c]);

    /*
     * TIM1 burst writes CCR1, CCR2, CCR3 in register order per update event:
     *   CCR1 ← Motor 1 / RL (PE9/CH1)
     *   CCR2 ← Motor 0 / FR (PE11/CH2)
     *   CCR3 ← Motor 3 / RR (PE13/CH3)
     */
    for (int b = 15; b >= 0; b--) {
        int slot = 15 - b;
        s_dma_buf_tim1[slot * 3 + 0] = (frame[1] & (1U << b)) ? DS_T1H : DS_T0H;
        s_dma_buf_tim1[slot * 3 + 1] = (frame[0] & (1U << b)) ? DS_T1H : DS_T0H;
        s_dma_buf_tim1[slot * 3 + 2] = (frame[3] & (1U << b)) ? DS_T1H : DS_T0H;
    }
    s_dma_buf_tim1[16 * 3 + 0] = 0U; // gap slot 1
    s_dma_buf_tim1[16 * 3 + 1] = 0U;
    s_dma_buf_tim1[16 * 3 + 2] = 0U;
    s_dma_buf_tim1[17 * 3 + 0] = 0U; // gap slot 2 — DMA TC fires here (after bit 0 completes)
    s_dma_buf_tim1[17 * 3 + 1] = 0U;
    s_dma_buf_tim1[17 * 3 + 2] = 0U;

    /* TIM4 burst writes CCR2: Motor 2 / FL (PD13/CH2) */
    for (int b = 15; b >= 0; b--)
        s_dma_buf_tim4[15 - b] = (frame[2] & (1U << b)) ? DS_T1H : DS_T0H;
    s_dma_buf_tim4[16] = 0U; // gap slot 1
    s_dma_buf_tim4[17] = 0U; // gap slot 2 — DMA TC fires here
}

/* ── switch_to_output ────────────────────────────────────────────────────── */
static void switch_to_output(void)
{
    /*
     * PE9/PE11/PE13 → TIM1_CH1/CH2/CH3 (AF1), push-pull, medium speed, pull-up.
     * MODER bits: pin9=18-19, pin11=22-23, pin13=26-27.
     * AFRH bits:  pin9= 4-7,  pin11=12-15, pin13=20-23.
     */
    GPIOE->MODER   = (GPIOE->MODER
                      & ~(0x3U << 18) & ~(0x3U << 22) & ~(0x3U << 26))
                     | (0x2U << 18) | (0x2U << 22) | (0x2U << 26);
    /* Medium speed (0b01 = ~25 MHz) per ArduPilot bdshot_setup_group_ic_DMA():
     * "bdshot requires less than MID2 speed to avoid noise when switching
     *  from output to input" — very-high slew rate causes ringing that the
     *  ESC captures as false edges on the telemetry window. */
    GPIOE->OSPEEDR = (GPIOE->OSPEEDR
                      & ~(0x3U << 18) & ~(0x3U << 22) & ~(0x3U << 26))
                     | (0x1U << 18) | (0x1U << 22) | (0x1U << 26);
    /* Keep pull-up active in output mode (ArduPilot bdshot uses PUPDR_PULLUP
     * on output pins); this ensures a defined HIGH state during the brief
     * MODER transition from input → AF at the start of each frame. */
    GPIOE->PUPDR   = (GPIOE->PUPDR
                      & ~(0x3U << 18) & ~(0x3U << 22) & ~(0x3U << 26))
                     | (0x1U << 18) | (0x1U << 22) | (0x1U << 26);
    GPIOE->AFRH     = (GPIOE->AFRH
                       & ~(0xFU <<  4) & ~(0xFU << 12) & ~(0xFU << 20))
                      | (0x1U <<  4) | (0x1U << 12) | (0x1U << 20);

    /*
     * PD13 → TIM4_CH2 (AF2), push-pull, medium speed, pull-up.
     * MODER bits: pin13=26-27. AFRH bits: pin13=20-23.
     */
    GPIOD->MODER   = (GPIOD->MODER & ~(0x3U << 26)) | (0x2U << 26);
    GPIOD->OSPEEDR = (GPIOD->OSPEEDR & ~(0x3U << 26)) | (0x1U << 26);
    GPIOD->PUPDR   = (GPIOD->PUPDR & ~(0x3U << 26)) | (0x1U << 26);
    GPIOD->AFRH     = (GPIOD->AFRH & ~(0xFU << 20)) | (0x2U << 20);

    /* CCxS (direction) bits are only writable when CCxE=0 — disable first.
     *
     * Bidirectional DShot REQUIRES inverted output (idle=HIGH, pulses LOW).
     * After each frame we switch pins to input+pull-up; the pull-up holds the
     * line HIGH between frames.  Standard DShot idles LOW, so the ESC would
     * see an invalid HIGH inter-frame gap and reject every frame.  PWM mode 2
     * (OC1M=7) gives output LOW while CNT < CCR, HIGH otherwise — idle=HIGH
     * when CCR=0, bit pulses go LOW for DS_T0H or DS_T1H ticks.
     *
     * BLHeli_32 v32.7+ auto-detects bidirectional from the inverted CRC; no
     * BLHeli Suite configuration required. */
    TIM1->CCER  = 0U;
    TIM1->CCMR1 = (7U << 4)  | TIM_CCMR1_OC1PE   // CH1: PWM mode 2 (inverted, idle=HIGH)
                | (7U << 12) | TIM_CCMR1_OC2PE;   // CH2: PWM mode 2
    TIM1->CCMR2 = (7U << 4)  | TIM_CCMR2_OC3PE;  // CH3: PWM mode 2
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->ARR   = DS_ARR;

    TIM4->CCER  = 0U;
    TIM4->CCMR1 = (7U << 12) | TIM_CCMR1_OC2PE;  // CH2: PWM mode 2 (inverted, idle=HIGH)
    TIM4->CCER  = TIM_CCER_CC2E;
    TIM4->ARR   = DS_ARR;
}

/* ── switch_tim1_to_input_capture ───────────────────────────────────────── */
static void switch_tim1_to_input_capture(void)
{
    s_edge_idx[0] = s_edge_idx[1] = s_edge_idx[2] = 0;

    /* PE9/PE11/PE13 → inputs with pull-up.  Pull-up holds the line HIGH between
     * ESC response edges so the capture threshold is clean. */
    GPIOE->MODER &= ~((0x3U << 18) | (0x3U << 22) | (0x3U << 26));
    GPIOE->PUPDR  = (GPIOE->PUPDR & ~((0x3U << 18) | (0x3U << 22) | (0x3U << 26)))
                  | (0x1U << 18) | (0x1U << 22) | (0x1U << 26);

    /* CCxS (direction) bits are only writable when CCxE=0 — disable first. */
    TIM1->CCER  = 0U;
    /* TIM1 input capture: CH1/CH2/CH3 on both edges, ~100 µs timeout.
     * IC1F/IC2F/IC3F = 0b0010 → 4-sample filter at fCK_INT (200 MHz).
     * ArduPilot bdshot_config_icu_dshot() uses TIM_CCMR1_IC1F_1 for the same
     * reason: any line ringing when the GPIO switches from AF-output to input
     * appears as sub-20 ns glitches; 4-sample filter rejects them before they
     * corrupt the GCR edge timestamps. */
    TIM1->CCMR1 = (1U << 0) | (2U << 4)   // CC1S=TI1, IC1F=0b0010 (4-sample)
                | (1U << 8) | (2U << 12);  // CC2S=TI2, IC2F=0b0010
    TIM1->CCMR2 = (1U << 0) | (2U << 4);  // CC3S=TI3, IC3F=0b0010
    TIM1->CCER  = (TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP)
                | (TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP)
                | (TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP);
    TIM1->ARR   = 20000U;
    TIM1->CNT   = 0U;
    TIM1->SR    = 0U;
    TIM1->DIER  = TIM_DIER_CC1IE | TIM_DIER_CC2IE |
                  TIM_DIER_CC3IE | TIM_DIER_UIE;
    TIM1->CR1  |= TIM_CR1_CEN;
}

/* ── switch_tim4_to_input_capture ───────────────────────────────────────── */
static void switch_tim4_to_input_capture(void)
{
    s_edge_idx[3] = 0;

    /* PD13 → input with pull-up. */
    GPIOD->MODER &= ~(0x3U << 26);
    GPIOD->PUPDR  = (GPIOD->PUPDR & ~(0x3U << 26)) | (0x1U << 26);

    /* CCxS (direction) bits are only writable when CCxE=0 — disable first. */
    TIM4->CCER  = 0U;
    /* TIM4 input capture: CH2 on both edges, ~100 µs timeout.
     * IC2F = 0b0010 → 4-sample filter, same reasoning as TIM1 above. */
    TIM4->CCMR1 = (1U << 8) | (2U << 12); // CC2S=TI2, IC2F=0b0010
    TIM4->CCER  = (TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP);
    TIM4->ARR   = 20000U;
    TIM4->CNT   = 0U;
    TIM4->SR    = 0U;
    TIM4->DIER  = TIM_DIER_CC2IE | TIM_DIER_UIE;
    TIM4->CR1  |= TIM_CR1_CEN;
}

/* ── decode_gcr ──────────────────────────────────────────────────────────── */
static bool decode_gcr(const uint32_t edges[], uint8_t edge_count, uint32_t *erpm_out)
{
    /*
     * GCR response runs at 5/4 × DShot rate → 750 kHz for DShot600.
     * At 200 MHz timer: 200e6 / 750e3 = 266.7 ≈ 267 ticks per GCR bit.
     *
     * edges[0] is the timestamp of the first falling edge (frame start).
     * The inter-frame gap (idle HIGH time before the ESC responds) ends at
     * edges[0].  Decoding begins with the run from edges[0] to edges[1].
     *
     * Encoding is NRZI: each edge (transition) marks a 1; bit clocks with
     * no transition are 0.  Run of n clocks between edges → bits (1 << (n-1)).
     * This matches the GCR table which was designed for the NRZI representation.
     *
     * BLHeli_32 inverts the 4-bit CRC nibble (XOR 0xF) before transmitting,
     * so the CRC check is: computed_crc ^ 0xF == received_crc.
     */
    const uint32_t bit_ticks = (DS_ARR + 1U) * 4U / 5U; // 267 ticks

    if (edge_count < 2U) return false;

    uint32_t bits        = 0U;
    uint32_t bits_filled = 0U;

    /*
     * DShot bidir uses NRZI encoding: each transition edge marks a 1, and
     * the subsequent bit clocks with no transition are 0s.  The GCR table
     * was designed for this representation — not for raw NRZ levels.
     *
     * For each run of n bit-clocks between consecutive edges, encode as
     * "1 followed by (n-1) zeros" = (1 << (n-1)) after shifting left by n.
     * This exactly matches ArduPilot's bdshot_decode_telemetry_packet():
     *   value <<= len;  value |= 1U << (len - 1U);
     */
    for (uint8_t i = 1U; i <= edge_count && bits_filled < 21U; i++) {
        uint32_t n;
        if (i < edge_count) {
            uint32_t diff = edges[i] - edges[i - 1U];
            n = (diff + bit_ticks / 2U) / bit_ticks;
            if (n == 0U) n = 1U;
        } else {
            n = 21U - bits_filled; // last run: fill remaining zeros
        }
        if (n > 21U - bits_filled) n = 21U - bits_filled;

        bits = (bits << n) | (1U << (n - 1U));
        bits_filled += n;
    }

    if (bits_filled < 21U) return false;

    uint32_t gcr20 = bits & 0xFFFFFU; // bottom 20 bits = 4 × 5-bit GCR codes
    uint16_t frame = 0U;
    for (int g = 3; g >= 0; g--) {
        uint8_t code   = (uint8_t)((gcr20 >> (g * 5U)) & 0x1FU);
        uint8_t nibble = kGcrDecode[code];
        if (nibble == 0xFF) return false;
        frame = (uint16_t)((frame << 4U) | nibble);
    }

    uint16_t val = frame >> 4U;
    uint8_t  crc = (uint8_t)((val ^ (val >> 4U) ^ (val >> 8U)) & 0x0FU);
    if ((crc ^ 0x0FU) != (frame & 0x0FU)) return false; // BLHeli_32 inverts CRC

    uint16_t exponent = (val >> 9U) & 0x7U;
    uint16_t mantissa = val & 0x1FFU;
    *erpm_out = (uint32_t)mantissa << exponent;
    return true;
}

/* ── decode_tim1_channels (motors 0, 1, 3) ──────────────────────────────── */
/* ISR captures edges in hardware-channel order: s_edges[0]=CC3/PE13=Motor3(RR),
 * s_edges[1]=CC2/PE11=Motor0(FR), s_edges[2]=CC1/PE9=Motor1(RL).
 * Write decoded RPM to the motor-indexed s_telem slot. */
static constexpr int kChanToMotor[3] = {3, 0, 1};

static void decode_tim1_channels(void)
{
    /* Called from OSAL ISR (I-locked state). Write s_telem directly — no mutex. */
    for (int c = 0; c < 3; c++) {
        int m = kChanToMotor[c];
        if (s_edge_idx[c] < 2) { s_telem[m].valid = false; continue; }
        uint32_t erpm = 0U;
        if (decode_gcr(s_edges[c], s_edge_idx[c], &erpm)) {
            s_telem[m].erpm  = erpm;
            s_telem[m].valid = true;
        } else {
            s_telem[m].valid = false;
        }
    }
}

/* ── decode_tim4_channel (motor 2 / FL) ─────────────────────────────────── */
static void decode_tim4_channel(void)
{
    /* Called from OSAL ISR (I-locked state). Write s_telem directly — no mutex. */
    if (s_edge_idx[3] >= 2) {
        uint32_t erpm = 0U;
        if (decode_gcr(s_edges[3], s_edge_idx[3], &erpm)) {
            s_telem[2].erpm  = erpm;
            s_telem[2].valid = true;
        } else {
            s_telem[2].valid = false;
        }
    } else {
        s_telem[2].valid = false;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void dshot_init(void)
{
    /* ── Enable peripheral clocks ──────────────────────────────────────── */
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
    RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
    RCC->AHB4ENR  |= RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIODEN;

    /* ── GPIO → motor output AF mode ───────────────────────────────────── */
    switch_to_output();

    /* ── TIM1 base config (Motors 0-2, APB2 @ 200 MHz) ────────────────── */
    TIM1->CR1  = 0U;
    TIM1->CR2  = 0U;
    TIM1->PSC  = 0U;
    TIM1->ARR  = DS_ARR;
    TIM1->RCR  = 0U;
    TIM1->BDTR = TIM_BDTR_MOE; // required for advanced timer TIM1
    TIM1->EGR  = TIM_EGR_UG;  // load shadow registers
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCR4 = 0U;
    /* DMA burst: DBA = CCR1 offset/4, DBL = 2 (3 registers: CCR1/CCR2/CCR3). */
    TIM1->DCR  = ((2U) << TIM_DCR_DBL_Pos)
               | (((uint32_t)(&TIM1->CCR1) - (uint32_t)TIM1) >> 2) << TIM_DCR_DBA_Pos;
    TIM1->DIER = TIM_DIER_UDE;

    /* ── TIM4 base config (Motor 3, APB1 timer clock @ 200 MHz) ────────── */
    TIM4->CR1  = 0U;
    TIM4->CR2  = 0U;
    TIM4->PSC  = 0U;
    TIM4->ARR  = DS_ARR;
    TIM4->EGR  = TIM_EGR_UG;
    TIM4->CCR1 = 0U;
    TIM4->CCR2 = 0U;
    TIM4->CCR3 = 0U;
    TIM4->CCR4 = 0U;
    /* DMA burst: DBA = CCR2 offset/4, DBL = 0 (1 register: CCR2 only). */
    TIM4->DCR  = ((0U) << TIM_DCR_DBL_Pos)
               | (((uint32_t)(&TIM4->CCR2) - (uint32_t)TIM4) >> 2) << TIM_DCR_DBA_Pos;
    TIM4->DIER = TIM_DIER_UDE;

    /* ── DMA stream for TIM1_UP (DMAMUX source 15) ─────────────────────── */
    s_dma_tim1 = dmaStreamAlloc(STM32_DMA_STREAM_ID_ANY, CORTEX_MINIMUM_PRIORITY,
                                 dshot_dma_tc_tim1, nullptr);
    osalDbgAssert(s_dma_tim1 != nullptr, "DMA alloc failed for TIM1_UP");
    dmaSetRequestSource(s_dma_tim1, DMAMUX_TIM1_UP);
    dmaStreamSetPeripheral(s_dma_tim1, &TIM1->DMAR);
    dmaStreamSetMemory0(s_dma_tim1, s_dma_buf_tim1);
    dmaStreamSetTransactionSize(s_dma_tim1, 54U); // 18 slots × 3 channels
    s_dma_tim1->stream->FCR = DMA_SxFCR_DMDIS;   // direct mode (no FIFO)
    dmaStreamSetMode(s_dma_tim1,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos) // 32-bit memory
        | (0x2U << DMA_SxCR_PSIZE_Pos) // 32-bit peripheral
        | (0x1U << DMA_SxCR_DIR_Pos)   // memory → peripheral
        | (0x3U << DMA_SxCR_PL_Pos)    // very-high priority (matches ArduPilot PL=3)
        | DMA_SxCR_TCIE);              // TC interrupt

    /* ── DMA stream for TIM4_UP (DMAMUX source 32) ─────────────────────── */
    s_dma_tim4 = dmaStreamAlloc(STM32_DMA_STREAM_ID_ANY, CORTEX_MINIMUM_PRIORITY,
                                 dshot_dma_tc_tim4, nullptr);
    osalDbgAssert(s_dma_tim4 != nullptr, "DMA alloc failed for TIM4_UP");
    dmaSetRequestSource(s_dma_tim4, DMAMUX_TIM4_UP);
    dmaStreamSetPeripheral(s_dma_tim4, &TIM4->DMAR);
    dmaStreamSetMemory0(s_dma_tim4, s_dma_buf_tim4);
    dmaStreamSetTransactionSize(s_dma_tim4, 18U); // 18 slots × 1 channel
    s_dma_tim4->stream->FCR = DMA_SxFCR_DMDIS;
    dmaStreamSetMode(s_dma_tim4,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x1U << DMA_SxCR_DIR_Pos)
        | (0x3U << DMA_SxCR_PL_Pos)    // very-high priority
        | DMA_SxCR_TCIE);

    /* ── Enable CC interrupts ───────────────────────────────────────────── */
    nvicEnableVector(TIM1_CC_IRQn, CORTEX_MINIMUM_PRIORITY);
    nvicEnableVector(TIM4_IRQn,    CORTEX_MINIMUM_PRIORITY);
}

void dshot_write(const uint16_t throttle[4])
{
    build_dma_buf(throttle);

    /* Stop both timers; restore motor pins to AF output mode. */
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    switch_to_output();
    TIM1->CNT = 0U; TIM4->CNT = 0U;

    /* After input capture the active CCR registers hold captured timestamps
     * (e.g. 15319) which are larger than ARR=333.  With OC1PE=1 the active
     * register is only refreshed at TIM_UP, so period-0 would have CCR>ARR
     * → output HIGH the entire first 1.67 µs → ESC sees an overlong pulse
     * and rejects the frame.  Writing CCR=0 then EGR=UG flushes the shadow
     * into the active register immediately while the timer is stopped. */
    TIM1->CCR1 = 0U; TIM1->CCR2 = 0U; TIM1->CCR3 = 0U;
    TIM4->CCR2 = 0U;
    TIM1->EGR  = TIM_EGR_UG;  // shadow → active, also resets CNT
    TIM4->EGR  = TIM_EGR_UG;
    TIM1->SR   = 0U; TIM4->SR  = 0U;  // clear UIF set by EGR

    /* Restore DMA enable bits.  switch_*_to_input_capture() overwrites DIER
     * (clearing UDE) so it must be re-asserted each frame.  Also re-assert
     * TIM1 BDTR.MOE — advanced timers silence CC outputs if MOE drops. */
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->DIER = TIM_DIER_UDE;
    TIM4->DIER = TIM_DIER_UDE;

    /* Reload DMA streams.  dmaStreamDisable() clears all interrupt-enable bits
     * (TCIE/HTIE/TEIE/DMEIE) as a ChibiOS workaround for bug 3607518, so
     * dmaStreamSetMode() must be called after every disable to restore TCIE. */
    dmaStreamDisable(s_dma_tim1);
    dmaStreamSetMode(s_dma_tim1,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x1U << DMA_SxCR_DIR_Pos)
        | (0x3U << DMA_SxCR_PL_Pos)    // very-high priority (matches init and TIM4)
        | DMA_SxCR_TCIE);
    dmaStreamSetMemory0(s_dma_tim1, s_dma_buf_tim1);
    dmaStreamSetTransactionSize(s_dma_tim1, 54U);
    dmaStreamEnable(s_dma_tim1);

    dmaStreamDisable(s_dma_tim4);
    dmaStreamSetMode(s_dma_tim4,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x1U << DMA_SxCR_DIR_Pos)
        | (0x3U << DMA_SxCR_PL_Pos)    // very-high priority
        | DMA_SxCR_TCIE);
    dmaStreamSetMemory0(s_dma_tim4, s_dma_buf_tim4);
    dmaStreamSetTransactionSize(s_dma_tim4, 18U);
    dmaStreamEnable(s_dma_tim4);

    /* Start both timers back-to-back to minimise inter-frame skew. */
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_CEN;
}

void dshot_get_telemetry(ESCTelemetry out[4])
{
    chSysLock();
    memcpy(out, s_telem, 4 * sizeof(ESCTelemetry));
    chSysUnlock();
}

void dshot_get_diag(DShotDiag *out)
{
    chSysLock();
    out->dma_tc[0]  = s_dma_tc_count[0];
    out->dma_tc[1]  = s_dma_tc_count[1];
    out->cc_isr[0]  = s_cc_isr_count[0];
    out->cc_isr[1]  = s_cc_isr_count[1];
    for (int c = 0; c < 4; c++) {
        out->edge_cnt[c] = s_last_edge_cnt[c];
        for (int k = 0; k < 5; k++)
            out->edges[c][k] = s_last_edges[c][k];
    }
    chSysUnlock();
}
