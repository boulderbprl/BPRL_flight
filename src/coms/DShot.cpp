#include "src/coms/DShot.hpp"
#include "ch.h"
#include "hal.h"
#include <cstring>

/*
 * DShot600 bidirectional — ArduPilot-style implementation.
 *
 * Motor pin mapping:
 *   Motor 0 (FR) → PE11 = TIM1_CH2  (AUX 3, AF1)
 *   Motor 1 (RL) → PE9  = TIM1_CH1  (AUX 4, AF1)
 *   Motor 2 (FL) → PD13 = TIM4_CH2  (AUX 5, AF2)
 *   Motor 3 (RR) → PE13 = TIM1_CH3  (AUX 2, AF1)
 *
 * Key design (matching ArduPilot bdshot):
 *   - GPIO stays in AF mode at all times — MODER is never changed after init.
 *   - TX: CCMR CC_nS=00 (output mode), timer drives the pin via AF.
 *   - RX: CCMR CC_nS=01 (IC mode), timer reads the pin; output driver goes
 *     high-Z so the ESC can drive the line for GCR telemetry.
 *   - No glitch on pin transitions because MODER never toggles.
 *   - TIM1_UP IRQ handles the TIM1 IC timeout (UIE fires TIM1_UP, a separate
 *     vector from TIM1_CC — the old code had no handler for TIM1_UP so the
 *     IC timeout was silently dropped every frame).
 */

/* ── DShot600 timing (200 MHz timer clock, PSC=0) ───────────────────────── */
static constexpr uint32_t DS_ARR = 333U; // 334 ticks = 1.667 µs per bit

/*
 * CCR = LOW pulse width directly (ArduPilot DSHOT_BIT_x_TICKS convention).
 * PWM mode 2: output LOW when CNT < CCR, HIGH when CNT >= CCR.
 * CCR=0 → always HIGH = bidir idle.
 * CCR=125 → LOW 625 ns  = '0' bit (37.5% of period).
 * CCR=250 → LOW 1250 ns = '1' bit (75.0% of period).
 */
static constexpr uint32_t DS_T0H = 125U;
static constexpr uint32_t DS_T1H = 250U;

/* GCR response rate = 5/4 × DShot600 = 750 kHz → 267 ticks per GCR bit */
static constexpr uint32_t GCR_BIT_TICKS = (DS_ARR + 1U) * 4U / 5U;

/* ── DMAMUX1 request IDs ────────────────────────────────────────────────── */
static constexpr uint32_t DMAMUX_TIM1_UP = 15U;
static constexpr uint32_t DMAMUX_TIM4_UP = 32U;

/* ── GCR decode table: 5-bit code → 4-bit nibble (0xFF = invalid) ───────── */
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

/* ── DMA transmit buffers (nocache — STM32H7 SRAM3, bypasses D-cache) ───── */
static uint32_t s_dma_buf_tim1[18 * 3] __attribute__((aligned(32), section(".nocache")));
static uint32_t s_dma_buf_tim4[18]     __attribute__((aligned(32), section(".nocache")));

/* ── Telemetry & edge capture state ─────────────────────────────────────── */
static ESCTelemetry s_telem[4] = {};
static uint32_t s_edges[4][21];
static uint8_t  s_edge_idx[4];

/* ── Diagnostics ────────────────────────────────────────────────────────── */
static volatile uint32_t s_dma_tc_count[2] = {};
static volatile uint32_t s_cc_isr_count[2] = {};
static uint8_t  s_last_edge_cnt[4] = {};
static uint32_t s_last_edges[4][5] = {};

/* ── ChibiOS DMA handles ────────────────────────────────────────────────── */
static const stm32_dma_stream_t *s_dma_tim1 = nullptr;
static const stm32_dma_stream_t *s_dma_tim4 = nullptr;

/* ── Forward declarations ───────────────────────────────────────────────── */
static uint16_t make_frame(uint16_t thr);
static void     build_dma_buf(const uint16_t throttle[4]);
static void     switch_to_output(void);
static void     switch_tim1_to_ic(void);
static void     switch_tim4_to_ic(void);
static void     decode_tim1_channels(void);
static void     decode_tim4_channel(void);
static bool     decode_gcr(const uint32_t edges[], uint8_t edge_count, uint32_t *erpm_out);

/* ═══════════════════════════════════════════════════════════════════════════
 * DMA TC callbacks — stop timer, start input capture.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void dshot_dma_tc_tim1(void *param, uint32_t flags)
{
    (void)param; (void)flags;
    s_dma_tc_count[0]++;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    switch_tim1_to_ic();
}

static void dshot_dma_tc_tim4(void *param, uint32_t flags)
{
    (void)param; (void)flags;
    s_dma_tc_count[1]++;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    switch_tim4_to_ic();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TIM1_UP IRQ — UIE timeout for TIM1 input capture.
 *
 * UIE fires TIM1_UP (IRQ 25), a separate vector from TIM1_CC (IRQ 27).
 * The old code had no handler here so IC always timed out silently.
 * ═══════════════════════════════════════════════════════════════════════════ */
OSAL_IRQ_HANDLER(STM32_TIM1_UP_HANDLER)
{
    OSAL_IRQ_PROLOGUE();

    uint32_t sr = TIM1->SR;
    TIM1->SR = 0U;

    if (sr & TIM_SR_UIF) {
        s_cc_isr_count[0]++;
        TIM1->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC2IE |
                        TIM_DIER_CC3IE | TIM_DIER_UIE);
        TIM1->CR1  &= ~TIM_CR1_CEN;
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
 * TIM1_CC ISR — edge capture for motors 0, 1, 3.
 *   CH3/PE13 → s_edges[0] (Motor 3 RR)
 *   CH2/PE11 → s_edges[1] (Motor 0 FR)
 *   CH1/PE9  → s_edges[2] (Motor 1 RL)
 * ═══════════════════════════════════════════════════════════════════════════ */
OSAL_IRQ_HANDLER(STM32_TIM1_CC_HANDLER)
{
    OSAL_IRQ_PROLOGUE();

    uint32_t sr = TIM1->SR;
    TIM1->SR = 0U;

    if ((sr & TIM_SR_CC3IF) && s_edge_idx[0] < 21)
        s_edges[0][s_edge_idx[0]++] = TIM1->CCR3;
    if ((sr & TIM_SR_CC2IF) && s_edge_idx[1] < 21)
        s_edges[1][s_edge_idx[1]++] = TIM1->CCR2;
    if ((sr & TIM_SR_CC1IF) && s_edge_idx[2] < 21)
        s_edges[2][s_edge_idx[2]++] = TIM1->CCR1;

    bool all_done = (s_edge_idx[0] >= 21) &&
                    (s_edge_idx[1] >= 21) &&
                    (s_edge_idx[2] >= 21);
    if (all_done) {
        s_cc_isr_count[0]++;
        TIM1->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC2IE |
                        TIM_DIER_CC3IE | TIM_DIER_UIE);
        TIM1->CR1  &= ~TIM_CR1_CEN;
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
 *   CH2/PD13 → s_edges[3]
 * ═══════════════════════════════════════════════════════════════════════════ */
OSAL_IRQ_HANDLER(STM32_TIM4_HANDLER)
{
    OSAL_IRQ_PROLOGUE();

    uint32_t sr = TIM4->SR;
    TIM4->SR = 0U;

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
    /*
     * DShot frame: [15:5] = 11-bit throttle, [4] = telem bit, [3:0] = CRC.
     *
     * telem bit = 0.  Values 0–47 with telem=1 are DShot SPECIAL COMMANDS
     * (command 0 = MOTOR STOP, etc.).  Setting telem=1 unconditionally was
     * sending MOTOR STOP at 500 Hz, preventing the ESC from ever arming.
     * ArduPilot only sets telem=1 once per 32 packets to request serial
     * telemetry; for normal throttle packets it is always 0.
     *
     * Inverted CRC (~) signals bidirectional DShot to the ESC — this is what
     * triggers the GCR eRPM response, independent of the telem bit.
     */
    uint16_t val = (uint16_t)(thr << 1);                       // telem bit = 0
    uint16_t crc = (~(val ^ (val >> 4) ^ (val >> 8))) & 0x0FU; // inverted CRC → bidir
    return (uint16_t)((val << 4) | crc);
}

/* ── build_dma_buf ───────────────────────────────────────────────────────── */
static void build_dma_buf(const uint16_t throttle[4])
{
    uint16_t frame[4];
    for (int c = 0; c < 4; c++) frame[c] = make_frame(throttle[c]);

    /*
     * TIM1 burst: each UEV loads CCR1, CCR2, CCR3 simultaneously.
     * Buffer layout per slot: [CCR1=Motor1/RL, CCR2=Motor0/FR, CCR3=Motor3/RR]
     */
    for (int b = 15; b >= 0; b--) {
        int slot = 15 - b;
        s_dma_buf_tim1[slot * 3 + 0] = (frame[1] & (1U << b)) ? DS_T1H : DS_T0H;
        s_dma_buf_tim1[slot * 3 + 1] = (frame[0] & (1U << b)) ? DS_T1H : DS_T0H;
        s_dma_buf_tim1[slot * 3 + 2] = (frame[3] & (1U << b)) ? DS_T1H : DS_T0H;
    }
    /* Two gap slots: CCR=0 → idle HIGH, DMA TC fires during slot 17 */
    s_dma_buf_tim1[16 * 3 + 0] = 0U;
    s_dma_buf_tim1[16 * 3 + 1] = 0U;
    s_dma_buf_tim1[16 * 3 + 2] = 0U;
    s_dma_buf_tim1[17 * 3 + 0] = 0U;
    s_dma_buf_tim1[17 * 3 + 1] = 0U;
    s_dma_buf_tim1[17 * 3 + 2] = 0U;

    /* TIM4: CCR2 = Motor 2 / FL */
    for (int b = 15; b >= 0; b--)
        s_dma_buf_tim4[15 - b] = (frame[2] & (1U << b)) ? DS_T1H : DS_T0H;
    s_dma_buf_tim4[16] = 0U;
    s_dma_buf_tim4[17] = 0U;
}

/* ── switch_to_output ────────────────────────────────────────────────────── */
/*
 * Configure TIM1 CH1/CH2/CH3 and TIM4 CH2 for bidirectional DShot output.
 *
 * ArduPilot approach: GPIO pins stay in AF mode at all times.  We switch
 * CC_nS from 01 (IC) back to 00 (output) in CCMR.  This is the only change
 * needed — no MODER toggle, no GPIO glitch.
 *
 * With CC_nS=00 and the timer in output mode, the AF output driver takes
 * over the pin.  CCR=0 + PWM mode 2 → output HIGH (bidir idle).
 */
static void switch_to_output(void)
{
    /* TIM1: set all three channels to PWM mode 2 output. */
    TIM1->CCER  = 0U;  // CCxS writable only when CCxE=0
    TIM1->CCMR1 = (7U << 4)  | TIM_CCMR1_OC1PE   // CH1: mode 2, preload
                | (7U << 12) | TIM_CCMR1_OC2PE;   // CH2: mode 2, preload
    TIM1->CCMR2 = (7U << 4)  | TIM_CCMR2_OC3PE;  // CH3: mode 2, preload
    TIM1->ARR   = DS_ARR;
    TIM1->CCR1  = 0U; TIM1->CCR2 = 0U; TIM1->CCR3 = 0U;
    TIM1->EGR   = TIM_EGR_UG;  // flush shadow → active (CCR=0, CNT=0)
    TIM1->SR    = 0U;
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    // CCR=0, mode 2: output HIGH (bidir idle) immediately on re-enable.

    /* TIM4: CH2 to PWM mode 2 output. */
    TIM4->CCER  = 0U;
    TIM4->CCMR1 = (7U << 12) | TIM_CCMR1_OC2PE;  // CH2: mode 2, preload
    TIM4->ARR   = DS_ARR;
    TIM4->CCR2  = 0U;
    TIM4->EGR   = TIM_EGR_UG;
    TIM4->SR    = 0U;
    TIM4->CCER  = TIM_CCER_CC2E;
    // No MODER change — GPIO stays AF throughout.
}

/* ── switch_tim1_to_ic ───────────────────────────────────────────────────── */
/*
 * Switch TIM1 CH1/CH2/CH3 to input capture on both edges.
 *
 * CC_nS=01: channel reads from TI_n (its own AF pin).  The output driver
 * is disabled by the IC mode — the pin becomes high-Z so the ESC can drive
 * it for GCR telemetry.  MODER stays AF; no glitch on the wire.
 *
 * IC1F=0b0010: 4-sample noise filter (matches ArduPilot IC1F_1 constant).
 */
static void switch_tim1_to_ic(void)
{
    s_edge_idx[0] = s_edge_idx[1] = s_edge_idx[2] = 0;

    TIM1->CCER  = 0U;
    TIM1->CCMR1 = (1U << 0) | (2U << 4)   // CC1S=TI1, IC1F=4-sample
                | (1U << 8) | (2U << 12);  // CC2S=TI2, IC2F=4-sample
    TIM1->CCMR2 = (1U << 0) | (2U << 4);  // CC3S=TI3, IC3F=4-sample
    /* Both edges (CC1P=1, CC1NP=1) on each channel. */
    TIM1->CCER  = (TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP)
                | (TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP)
                | (TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP);
    TIM1->ARR   = 20000U;  // 100 µs timeout at 200 MHz
    TIM1->CNT   = 0U;
    TIM1->SR    = 0U;
    /* Enable CC interrupts for edge capture and UIE for timeout.
     * UIE fires TIM1_UP (IRQ 25) which now has its own handler. */
    TIM1->DIER  = TIM_DIER_CC1IE | TIM_DIER_CC2IE |
                  TIM_DIER_CC3IE | TIM_DIER_UIE;
    TIM1->CR1  |= TIM_CR1_CEN;
}

/* ── switch_tim4_to_ic ───────────────────────────────────────────────────── */
static void switch_tim4_to_ic(void)
{
    s_edge_idx[3] = 0;

    TIM4->CCER  = 0U;
    TIM4->CCMR1 = (1U << 8) | (2U << 12); // CC2S=TI2, IC2F=4-sample
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
    if (edge_count < 2U) return false;

    uint32_t bits = 0U, bits_filled = 0U;

    for (uint8_t i = 1U; i <= edge_count && bits_filled < 21U; i++) {
        uint32_t n;
        if (i < edge_count) {
            uint32_t diff = edges[i] - edges[i - 1U];
            n = (diff + GCR_BIT_TICKS / 2U) / GCR_BIT_TICKS;
            if (n == 0U) n = 1U;
        } else {
            n = 21U - bits_filled;
        }
        if (n > 21U - bits_filled) n = 21U - bits_filled;
        bits = (bits << n) | (1U << (n - 1U));
        bits_filled += n;
    }

    if (bits_filled < 21U) return false;

    uint32_t gcr20 = bits & 0xFFFFFU;
    uint16_t frame = 0U;
    for (int g = 3; g >= 0; g--) {
        uint8_t code   = (uint8_t)((gcr20 >> (g * 5U)) & 0x1FU);
        uint8_t nibble = kGcrDecode[code];
        if (nibble == 0xFF) return false;
        frame = (uint16_t)((frame << 4U) | nibble);
    }

    uint16_t val = frame >> 4U;
    uint8_t  crc = (uint8_t)((val ^ (val >> 4U) ^ (val >> 8U)) & 0x0FU);
    if ((crc ^ 0x0FU) != (frame & 0x0FU)) return false;

    uint16_t exponent = (val >> 9U) & 0x7U;
    uint16_t mantissa = val & 0x1FFU;
    *erpm_out = (uint32_t)mantissa << exponent;
    return true;
}

/* ── decode_tim1_channels ───────────────────────────────────────────────── */
static constexpr int kChanToMotor[3] = {3, 0, 1};

static void decode_tim1_channels(void)
{
    for (int c = 0; c < 3; c++) {
        int m = kChanToMotor[c];
        uint32_t erpm = 0U;
        if (s_edge_idx[c] >= 2U && decode_gcr(s_edges[c], s_edge_idx[c], &erpm)) {
            s_telem[m].erpm  = erpm;
            s_telem[m].valid = true;
        } else {
            s_telem[m].valid = false;
        }
    }
}

/* ── decode_tim4_channel ────────────────────────────────────────────────── */
static void decode_tim4_channel(void)
{
    uint32_t erpm = 0U;
    if (s_edge_idx[3] >= 2U && decode_gcr(s_edges[3], s_edge_idx[3], &erpm)) {
        s_telem[2].erpm  = erpm;
        s_telem[2].valid = true;
    } else {
        s_telem[2].valid = false;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void dshot_init(void)
{
    /* ── Enable peripheral clocks ─────────────────────────────────────── */
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
    RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
    RCC->AHB4ENR  |= RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIODEN;

    /* ── GPIO: set AF mode ONCE and never change ─────────────────────── */
    /*
     * ArduPilot approach: pin stays in AF mode throughout TX and RX.
     * Switching between output and IC is done by changing CC_nS in CCMR,
     * not by toggling MODER.  This eliminates the glitch that was causing
     * false IC captures at CNT=0 on every frame.
     *
     * Medium speed (0b01 = ~25 MHz): matches ArduPilot bdshot GPIO speed.
     * Pull-up: holds line HIGH when timer IC output driver is disabled.
     *
     * PE9=TIM1_CH1, PE11=TIM1_CH2, PE13=TIM1_CH3 → AF1
     */
    GPIOE->MODER   = (GPIOE->MODER
                      & ~(0x3U << 18) & ~(0x3U << 22) & ~(0x3U << 26))
                     | (0x2U << 18) | (0x2U << 22) | (0x2U << 26);
    GPIOE->OSPEEDR = (GPIOE->OSPEEDR
                      & ~(0x3U << 18) & ~(0x3U << 22) & ~(0x3U << 26))
                     | (0x1U << 18) | (0x1U << 22) | (0x1U << 26);
    GPIOE->PUPDR   = (GPIOE->PUPDR
                      & ~(0x3U << 18) & ~(0x3U << 22) & ~(0x3U << 26))
                     | (0x1U << 18) | (0x1U << 22) | (0x1U << 26);
    GPIOE->AFRH    = (GPIOE->AFRH
                      & ~(0xFU <<  4) & ~(0xFU << 12) & ~(0xFU << 20))
                     | (0x1U <<  4) | (0x1U << 12) | (0x1U << 20);

    /* PD13=TIM4_CH2 → AF2 */
    GPIOD->MODER   = (GPIOD->MODER & ~(0x3U << 26)) | (0x2U << 26);
    GPIOD->OSPEEDR = (GPIOD->OSPEEDR & ~(0x3U << 26)) | (0x1U << 26);
    GPIOD->PUPDR   = (GPIOD->PUPDR & ~(0x3U << 26)) | (0x1U << 26);
    GPIOD->AFRH    = (GPIOD->AFRH & ~(0xFU << 20)) | (0x2U << 20);

    /* ── TIM1 base configuration ─────────────────────────────────────── */
    TIM1->CR1  = 0U;
    TIM1->CR2  = 0U;
    TIM1->PSC  = 0U;
    TIM1->ARR  = DS_ARR;
    TIM1->RCR  = 0U;
    TIM1->BDTR = TIM_BDTR_MOE;  // main output enable (required for TIM1)
    TIM1->EGR  = TIM_EGR_UG;
    TIM1->CCR1 = 0U; TIM1->CCR2 = 0U; TIM1->CCR3 = 0U; TIM1->CCR4 = 0U;
    /* DMA burst: DBA=CCR1 word offset, DBL=2 (3 registers: CCR1/CCR2/CCR3) */
    TIM1->DCR  = ((2U) << TIM_DCR_DBL_Pos)
               | (((uint32_t)(&TIM1->CCR1) - (uint32_t)TIM1) >> 2U) << TIM_DCR_DBA_Pos;
    TIM1->DIER = TIM_DIER_UDE;

    /* ── TIM4 base configuration ─────────────────────────────────────── */
    TIM4->CR1  = 0U;
    TIM4->CR2  = 0U;
    TIM4->PSC  = 0U;
    TIM4->ARR  = DS_ARR;
    TIM4->EGR  = TIM_EGR_UG;
    TIM4->CCR1 = 0U; TIM4->CCR2 = 0U; TIM4->CCR3 = 0U; TIM4->CCR4 = 0U;
    /* DMA burst: DBA=CCR2 word offset, DBL=0 (1 register: CCR2) */
    TIM4->DCR  = ((0U) << TIM_DCR_DBL_Pos)
               | (((uint32_t)(&TIM4->CCR2) - (uint32_t)TIM4) >> 2U) << TIM_DCR_DBA_Pos;
    TIM4->DIER = TIM_DIER_UDE;

    /* Set initial output mode so pins drive HIGH (bidir idle) from the start */
    switch_to_output();

    /* ── DMA for TIM1_UP ─────────────────────────────────────────────── */
    s_dma_tim1 = dmaStreamAlloc(STM32_DMA_STREAM_ID_ANY, CORTEX_MINIMUM_PRIORITY,
                                 dshot_dma_tc_tim1, nullptr);
    osalDbgAssert(s_dma_tim1 != nullptr, "DMA alloc TIM1_UP");
    /* Explicit null guard: if assertions are compiled out and alloc fails,
     * rapid-flash the activity LED so the failure is impossible to miss.
     * Any code path past here with s_dma_tim1==nullptr would silent-crash. */
    if (s_dma_tim1 == nullptr) {
        for (;;) {
            for (int i = 0; i < 20; i++) {
                palToggleLine(LINE_LED_ACTIVITY);
                volatile uint32_t n = 2000000U; while (n--) {}
            }
            volatile uint32_t n = 16000000U; while (n--) {}
        }
    }
    dmaSetRequestSource(s_dma_tim1, DMAMUX_TIM1_UP);
    dmaStreamSetPeripheral(s_dma_tim1, &TIM1->DMAR);
    dmaStreamSetMemory0(s_dma_tim1, s_dma_buf_tim1);
    dmaStreamSetTransactionSize(s_dma_tim1, 54U);
    /* FIFO full-threshold: pre-loads 4 words so all 3 burst requests are
     * served without re-fetching mid-burst (matches ArduPilot FTH_FULL). */
    s_dma_tim1->stream->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH;
    dmaStreamSetMode(s_dma_tim1,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x1U << DMA_SxCR_DIR_Pos)
        | (0x3U << DMA_SxCR_PL_Pos)
        | DMA_SxCR_TCIE);

    /* ── DMA for TIM4_UP ─────────────────────────────────────────────── */
    s_dma_tim4 = dmaStreamAlloc(STM32_DMA_STREAM_ID_ANY, CORTEX_MINIMUM_PRIORITY,
                                 dshot_dma_tc_tim4, nullptr);
    osalDbgAssert(s_dma_tim4 != nullptr, "DMA alloc TIM4_UP");
    if (s_dma_tim4 == nullptr) {
        for (;;) {
            for (int i = 0; i < 30; i++) {  /* 30-flash = distinct from TIM1 failure (20) */
                palToggleLine(LINE_LED_ACTIVITY);
                volatile uint32_t n = 2000000U; while (n--) {}
            }
            volatile uint32_t n = 16000000U; while (n--) {}
        }
    }
    dmaSetRequestSource(s_dma_tim4, DMAMUX_TIM4_UP);
    dmaStreamSetPeripheral(s_dma_tim4, &TIM4->DMAR);
    dmaStreamSetMemory0(s_dma_tim4, s_dma_buf_tim4);
    dmaStreamSetTransactionSize(s_dma_tim4, 18U);
    s_dma_tim4->stream->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH;
    dmaStreamSetMode(s_dma_tim4,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x1U << DMA_SxCR_DIR_Pos)
        | (0x3U << DMA_SxCR_PL_Pos)
        | DMA_SxCR_TCIE);

    /* ── Enable IRQs ─────────────────────────────────────────────────── */
    /*
     * Use the same priority as the rest of the timer IRQs in mcuconf.h
     * (STM32_IRQ_TIM1_CC_PRIORITY = STM32_IRQ_TIM4_PRIORITY = 7).
     *
     * CORTEX_MINIMUM_PRIORITY (15) was too low: any higher-priority ISR
     * (SPI DMA at ~12, CAN at ~12, USB at ~12) could preempt the CC edge-
     * capture handler mid-GCR-frame, causing delayed CCR reads, missed edges
     * (the CCR gets overwritten by the next capture before we read it), and
     * corrupt run-length timestamps.  At 750 kHz GCR each bit is only 1.33 µs;
     * even a 5 µs ISR preemption loses multiple edges.
     *
     * These ISRs never call any ChibiOS ch* functions, so they are safe at
     * any priority level (including above the RTOS kernel lock boundary).
     */
    nvicEnableVector(TIM1_UP_IRQn,  STM32_IRQ_TIM1_UP_PRIORITY); // UIE timeout
    nvicEnableVector(TIM1_CC_IRQn,  STM32_IRQ_TIM1_CC_PRIORITY); // CC edge capture
    nvicEnableVector(TIM4_IRQn,     STM32_IRQ_TIM4_PRIORITY);    // CC + UIE (shared)
}

void dshot_write(const uint16_t throttle[4])
{
    build_dma_buf(throttle);

    /* Stop both timers in whatever state they're in (IC or idle). */
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;

    /* Disable all timer interrupts and DMA requests BEFORE calling
     * switch_to_output().  switch_to_output() calls EGR=UG which
     * generates a software UEV.  If UIE or CC_nIE are still set in
     * DIER from the previous IC session, that UEV immediately queues
     * the TIM1_UP or TIM1_CC IRQ — wasting an interrupt slot and
     * potentially decoding stale edge data. */
    TIM1->DIER = 0U;
    TIM4->DIER = 0U;
    TIM1->SR   = 0U;
    TIM4->SR   = 0U;

    /* Switch to output mode (CC_nS=00 → timer drives pin HIGH via AF).
     * No MODER change needed — pin is already in AF mode. */
    switch_to_output();

    /* Restore timer control for DShot TX. */
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->DIER = TIM_DIER_UDE;
    TIM4->DIER = TIM_DIER_UDE;

    /* Reload DMA.  dmaStreamDisable() clears TCIE (ChibiOS bug workaround),
     * so dmaStreamSetMode() must restore it after every disable. */
    dmaStreamDisable(s_dma_tim1);
    dmaStreamSetMode(s_dma_tim1,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x1U << DMA_SxCR_DIR_Pos)
        | (0x3U << DMA_SxCR_PL_Pos)
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
        | (0x3U << DMA_SxCR_PL_Pos)
        | DMA_SxCR_TCIE);
    dmaStreamSetMemory0(s_dma_tim4, s_dma_buf_tim4);
    dmaStreamSetTransactionSize(s_dma_tim4, 18U);
    dmaStreamEnable(s_dma_tim4);

    /* Start both timers. CNT=0 from switch_to_output's EGR=UG.
     * Period 0 is idle HIGH (CCR=0, mode 2) before first DMA UEV. */
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
