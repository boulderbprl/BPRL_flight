#include "src/coms/DShot.hpp"
#include "ch.h"
#include "hal.h"
#include <cstring>

/*
 * DShot600 bidirectional implementation for STM32H7 (CubeBlue H7 / CubeOrange+).
 *
 * Motor pin mapping (CubePilot standard carrier, AUX OUT rail):
 *   Motor 0 (FR) → PE13 = TIM1_CH3  (AUX 2, AF1)
 *   Motor 1 (RL) → PE11 = TIM1_CH2  (AUX 3, AF1)
 *   Motor 2 (FL) → PE9  = TIM1_CH1  (AUX 4, AF1)
 *   Motor 3 (RR) → PD13 = TIM4_CH2  (AUX 5, AF2)
 *
 * TIM1 (Motors 0-2): APB2 timer clock @ 200 MHz.
 *   DMA burst over CCR1/CCR2/CCR3 (DBL=2, 3 regs).
 *   DMA buffer: 17 slots × 3 words = 51 uint32_t.
 *   DMAMUX1 source 15 = STM32_DMAMUX1_TIM1_UP.
 *
 * TIM4 (Motor 3): APB1 timer clock @ 200 MHz (APB1 prescaler != 1 → ×2).
 *   DMA burst over CCR2 only (DBL=0, 1 reg).
 *   DMA buffer: 17 slots × 1 word = 17 uint32_t.
 *   DMAMUX1 source 32 = STM32_DMAMUX1_TIM4_UP.
 *
 * DMA burst interleave for TIM1 (CCR1 is first in address order):
 *   slot b → [CCR1 = Motor2 bit b, CCR2 = Motor1 bit b, CCR3 = Motor0 bit b]
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
static uint32_t s_dma_buf_tim1[17 * 3]; // TIM1: 51 words (3 motors × 17 slots)
static uint32_t s_dma_buf_tim4[17];     // TIM4: 17 words (1 motor × 17 slots)

/* ── Telemetry state ─────────────────────────────────────────────────────── */
static ESCTelemetry s_telem[4] = {};
/* No mutex — decode_* run inside OSAL ISRs (I-locked), so they write
 * directly.  dshot_get_telemetry() uses chSysLock/Unlock for atomic reads. */

/* ── Input capture edge buffers (one array per motor) ───────────────────── */
static uint32_t s_edges[4][21];
static uint8_t  s_edge_idx[4];

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
static bool     decode_gcr(const uint32_t edges[21], uint32_t *erpm_out);

/* ═══════════════════════════════════════════════════════════════════════════
 * DMA TC callbacks — stop timer, hand off to input capture.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void dshot_dma_tc_tim1(void *param, uint32_t flags)
{
    (void)param; (void)flags;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    switch_tim1_to_input_capture();
}

static void dshot_dma_tc_tim4(void *param, uint32_t flags)
{
    (void)param; (void)flags;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    switch_tim4_to_input_capture();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TIM1 CC ISR — edge capture for motors 0, 1, 2.
 *   Motor 0 → TIM1_CH3/CCR3 → PE13 → s_edges[0]
 *   Motor 1 → TIM1_CH2/CCR2 → PE11 → s_edges[1]
 *   Motor 2 → TIM1_CH1/CCR1 → PE9  → s_edges[2]
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
        TIM1->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC2IE |
                        TIM_DIER_CC3IE | TIM_DIER_UIE);
        TIM1->CR1  &= ~TIM_CR1_CEN;
        decode_tim1_channels();
    }

    OSAL_IRQ_EPILOGUE();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TIM4 ISR — edge capture for motor 3.
 *   Motor 3 → TIM4_CH2/CCR2 → PD13 → s_edges[3]
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
        TIM4->DIER &= ~(TIM_DIER_CC2IE | TIM_DIER_UIE);
        TIM4->CR1  &= ~TIM_CR1_CEN;
        decode_tim4_channel();
    }

    OSAL_IRQ_EPILOGUE();
}

/* ── make_frame ──────────────────────────────────────────────────────────── */
static uint16_t make_frame(uint16_t thr)
{
    uint16_t val = (uint16_t)((thr << 1) | 0U); // bidir telemetry bit = 0
    uint16_t crc = (val ^ (val >> 4) ^ (val >> 8)) & 0x0FU;
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
     *   CCR1 ← Motor 2 (PE9/CH1)
     *   CCR2 ← Motor 1 (PE11/CH2)
     *   CCR3 ← Motor 0 (PE13/CH3)
     */
    for (int b = 15; b >= 0; b--) {
        int slot = 15 - b;
        s_dma_buf_tim1[slot * 3 + 0] = (frame[2] & (1U << b)) ? DS_T1H : DS_T0H;
        s_dma_buf_tim1[slot * 3 + 1] = (frame[1] & (1U << b)) ? DS_T1H : DS_T0H;
        s_dma_buf_tim1[slot * 3 + 2] = (frame[0] & (1U << b)) ? DS_T1H : DS_T0H;
    }
    s_dma_buf_tim1[16 * 3 + 0] = 0U; // inter-frame gap
    s_dma_buf_tim1[16 * 3 + 1] = 0U;
    s_dma_buf_tim1[16 * 3 + 2] = 0U;

    /* TIM4 burst writes CCR2: Motor 3 (PD13/CH2) */
    for (int b = 15; b >= 0; b--)
        s_dma_buf_tim4[15 - b] = (frame[3] & (1U << b)) ? DS_T1H : DS_T0H;
    s_dma_buf_tim4[16] = 0U;
}

/* ── switch_to_output ────────────────────────────────────────────────────── */
static void switch_to_output(void)
{
    /*
     * PE9/PE11/PE13 → TIM1_CH1/CH2/CH3 (AF1), push-pull, very-high speed.
     * MODER bits: pin9=18-19, pin11=22-23, pin13=26-27.
     * AFRH bits:  pin9= 4-7,  pin11=12-15, pin13=20-23.
     */
    GPIOE->MODER   = (GPIOE->MODER
                      & ~(0x3U << 18) & ~(0x3U << 22) & ~(0x3U << 26))
                     | (0x2U << 18) | (0x2U << 22) | (0x2U << 26);
    GPIOE->OSPEEDR |= (0x3U << 18) | (0x3U << 22) | (0x3U << 26);
    GPIOE->PUPDR    = (GPIOE->PUPDR
                       & ~(0x3U << 18) & ~(0x3U << 22) & ~(0x3U << 26));
    GPIOE->AFRH     = (GPIOE->AFRH
                       & ~(0xFU <<  4) & ~(0xFU << 12) & ~(0xFU << 20))
                      | (0x1U <<  4) | (0x1U << 12) | (0x1U << 20);

    /*
     * PD13 → TIM4_CH2 (AF2), push-pull, very-high speed.
     * MODER bits: pin13=26-27. AFRH bits: pin13=20-23.
     */
    GPIOD->MODER   = (GPIOD->MODER & ~(0x3U << 26)) | (0x2U << 26);
    GPIOD->OSPEEDR |= (0x3U << 26);
    GPIOD->PUPDR    = (GPIOD->PUPDR & ~(0x3U << 26));
    GPIOD->AFRH     = (GPIOD->AFRH & ~(0xFU << 20)) | (0x2U << 20);

    /* TIM1: CH1/CH2/CH3 → PWM mode 2, preload enabled. CH4 unused. */
    TIM1->CCMR1 = (6U << 4)  | TIM_CCMR1_OC1PE
                | (6U << 12) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6U << 4)  | TIM_CCMR2_OC3PE;
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->ARR   = DS_ARR;

    /* TIM4: CH2 → PWM mode 2, preload enabled. */
    TIM4->CCMR1 = (6U << 12) | TIM_CCMR1_OC2PE;
    TIM4->CCER  = TIM_CCER_CC2E;
    TIM4->ARR   = DS_ARR;
}

/* ── switch_tim1_to_input_capture ───────────────────────────────────────── */
static void switch_tim1_to_input_capture(void)
{
    s_edge_idx[0] = s_edge_idx[1] = s_edge_idx[2] = 0;

    /* PE9/PE11/PE13 → floating inputs. */
    GPIOE->MODER &= ~((0x3U << 18) | (0x3U << 22) | (0x3U << 26));
    GPIOE->PUPDR &= ~((0x3U << 18) | (0x3U << 22) | (0x3U << 26));

    /* TIM1 input capture: CH1/CH2/CH3 on both edges, ~100 µs timeout. */
    TIM1->CCMR1 = (1U << 0) | (1U << 8);  // CC1S=TI1, CC2S=TI2
    TIM1->CCMR2 = (1U << 0);               // CC3S=TI3
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

    /* PD13 → floating input. */
    GPIOD->MODER &= ~(0x3U << 26);
    GPIOD->PUPDR &= ~(0x3U << 26);

    /* TIM4 input capture: CH2 on both edges, ~100 µs timeout. */
    TIM4->CCMR1 = (1U << 8);  // CC2S=TI2
    TIM4->CCER  = (TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP);
    TIM4->ARR   = 20000U;
    TIM4->CNT   = 0U;
    TIM4->SR    = 0U;
    TIM4->DIER  = TIM_DIER_CC2IE | TIM_DIER_UIE;
    TIM4->CR1  |= TIM_CR1_CEN;
}

/* ── decode_gcr ──────────────────────────────────────────────────────────── */
static bool decode_gcr(const uint32_t edges[21], uint32_t *erpm_out)
{
    const uint32_t bit_ticks = DS_ARR + 1U; // 334 ticks per bit

    uint32_t bits = 0U;
    int      bits_filled = 0;
    uint8_t  level = 1U; // ESC response idles high

    for (int i = 0; i < 21 && bits_filled < 21; i++) {
        uint32_t width;
        if (i == 0) {
            width = edges[0];
        } else {
            uint32_t prev = edges[i - 1];
            uint32_t cur  = edges[i];
            width = (cur >= prev) ? (cur - prev) : (20001U - prev + cur);
        }

        uint32_t n = (width + bit_ticks / 2U) / bit_ticks;
        if (n == 0U) n = 1U;
        if (n > (uint32_t)(21 - bits_filled)) n = (uint32_t)(21 - bits_filled);

        for (uint32_t k = 0; k < n && bits_filled < 21; k++) {
            bits = (bits << 1) | level;
            bits_filled++;
        }
        level ^= 1U;
    }

    if (bits_filled < 21) return false;

    uint32_t gcr20 = bits & 0xFFFFFU;
    uint16_t frame = 0U;
    for (int g = 3; g >= 0; g--) {
        uint8_t code   = (uint8_t)((gcr20 >> (g * 5)) & 0x1FU);
        uint8_t nibble = kGcrDecode[code];
        if (nibble == 0xFF) return false;
        frame = (uint16_t)((frame << 4) | nibble);
    }

    uint16_t val = frame >> 4;
    uint8_t  crc = (uint8_t)((val ^ (val >> 4) ^ (val >> 8)) & 0x0FU);
    if (crc != (frame & 0x0FU)) return false;

    uint16_t exponent = (val >> 9) & 0x7U;
    uint16_t mantissa = (val >> 0) & 0x1FFU;
    *erpm_out = (uint32_t)mantissa << exponent;
    return true;
}

/* ── decode_tim1_channels (motors 0, 1, 2) ──────────────────────────────── */
static void decode_tim1_channels(void)
{
    /* Called from OSAL ISR (I-locked state). Write s_telem directly — no mutex. */
    for (int c = 0; c < 3; c++) {
        if (s_edge_idx[c] < 2) { s_telem[c].valid = false; continue; }
        uint32_t erpm = 0U;
        if (decode_gcr(s_edges[c], &erpm)) {
            s_telem[c].erpm  = erpm;
            s_telem[c].valid = true;
        } else {
            s_telem[c].valid = false;
        }
    }
}

/* ── decode_tim4_channel (motor 3) ──────────────────────────────────────── */
static void decode_tim4_channel(void)
{
    /* Called from OSAL ISR (I-locked state). Write s_telem directly — no mutex. */
    if (s_edge_idx[3] >= 2) {
        uint32_t erpm = 0U;
        if (decode_gcr(s_edges[3], &erpm)) {
            s_telem[3].erpm  = erpm;
            s_telem[3].valid = true;
        } else {
            s_telem[3].valid = false;
        }
    } else {
        s_telem[3].valid = false;
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
    dmaStreamSetTransactionSize(s_dma_tim1, 51U); // 17 slots × 3 channels
    s_dma_tim1->stream->FCR = DMA_SxFCR_DMDIS;   // direct mode (no FIFO)
    dmaStreamSetMode(s_dma_tim1,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos) // 32-bit memory
        | (0x2U << DMA_SxCR_PSIZE_Pos) // 32-bit peripheral
        | (0x1U << DMA_SxCR_DIR_Pos)   // memory → peripheral
        | DMA_SxCR_TCIE);              // TC interrupt

    /* ── DMA stream for TIM4_UP (DMAMUX source 32) ─────────────────────── */
    s_dma_tim4 = dmaStreamAlloc(STM32_DMA_STREAM_ID_ANY, CORTEX_MINIMUM_PRIORITY,
                                 dshot_dma_tc_tim4, nullptr);
    osalDbgAssert(s_dma_tim4 != nullptr, "DMA alloc failed for TIM4_UP");
    dmaSetRequestSource(s_dma_tim4, DMAMUX_TIM4_UP);
    dmaStreamSetPeripheral(s_dma_tim4, &TIM4->DMAR);
    dmaStreamSetMemory0(s_dma_tim4, s_dma_buf_tim4);
    dmaStreamSetTransactionSize(s_dma_tim4, 17U); // 17 slots × 1 channel
    s_dma_tim4->stream->FCR = DMA_SxFCR_DMDIS;
    dmaStreamSetMode(s_dma_tim4,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x1U << DMA_SxCR_DIR_Pos)
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
    TIM1->SR  = 0U; TIM4->SR  = 0U;

    /* Reload DMA streams. */
    dmaStreamDisable(s_dma_tim1);
    dmaStreamSetMemory0(s_dma_tim1, s_dma_buf_tim1);
    dmaStreamSetTransactionSize(s_dma_tim1, 51U);
    dmaStreamEnable(s_dma_tim1);

    dmaStreamDisable(s_dma_tim4);
    dmaStreamSetMemory0(s_dma_tim4, s_dma_buf_tim4);
    dmaStreamSetTransactionSize(s_dma_tim4, 17U);
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
