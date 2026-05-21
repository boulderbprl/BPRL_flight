#include "src/coms/DShot.hpp"
#include "ch.h"
#include "hal.h"
#include <cstring>

/*
 * DShot600 bidirectional implementation for STM32H7 (CubeBlue H7 / CubeOrange+).
 *
 * Motor pin mapping:
 *   Motor 1 (FR) → PA8  = TIM1_CH1  (AF1)
 *   Motor 2 (RL) → PA9  = TIM1_CH2  (AF1)
 *   Motor 3 (FL) → PA10 = TIM1_CH3  (AF1)
 *   Motor 4 (RR) → PE14 = TIM1_CH4  (AF1)
 *
 * PA11 (the other TIM1_CH4 mapping) is NOT used: it is the USB OTG_FS D- line
 * and must remain in USB AF10.  PE14 is the alternate mapping for TIM1_CH4.
 * Verify your carrier board connects motor 4 ESC signal to PE14.
 *
 * TIM1 is on APB2 (assumed 200 MHz). Adjust ARR/CCR values below if APB2
 * differs — see "APB2 clock" note in the plan.
 *
 * DMA burst: TIM1_UP event triggers DMA2 to push 4 CCR values per timer
 * period (CCR1-4 interleaved), clocking out all 4 motors simultaneously.
 *
 * After the 16-bit DShot frame completes, the DMA TC ISR switches the pins
 * to input capture, waits ~30 µs, then captures 21 GCR-encoded edges from
 * each ESC and decodes eRPM.
 */

/* ── DShot600 timing constants (APB2 = 200 MHz, PSC = 0) ─────────────────── */
static constexpr uint32_t DS_ARR  = 333U; // bit period = 334 ticks = 1.667 µs
static constexpr uint32_t DS_T0H  = 125U; // '0' high time = 625 ns
static constexpr uint32_t DS_T1H  = 250U; // '1' high time = 1250 ns

/* ── GCR decode table: 5-bit code → 4-bit nibble (0xFF = invalid) ────────── */
static constexpr uint8_t kGcrDecode[32] = {
    0xFF, 0xFF, 0xFF, 0xFF, // 0x00-0x03 invalid
    0xFF, 0xFF, 0xFF, 0xFF, // 0x04-0x07 invalid
    0xFF, 0x09, 0x0A, 0x0B, // 0x08-0x0B
    0xFF, 0x0D, 0x0E, 0x0F, // 0x0C-0x0F
    0xFF, 0xFF, 0x02, 0x03, // 0x10-0x13
    0xFF, 0x05, 0x06, 0x07, // 0x14-0x17
    0xFF, 0x00, 0x08, 0x01, // 0x18-0x1B
    0xFF, 0x04, 0x0C, 0xFF, // 0x1C-0x1F
};

/* ── DMA transmit buffer ─────────────────────────────────────────────────── */
/*
 * Interleaved layout for TIM1 DMA burst (DBL=3, DBA=CCR1):
 *   slot 0: [CCR1 bit15, CCR2 bit15, CCR3 bit15, CCR4 bit15]
 *   slot 1: [CCR1 bit14, CCR2 bit14, CCR3 bit14, CCR4 bit14]
 *   ...
 *   slot 15: [CCR1 bit0, CCR2 bit0, CCR3 bit0, CCR4 bit0]
 *   slot 16: [0, 0, 0, 0]  ← inter-frame gap (CCR=0 → output low)
 * Total: 17 slots × 4 channels = 68 uint32_t values.
 */
static uint32_t s_dma_buf[17 * 4];

/* ── Telemetry state ─────────────────────────────────────────────────────── */
static ESCTelemetry s_telem[4]    = {};
static mutex_t      s_telem_mtx;

/* ── Input capture edge buffer ───────────────────────────────────────────── */
/*
 * 21 edges per channel. Each entry is a raw TIM1 counter value at the edge.
 * Capture is interrupt-driven; s_edge_idx[c] counts edges received so far.
 */
static uint32_t s_edges[4][21];
static uint8_t  s_edge_idx[4];

/* ── Forward declarations ────────────────────────────────────────────────── */
static uint16_t make_frame(uint16_t thr);
static void     build_dma_buf(const uint16_t throttle[4]);
static void     switch_to_output(void);
static void     switch_to_input_capture(void);
static void     decode_all_channels(void);
static bool     decode_gcr(const uint32_t edges[21], uint32_t *erpm_out);

/* ChibiOS DMA stream handle — allocated in dshot_init(). */
static const stm32_dma_stream_t *s_dma6 = nullptr;

/* ═══════════════════════════════════════════════════════════════════════════
 * DMA TC callback — invoked by ChibiOS DMAv2 ISR after all burst slots done.
 * Registered via dmaStreamAlloc(); no OSAL_IRQ prologue/epilogue needed.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void dshot_dma_tc(void *param, uint32_t flags)
{
    (void)param; (void)flags;

    /* Stop TIM1. DMA stream was already disabled by ChibiOS ISR. */
    TIM1->CR1 &= ~TIM_CR1_CEN;

    /* Switch all 4 motor pins to floating input for ESC response capture. */
    switch_to_input_capture();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TIM1 capture-compare interrupt — records edge timestamps.
 * ═══════════════════════════════════════════════════════════════════════════ */
OSAL_IRQ_HANDLER(STM32_TIM1_CC_HANDLER)
{
    OSAL_IRQ_PROLOGUE();

    uint32_t sr = TIM1->SR;
    TIM1->SR = 0; // clear all CC flags

    /* Record capture for each active channel. */
    if ((sr & TIM_SR_CC1IF) && s_edge_idx[0] < 21)
        s_edges[0][s_edge_idx[0]++] = TIM1->CCR1;

    if ((sr & TIM_SR_CC2IF) && s_edge_idx[1] < 21)
        s_edges[1][s_edge_idx[1]++] = TIM1->CCR2;

    if ((sr & TIM_SR_CC3IF) && s_edge_idx[2] < 21)
        s_edges[2][s_edge_idx[2]++] = TIM1->CCR3;

    if ((sr & TIM_SR_CC4IF) && s_edge_idx[3] < 21)
        s_edges[3][s_edge_idx[3]++] = TIM1->CCR4;

    /* Once all 4 channels have 21 edges (or timer OVF fires), decode. */
    bool all_done = (s_edge_idx[0] >= 21) && (s_edge_idx[1] >= 21) &&
                    (s_edge_idx[2] >= 21) && (s_edge_idx[3] >= 21);
    if (all_done || (sr & TIM_SR_UIF)) {
        TIM1->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC2IE |
                        TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);
        TIM1->CR1  &= ~TIM_CR1_CEN;
        decode_all_channels();
        switch_to_output();
    }

    OSAL_IRQ_EPILOGUE();
}

/* ── make_frame ──────────────────────────────────────────────────────────── */
static uint16_t make_frame(uint16_t thr)
{
    /* Bidirectional mode: telemetry request bit = 0. */
    uint16_t val = (uint16_t)((thr << 1) | 0U);
    uint16_t crc = (val ^ (val >> 4) ^ (val >> 8)) & 0x0FU;
    return (uint16_t)((val << 4) | crc);
}

/* ── build_dma_buf ───────────────────────────────────────────────────────── */
static void build_dma_buf(const uint16_t throttle[4])
{
    uint16_t frame[4];
    for (int c = 0; c < 4; c++)
        frame[c] = make_frame(throttle[c]);

    /* Interleave: slot b (MSB first) → 4 CCR values. */
    for (int b = 15; b >= 0; b--) {
        int slot = 15 - b;
        for (int c = 0; c < 4; c++)
            s_dma_buf[slot * 4 + c] = (frame[c] & (1U << b)) ? DS_T1H : DS_T0H;
    }
    /* Inter-frame gap: CCR = 0 → output held low (inverted bidir = low = idle). */
    for (int c = 0; c < 4; c++)
        s_dma_buf[16 * 4 + c] = 0U;
}

/* ── switch_to_output ────────────────────────────────────────────────────── */
static void switch_to_output(void)
{
    /* PA8-PA10 → TIM1_CH1-3 (motors 1-3): AF1, push-pull, very-high speed.
     * MODER bits: pin8=16-17, pin9=18-19, pin10=20-21. */
    GPIOA->MODER   = (GPIOA->MODER
                      & ~(0x3U << 16) & ~(0x3U << 18) & ~(0x3U << 20))
                     | (0x2U << 16) | (0x2U << 18) | (0x2U << 20);
    GPIOA->OSPEEDR |= (0x3U << 16) | (0x3U << 18) | (0x3U << 20);
    GPIOA->PUPDR    = (GPIOA->PUPDR
                       & ~(0x3U << 16) & ~(0x3U << 18) & ~(0x3U << 20));
    /* AF1 for PA8-PA10 in AFRH (pin8=bits0-3, pin9=bits4-7, pin10=bits8-11). */
    GPIOA->AFRH = (GPIOA->AFRH & ~(0xFU << 0) & ~(0xFU << 4) & ~(0xFU << 8))
                | (0x1U << 0) | (0x1U << 4) | (0x1U << 8);

    /* PE14 → TIM1_CH4 (motor 4): AF1, push-pull, very-high speed.
     * PA11 is USB OTG_FS D- and must not be touched.
     * MODER bits: pin14=28-29; AFRH: pin14=bits24-27. */
    GPIOE->MODER   = (GPIOE->MODER & ~(0x3U << 28)) | (0x2U << 28);
    GPIOE->OSPEEDR |= (0x3U << 28);
    GPIOE->PUPDR    = (GPIOE->PUPDR & ~(0x3U << 28));
    GPIOE->AFRH     = (GPIOE->AFRH & ~(0xFU << 24)) | (0x1U << 24);

    /* TIM1 CH1-4 → PWM mode 2, preload enabled. */
    TIM1->CCMR1 = (6U << 4)  | TIM_CCMR1_OC1PE
                | (6U << 12) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6U << 4)  | TIM_CCMR2_OC3PE
                | (6U << 12) | TIM_CCMR2_OC4PE;
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E |
                  TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM1->ARR   = DS_ARR;
}

/* ── switch_to_input_capture ─────────────────────────────────────────────── */
static void switch_to_input_capture(void)
{
    /* Reset edge counters. */
    memset(s_edge_idx, 0, sizeof(s_edge_idx));

    /* PA8-PA10 (motors 1-3) → floating digital inputs (MODER=00).
     * TIM1 IC can sample the same pins without AF when MODER=00. */
    GPIOA->MODER  &= ~((0x3U << 16) | (0x3U << 18) | (0x3U << 20));
    GPIOA->PUPDR  &= ~((0x3U << 16) | (0x3U << 18) | (0x3U << 20));

    /* PE14 (motor 4) → floating digital input. */
    GPIOE->MODER  &= ~(0x3U << 28);
    GPIOE->PUPDR  &= ~(0x3U << 28);

    /*
     * TIM1 input capture: CH1-4 capture on both edges, no prescaler, no filter.
     * CCMR: CC1S=01 (IC1 mapped to TI1), OC1FE/OC1PE cleared.
     * CCER: CC1P=1, CC1NP=1 → capture on both edges.
     */
    TIM1->CCMR1 = (1U << 0) | (1U << 8);   // CC1S=TI1, CC2S=TI2
    TIM1->CCMR2 = (1U << 0) | (1U << 8);   // CC3S=TI3, CC4S=TI4
    TIM1->CCER  = (TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP)
                | (TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP)
                | (TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP)
                | (TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP);

    /*
     * Free-running counter for edge timestamp measurement.
     * Timeout via UIF: ~100 µs at 200 MHz → ARR ≈ 20000.
     */
    TIM1->ARR   = 20000U;
    TIM1->CNT   = 0U;
    TIM1->SR    = 0U;
    TIM1->DIER  = TIM_DIER_CC1IE | TIM_DIER_CC2IE |
                  TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE;
    TIM1->CR1  |= TIM_CR1_CEN;
}

/* ── decode_gcr ──────────────────────────────────────────────────────────── */
/*
 * Decode one channel's edge timestamps into an eRPM value.
 * Returns true on CRC pass; *erpm_out is set only on success.
 *
 * Edge width in timer ticks: one DShot600 bit = DS_ARR+1 = 334 ticks.
 * Each bit is a high→low or low→high transition; consecutive same-level
 * bits appear as a single wide edge (RLL encoding).
 */
static bool decode_gcr(const uint32_t edges[21], uint32_t *erpm_out)
{
    const uint32_t bit_ticks = DS_ARR + 1U; // 334 ticks per bit at 200 MHz

    /* Reconstruct the 21-bit GCR bitstream from edge widths. */
    uint32_t bits = 0U;
    int      bits_filled = 0;
    uint8_t  level = 1U; // ESC response line idles high

    for (int i = 0; i < 21 && bits_filled < 21; i++) {
        uint32_t width;
        if (i == 0) {
            width = edges[0]; // time from capture start to first edge
        } else {
            uint32_t prev = edges[i - 1];
            uint32_t cur  = edges[i];
            width = (cur >= prev) ? (cur - prev) : (20001U - prev + cur); // wrap
        }

        /* Number of bit periods this edge width spans (round to nearest). */
        uint32_t n = (width + bit_ticks / 2U) / bit_ticks;
        if (n == 0U) n = 1U;
        if (n > 21U - (uint32_t)bits_filled) n = (uint32_t)(21 - bits_filled);

        for (uint32_t k = 0; k < n && bits_filled < 21; k++) {
            bits = (bits << 1) | level;
            bits_filled++;
        }
        level ^= 1U; // toggle after each edge
    }

    if (bits_filled < 21)
        return false;

    /* Strip leading RLL bit → 20 GCR bits. */
    uint32_t gcr20 = bits & 0xFFFFFU;

    /* Decode 4 × 5-bit GCR groups → 4 nibbles → 16-bit frame. */
    uint16_t frame = 0U;
    for (int g = 3; g >= 0; g--) {
        uint8_t code   = (uint8_t)((gcr20 >> (g * 5)) & 0x1FU);
        uint8_t nibble = kGcrDecode[code];
        if (nibble == 0xFF) return false; // invalid GCR code
        frame = (uint16_t)((frame << 4) | nibble);
    }

    /* CRC check (same formula as command frame). */
    uint16_t val = frame >> 4;
    uint8_t  crc = (uint8_t)((val ^ (val >> 4) ^ (val >> 8)) & 0x0FU);
    if (crc != (frame & 0x0FU)) return false;

    /* Decode eRPM: [14:12] exponent, [11:3] mantissa (9 bits). */
    uint16_t exponent = (val >> 9) & 0x7U;
    uint16_t mantissa = (val >> 0) & 0x1FFU;
    *erpm_out = (uint32_t)mantissa << exponent;
    return true;
}

/* ── decode_all_channels ─────────────────────────────────────────────────── */
static void decode_all_channels(void)
{
    ESCTelemetry tmp[4];
    chMtxLock(&s_telem_mtx);
    memcpy(tmp, s_telem, sizeof(tmp));
    chMtxUnlock(&s_telem_mtx);

    for (int c = 0; c < 4; c++) {
        if (s_edge_idx[c] < 2) continue; // no response received
        uint32_t erpm = 0U;
        if (decode_gcr(s_edges[c], &erpm)) {
            tmp[c].erpm  = erpm;
            tmp[c].valid = true;
        } else {
            tmp[c].valid = false;
        }
    }

    chMtxLock(&s_telem_mtx);
    memcpy(s_telem, tmp, sizeof(tmp));
    chMtxUnlock(&s_telem_mtx);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void dshot_init(void)
{
    chMtxObjectInit(&s_telem_mtx);

    /* ── Enable clocks ─────────────────────────────────────────────────── */
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA2EN;
    RCC->AHB4ENR  |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOEEN; /* PA8-10 + PE14 */

    /* ── GPIO: PA8-PA11 → TIM1_CH1-4, AF1, push-pull, very-high speed ─── */
    switch_to_output(); // sets MODER/OSPEEDR/AFR for output

    /* ── TIM1 base configuration ────────────────────────────────────────── */
    TIM1->CR1   = 0U;
    TIM1->CR2   = 0U;
    TIM1->PSC   = 0U;              // no prescaler: 200 MHz tick
    TIM1->ARR   = DS_ARR;          // 334-tick period = 1.667 µs (DShot600)
    TIM1->RCR   = 0U;
    TIM1->BDTR  = TIM_BDTR_MOE;   // main output enable (required for TIM1)
    TIM1->EGR   = TIM_EGR_UG;     // load PSC/ARR shadow registers

    /* CCR registers initialised to idle (inverted: CCR=0 → output high = idle). */
    TIM1->CCR1  = 0U;
    TIM1->CCR2  = 0U;
    TIM1->CCR3  = 0U;
    TIM1->CCR4  = 0U;

    /* PWM mode 2 on CH1-4, preload enabled (set by switch_to_output). */

    /* Enable TIM1 DMA request on update event. */
    TIM1->DCR   = ((3U) << TIM_DCR_DBL_Pos)           // burst length = 4 registers
                | (((uint32_t)(&TIM1->CCR1) - (uint32_t)TIM1) >> 2) << TIM_DCR_DBA_Pos;
    TIM1->DIER  = TIM_DIER_UDE; // update DMA request enable

    /* ── DMA2 Stream 6 for TIM1_UP — allocated through ChibiOS DMA layer ── */
    s_dma6 = dmaStreamAlloc(STM32_DMA_STREAM_ID(2, 6), CORTEX_MINIMUM_PRIORITY,
                             dshot_dma_tc, nullptr);
    osalDbgAssert(s_dma6 != nullptr, "DMA2 stream 6 unavailable");

    /* Route TIM1_UP (request ID 11 on STM32H7) via DMAMUX. */
    dmaSetRequestSource(s_dma6, 11U);

    dmaStreamSetPeripheral(s_dma6, &TIM1->DMAR);
    dmaStreamSetMemory0(s_dma6, s_dma_buf);
    dmaStreamSetTransactionSize(s_dma6, 68U);              // 17 slots × 4 channels
    s_dma6->stream->FCR = DMA_SxFCR_DMDIS;                // direct mode (no FIFO)
    dmaStreamSetMode(s_dma6,
        DMA_SxCR_MINC                          // memory increment
        | (0x2U << DMA_SxCR_MSIZE_Pos)         // 32-bit memory
        | (0x2U << DMA_SxCR_PSIZE_Pos)         // 32-bit peripheral
        | (0x1U << DMA_SxCR_DIR_Pos)           // memory → peripheral
        | DMA_SxCR_TCIE);                      // TC interrupt → calls dshot_dma_tc

    /* Enable TIM1 CC interrupt for bidirectional capture. */
    nvicEnableVector(TIM1_CC_IRQn, CORTEX_MINIMUM_PRIORITY);
}

void dshot_write(const uint16_t throttle[4])
{
    build_dma_buf(throttle);

    /* Reset TIM1 and reload DMA stream. */
    TIM1->CR1   &= ~TIM_CR1_CEN;
    TIM1->CNT    = 0U;
    TIM1->SR     = 0U;

    dmaStreamDisable(s_dma6);
    dmaStreamSetMemory0(s_dma6, s_dma_buf);
    dmaStreamSetTransactionSize(s_dma6, 68U);
    dmaStreamEnable(s_dma6);

    /* Start TIM1 — DMA will feed CCR1-4 on each update event. */
    TIM1->CR1 |= TIM_CR1_CEN;
}

void dshot_get_telemetry(ESCTelemetry out[4])
{
    chMtxLock(&s_telem_mtx);
    memcpy(out, s_telem, 4 * sizeof(ESCTelemetry));
    chMtxUnlock(&s_telem_mtx);
}
