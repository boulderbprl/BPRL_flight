#include "src/coms/DShot.hpp"
#include "ch.h"
#include "hal.h"
#include <cstring>

/*
 * DShot600 bidirectional — single-stream burst DMA TX + dedicated IC, matching ArduPilot.
 *
 * Motor pin mapping (CubePilot standard carrier, AUX OUT):
 *   Motor 0 (FR) → PE11 = TIM1_CH2  (AUX 3, AF1)
 *   Motor 1 (RL) → PE9  = TIM1_CH1  (AUX 4, AF1)
 *   Motor 2 (FL) → PD13 = TIM4_CH2  (AUX 5, AF2)
 *   Motor 3 (RR) → PE13 = TIM1_CH3  (AUX 2, AF1)
 *
 * ── TIM1 TX burst mechanism ──────────────────────────────────────────────────
 *
 *   One DMA stream (s_dma_tx_tim1) writes to TIM1->DMAR via DCR burst mode:
 *     DCR: DBA=CCR1(13), DBL=3 (burst of 4: CCR1, CCR2, CCR3, CCR4)
 *
 *   Buffer: s_tx_buf_tim1[72] — interleaved stride-4 layout:
 *     [CCR1_b15, CCR2_b15, CCR3_b15, 0,
 *      CCR1_b14, CCR2_b14, CCR3_b14, 0,
 *      ...
 *      0, 0, 0, 0,   ← trailing zero row 1
 *      0, 0, 0, 0]   ← trailing zero row 2
 *   (CCR4 column is always 0 — CH4 is not used and CC4E is never set.)
 *
 *   One TIM1_UP event → FIFO (full threshold, 4 words) drains 4 words to DMAR.
 *   Timer routes these 4 DMAR writes to CCR1→CCR2→CCR3→CCR4, all within one UEV.
 *   CCR1/CCR2/CCR3 are thus updated simultaneously on the same UEV (~5 ns apart).
 *   72 words / 4 words per UEV = 18 UEVs = 16 data bits + 2 trailing zeros. ✓
 *
 *   This matches ArduPilot's send_pulses_DMAR exactly (ArduPilot uses DBL=3 for
 *   4 active channels; BPRL uses DBL=3 with CCR4 as a dummy padding column).
 *
 * ── DMA stream allocation ────────────────────────────────────────────────────
 *
 *   s_dma_tx_tim1 — TIM1 TX burst.
 *     DMAMUX=TIM1_UP(15), M→P, 72 words → &TIM1->DMAR, FIFO full threshold.
 *
 *   s_dma_ic_tim1 — TIM1 IC (dedicated, separate from TX stream).
 *     Slot 0 (M0/PE11): DMAMUX=TIM1_CH2(12), P→M, CC2S=TI2, DBA=CCR2, DBL=0
 *     Slot 1 (M1/PE9):  DMAMUX=TIM1_CH2(12), P→M, CC2S=TI1 (cross), DBA=CCR2
 *     Slot 2 (M3/PE13): DMAMUX=TIM1_CH3(13), P→M, CC3S=TI3, DBA=CCR3, DBL=0
 *
 *   s_dma_tim4 — TIM4 TX + IC (shared, one motor, no rotation).
 *     TX: DMAMUX=TIM4_UP(32), M→P, 18 words → &TIM4->DMAR (DCR: DBA=CCR2, DBL=0)
 *     IC: DMAMUX=TIM4_CH2(30), P→M, 22 words → &TIM4->DMAR (DCR unchanged)
 *
 * ── ArduPilot CC2 cross-capture trick for M1 ─────────────────────────────────
 *
 *   STM32 CCMR CC2S field selects which signal feeds the CC2 capture register:
 *     0b01 = TI2 → captures PE11 (CH2's own pin) — used for M0 (slot 0)
 *     0b10 = TI1 → captures PE9  (CH1's pin via cross-input) — used for M1 (slot 1)
 *   Both generate CC2 events; DMA reads from DMAR which maps to CCR2 via DCR.
 *   CC1 and CC1DE are never used; this matches ArduPilot exactly.
 *
 * ── Telemetry rates (400 Hz DShot frame rate) ────────────────────────────────
 *   M0 FR (PE11): ~133 Hz  (TIM1 IC rotation slot 0 of 3)
 *   M1 RL (PE9):  ~133 Hz  (TIM1 IC rotation slot 1 of 3)
 *   M3 RR (PE13): ~133 Hz  (TIM1 IC rotation slot 2 of 3)
 *   M2 FL (PD13):  400 Hz  (TIM4, every frame)
 */

/* ── DShot600 timing (200 MHz timer clock, PSC = 0) ─────────────────────── */
static constexpr uint32_t DS_ARR         = 333U;
static constexpr uint32_t DS_T0H         = 125U;
static constexpr uint32_t DS_T1H         = 250U;
static constexpr uint32_t GCR_BIT_TICKS  = (DS_ARR + 1U) * 4U / 5U; // 267

/* ── DMAMUX1 request IDs (stm32_dmamux.h, STM32H7xx) ────────────────────── */
static constexpr uint32_t DMAMUX_TIM1_CH2 = 12U;
static constexpr uint32_t DMAMUX_TIM1_CH3 = 13U;
static constexpr uint32_t DMAMUX_TIM1_UP  = 15U;
static constexpr uint32_t DMAMUX_TIM4_CH2 = 30U;
static constexpr uint32_t DMAMUX_TIM4_UP  = 32U;

/* ── Timer DCR word offsets (DBA = byte_offset / 4) ─────────────────────── */
static constexpr uint32_t DBA_CCR1 = 13U;  // 0x34/4
static constexpr uint32_t DBA_CCR2 = 14U;  // 0x38/4
static constexpr uint32_t DBA_CCR3 = 15U;  // 0x3C/4

/* ── GCR decode table ────────────────────────────────────────────────────── */
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

/* ── DMA TX buffers (.nocache — bypasses D-cache on STM32H743) ───────────── */
/* Interleaved stride-4: [CCR1, CCR2, CCR3, CCR4_dummy] × 19 rows = 76 words.
 * Row 0:    pre-word zeros  (dshot_pre=1, keeps line idle for one extra UEV)
 * Rows 1–16: data bits 15..0
 * Rows 17–18: trailing zeros (dshot_post=2)
 * One TIM1_UP drains 4 words from FIFO → all four CCRs updated per UEV.
 * With OC preload (OC1PE=1) the DMA write on UEVn takes effect on UEVn+1,
 * so the pre-word absorbs this one-UEV delay — matches ArduPilot exactly. */
static uint32_t s_tx_buf_tim1[76] __attribute__((aligned(32), section(".nocache")));
static uint32_t s_tx_buf_tim4[19] __attribute__((aligned(32), section(".nocache")));

/* ── DMA IC buffer (single buffer reused each rotation) ─────────────────── */
static uint32_t s_ic_buf_tim1[22] __attribute__((aligned(32), section(".nocache")));
static uint32_t s_ic_buf_tim4[22] __attribute__((aligned(32), section(".nocache")));

/* ── Phase state machine ─────────────────────────────────────────────────── */
enum class DshotPhase : uint8_t { IDLE, TX, IC };
static volatile DshotPhase s_phase_tim1 = DshotPhase::IDLE;
static volatile DshotPhase s_phase_tim4 = DshotPhase::IDLE;

/* ── TIM1 IC rotation ────────────────────────────────────────────────────── */
/*
 * Slot 0 → M0 (CH2/PE11): CC2S=TI2 direct,  DMAMUX=TIM1_CH2, DBA=CCR2
 * Slot 1 → M1 (CH1/PE9):  CC2S=TI1 cross,   DMAMUX=TIM1_CH2, DBA=CCR2
 * Slot 2 → M3 (CH3/PE13): CC3S=TI3 direct,  DMAMUX=TIM1_CH3, DBA=CCR3
 */
static uint8_t s_telem_slot = 0U;

static constexpr uint32_t kIcDmamux[3]  = { DMAMUX_TIM1_CH2, DMAMUX_TIM1_CH2, DMAMUX_TIM1_CH3 };
static constexpr uint32_t kIcDba[3]     = { DBA_CCR2, DBA_CCR2, DBA_CCR3 };
static constexpr int       kSlotMotor[3] = { 0, 1, 3 };

/* CCMR1 CC2S field (bits [9:8]): TI2=direct(PE11), TI1=cross(PE9) */
static constexpr uint32_t kCC2S_TI2 = (1U << 8);  // 0b01 = CH2's own pin
static constexpr uint32_t kCC2S_TI1 = (2U << 8);  // 0b10 = CH1's pin (cross)
static constexpr uint32_t kIC2F     = (2U << 12); // IC2F = 4-sample noise filter

/* ── Telemetry & diagnostics ─────────────────────────────────────────────── */
static ESCTelemetry      s_telem[4]           = {};
static volatile uint32_t s_dma_tc_count[2]   = {};  // TX TC per timer
static volatile uint32_t s_ic_done_count[2]  = {};  // IC complete per timer
static uint8_t  s_last_edge_cnt[4]           = {};
static uint32_t s_last_edges[4][5]           = {};

/* ── DMA stream handles ──────────────────────────────────────────────────── */
static const stm32_dma_stream_t *s_dma_tx_tim1  = nullptr;  // TIM1 TX burst → DMAR
static const stm32_dma_stream_t *s_dma_ic_tim1  = nullptr;  // TIM1 IC capture
static const stm32_dma_stream_t *s_dma_tim4     = nullptr;  // TIM4 TX + IC

/* ── Forward declarations ────────────────────────────────────────────────── */
static uint16_t make_frame(uint16_t thr);
static void     build_tx_buf(const uint16_t throttle[4]);
static void     reset_pwm_for_tx(void);
static void     start_ic_tim1(void);
static void     start_ic_tim4(void);
static void     finish_ic_tim1(uint8_t n_edges);
static void     finish_ic_tim4(uint8_t n_edges);
static bool     decode_gcr(const uint32_t *buf, uint8_t n_edges, uint32_t *erpm_out);

/* ═══════════════════════════════════════════════════════════════════════════
 * DMA TC callbacks (priority 7 — fires before any OS tick can delay it)
 * ═══════════════════════════════════════════════════════════════════════════ */
/* TX complete (fires on s_dma_tx_tim1 TC after all 72 burst words transferred). */
static void dshot_dma_tc_tx_tim1(void *p, uint32_t flags)
{
    (void)p; (void)flags;
    if (s_phase_tim1 != DshotPhase::TX) return;
    s_dma_tc_count[0]++;
    TIM1->CR1  &= ~TIM_CR1_CEN;
    s_phase_tim1 = DshotPhase::IC;
    start_ic_tim1();
}

/* IC complete (fires on s_dma_ic_tim1 TC — all 22 edges captured). */
static void dshot_dma_tc_ic_tim1(void *p, uint32_t flags)
{
    (void)p; (void)flags;
    if (s_phase_tim1 != DshotPhase::IC) return;
    TIM1->DIER  = 0U;
    TIM1->CR1  &= ~TIM_CR1_CEN;
    s_phase_tim1 = DshotPhase::IDLE;
    s_ic_done_count[0]++;
    finish_ic_tim1(22U);
}

static void dshot_dma_tc_tim4(void *p, uint32_t flags)
{
    (void)p; (void)flags;
    if (s_phase_tim4 == DshotPhase::TX) {
        s_dma_tc_count[1]++;
        TIM4->CR1  &= ~TIM_CR1_CEN;
        s_phase_tim4 = DshotPhase::IC;
        start_ic_tim4();
    } else if (s_phase_tim4 == DshotPhase::IC) {
        TIM4->DIER  = 0U;
        TIM4->CR1  &= ~TIM_CR1_CEN;
        s_phase_tim4 = DshotPhase::IDLE;
        s_ic_done_count[1]++;
        finish_ic_tim4(22U);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TIM1_UP ISR — UIE timeout for TIM1 IC (100 µs fallback).
 * ═══════════════════════════════════════════════════════════════════════════ */
OSAL_IRQ_HANDLER(STM32_TIM1_UP_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    uint32_t sr = TIM1->SR;
    TIM1->SR = 0U;

    if ((sr & TIM_SR_UIF) && (s_phase_tim1 == DshotPhase::IC)) {
        TIM1->DIER = 0U;
        TIM1->CR1  &= ~TIM_CR1_CEN;
        dmaStreamDisable(s_dma_ic_tim1);
        uint8_t captured = (uint8_t)(22U - dmaStreamGetTransactionSize(s_dma_ic_tim1));
        s_phase_tim1 = DshotPhase::IDLE;
        s_ic_done_count[0]++;
        finish_ic_tim1(captured);
    }
    OSAL_IRQ_EPILOGUE();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TIM1_CC ISR — safety only; CCxIE is never set in DMA-IC mode.
 * ═══════════════════════════════════════════════════════════════════════════ */
OSAL_IRQ_HANDLER(STM32_TIM1_CC_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    TIM1->SR = 0U;
    OSAL_IRQ_EPILOGUE();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TIM4 ISR — UIE timeout for TIM4 IC.
 * ═══════════════════════════════════════════════════════════════════════════ */
OSAL_IRQ_HANDLER(STM32_TIM4_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    uint32_t sr = TIM4->SR;
    TIM4->SR = 0U;

    if ((sr & TIM_SR_UIF) && (s_phase_tim4 == DshotPhase::IC)) {
        TIM4->DIER = 0U;
        TIM4->CR1  &= ~TIM_CR1_CEN;
        dmaStreamDisable(s_dma_tim4);
        uint8_t captured = (uint8_t)(22U - dmaStreamGetTransactionSize(s_dma_tim4));
        s_phase_tim4 = DshotPhase::IDLE;
        s_ic_done_count[1]++;
        finish_ic_tim4(captured);
    }
    OSAL_IRQ_EPILOGUE();
}

/* ── make_frame ──────────────────────────────────────────────────────────── */
static uint16_t make_frame(uint16_t thr)
{
    uint16_t val = (uint16_t)(thr << 1);                        // telem bit = 0
    uint16_t crc = (~(val ^ (val >> 4) ^ (val >> 8))) & 0x0FU; // inverted → bidir
    return (uint16_t)((val << 4) | crc);
}

/* ── build_tx_buf ────────────────────────────────────────────────────────── */
static void build_tx_buf(const uint16_t throttle[4])
{
    uint16_t frame[4];
    for (int c = 0; c < 4; c++) frame[c] = make_frame(throttle[c]);

    /* Row 0: pre-word zeros — idle period before bit 15 (dshot_pre=1).
     * With OC preload, the DMA write on UEVn is shadowed until UEVn+1.
     * This pre-row absorbs that one-UEV delay so bit 15 fires on UEV3,
     * matching ArduPilot's fill_DMA_buffer_dshot() layout exactly. */
    s_tx_buf_tim1[0] = 0U; s_tx_buf_tim1[1] = 0U;
    s_tx_buf_tim1[2] = 0U; s_tx_buf_tim1[3] = 0U;

    /* Rows 1–16: data bits 15..0, interleaved [CCR1=M1, CCR2=M0, CCR3=M3, 0]. */
    for (int b = 15; b >= 0; b--) {
        int row = (16 - b) * 4;
        s_tx_buf_tim1[row + 0] = (frame[1] & (1U << b)) ? DS_T1H : DS_T0H;
        s_tx_buf_tim1[row + 1] = (frame[0] & (1U << b)) ? DS_T1H : DS_T0H;
        s_tx_buf_tim1[row + 2] = (frame[3] & (1U << b)) ? DS_T1H : DS_T0H;
        s_tx_buf_tim1[row + 3] = 0U;
    }
    /* Rows 17–18: trailing zeros. */
    for (int i = 68; i < 76; i++) s_tx_buf_tim1[i] = 0U;

    /* TIM4: pre-word, 16 data bits, 2 trailing zeros (19 words). */
    s_tx_buf_tim4[0] = 0U;
    for (int b = 15; b >= 0; b--)
        s_tx_buf_tim4[16 - b] = (frame[2] & (1U << b)) ? DS_T1H : DS_T0H;
    s_tx_buf_tim4[17] = 0U;
    s_tx_buf_tim4[18] = 0U;
}

/* ── reset_pwm_for_tx ────────────────────────────────────────────────────── */
/*
 * Equivalent to ArduPilot bdshot_reset_pwm() = pwmStop() + pwmStart().
 * rccResetTIMx() performs a full APB peripheral reset (all regs → 0),
 * then we replicate the exact register writes ChibiOS pwmStart() makes
 * for PWM_OUTPUT_ACTIVE_LOW (OCxM=6, CCxP=1) with preload.
 * After this function both timers are running; DMA is armed in dshot_write().
 * DCR is written here (driver does not touch it).
 */
static void reset_pwm_for_tx(void)
{
    rccResetTIM1();
    TIM1->PSC   = 0U;
    TIM1->ARR   = DS_ARR;
    TIM1->CR2   = 0U;
    TIM1->CCMR1 = STM32_TIM_CCMR1_OC1M(6) | TIM_CCMR1_OC1PE
                | STM32_TIM_CCMR1_OC2M(6) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = STM32_TIM_CCMR2_OC3M(6) | TIM_CCMR2_OC3PE
                | STM32_TIM_CCMR2_OC4M(6) | TIM_CCMR2_OC4PE;
    TIM1->CCR1  = 0U; TIM1->CCR2 = 0U; TIM1->CCR3 = 0U; TIM1->CCR4 = 0U;
    TIM1->CCER  = STM32_TIM_CCER_CC1P | STM32_TIM_CCER_CC1E
                | STM32_TIM_CCER_CC2P | STM32_TIM_CCER_CC2E
                | STM32_TIM_CCER_CC3P | STM32_TIM_CCER_CC3E;
    TIM1->EGR   = TIM_EGR_UG;
    TIM1->SR    = 0U;
    TIM1->DIER  = TIM_DIER_UDE;
    TIM1->BDTR  = TIM_BDTR_MOE;
    TIM1->DCR   = (3U << TIM_DCR_DBL_Pos) | (DBA_CCR1 << TIM_DCR_DBA_Pos);
    TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;

    rccResetTIM4();
    TIM4->PSC   = 0U;
    TIM4->ARR   = DS_ARR;
    TIM4->CR2   = 0U;
    TIM4->CCMR1 = STM32_TIM_CCMR1_OC2M(6) | TIM_CCMR1_OC2PE;
    TIM4->CCR2  = 0U;
    TIM4->CCER  = STM32_TIM_CCER_CC2P | STM32_TIM_CCER_CC2E;
    TIM4->EGR   = TIM_EGR_UG;
    TIM4->SR    = 0U;
    TIM4->DIER  = TIM_DIER_UDE;
    TIM4->DCR   = (0U << TIM_DCR_DBL_Pos) | (DBA_CCR2 << TIM_DCR_DBA_Pos);
    TIM4->CNT   = DS_ARR / 2U;   // half-period offset → interleaves DMA bursts
    TIM4->SR    = 0U;
    TIM4->CR1   = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
}

/* ── start_ic_tim1 ───────────────────────────────────────────────────────── */
/*
 * Configure s_dma_ic_tim1 for the current rotation slot and start IC capture.
 * Called from dshot_dma_tc_tx_tim1 (first TX TC, priority 7) after TIM1 is stopped.
 *
 * TIM1 is fully stopped before this is called (CR1 &= ~CEN in the TC callback),
 * so DCR can be safely changed and no UDE requests are in-flight.
 */
static void start_ic_tim1(void)
{
    uint8_t slot = s_telem_slot;

    /* Reconfigure the dedicated IC stream: P→M, DMAMUX = CCx DMA request. */
    dmaSetRequestSource(s_dma_ic_tim1, kIcDmamux[slot]);
    dmaStreamSetPeripheral(s_dma_ic_tim1, &TIM1->DMAR);
    dmaStreamSetMemory0(s_dma_ic_tim1, s_ic_buf_tim1);
    dmaStreamSetTransactionSize(s_dma_ic_tim1, 22U);
    s_dma_ic_tim1->stream->FCR = 0U;   // direct mode
    dmaStreamSetMode(s_dma_ic_tim1,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x0U << DMA_SxCR_DIR_Pos)   // P→M
        | (0x3U << DMA_SxCR_PL_Pos)
        | DMA_SxCR_TCIE);

    /* Change DCR to point at the single capture register for this slot.
     * Safe to do now: timer is stopped and TX DMA is idle. */
    TIM1->DCR = (0U << TIM_DCR_DBL_Pos) | (kIcDba[slot] << TIM_DCR_DBA_Pos);

    /* Switch the selected channel to IC mode; disable outputs on others.
     * CCER must be cleared before writing CCxS (hardware restriction). */
    TIM1->CCER  = 0U;
    TIM1->CCMR1 = 0U;
    TIM1->CCMR2 = 0U;

    if (slot <= 1U) {
        /* Slots 0 and 1 use CC2: CC2S selects which pin feeds CC2.
         * Slot 0 (M0/PE11): CC2S=TI2 (CH2's own input, direct).
         * Slot 1 (M1/PE9):  CC2S=TI1 (cross-input from CH1/PE9). */
        uint32_t cc2s = (slot == 0U) ? kCC2S_TI2 : kCC2S_TI1;
        TIM1->CCMR1 = cc2s | kIC2F;
        TIM1->CCER  = TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP;
        TIM1->DIER  = TIM_DIER_CC2DE | TIM_DIER_UIE;
    } else {
        /* Slot 2 (M3/PE13): CC3S=TI3 direct. */
        TIM1->CCMR2 = (1U << 0) | (2U << 4);  // CC3S=TI3, IC3F=4-sample
        TIM1->CCER  = TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP;
        TIM1->DIER  = TIM_DIER_CC3DE | TIM_DIER_UIE;
    }

    TIM1->ARR = 20000U;   // 100 µs hardware timeout
    TIM1->CNT = 0U;
    TIM1->SR  = 0U;

    dmaStreamEnable(s_dma_ic_tim1);
    /* URS=1: only counter overflow (not software EGR=UG) sets UIF, so no spurious
     * UIF if EGR is called elsewhere.  UDIS must NOT be set — BPRL uses the
     * counter-overflow UIE as a 100 µs hardware timeout for the IC phase. */
    TIM1->CR1 = TIM_CR1_URS | TIM_CR1_CEN;
}

/* ── start_ic_tim4 ───────────────────────────────────────────────────────── */
static void start_ic_tim4(void)
{
    dmaSetRequestSource(s_dma_tim4, DMAMUX_TIM4_CH2);
    dmaStreamSetPeripheral(s_dma_tim4, &TIM4->DMAR);
    dmaStreamSetMemory0(s_dma_tim4, s_ic_buf_tim4);
    dmaStreamSetTransactionSize(s_dma_tim4, 22U);
    s_dma_tim4->stream->FCR = 0U;
    dmaStreamSetMode(s_dma_tim4,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x0U << DMA_SxCR_DIR_Pos)
        | (0x3U << DMA_SxCR_PL_Pos)
        | DMA_SxCR_TCIE);

    /* TIM4 DCR: DBA=CCR2, DBL=0 — same as TX, no change needed. */

    TIM4->CCER  = 0U;
    TIM4->CCMR1 = kCC2S_TI2 | kIC2F;  // CC2S=TI2 (PD13 direct), IC2F=4-sample
    TIM4->CCER  = TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP;
    TIM4->ARR   = 20000U;
    TIM4->CNT   = 0U;
    TIM4->SR    = 0U;
    TIM4->DIER  = TIM_DIER_CC2DE | TIM_DIER_UIE;

    dmaStreamEnable(s_dma_tim4);
    TIM4->CR1 = TIM_CR1_URS | TIM_CR1_CEN;
}

/* ── finish_ic_tim1 ──────────────────────────────────────────────────────── */
static void finish_ic_tim1(uint8_t n_edges)
{
    int motor = kSlotMotor[s_telem_slot];

    s_last_edge_cnt[motor] = n_edges;
    uint8_t keep = (n_edges < 5U) ? n_edges : 5U;
    for (uint8_t k = 0; k < keep; k++) s_last_edges[motor][k] = s_ic_buf_tim1[k];

    uint32_t erpm = 0U;
    if (decode_gcr(s_ic_buf_tim1, n_edges, &erpm)) {
        s_telem[motor].erpm  = erpm;
        s_telem[motor].valid = true;
    } else {
        s_telem[motor].valid = false;
    }

    /* Advance rotation 0→1→2→0. */
    if (++s_telem_slot >= 3U) s_telem_slot = 0U;
}

/* ── finish_ic_tim4 ──────────────────────────────────────────────────────── */
static void finish_ic_tim4(uint8_t n_edges)
{
    s_last_edge_cnt[2] = n_edges;
    uint8_t keep = (n_edges < 5U) ? n_edges : 5U;
    for (uint8_t k = 0; k < keep; k++) s_last_edges[2][k] = s_ic_buf_tim4[k];

    uint32_t erpm = 0U;
    if (decode_gcr(s_ic_buf_tim4, n_edges, &erpm)) {
        s_telem[2].erpm  = erpm;
        s_telem[2].valid = true;
    } else {
        s_telem[2].valid = false;
    }
}

/* ── decode_gcr ──────────────────────────────────────────────────────────── */
/*
 * Decode 21-bit GCR from CCR timestamps captured by DMA.
 * Matches ArduPilot/BetaFlight bdshot_decode_telemetry_packet exactly.
 *
 * GCR encoding: a run of n consecutive identical bits is represented as a
 * '1' at the MSB followed by (n-1) '0's.  This is the transition-marking
 * convention — it is level-independent (no need to track HIGH vs LOW).
 *   n=1 → 0b1         n=2 → 0b10        n=3 → 0b100
 * Building the 21-bit word: for each inter-edge interval of n bit-widths:
 *   bits = (bits << n) | (1 << (n-1))
 *
 * CRC: the ESC response uses the INVERTED nibble sum (same as the bidir TX
 * frame from the FC).  The 4 nibbles of the decoded 20-bit frame satisfy
 *   n0 ^ n1 ^ n2 ^ n3 == 0xF
 * which is equivalent to (frame ^ frame>>4 ^ frame>>8 ^ frame>>12) & 0xF == 0xF.
 */
static bool decode_gcr(const uint32_t *buf, uint8_t n_edges, uint32_t *erpm_out)
{
    if (n_edges < 2U) return false;

    uint32_t bits        = 0U;
    uint32_t bits_filled = 0U;

    for (uint8_t i = 0U; i < n_edges && bits_filled < 21U; i++) {
        uint32_t n;
        if (i + 1U < n_edges) {
            uint32_t diff = buf[i + 1U] - buf[i];
            n = (diff + GCR_BIT_TICKS / 2U) / GCR_BIT_TICKS;
            if (n == 0U) n = 1U;
        } else {
            n = 21U - bits_filled;
        }
        if (n > 21U - bits_filled) n = 21U - bits_filled;

        /* Transition-marking: '1' at MSB of the run, '0' below it. */
        bits = (bits << n) | (1U << (n - 1U));
        bits_filled += n;
    }

    if (bits_filled < 21U) return false;

    /* Decode 4 × 5-bit GCR codes → 4-bit nibbles → 20-bit value. */
    uint32_t gcr20 = bits & 0xFFFFFU;
    uint16_t frame = 0U;
    for (int g = 3; g >= 0; g--) {
        uint8_t code   = (uint8_t)((gcr20 >> (g * 5U)) & 0x1FU);
        uint8_t nibble = kGcrDecode[code];
        if (nibble == 0xFF) return false;
        frame = (uint16_t)((frame << 4U) | nibble);
    }

    /* Inverted nibble CRC: the XOR of all four nibbles must equal 0xF. */
    uint16_t csum = frame ^ (frame >> 4U) ^ (frame >> 8U) ^ (frame >> 12U);
    if ((csum & 0x0FU) != 0x0FU) return false;

    /* eRPM: [15:12]=nibble0 is CRC, [11:9]=3-bit exponent, [8:0]=9-bit mantissa.
     * The encoded value is the electrical revolution PERIOD in microseconds.
     * eRPM = 60,000,000 / period_us  (same as ArduPilot's conversion). */
    uint16_t val      = (frame >> 4U) & 0xFFFU;
    uint16_t exponent = (val >> 9U) & 0x7U;
    uint16_t mantissa =  val         & 0x1FFU;
    uint32_t period   = (uint32_t)mantissa << exponent;
    *erpm_out = (period > 0U) ? (60000000U / period) : 0U;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void dshot_init(void)
{
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
    RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
    RCC->AHB4ENR  |= RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIODEN;

    /* ── GPIO: AF mode once, never changed ───────────────────────────────── */
    /* PE9=TIM1_CH1, PE11=TIM1_CH2, PE13=TIM1_CH3 → AF1, medium speed, pull-up
     * BiDir DShot requires medium speed (not high/very-high) to avoid ringing
     * on the IC→OC output reconnect transition — matches ArduPilot bdshot. */
    GPIOE->MODER   = (GPIOE->MODER & ~(0x3U<<18) & ~(0x3U<<22) & ~(0x3U<<26))
                   | (0x2U<<18) | (0x2U<<22) | (0x2U<<26);
    GPIOE->OSPEEDR = (GPIOE->OSPEEDR & ~(0x3U<<18) & ~(0x3U<<22) & ~(0x3U<<26))
                   | (0x1U<<18) | (0x1U<<22) | (0x1U<<26);
    GPIOE->PUPDR   = (GPIOE->PUPDR & ~(0x3U<<18) & ~(0x3U<<22) & ~(0x3U<<26))
                   | (0x1U<<18) | (0x1U<<22) | (0x1U<<26);
    GPIOE->AFRH    = (GPIOE->AFRH & ~(0xFU<<4) & ~(0xFU<<12) & ~(0xFU<<20))
                   | (0x1U<<4) | (0x1U<<12) | (0x1U<<20);

    /* PD13=TIM4_CH2 → AF2, medium speed, pull-up */
    GPIOD->MODER   = (GPIOD->MODER & ~(0x3U<<26)) | (0x2U<<26);
    GPIOD->OSPEEDR = (GPIOD->OSPEEDR & ~(0x3U<<26)) | (0x1U<<26);
    GPIOD->PUPDR   = (GPIOD->PUPDR & ~(0x3U<<26)) | (0x1U<<26);
    GPIOD->AFRH    = (GPIOD->AFRH & ~(0xFU<<20)) | (0x2U<<20);

    /* ── TIM1 and TIM4 — replicates pwmStart() register sequence exactly ─────
     * rccResetTIM1/4 performs a full APB peripheral reset (all registers → 0),
     * equivalent to pwmStop()+pwmStart()'s rccResetTIMx call in ChibiOS.
     * Subsequent register writes match ArduPilot set_group_mode(active_high=false):
     *   CCMR: OCxM=6 (PWM mode 1), OC preload   → active-low DShot output
     *   CCER: CCxP=1 (inverted), CCxE=1          → pin HIGH when CCR=0 (idle)
     *   DIER: UDE                                 → DMA on update event
     *   BDTR: MOE                                 → TIM1 main output enable
     *   CR1:  ARPE|URS|CEN                        → matches ArduPilot CR1
     * DCR is not touched by ArduPilot's pwmStart(); set separately. */
    rccResetTIM1();
    TIM1->PSC   = 0U;
    TIM1->ARR   = DS_ARR;
    TIM1->CR2   = 0U;
    TIM1->CCMR1 = STM32_TIM_CCMR1_OC1M(6) | TIM_CCMR1_OC1PE
                | STM32_TIM_CCMR1_OC2M(6) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = STM32_TIM_CCMR2_OC3M(6) | TIM_CCMR2_OC3PE
                | STM32_TIM_CCMR2_OC4M(6) | TIM_CCMR2_OC4PE;
    TIM1->CCR1  = 0U; TIM1->CCR2 = 0U; TIM1->CCR3 = 0U; TIM1->CCR4 = 0U;
    TIM1->CCER  = STM32_TIM_CCER_CC1P | STM32_TIM_CCER_CC1E
                | STM32_TIM_CCER_CC2P | STM32_TIM_CCER_CC2E
                | STM32_TIM_CCER_CC3P | STM32_TIM_CCER_CC3E;
    TIM1->EGR   = TIM_EGR_UG;
    TIM1->SR    = 0U;
    TIM1->DIER  = TIM_DIER_UDE;
    TIM1->BDTR  = TIM_BDTR_MOE;
    TIM1->DCR   = (3U << TIM_DCR_DBL_Pos) | (DBA_CCR1 << TIM_DCR_DBA_Pos);
    TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;

    rccResetTIM4();
    TIM4->PSC   = 0U;
    TIM4->ARR   = DS_ARR;
    TIM4->CR2   = 0U;
    TIM4->CCMR1 = STM32_TIM_CCMR1_OC2M(6) | TIM_CCMR1_OC2PE;
    TIM4->CCR2  = 0U;
    TIM4->CCER  = STM32_TIM_CCER_CC2P | STM32_TIM_CCER_CC2E;
    TIM4->EGR   = TIM_EGR_UG;
    TIM4->SR    = 0U;
    TIM4->DIER  = TIM_DIER_UDE;
    TIM4->DCR   = (0U << TIM_DCR_DBL_Pos) | (DBA_CCR2 << TIM_DCR_DBA_Pos);
    /* Phase offset: TIM4 UEV fires ~0.84µs before TIM1 (interleaves DMA bursts). */
    TIM4->CNT   = DS_ARR / 2U;
    TIM4->SR    = 0U;
    TIM4->CR1   = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;

    /* ── DMA streams for TIM1: one TX burst stream + one IC stream ──────── */
    s_dma_tx_tim1 = dmaStreamAlloc(STM32_DMA_STREAM_ID_ANY,
                                    STM32_IRQ_TIM1_CC_PRIORITY,
                                    dshot_dma_tc_tx_tim1, nullptr);
    osalDbgAssert(s_dma_tx_tim1 != nullptr, "DMA alloc TIM1 TX");
    if (s_dma_tx_tim1 == nullptr) {
        for (;;) {
            for (int i = 0; i < 20; i++) {
                palToggleLine(LINE_LED_ACTIVITY);
                volatile uint32_t n = 2000000U; while (n--) {}
            }
            volatile uint32_t n = 16000000U; while (n--) {}
        }
    }
    s_dma_ic_tim1 = dmaStreamAlloc(STM32_DMA_STREAM_ID_ANY,
                                    STM32_IRQ_TIM1_CC_PRIORITY,
                                    dshot_dma_tc_ic_tim1, nullptr);
    osalDbgAssert(s_dma_ic_tim1 != nullptr, "DMA alloc TIM1 IC");
    if (s_dma_ic_tim1 == nullptr) {
        for (;;) {
            for (int i = 0; i < 20; i++) {
                palToggleLine(LINE_LED_ACTIVITY);
                volatile uint32_t n = 2000000U; while (n--) {}
            }
            volatile uint32_t n = 16000000U; while (n--) {}
        }
    }
    /* TX stream configured in dshot_write(); IC stream in start_ic_tim1(). */

    /* ── DMA stream for TIM4 (shared TX/IC) ─────────────────────────────── */
    s_dma_tim4 = dmaStreamAlloc(STM32_DMA_STREAM_ID_ANY,
                                 STM32_IRQ_TIM4_PRIORITY,
                                 dshot_dma_tc_tim4, nullptr);
    osalDbgAssert(s_dma_tim4 != nullptr, "DMA alloc TIM4");
    if (s_dma_tim4 == nullptr) {
        for (;;) {
            for (int i = 0; i < 30; i++) {
                palToggleLine(LINE_LED_ACTIVITY);
                volatile uint32_t n = 2000000U; while (n--) {}
            }
            volatile uint32_t n = 16000000U; while (n--) {}
        }
    }
    dmaSetRequestSource(s_dma_tim4, DMAMUX_TIM4_UP);
    dmaStreamSetPeripheral(s_dma_tim4, &TIM4->DMAR);
    dmaStreamSetMemory0(s_dma_tim4, s_tx_buf_tim4);
    dmaStreamSetTransactionSize(s_dma_tim4, 19U);
    s_dma_tim4->stream->FCR = 0U;    // direct mode: 1 word/TIM4_UP → CCR2 (DBL=0)
    dmaStreamSetMode(s_dma_tim4,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x1U << DMA_SxCR_DIR_Pos)
        | (0x3U << DMA_SxCR_PL_Pos)
        | DMA_SxCR_TCIE);
    dmaStreamEnable(s_dma_tim4);

    /* ── IRQ vectors ─────────────────────────────────────────────────────── */
    nvicEnableVector(TIM1_UP_IRQn, STM32_IRQ_TIM1_UP_PRIORITY);
    nvicEnableVector(TIM1_CC_IRQn, STM32_IRQ_TIM1_CC_PRIORITY);
    nvicEnableVector(TIM4_IRQn,    STM32_IRQ_TIM4_PRIORITY);
}

void dshot_write(const uint16_t throttle[4])
{
    build_tx_buf(throttle);

    /* ── Abort any in-flight IC ──────────────────────────────────────────── */
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM1->DIER = 0U;
    TIM4->DIER = 0U;
    TIM1->SR   = 0U;
    TIM4->SR   = 0U;
    dmaStreamDisable(s_dma_tx_tim1);
    dmaStreamDisable(s_dma_ic_tim1);
    dmaStreamDisable(s_dma_tim4);

    /* IDLE before reconfiguring — any stale DMA TC sees IDLE and returns. */
    s_phase_tim1 = DshotPhase::IDLE;
    s_phase_tim4 = DshotPhase::IDLE;

    /* ── Full IC→OC reset (matches ArduPilot bdshot_reset_pwm) ──────────── */
    /* rccResetTIMx → complete register wipe, then fresh CCMR/CCER/BDTR/DIER/CR1.
     * Both timers running after this call; DMA is armed below.
     * DCR, BDTR, and DIER are all set inside reset_pwm_for_tx(). */
    reset_pwm_for_tx();

    /* ── Reload TIM1 TX DMA (single burst stream → DMAR, 76 words) ─────── */
    dmaSetRequestSource(s_dma_tx_tim1, DMAMUX_TIM1_UP);
    dmaStreamSetPeripheral(s_dma_tx_tim1, &TIM1->DMAR);
    dmaStreamSetMemory0(s_dma_tx_tim1, s_tx_buf_tim1);
    dmaStreamSetTransactionSize(s_dma_tx_tim1, 76U);
    s_dma_tx_tim1->stream->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH;
    dmaStreamSetMode(s_dma_tx_tim1,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x1U << DMA_SxCR_DIR_Pos)
        | (0x3U << DMA_SxCR_PL_Pos)
        | DMA_SxCR_TCIE
        | DMA_SxCR_MBURST_0          // MBURST=INCR4: memory reads 4 words/burst into FIFO
        | DMA_SxCR_PBURST_0);        // PBURST=INCR4: all 4 CCR words drain to DMAR atomically per TIM1_UP
    dmaStreamEnable(s_dma_tx_tim1);

    dmaSetRequestSource(s_dma_tim4, DMAMUX_TIM4_UP);
    dmaStreamSetPeripheral(s_dma_tim4, &TIM4->DMAR);
    dmaStreamSetMemory0(s_dma_tim4, s_tx_buf_tim4);
    dmaStreamSetTransactionSize(s_dma_tim4, 19U);
    s_dma_tim4->stream->FCR = 0U;    // direct mode: 1 word/TIM4_UP → CCR2 (DBL=0)
    dmaStreamSetMode(s_dma_tim4,
          DMA_SxCR_MINC
        | (0x2U << DMA_SxCR_MSIZE_Pos)
        | (0x2U << DMA_SxCR_PSIZE_Pos)
        | (0x1U << DMA_SxCR_DIR_Pos)
        | (0x3U << DMA_SxCR_PL_Pos)
        | DMA_SxCR_TCIE);
    dmaStreamEnable(s_dma_tim4);

    /* ── Arm phase and let running timers drive DMA ─────────────────────── */
    /* Both timers are already running from reset_pwm_for_tx() (pwmStart sets
     * CR1=ARPE|URS|CEN). TIM4 CNT offset is also set there. DMA responds to
     * the next UEV; dshot_pre=1 ensures a clean idle period regardless of
     * where in the period DMA was armed. */
    s_phase_tim1 = DshotPhase::TX;
    s_phase_tim4 = DshotPhase::TX;
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
    out->dma_tc[0] = s_dma_tc_count[0];
    out->dma_tc[1] = s_dma_tc_count[1];
    out->cc_isr[0] = s_ic_done_count[0];
    out->cc_isr[1] = s_ic_done_count[1];
    for (int c = 0; c < 4; c++) {
        out->edge_cnt[c] = s_last_edge_cnt[c];
        for (int k = 0; k < 5; k++)
            out->edges[c][k] = s_last_edges[c][k];
    }
    chSysUnlock();
}
