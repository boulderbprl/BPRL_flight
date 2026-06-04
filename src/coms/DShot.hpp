#pragma once
#include <cstdint>

/*
 * DShot600 bidirectional motor output driver.
 *
 * Motors: PE9=TIM1_CH1(M1), PE11=TIM1_CH2(M0), PE13=TIM1_CH3(M3), PD13=TIM4_CH2(M2).
 * TX: 3 separate DMA streams write to TIM1 CCR1/CCR2/CCR3 simultaneously (no CCR skew).
 *     1 DMA stream writes to TIM4 CCR2 via DMAR.
 * IC: dedicated DMA stream captures GCR edges; TIM1 rotates M0→M1→M3 (~133 Hz each).
 *     TIM4 captures M2 every frame (400 Hz).
 *
 * Before use: enable "Bidirectional DSHOT" on the ESC (BLHeli32 / AM32).
 * Call dshot_init() once at startup.
 * Call dshot_write() each control tick (returns immediately — DMA driven).
 * Call dshot_get_telemetry() to snapshot the latest eRPM values (thread-safe).
 */

struct ESCTelemetry {
    uint32_t erpm;  // electrical RPM (0 until first valid GCR frame)
    bool     valid; // true once at least one CRC-passing frame has arrived
};

struct DShotDiag {
    uint32_t dma_tc[2];    // DMA TC fire count: [0]=TIM1, [1]=TIM4
    uint32_t cc_isr[2];    // CC ISR decode count: [0]=TIM1, [1]=TIM4
    uint8_t  edge_cnt[4];  // edge count from last completed frame, per motor
    uint32_t edges[4][5];  // first 5 edge timestamps from last frame, per motor
};

void dshot_init(void);
void dshot_write(const uint16_t throttle[4]); // 0=disarm, 48-2047=throttle
void dshot_get_telemetry(ESCTelemetry out[4]); // thread-safe snapshot
void dshot_get_diag(DShotDiag *out);           // thread-safe diagnostic snapshot
