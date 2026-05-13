#pragma once
#include <cstdint>

/*
 * DShot600 bidirectional motor output driver — TIM1 CH1-4 via DMA burst.
 *
 * Output: inverted DShot600 (bidirectional mode) on PA8-PA11.
 * Input:  GCR-encoded eRPM response captured on the same pins after each frame.
 *
 * Call dshot_init() once at startup (replaces PWM init).
 * Call dshot_write() each control tick; it returns immediately (DMA-driven).
 * Call dshot_get_telemetry() to snapshot the latest eRPM values.
 *
 * Before use: enable "Bidirectional DSHOT" in BLHeli Suite 32 on the P60A V2.
 */

struct ESCTelemetry {
    uint32_t erpm;  // electrical RPM (0 until first valid GCR frame)
    bool     valid; // true once at least one CRC-passing frame has arrived
};

void dshot_init(void);
void dshot_write(const uint16_t throttle[4]); // 0=disarm, 48-2047=throttle
void dshot_get_telemetry(ESCTelemetry out[4]); // thread-safe snapshot
