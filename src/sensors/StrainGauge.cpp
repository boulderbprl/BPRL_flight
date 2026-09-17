#include "src/sensors/StrainGauge.hpp"
#include "src/coms/I2C.hpp"
#include <cstring>
#include <cstdint>

MUTEX_DECL(strainGauge_mtx);
StrainGaugeRaw g_strain_gauge = {};

namespace {

constexpr uint8_t  kI2CAddr      = 0x09; 
constexpr uint8_t  kChunkChannels = 12;
constexpr uint8_t  kChunkBytes    = kChunkChannels * sizeof(int16_t);  // 24
constexpr uint8_t  kNumRawChunks  = 3;  // wire protocol still reads all 3 — see
                                         // StrainGauge.hpp for why chunks 1
                                         // and 2 are discarded as duplicates
                                         // of chunk 0
constexpr uint8_t  kPadPerChunk   = 3;  // leading zero/reserved slots per chunk
constexpr uint8_t  kTriggerByte   = 'P';
constexpr uint8_t  kCalByte       = 0x5A;
constexpr uint8_t  kResetByte     = 'R';

volatile bool s_cal_pending = false;

/*
 * DMA RX buffer — .nocache, same rationale as StrainRate.cpp's s_i2c_rx:
 * ChibiOS's I2Cv2 driver does no D-cache maintenance around DMA transfers,
 * and thread stacks live in cacheable AXI SRAM, so a stack-allocated buffer
 * would read stale/corrupt bytes after the DMA write. Reused across all 3
 * chunks of a read cycle since only one chunk is in flight at a time.
 */
static __attribute__((section(".nocache"))) uint8_t s_chunk_rx[kChunkBytes];

bool write_byte(uint8_t b)
{
    return i2cMasterTransmitTimeout(&I2CD2, kI2CAddr, &b, 1, nullptr, 0, TIME_US2I(1500)) == MSG_OK;
}

void strain_gauge_poll(void *ctx)
{
    (void)ctx;

    // Same absent-sensor backoff rationale as StrainRate.cpp — this board
    // is often unplugged on the bench, and a full poll here is up to 4 I2C
    // transactions (1 trigger write + 3 chunked read-backs), each costing a
    // timeout when nothing's connected. Back off to ~1 Hz retries after a
    // few consecutive failures so an absent sensor costs next to nothing.
    static uint32_t s_fail_streak    = 0;
    static uint32_t s_skip_ticks     = 0;
    static bool     s_was_connected  = false;  // for auto-zero on (re)connect, below
    constexpr uint32_t kFailStreakBeforeBackoff = 3;
    constexpr uint32_t kBackoffTicks            = 200;  // ~1 s at 200 Hz

    if (s_skip_ticks > 0) {
        s_skip_ticks--;
        return;
    }

    i2cAcquireBus(&I2CD2);

    bool ok = true;

    // Calibration byte, if queued by strain_gauge_calibrate() — sent from
    // here (inside the poll callback) since only the callback holds the bus
    // mutex at the right time, mirroring AP_Strain_Backend::timer()'s
    // _cal_pending handling.
    if (s_cal_pending) {
        s_cal_pending = false;
        ok = write_byte(kCalByte);
    }

    if (ok && !write_byte(kTriggerByte)) {
        ok = false;
    }

    // Full raw wire read (all 3 chunks, 36 int16s) — chunks 1 and 2 are
    // discarded below as confirmed duplicates of chunk 0 (see StrainGauge.hpp).
    int16_t raw[kNumRawChunks * kChunkChannels];
    bool need_reset = false;
    if (ok) {
        for (uint8_t chunk = 0; chunk < kNumRawChunks; chunk++) {
            uint8_t idx = chunk;
            msg_t status = i2cMasterTransmitTimeout(&I2CD2, kI2CAddr, &idx, 1,
                                                      s_chunk_rx, kChunkBytes,
                                                      TIME_US2I(1500));
            if (status != MSG_OK) {
                // MSG_RESET means the driver hit an error and is now
                // I2C_LOCKED — without resetting here it stays locked
                // forever (MSG_TIMEOUT never fires again on its own).
                // The actual reset call is deferred until after
                // i2cReleaseBus() below (see there for why) — it must never
                // run while this thread still holds the bus.
                need_reset = (status == MSG_TIMEOUT || status == MSG_RESET);
                ok = false;
                break;
            }
            memcpy(&raw[chunk * kChunkChannels], s_chunk_rx, kChunkBytes);
        }
    }

    i2cReleaseBus(&I2CD2);

    // i2c_drv_reset() calls i2cStop()/i2cStart() on the driver itself — it
    // must run only after this thread has released the bus (matching
    // StrainRate.cpp's i2c poll), never while still holding it. Calling it
    // inside the acquire/release bracket (an earlier version of this file
    // did) restarts the driver out from under the mutual-exclusion lock
    // this thread is still holding, leaving it in an undefined state.
    if (need_reset) i2c_drv_reset();

    if (!ok) {
        if (s_fail_streak < kFailStreakBeforeBackoff) s_fail_streak++;
        if (s_fail_streak >= kFailStreakBeforeBackoff) {
            s_skip_ticks = kBackoffTicks - 1;
            // Confirmed disconnected (not just one flaky transaction) — treat
            // the next successful read as a fresh connection so the arms
            // re-zero automatically once the board is plugged back in.
            s_was_connected = false;
        }
        return;
    }

    s_fail_streak = 0;

    // Auto-zero on (re)connect: the first successful read after boot or
    // after a confirmed disconnect queues a calibration byte for the *next*
    // poll tick (mirrors strain_gauge_calibrate()) — this tick's frame is
    // published un-zeroed, and the following tick (~5 ms later) reflects
    // the zeroed reading.
    if (!s_was_connected) {
        s_was_connected = true;
        s_cal_pending    = true;
    }

    // Extract the 9 real channels from chunk 0 (its leading 3 are always-zero
    // padding). raw[12..35] (chunks 1 and 2) is discarded — bench-confirmed
    // duplicate of chunk 0, not independent data.
    chMtxLock(&strainGauge_mtx);
    memcpy(g_strain_gauge.data, &raw[kPadPerChunk],
           STRAIN_GAUGE_NUM_CHANNELS * sizeof(int16_t));
    g_strain_gauge.valid          = true;
    g_strain_gauge.last_update_ms = (uint32_t)TIME_I2MS(chVTGetSystemTime());
    g_strain_gauge.update_count++;
    chMtxUnlock(&strainGauge_mtx);
}

} // namespace

void strain_gauge_init(void)
{
    bprl_i2c_register(kI2CAddr, strain_gauge_poll, nullptr);
}

void strain_gauge_calibrate(void)
{
    s_cal_pending = true;
}

bool strain_gauge_reset(void)
{
    i2cAcquireBus(&I2CD2);
    bool ok = write_byte(kResetByte);
    i2cReleaseBus(&I2CD2);
    return ok;
}
