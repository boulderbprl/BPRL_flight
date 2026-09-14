#include "src/sensors/StrainGauge.hpp"
#include "src/coms/I2C.hpp"
#include <cstring>
#include <cstdint>

MUTEX_DECL(strainGauge_mtx);
StrainGaugeRaw g_strain_gauge[STRAIN_GAUGE_NUM_SENSORS] = {};

namespace {

constexpr uint8_t  kI2CAddrBase   = 0x09;  // node n -> 0x09 + 7*n (0x09, 0x10)
constexpr uint8_t  kAddrStride    = 7U;
constexpr uint8_t  kChunkChannels = 12;
constexpr uint8_t  kChunkBytes    = kChunkChannels * sizeof(int16_t);            // 24
constexpr uint8_t  kNumChunks     = STRAIN_GAUGE_NUM_CHANNELS / kChunkChannels;  // 3
constexpr uint8_t  kTriggerByte   = 'P';
constexpr uint8_t  kCalByte       = 0x5A;
constexpr uint8_t  kResetByte     = 'R';

volatile bool s_cal_pending[STRAIN_GAUGE_NUM_SENSORS] = {};

/*
 * DMA RX buffer, one per node — .nocache, same rationale as StrainRate.cpp's
 * s_i2c_rx: ChibiOS's I2Cv2 driver does no D-cache maintenance around DMA
 * transfers, and thread stacks live in cacheable AXI SRAM, so a
 * stack-allocated buffer would read stale/corrupt bytes after the DMA
 * write. Per-node (not per-chunk) since only one chunk is in flight per
 * node at a time — reused across all 3 chunks of a node's read cycle.
 */
static __attribute__((section(".nocache"))) uint8_t s_chunk_rx[STRAIN_GAUGE_NUM_SENSORS][kChunkBytes];

bool write_byte(uint8_t addr, uint8_t b)
{
    return i2cMasterTransmitTimeout(&I2CD2, addr, &b, 1, nullptr, 0, TIME_US2I(1500)) == MSG_OK;
}

void strain_gauge_poll(void *ctx)
{
    const uint8_t node = (uint8_t)(intptr_t)ctx;
    const uint8_t addr = (uint8_t)(kI2CAddrBase + kAddrStride * node);

    // Same absent-sensor backoff rationale as StrainRate.cpp — these boards
    // are often unplugged on the bench, and a full poll here is up to 4 I2C
    // transactions (1 trigger write + 3 chunked read-backs), each costing a
    // timeout when nothing's connected. Back off to ~1 Hz retries after a
    // few consecutive failures so an absent node costs next to nothing.
    static uint32_t s_fail_streak[STRAIN_GAUGE_NUM_SENSORS] = {};
    static uint32_t s_skip_ticks[STRAIN_GAUGE_NUM_SENSORS]  = {};
    constexpr uint32_t kFailStreakBeforeBackoff = 3;
    constexpr uint32_t kBackoffTicks            = 200;  // ~1 s at 200 Hz

    if (s_skip_ticks[node] > 0) {
        s_skip_ticks[node]--;
        return;
    }

    i2cAcquireBus(&I2CD2);

    bool ok = true;

    // Calibration byte, if queued by strain_gauge_calibrate() — sent from
    // here (inside the poll callback) since only the callback holds the bus
    // mutex at the right time, mirroring AP_Strain_Backend::timer()'s
    // _cal_pending handling.
    if (s_cal_pending[node]) {
        s_cal_pending[node] = false;
        ok = write_byte(addr, kCalByte);
    }

    if (ok && !write_byte(addr, kTriggerByte)) {
        ok = false;
    }

    int16_t chunk_data[STRAIN_GAUGE_NUM_CHANNELS];
    bool need_reset = false;
    if (ok) {
        for (uint8_t chunk = 0; chunk < kNumChunks; chunk++) {
            uint8_t idx = chunk;
            msg_t status = i2cMasterTransmitTimeout(&I2CD2, addr, &idx, 1,
                                                      s_chunk_rx[node], kChunkBytes,
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
            memcpy(&chunk_data[chunk * kChunkChannels], s_chunk_rx[node], kChunkBytes);
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
        if (s_fail_streak[node] < kFailStreakBeforeBackoff) s_fail_streak[node]++;
        if (s_fail_streak[node] >= kFailStreakBeforeBackoff) s_skip_ticks[node] = kBackoffTicks - 1;
        return;
    }

    s_fail_streak[node] = 0;

    chMtxLock(&strainGauge_mtx);
    memcpy(g_strain_gauge[node].data, chunk_data, sizeof(chunk_data));
    g_strain_gauge[node].valid          = true;
    g_strain_gauge[node].last_update_ms = (uint32_t)TIME_I2MS(chVTGetSystemTime());
    chMtxUnlock(&strainGauge_mtx);
}

} // namespace

void strain_gauge_init(void)
{
    for (uint8_t node = 0; node < STRAIN_GAUGE_NUM_SENSORS; node++) {
        bprl_i2c_register((uint8_t)(kI2CAddrBase + kAddrStride * node),
                           strain_gauge_poll, (void *)(intptr_t)node);
    }
}

void strain_gauge_calibrate(uint8_t node)
{
    if (node < STRAIN_GAUGE_NUM_SENSORS) s_cal_pending[node] = true;
}

bool strain_gauge_reset(uint8_t node)
{
    if (node >= STRAIN_GAUGE_NUM_SENSORS) return false;
    const uint8_t addr = (uint8_t)(kI2CAddrBase + kAddrStride * node);
    i2cAcquireBus(&I2CD2);
    bool ok = write_byte(addr, kResetByte);
    i2cReleaseBus(&I2CD2);
    return ok;
}
