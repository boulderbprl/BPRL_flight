#include "src/uptime.hpp"
#include "src/sensors/StrainGauge.hpp"
#include "src/coms/I2C.hpp"
#include <cstring>
#include <cstdint>

MUTEX_DECL(strainGauge_mtx);
StrainGaugeRaw g_strain_gauge = {};

namespace {

// Wire protocol per src/sensors/ConBoard_I2C_Reference.md — see
// StrainGauge.hpp for the full description.
constexpr uint8_t  kI2CAddr       = 0x09;
constexpr uint8_t  kNumChunks     = 2;
constexpr uint8_t  kChunkChannels = STRAIN_GAUGE_NUM_CHANNELS / kNumChunks;  // 18
constexpr uint8_t  kChunkBytes    = kChunkChannels * sizeof(int16_t);        // 36
constexpr uint8_t  kCalByte       = 'Z';  // 0x5A
constexpr uint8_t  kResetByte     = 'R';  // 0x52

// A 36-byte read is ~0.85 ms on the wire at 400 kHz; the chunk-select write
// is a single byte.
constexpr sysinterval_t kWriteTimeout = TIME_US2I(1500);
constexpr sysinterval_t kReadTimeout  = TIME_US2I(2500);

volatile bool s_cal_pending = false;

/*
 * DMA RX buffer — .nocache, same rationale as StrainRate.cpp's s_i2c_rx:
 * ChibiOS's I2Cv2 driver does no D-cache maintenance around DMA transfers,
 * and thread stacks live in cacheable AXI SRAM, so a stack-allocated buffer
 * would read stale/corrupt bytes after the DMA write. Reused across both
 * chunks of a read cycle since only one chunk is in flight at a time.
 */
static __attribute__((section(".nocache"))) uint8_t s_chunk_rx[kChunkBytes];

/*
 * DMA TX buffer — .nocache for the mirror-image reason. The I2C driver
 * hands the caller's pointer straight to the TX DMA stream with no cache
 * flush, so the DMA reads the byte from RAM, not from the D-cache. A byte
 * just written to a stack variable is normally still sitting in the cache,
 * and RAM holds whatever that stack slot contained the last time its cache
 * line happened to be written back. Sending select/command bytes from the
 * stack (as this driver used to) put a stale byte on the wire: both chunk
 * selects of a poll went out as the same value, so both reads returned the
 * same chunk, and the 'Z' zero command never reached the board as 'Z'.
 * Bytes sent from this buffer are what was written to it. Only used with
 * the bus held, so one shared byte is enough.
 */
static __attribute__((section(".nocache"))) uint8_t s_tx_byte;

msg_t write_byte(uint8_t b)
{
    s_tx_byte = b;
    return i2cMasterTransmitTimeout(&I2CD2, kI2CAddr, &s_tx_byte, 1, nullptr, 0, kWriteTimeout);
}

// Absent-sensor backoff state — see strain_gauge_poll().
uint32_t s_fail_streak   = 0;
uint32_t s_skip_ticks    = 0;
bool     s_was_connected = false;  // for auto-zero on (re)connect
constexpr uint32_t kFailStreakBeforeBackoff = 3;
constexpr uint32_t kBackoffTicks            = 400;  // ~1 s at 400 Hz

void note_failure(void)
{
    if (s_fail_streak < kFailStreakBeforeBackoff) s_fail_streak++;
    if (s_fail_streak >= kFailStreakBeforeBackoff) {
        s_skip_ticks = kBackoffTicks - 1;
        // Confirmed disconnected (not just one flaky transaction) — treat
        // the next successful read as a fresh connection so the board
        // re-zeroes automatically once it is plugged back in.
        s_was_connected = false;
    }
}

void strain_gauge_poll(void *ctx)
{
    (void)ctx;

    // Same absent-sensor backoff rationale as StrainRate.cpp — this board
    // is often unplugged on the bench, and a full poll here is 4 I2C
    // transactions (a chunk-select write + a read, twice), each costing a
    // timeout when nothing's connected. Back off to ~1 Hz retries after a
    // few consecutive failures so an absent sensor costs next to nothing.
    if (s_skip_ticks > 0) {
        s_skip_ticks--;
        return;
    }

    i2cAcquireBus(&I2CD2);

    msg_t   status    = MSG_OK;
    bool    cal_sent  = false;
    uint8_t chunks_ok = 0;  // chunks read successfully this poll (in order)

    // Zero command, if queued by strain_gauge_calibrate() — sent from here
    // (inside the poll callback) since only the callback holds the bus
    // mutex at the right time. The board needs ~1 ms after 'Z' before its
    // data is good again, so this tick sends the command and nothing else;
    // the next tick (2.5 ms later) reads the zeroed frame.
    int16_t raw[STRAIN_GAUGE_NUM_CHANNELS];
    if (s_cal_pending) {
        s_cal_pending = false;
        status   = write_byte(kCalByte);
        cal_sent = true;
    } else {
        // The board does not auto-advance: each chunk needs its own select
        // write (0x00 / 0x01), sent as a complete transaction ending in a
        // STOP, followed by a separate 36-byte read — exactly the
        // endTransmission() + requestFrom() sequence in the reference. Do
        // not fold these into one write-then-read (repeated START)
        // transaction: the board applies the select byte from its
        // receive handler, which a repeated START is not guaranteed to
        // run before the read is serviced.
        for (uint8_t chunk = 0; chunk < kNumChunks; chunk++) {
            status = write_byte(chunk);
            if (status != MSG_OK) break;
            status = i2cMasterReceiveTimeout(&I2CD2, kI2CAddr, s_chunk_rx,
                                             kChunkBytes, kReadTimeout);
            if (status != MSG_OK) break;
            // int16_t little-endian on the wire (nRF52840 native), same as
            // this MCU — copy straight across.
            memcpy(&raw[chunk * kChunkChannels], s_chunk_rx, kChunkBytes);
            chunks_ok++;
        }
    }

    i2cReleaseBus(&I2CD2);

    // Per-chunk receive counters — bumped for every chunk that actually
    // arrived, including a chunk 0 whose chunk 1 then failed (that frame
    // is still not published, below).
    if (chunks_ok > 0) {
        chMtxLock(&strainGauge_mtx);
        for (uint8_t chunk = 0; chunk < chunks_ok; chunk++) {
            g_strain_gauge.chunk_count[chunk]++;
        }
        chMtxUnlock(&strainGauge_mtx);
    }

    if (status != MSG_OK) {
        // MSG_TIMEOUT / MSG_RESET leave the driver in I2C_LOCKED — without
        // a reset it stays locked forever. i2c_drv_reset() calls
        // i2cStop()/i2cStart() on the driver itself, so it must run only
        // after this thread has released the bus (matching StrainRate.cpp's
        // i2c poll), never inside the acquire/release bracket above.
        i2c_drv_reset();
        note_failure();
        return;
    }

    s_fail_streak = 0;
    if (cal_sent) return;

    // Auto-zero on (re)connect: the first successful read after boot or
    // after a confirmed disconnect queues a zero command for the *next*
    // poll tick (mirrors strain_gauge_calibrate()) — this tick's frame is
    // published un-zeroed, and the first frame after that reflects the
    // zeroed reading.
    if (!s_was_connected) {
        s_was_connected = true;
        s_cal_pending   = true;
    }

    chMtxLock(&strainGauge_mtx);
    memcpy(g_strain_gauge.data, raw, sizeof(g_strain_gauge.data));
    g_strain_gauge.valid          = true;
    g_strain_gauge.last_update_ms = bprl_millis();
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
    bool ok = write_byte(kResetByte) == MSG_OK;
    i2cReleaseBus(&I2CD2);
    return ok;
}
