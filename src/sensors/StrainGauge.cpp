#include "src/uptime.hpp"
#include "src/sensors/StrainGauge.hpp"
#include "src/coms/I2C.hpp"
#include "src/threads.hpp"   // CONTROL_DT_S
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
    const bool same = g_strain_gauge.valid &&
                      memcmp(g_strain_gauge.data, raw, sizeof(g_strain_gauge.data)) == 0;
    g_strain_gauge.frozen_frames  = same ? g_strain_gauge.frozen_frames + 1 : 0;
    memcpy(g_strain_gauge.data, raw, sizeof(g_strain_gauge.data));
    g_strain_gauge.valid          = true;
    g_strain_gauge.last_update_ms = bprl_millis();
    g_strain_gauge.last_update_us = bprl_micros();
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

SensorHealth strain_gauge_health(const StrainGaugeRaw &snap, uint32_t now_us)
{
    if (!snap.valid) return SensorHealth::BAD;

    // Signed for the same reason as EncoderRpmMonitor: a frame published
    // after the caller read now_us is fresh, not wrapped-around stale.
    const int32_t age_us = (int32_t)(now_us - snap.last_update_us);
    if (age_us > (int32_t)STRAIN_GAUGE_STALE_US ||
        snap.frozen_frames >= STRAIN_GAUGE_FROZEN_FRAMES) {
        return SensorHealth::BAD;
    }
    if (age_us > (int32_t)STRAIN_GAUGE_FRESH_US ||
        snap.frozen_frames >= STRAIN_GAUGE_HOLD_FRAMES) {
        return SensorHealth::HOLDING;
    }
    return SensorHealth::GOOD;
}

// Fit taps are STRAIN_FIT_TAP_DT_S apart; at CONTROL_DT_S that's every
// kTapStride-th tick.
static constexpr int kTapStride = (int)(STRAIN_FIT_TAP_DT_S / CONTROL_DT_S + 0.5f);
static_assert(kTapStride >= 1, "strain fit taps closer together than one control tick");
static_assert((STRAIN_FIT_N_TAPS - 1) * kTapStride + 1 <= 24,
              "StrainAngAccel::kHistory too short for the fit's tap history");
static_assert((STRAIN_FIT_RPM_TAPS > 0 ? STRAIN_FIT_RPM_TAPS - 1 : 0) * kTapStride + 1 <= 24,
              "StrainAngAccel::kHistory too short for the fit's RPM tap history");
static_assert((STRAIN_FIT_FUSE_IMU_TAPS > 0 ? STRAIN_FIT_FUSE_IMU_TAPS - 1 : 0) * kTapStride + 1 <= 24 &&
              (STRAIN_FIT_FUSE_RPM_TAPS > 0 ? STRAIN_FIT_FUSE_RPM_TAPS - 1 : 0) * kTapStride + 1 <= 24,
              "StrainAngAccel::kHistory too short for the fused fit's tap history");
static_assert(!STRAIN_FIT_FUSE || STRAIN_FIT_FUSE_IMU_TAPS >= 1, "fused fit without IMU taps");
static_assert(STRAIN_GAUGE_HPF_HZ == STRAIN_FIT_HPF_HZ,
              "strain high-pass doesn't match the one the fit was made with");
static_assert(STRAIN_FIT_N_SIG == 2 * STRAIN_FIT_N_GROUPS * (1 + STRAIN_FIT_USE_SQ),
              "StrainGaugeFit.hpp inputs per tap don't match STRAIN_FIT_USE_SQ");
static_assert(STRAIN_FIT_Q_N_SIG == 2 * STRAIN_FIT_N_GROUPS * (1 + STRAIN_FIT_Q_USE_SQ),
              "StrainGaugeFit.hpp q_dot inputs per tap don't match STRAIN_FIT_Q_USE_SQ");
static_assert(STRAIN_FIT_Q_SEPARATE ||
              (STRAIN_FIT_Q_N_SIG == STRAIN_FIT_N_SIG && STRAIN_FIT_Q_HPF_HZ == STRAIN_FIT_HPF_HZ &&
               STRAIN_FIT_Q_USE_SQ == STRAIN_FIT_USE_SQ),
              "q_dot shares the roll block but StrainGaugeFit.hpp gives it a different layout/high-pass");

void StrainAngAccel::update(const StrainGaugeRaw &snap, const uint32_t rpm[4])
{
    if (!snap.valid) return;

    // 1. Group signals, raw counts: means [0..11], gradients [12..23].
    float sig[kSignals];
    for (int k = 0; k < STRAIN_FIT_N_GROUPS; k++) {
        const StrainFitGroup &g = kStrainFitGroups[k];
        float sum = 0.0f, first = 0.0f, last = 0.0f;
        for (int m = 0; m < g.n; m++) {
            const float s = (float)snap.data[g.ch[m]] * g.inv_gain[m];
            sum += s;
            if (m == 0) first = s;
            last = s;
        }
        sig[k]                       = (g.n > 0) ? sum / (float)g.n : 0.0f;
        sig[STRAIN_FIT_N_GROUPS + k] = (first - last) * g.inv_steps;   // inv_steps is 0 for n < 2
    }

    // 2. High-pass, then the shared angular-acceleration low-pass chain, into
    // the next ring row.
    _head = (_head + 1) % kHistory;
    const float alpha_extra = lowpass_alpha(_filt.lpf_extra_hz, CONTROL_DT_S);
    for (int i = 0; i < kSignals; i++) {
        const float hp  = highpass1p(sig[i], _hp[i], STRAIN_GAUGE_HPF_HZ, CONTROL_DT_S);
        const float lp2 = lowpass2p(hp, _lp2[i], _filt.lpf_hz, CONTROL_DT_S);
        _lp1[i]         = lowpass(lp2, _lp1[i], alpha_extra);
        _hist[_head][i] = _lp1[i];
        // 2b. signed square, fixed scale (STRAIN_FIT_USE_SQ), after the 24 signals
#if STRAIN_FIT_USE_SQ
        const float v = _lp1[i];
        _hist[_head][kSignals + i] = v * (v < 0.0f ? -v : v) * kStrainFitSqInvScale[i];
#endif
    }
#if STRAIN_FIT_Q_SEPARATE
    // 2c. Pitch block: same group signals, the pitch fit's own high-pass, the
    // same low-pass chain, its own signed squares — after the roll block.
    for (int i = 0; i < kSignals; i++) {
        const float hp  = highpass1p(sig[i], _hpq[i], STRAIN_FIT_Q_HPF_HZ, CONTROL_DT_S);
        const float lp2 = lowpass2p(hp, _lp2q[i], _filt.lpf_hz, CONTROL_DT_S);
        _lp1q[i]        = lowpass(lp2, _lp1q[i], alpha_extra);
        _hist[_head][kQOffset + i] = _lp1q[i];
#if STRAIN_FIT_Q_USE_SQ
        const float v = _lp1q[i];
        _hist[_head][kQOffset + kSignals + i] = v * (v < 0.0f ? -v : v) * kStrainFitSqInvScaleQ[i];
#endif
    }
#endif
    // RPM inputs, unfiltered — as the fit used the logged RPMS.
    for (int m = 0; m < 4; m++) {
        const float r = (float)rpm[m] * STRAIN_FIT_RPM_SCALE;
        _rpm_hist[_head][m] = STRAIN_FIT_RPM_SQUARE ? r * r : r;
    }
    if (_ticks < kHistory) _ticks++;

    // 3. Weighted taps, k, k - stride, ... Before the history fills the
    // older rows are still 0 — don't use the estimate until ready().
    float p = kStrainFitBiasP;
    float q = kStrainFitBiasQ;
#if STRAIN_FIT_FUSE
    // 5. The fused fit's strain + RPM part, on the roll block's rows; fuse() adds the IMU taps.
    float pf = kStrainFuseBiasP;
    float qf = kStrainFuseBiasQ;
#endif
    for (int j = 0; j < STRAIN_FIT_N_TAPS; j++) {
        const float *row = _hist[(_head - j * kTapStride + kHistory) % kHistory];
        for (int i = 0; i < kFitSignals; i++) p += kStrainFitWP[j][i] * row[i];
        for (int i = 0; i < kFitSignalsQ; i++) q += kStrainFitWQ[j][i] * row[kQOffset + i];
#if STRAIN_FIT_FUSE
        for (int i = 0; i < kFitSignals; i++) {
            pf += kStrainFuseWP[j][i] * row[i];
            qf += kStrainFuseWQ[j][i] * row[i];
        }
#endif
    }
#if STRAIN_FIT_FUSE
    for (int j = 0; j < STRAIN_FIT_FUSE_RPM_TAPS; j++) {
        const float *row = _rpm_hist[(_head - j * kTapStride + kHistory) % kHistory];
        for (int m = 0; m < 4; m++) {
            pf += kStrainFuseRpmWP[j][m] * row[m];
            qf += kStrainFuseRpmWQ[j][m] * row[m];
        }
    }
    _fz_part_p = pf;
    _fz_part_q = qf;
#endif
    for (int j = 0; j < STRAIN_FIT_RPM_TAPS; j++) {
        const float *row = _rpm_hist[(_head - j * kTapStride + kHistory) % kHistory];
        for (int m = 0; m < 4; m++) {
            p += kStrainFitRpmWP[j][m] * row[m];
            q += kStrainFitRpmWQ[j][m] * row[m];
        }
    }
    _p_dot = ready() ? p : 0.0f;
    _q_dot = ready() ? q : 0.0f;
}

void StrainAngAccel::fuse(float imu_p_dot, float imu_q_dot)
{
#if STRAIN_FIT_FUSE
    _imu_head = (_imu_head + 1) % kHistory;
    _imu_hist[_imu_head][0] = imu_p_dot;
    _imu_hist[_imu_head][1] = imu_q_dot;
    if (_imu_ticks < kHistory) _imu_ticks++;

    float pf = _fz_part_p;
    float qf = _fz_part_q;
    for (int j = 0; j < STRAIN_FIT_FUSE_IMU_TAPS; j++) {
        const float *row = _imu_hist[(_imu_head - j * kTapStride + kHistory) % kHistory];
        pf += kStrainFuseImuWP[j][0] * row[0] + kStrainFuseImuWP[j][1] * row[1];
        qf += kStrainFuseImuWQ[j][0] * row[0] + kStrainFuseImuWQ[j][1] * row[1];
    }
    _pf_dot = fused_ready() ? pf : 0.0f;
    _qf_dot = fused_ready() ? qf : 0.0f;
#else
    (void)imu_p_dot;
    (void)imu_q_dot;
#endif
}

bool strain_gauge_reset(void)
{
    i2cAcquireBus(&I2CD2);
    bool ok = write_byte(kResetByte) == MSG_OK;
    i2cReleaseBus(&I2CD2);
    return ok;
}
