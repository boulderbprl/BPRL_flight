#pragma once
#include "ch.h"
#include "hal.h"
#include "src/sensors/SensorHealth.hpp"
#include "src/sensors/StrainGaugeFit.hpp"
#include "src/math/math.hpp"
#include "configs/DroneConfig.hpp"

/*
 * Strain gauge array sensor — the ADC "ConBoard" (nRF52840), a single I2C
 * node at 0x09 on I2C2, 400 kHz. This is a different physical sensor from
 * src/sensors/StrainRate.hpp (4 channels, single sensor at 0x11); only one
 * of strain_gauge_init()/strain_rate_init() should be called from main.cpp.
 *
 * Wire protocol — transcribed from src/sensors/ConBoard_I2C_Reference.md
 * (board firmware V2_ADC_ConBoard.cpp), which supersedes the earlier
 * AP_Strain-derived 'P' trigger + 3 x 12-channel chunk protocol this driver
 * used to speak:
 *
 *   36 ADC channels, each an int16_t (little-endian), 72 bytes total, split
 *   into two 36-byte chunks because of the board's I2C buffer limit:
 *
 *     chunk 0 (select byte 0x00) -> channels  0-17
 *     chunk 1 (select byte 0x01) -> channels 18-35
 *
 *   For each chunk: write the 1-byte chunk index (complete transaction,
 *   STOP), then read 36 bytes. The board does not auto-advance, so every
 *   read must be preceded by its own select write.
 *
 *   Single bytes >= 0x02 are commands, with no reply: 'Z' (0x5A) zeroes all
 *   channels, 'R' (0x52) resets all ADCs, 'D' (0x44) toggles the board's
 *   debug serial plotter, 'P' (0x50) is a no-op poll marker (no longer
 *   sent — it is not needed to get data). Allow ~1 ms after 'Z'/'R' before
 *   reading.
 *
 * Units: each count is filtered_voltage x 10,000,000, i.e. 0.1 uV/count,
 * +/-3.2767 mV full scale — see STRAIN_GAUGE_COUNTS_PER_VOLT.
 *
 * Timing: the board samples at 500 Hz and low-passes each channel with a
 * 2nd-order Butterworth (fc = 100 Hz) before it reaches the bus, and a read
 * always returns its last complete snapshot. I2CThread polls at 400 Hz on
 * this board (main.cpp's kRates.i2c), so nearly every poll sees a fresh
 * board sample. The two chunks are separate transactions ~1 ms apart, so
 * channels 0-17 and 18-35 of one published frame can come from adjacent
 * 2 ms board snapshots. A full poll is 2 x (1-byte write + 36-byte read),
 * ~1.9 ms at 400 kHz — most of I2CThread's 2.5 ms period, so there is no
 * room for another device on this bus at this rate, and a board that
 * stretches the clock will push the achieved rate below 400 Hz (see the
 * Hz field of the STN1/STN3 log messages).
 *
 * Channel -> arm mapping: 4 arms x 9 channels, in channel order. $STRAIN
 * and tools/telemetry.py use the same ARM1..ARM4 order:
 *
 *   data[ 0.. 8]  ARM1  front right
 *   data[ 9..17]  ARM2  back right
 *   data[18..26]  ARM3  back left
 *   data[27..35]  ARM4  front left
 *
 * The earlier "9 real channels, 5+4 split, chunks are duplicates" findings
 * were for the old protocol and no longer apply.
 */

#define STRAIN_GAUGE_NUM_CHANNELS      36  // 2 chunks x 18 int16_t
#define STRAIN_GAUGE_NUM_ARMS          4   // FR, BR, BL, FL — see note above
#define STRAIN_GAUGE_CHANNELS_PER_ARM  (STRAIN_GAUGE_NUM_CHANNELS / STRAIN_GAUGE_NUM_ARMS)  // 9

// volts = data[i] / STRAIN_GAUGE_COUNTS_PER_VOLT
#define STRAIN_GAUGE_COUNTS_PER_VOLT   10000000.0f

struct StrainGaugeRaw {
    int16_t  data[STRAIN_GAUGE_NUM_CHANNELS];  // raw counts, channel order 0-35
    bool     valid;          // true once at least one full frame (both chunks) has been read
    uint32_t last_update_ms; // chVTGetSystemTime()-based timestamp of the last good read
    uint32_t update_count;   // incremented on every successful frame read — diff over
                              // a fixed window (e.g. 1s) to get the sensor's actual Hz
    uint32_t chunk_count[2]; // per-chunk successful 36-byte reads ([0] = channels 0-17,
                              // [1] = channels 18-35), counted even when the other
                              // chunk of that poll failed — diff over a window for
                              // each chunk's own receive rate (logged as STN1/STN3 Hz)
    uint32_t last_update_us; // bprl_micros() of the last good read — for strain_gauge_health()
    uint32_t frozen_frames;  // consecutive published frames bit-identical to the one before
};

extern mutex_t        strainGauge_mtx;
extern StrainGaugeRaw g_strain_gauge;

// Whether the strain data is usable right now, evaluated once per
// ControlThread tick from a snapshot of g_strain_gauge. Whole-board checks
// only for now — the per-channel ones are in the TODO below.
//
//   GOOD     a frame within STRAIN_GAUGE_FRESH_US that isn't a repeat of
//            the frames before it.
//   HOLDING  no frame for STRAIN_GAUGE_FRESH_US..STRAIN_GAUGE_STALE_US, or
//            STRAIN_GAUGE_HOLD_FRAMES..STRAIN_GAUGE_FROZEN_FRAMES frames in a
//            row bit-identical across all 36 channels — a short hiccup.
//   BAD      no frame ever, none for more than STRAIN_GAUGE_STALE_US, or
//            STRAIN_GAUGE_FROZEN_FRAMES identical frames in a row: the board
//            answers I2C from its last snapshot even if its ADCs stop, and
//            live, noisy channels never repeat exactly for that long (at
//            400 Hz polling of a 500 Hz board a single repeat is normal).
//
// No recovery dwell here — that is FlightStateMachine's. Stateless.
#define STRAIN_GAUGE_FRESH_US        7500U   // 3 missed polls at 400 Hz
#define STRAIN_GAUGE_STALE_US        50000U  // 50 ms — the hold INDI tolerates
#define STRAIN_GAUGE_HOLD_FRAMES     2U      // 2 repeats in a row: not live
#define STRAIN_GAUGE_FROZEN_FRAMES   20U     // 50 ms at 400 Hz

SensorHealth strain_gauge_health(const StrainGaugeRaw &snap, uint32_t now_us);

/*
 * Strain -> angular acceleration (p_dot, q_dot), the firmware half of the
 * MATLAB strain calibration pipeline (strain_pdot_pipeline.m). Weights and
 * gauge groups come from StrainGaugeFit.hpp, generated from that script's
 * weights .mat (gust_calibration.m: gust_fit_weights.mat) by
 * tools/strain_fit_export.py. Per ControlThread
 * tick, on that tick's snapshot of g_strain_gauge:
 *
 *  1. Per arm and axis (gauges {0,3,6}, {1,4,7}, {2,5,8} of each arm, live
 *     ones only, each scaled by 1/gain to the group's first gauge):
 *       mean     = average of the scaled gauges
 *       gradient = (first - last scaled gauge) / node steps between them
 *     -> 12 means + 12 gradients.
 *  2. Each of the 24 signals: 1st-order high-pass at STRAIN_GAUGE_HPF_HZ
 *     (always on — drift and the re-zero steps), then the drone's
 *     AngAccelFilterConfig low-pass (2nd-order at lpf_hz, then 1st-order at
 *     lpf_extra_hz), the same chain the Unmixer puts on RPM, so the strain
 *     estimate and the RPM torque stay delay-matched. MATLAB filters the
 *     gauges before grouping; the filters and the grouping are both linear,
 *     so filtering the 24 group signals instead gives the same result for
 *     less work.
 *  2b. Fits with STRAIN_FIT_USE_SQ (gust_calibration.m use_sq) also take the
 *     signed square of every filtered signal, s * |s| * kStrainFitSqInvScale[i]
 *     (a fixed per-signal scale from fit time), stored after the 24 signals in
 *     the same ring row -> STRAIN_FIT_N_SIG = 48 inputs per tap.
 *  2c. With a separate pitch fit (STRAIN_FIT_Q_SEPARATE, strain_fit_export.py
 *     --pitch) the same 24 group signals also go through a second chain —
 *     high-pass at STRAIN_FIT_Q_HPF_HZ, the same low-pass — plus their own
 *     signed squares (STRAIN_FIT_Q_USE_SQ, kStrainFitSqInvScaleQ), stored
 *     after the roll block in the ring row. Only q_dot reads this block; roll
 *     (p_dot) is computed exactly as without it.
 *  3. A ring buffer keeps the filtered signals for the last
 *     (STRAIN_FIT_N_TAPS - 1) x STRAIN_FIT_TAP_DT_S; the fit reads taps
 *     STRAIN_FIT_TAP_DT_S apart (every 4th tick at 400 Hz) and
 *       p_dot = kStrainFitBiasP + sum(kStrainFitWP x taps of the roll block),
 *       q_dot = kStrainFitBiasQ + sum(kStrainFitWQ x taps of the pitch block
 *               — the roll block when STRAIN_FIT_Q_SEPARATE is 0).
 *     The estimate still updates every tick.
 *  4. Fits from gust_calibration.m (use_rpm) also take the 4 motor RPMs:
 *     STRAIN_FIT_RPM_TAPS taps, same spacing, of |rpm[i]| x
 *     STRAIN_FIT_RPM_SCALE (squared if STRAIN_FIT_RPM_SQUARE), unfiltered —
 *     the fit was made on the logged RPMS (= |rpm_raw|, which is
 *     EncoderNodeHealth::rpm_abs while the encoder is GOOD), with no filter.
 *     rpm[] is the ControlThread's logical-order rpm[] (FR, RL, FL, RR =
 *     RPMS.RPM0..3), so the estimate is only valid while the RPM health
 *     isn't BAD — the caller gates on that (threads.cpp). With
 *     STRAIN_FIT_RPM_TAPS 0 (a strain-only fit) rpm is ignored.
 *  5. Fused strain + IMU estimate (STRAIN_FIT_FUSE, strain_fit_export.py
 *     --fuse): a second weight set on the same ring rows as p_dot (the roll
 *     block) and the same RPM history — kStrainFuseW*, kStrainFuseRpmW* —
 *     plus STRAIN_FIT_FUSE_IMU_TAPS taps, same spacing, of the IMU p_dot and
 *     q_dot chain (StateManager::imu_p_dot()/imu_q_dot() = SACC.PdI/QdI),
 *     both axes into each output. update() sums the strain + RPM part;
 *     fuse(), called once the tick's IMU estimate exists (after
 *     StateManager::update()), adds the IMU taps. The fit was trained on
 *     the logged PdI/QdI chain, so the IMU chain config
 *     (AngAccelFilterConfig::imu_lpf_hz/imu_lpf_extra_hz) must stay as it was.
 *
 * Gauges are sampled and held between I2C frames — the board and the I2C
 * poll both run at ~400 Hz, so most ticks see a new frame.
 * The output is 0 until the first valid frame plus the tap history.
 * Health is strain_gauge_health()'s job, not this class's.
 */
// Strain high-pass cutoff: always the one the fit was made with (fit B: 1 Hz).
// A separate pitch fit has its own, STRAIN_FIT_Q_HPF_HZ (pitch fit P: 0.2 Hz).
#define STRAIN_GAUGE_HPF_HZ  STRAIN_FIT_HPF_HZ

class StrainAngAccel {
public:
    explicit StrainAngAccel(const AngAccelFilterConfig &filt) : _filt(filt) {}

    // Once per ControlThread tick (filters run at the fixed CONTROL_DT_S).
    // Does nothing until snap.valid. rpm = this tick's logical-order motor
    // RPM (step 4 above; unused by a strain-only fit).
    void update(const StrainGaugeRaw &snap, const uint32_t rpm[4]);

    // Fused strain + IMU estimate (step 5): once per tick, after update()
    // and after this tick's IMU p_dot/q_dot are computed. No-op (and
    // fused_ready() stays false) when StrainGaugeFit.hpp has no fused fit.
    void fuse(float imu_p_dot, float imu_q_dot);

    // Either fit reads RPM (the fused fit's RPM taps are a subset of these).
    static constexpr bool uses_rpm() { return STRAIN_FIT_RPM_TAPS > 0 || STRAIN_FIT_FUSE_RPM_TAPS > 0; }
    static constexpr bool has_fused() { return STRAIN_FIT_FUSE != 0; }

    float p_dot() const { return _p_dot; }   // rad/s^2
    float q_dot() const { return _q_dot; }   // rad/s^2
    bool  ready() const { return _ticks >= kHistory; }  // tap history full
    float fused_p_dot() const { return _pf_dot; }   // rad/s^2, 0 until fused_ready()
    float fused_q_dot() const { return _qf_dot; }
    bool  fused_ready() const { return has_fused() && ready() && _imu_ticks >= kHistory; }

private:
    static constexpr int kSignals    = 2 * STRAIN_FIT_N_GROUPS;   // means, then gradients (filtered)
    static constexpr int kFitSignals = STRAIN_FIT_N_SIG;          // + their signed squares (STRAIN_FIT_USE_SQ)
    static constexpr int kFitSignalsQ = STRAIN_FIT_Q_N_SIG;       // q_dot inputs per tap
    // q_dot's inputs start here in a ring row: after the roll block if pitch has its own (step 2c)
    static constexpr int kQOffset = STRAIN_FIT_Q_SEPARATE ? kFitSignals : 0;
    static constexpr int kRowLen  = kFitSignals + (STRAIN_FIT_Q_SEPARATE ? kFitSignalsQ : 0);
    static constexpr int kHistory = 24;   // ring length in ticks, >= (N_TAPS-1) x tap stride + 1 (checked in the .cpp)

    AngAccelFilterConfig _filt;
    Highpass1pState      _hp[kSignals];
    Biquad2pState        _lp2[kSignals];
    float                _lp1[kSignals] = {};
#if STRAIN_FIT_Q_SEPARATE
    Highpass1pState      _hpq[kSignals];    // pitch block (step 2c)
    Biquad2pState        _lp2q[kSignals];
    float                _lp1q[kSignals] = {};
#endif
    float                _hist[kHistory][kRowLen] = {};
    float                _rpm_hist[kHistory][4] = {};   // RPM inputs (step 4), same ring index as _hist
    int                  _head  = 0;   // _hist row written on the last update
    uint32_t             _ticks = 0;   // updates since the first valid frame (saturates at kHistory)
    float                _p_dot = 0.0f;
    float                _q_dot = 0.0f;
#if STRAIN_FIT_FUSE
    float                _imu_hist[kHistory][2] = {};   // IMU p_dot, q_dot (step 5), fuse()'s own ring
    int                  _imu_head = 0;
    float                _fz_part_p = 0.0f;   // fused fit's strain + RPM part, from update()
    float                _fz_part_q = 0.0f;
#endif
    uint32_t             _imu_ticks = 0;      // fuse() calls (saturates at kHistory)
    float                _pf_dot = 0.0f;
    float                _qf_dot = 0.0f;
};

/*
 * TODO — per-channel health, and what the fit does when a channel dies.
 * The fit itself (StrainAngAccel above) is in; it uses the channels that
 * were live when it was fitted and nothing here checks them yet:
 *
 *  1. (Done — StrainAngAccel.) Measure its delay against the RPM -> torque
 *     path INDI differences it against before flying INDI on it.
 *
 *  2. Per-channel freeze. A channel whose (max - min) over a 50 ms window
 *     (20 frames) is <= 1 count, two windows in a row (100 ms), is frozen.
 *     Two compares per channel per frame, plus a reset every 20 frames.
 *     Catches a channel stuck flickering between two values too, which
 *     comparing consecutive 50 ms averages would miss. Only channels with a
 *     non-zero weight need checking.
 *
 *  3. Railed channel. |data[i]| >= ~32700 (the +/-3.2767 mV full scale) is
 *     saturated — treat it like frozen, since it also stops changing.
 *
 *  4. Spike on one channel. Clamp each channel's frame-to-frame step: a
 *     step larger than anything the arm can physically do holds the
 *     previous value for that frame. No added delay (a median-of-3 would
 *     cost 2.5 ms at 400 Hz). The board's 100 Hz low-pass already softens
 *     spikes; this only catches gross ones.
 *
 *  5. What a bad channel does. For now: any weighted channel frozen,
 *     railed or repeatedly clamped -> strain_gauge_health() returns BAD, so
 *     the STRAIN/FUSED axes fly the IMU estimate — refit with fewer channels in MATLAB rather
 *     than fly a fit missing a term. Dropout tolerance (re-solved weight
 *     sets per dropped channel, or rescaling the rest) is out of scope; note
 *     that just zeroing a weight shifts the estimate by weight x value, and
 *     channels on one arm are strongly correlated.
 *
 *  6. Auto-zero. strain_gauge_poll() re-zeroes the board ('Z') on every
 *     reconnect. Mid-flight that zeroes under load and biases every channel
 *     — block it while armed (keep the frames BAD until disarm instead).
 *
 *  7. Chunk skew. Channels 0-17 and 18-35 of one frame can come from
 *     adjacent 2 ms board snapshots (see Timing above) — worth knowing when
 *     fitting weights across both halves.
 */

// Call after i2c_drv_init() in main.cpp. Registers the I2C poll function
// for the sensor at 0x09. Do not also call strain_rate_init() — see the
// mutual-exclusivity note above.
void strain_gauge_init(void);

// Queue the zero command ('Z', 0x5A) to be sent on the next poll tick (the
// actual I2C write happens from inside the poll callback, which already owns
// the bus). That tick publishes no frame; the next one is zeroed.
void strain_gauge_calibrate(void);

// Send the ADC reset byte ('R', 0x52) immediately (blocking on the I2C bus
// mutex). Returns false if the write fails.
bool strain_gauge_reset(void);
