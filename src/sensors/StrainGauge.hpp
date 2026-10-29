#pragma once
#include "ch.h"
#include "hal.h"
#include "src/sensors/SensorHealth.hpp"

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
 * TODO — strain -> angular acceleration fit and per-channel health.
 * Strain is only logged today; nothing here feeds a controller yet. When
 * the fit goes in (in these StrainGauge files):
 *
 *  1. Fit. p_dot/q_dot = sum over the ~30 good channels of weight[i] x
 *     data[i] (+ bias), weights fitted in MATLAB (strain cal pipeline) and
 *     imported as a static table. Channels left out of the fit get weight 0.
 *     Applied every published frame: ~30 multiply-adds per axis, nothing.
 *     Keep the result's delay matched to the RPM -> torque path INDI
 *     differences it against (~10 ms target).
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
 *     INDI falls back to PID — refit with fewer channels in MATLAB rather
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
