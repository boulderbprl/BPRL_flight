#pragma once
#include "ch.h"
#include "hal.h"

/*
 * Strain gauge array sensor — a single I2C node at 0x09. Mirrors
 * ArduPilot's AP_Strain library (Documents/ardupilot/libraries/AP_Strain/
 * AP_Strain*.{h,cpp}), specifically its "full" (non-USE_STRAIN_RATE_SENSOR)
 * path — this is a different physical sensor from src/sensors/
 * StrainRate.hpp, which mirrors AP_Strain's other, mutually-exclusive
 * USE_STRAIN_RATE_SENSOR path (4 channels, single sensor at 0x11). Per
 * ArduPilot's own #ifdef, only one of the two is ever active at a time —
 * for now, only one of strain_gauge_init()/strain_rate_init() should be
 * called from main.cpp.
 *
 * Wire protocol, transcribed from AP_Strain_Backend::get_reading() (the
 * #else / non-rate branch), with the payload type adjusted per the sensor
 * firmware's own rewrite from float32 to int16 (ArduPilot's upstream
 * driver still sends float32 — this project's sensor firmware diverges
 * here):
 *   1. Write 'P' (0x50)              — trigger the sensor to capture a frame.
 *   2. For chunk in 0..2:
 *        write the chunk index (0/1/2), then read 24 bytes back
 *        (12 x int16_t, little-endian).
 *
 * Bench-verified (2026-09, tools/telemetry.py strain): the "chunk index"
 * byte does not actually select distinct data on this sensor. All 3 chunks
 * return the same live register snapshot; small differences seen between
 * them in early testing were just measurement noise between two back-to-
 * back reads of the same live signal, not evidence of separate data banks
 * (confirmed once two "different" chunks came back reading identically at
 * idle, and again when physically loading one arm moved the same channels
 * in both "chunk 0" and "chunk 1" reads). Within one chunk's 12 int16s,
 * the first 3 are always zero (padding/reserved) and the last 9 are real.
 * So this device exposes exactly 9 real channels, once — chunks 1 and 2
 * are pure duplicates of chunk 0 and are read but discarded.
 *
 * Of those 9 real channels, a physical touch test (flexing one arm at a
 * time and watching which channels moved) found the arm boundary sits
 * mid-array, not chunk-aligned: channels 0-4 (5 channels) respond to one
 * physical arm, channels 5-8 (4 channels) respond to the other. Labeled
 * below as ARM1/ARM2 in that order — swap the labels if that turns out
 * backwards relative to which physical arm you called "1" on the bench.
 *
 * Open question, not yet resolved: this is only 2 of the drone's 4 arms.
 * Whether the other 2 live on a separate I2C address, need a bank-select
 * command, or simply aren't wired to this bench unit yet is still unknown.
 *
 * Calibration (AP_Strain_Backend::calibrate(), byte 0x5A) and reset
 * (AP_Strain_Backend::reset(), byte 'R'/0x52) are separate single-byte
 * writes with no reply expected.
 *
 * Timing note: a full read is 1 trigger write + 3 chunked
 * write-then-read-24-bytes transactions — noticeably heavier than this
 * project's other I2C devices (StrainRate.hpp's single 8-byte read). At
 * 400 kHz that's roughly ~0.7 ms/chunk including addressing overhead, so
 * ~2 ms for a full read cycle — well within I2CThread's 200 Hz / 5 ms
 * budget (see src/threads.cpp); bench-confirmed running at 200 Hz. Chunks
 * 1 and 2 are still requested over the wire (rather than dropping to a
 * single chunk transaction) since changing the transaction count on this
 * still not-fully-understood sensor risks desyncing its firmware.
 */

#define STRAIN_GAUGE_NUM_CHANNELS    9  // total real channels (see note above)
#define STRAIN_GAUGE_ARM1_CHANNELS   5  // data[0..4]
#define STRAIN_GAUGE_ARM2_CHANNELS   4  // data[5..8]

struct StrainGaugeRaw {
    int16_t  data[STRAIN_GAUGE_NUM_CHANNELS];  // 9 real strain measurements
    bool     valid;          // true once at least one full frame has been read
    uint32_t last_update_ms; // chVTGetSystemTime()-based timestamp of the last good read
    uint32_t update_count;   // incremented on every successful frame read — diff over
                              // a fixed window (e.g. 1s) to get the sensor's actual Hz
};

extern mutex_t        strainGauge_mtx;
extern StrainGaugeRaw g_strain_gauge;

// Call after i2c_drv_init() in main.cpp. Registers the I2C poll function
// for the sensor at 0x09. Do not also call strain_rate_init() — see the
// mutual-exclusivity note above.
void strain_gauge_init(void);

// Queue a calibration byte (0x5A) to be sent on the next poll tick — mirrors
// AP_Strain_Backend's _cal_pending flag (the actual I2C write happens from
// inside the poll callback, which already owns the bus).
void strain_gauge_calibrate(void);

// Send the reset byte ('R') immediately (blocking on the I2C bus mutex).
// Returns false if the write fails.
bool strain_gauge_reset(void);
