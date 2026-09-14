#pragma once
#include "ch.h"
#include "hal.h"

/*
 * Strain gauge array sensor — up to 2 sensor nodes, 36 raw measurements
 * each, I2C addr 0x09 / 0x10 (0x09 + 7*node). Mirrors ArduPilot's AP_Strain
 * library (Documents/ardupilot/libraries/AP_Strain/AP_Strain*.{h,cpp}),
 * specifically its "full" (non-USE_STRAIN_RATE_SENSOR) path — this is a
 * different physical sensor from src/sensors/StrainRate.hpp, which mirrors
 * AP_Strain's other, mutually-exclusive USE_STRAIN_RATE_SENSOR path
 * (4 channels, single sensor at 0x11). Per ArduPilot's own #ifdef, only one
 * of the two is ever active at a time — for now, only one of
 * strain_gauge_init()/strain_rate_init() should be called from main.cpp.
 *
 * Wire protocol per node, transcribed from AP_Strain_Backend::get_reading()
 * (the #else / non-rate branch), with the payload type adjusted per the
 * sensor firmware's own rewrite from float32 to int16 (ArduPilot's upstream
 * driver still sends float32 — this project's sensor firmware diverges here):
 *   1. Write 'P' (0x50)              — trigger the sensor to capture a frame.
 *   2. For chunk in 0..2:
 *        write the chunk index (0/1/2), then read 24 bytes back
 *        (12 x int16_t, little-endian) — 3 chunks x 12 = 36 int16s total.
 * Calibration (AP_Strain_Backend::calibrate(), byte 0x5A) and reset
 * (AP_Strain_Backend::reset(), byte 'R'/0x52) are separate single-byte
 * writes with no reply expected.
 *
 * Timing note: a full node read is 1 trigger write + 3 chunked
 * write-then-read-24-bytes transactions — noticeably heavier than this
 * project's other I2C devices (StrainRate.hpp's single 8-byte read). At
 * 400 kHz that's roughly ~0.7 ms/chunk including addressing overhead, so
 * ~2 ms for one node's full cycle — with both nodes registered, a single
 * I2CThread tick (200 Hz / 5 ms budget, see src/threads.cpp) stays within
 * budget under normal conditions, but is worth re-checking against real
 * hardware timing once available.
 */

#define STRAIN_GAUGE_NUM_SENSORS   2     // node n -> I2C addr 0x09 + 7*n (0x09, 0x10)
#define STRAIN_GAUGE_NUM_CHANNELS  36    // raw measurements per node

struct StrainGaugeRaw {
    int16_t  data[STRAIN_GAUGE_NUM_CHANNELS];  // 36 raw strain measurements
    bool     valid;          // true once at least one full 36-channel frame has been read
    uint32_t last_update_ms; // chVTGetSystemTime()-based timestamp of the last good read
};

extern mutex_t        strainGauge_mtx;
extern StrainGaugeRaw g_strain_gauge[STRAIN_GAUGE_NUM_SENSORS];

// Call after i2c_drv_init() in main.cpp. Registers one I2C poll function per
// sensor node (0x09, 0x10). Do not also call strain_rate_init() — see the
// mutual-exclusivity note above.
void strain_gauge_init(void);

// Queue a calibration byte (0x5A) to be sent to one node on its next poll
// tick — mirrors AP_Strain_Backend's _cal_pending flag (the actual I2C write
// happens from inside the poll callback, which already owns the bus).
void strain_gauge_calibrate(uint8_t node);

// Send the reset byte ('R') to one node immediately (blocking on the I2C
// bus mutex). Returns false if the node index is invalid or the write fails.
bool strain_gauge_reset(uint8_t node);
