#pragma once
#include "ch.h"
#include "hal.h"

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
 * Hz field of the STR1/STR3 log messages).
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
                              // each chunk's own receive rate (logged as STR1/STR3 Hz)
};

extern mutex_t        strainGauge_mtx;
extern StrainGaugeRaw g_strain_gauge;

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
