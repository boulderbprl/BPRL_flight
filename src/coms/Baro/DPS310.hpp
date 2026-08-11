#pragma once
#include "hal.h"

/*
 * Driver for Infineon DPS310 barometric pressure sensor — I2C, address 0x77.
 * Orqa QuadCore H7's only barometer (BPRL_BOARD_ORQA); the Cube boards use
 * MS5611 on SPI instead (src/coms/Baro/MS5611.hpp).
 *
 * Unlike MS5611's driver, this one follows the procedural I2C-poll pattern
 * used elsewhere in src/coms (see src/sensors/StrainRate.cpp) rather than
 * exposing a class with .read() called from SPIThread: the sensor runs in
 * DPS310's own continuous background-measurement mode, so each poll is a
 * single "is a fresh sample ready, and if so read+compensate it" check —
 * there's no multi-step conversion state machine to drive, so a bare
 * function pair (init + poll) fits the existing I2C device convention
 * better than a class does.
 *
 * dps310_init() registers the poll callback with bprl_i2c_register() —
 * call it once from main() (see src/sensors/StrainRate.cpp's strain_rate_init()
 * for the same pattern). i2c_drv_init() must run first so I2CD2 is started.
 * The poll callback runs from I2CThread (200 Hz) and writes g_baro/baro_mtx
 * directly, same contract as SPIThread's baro1.read() path on the Cube
 * boards: alt_m is POSITIVE UP, relative to the reference altitude captured
 * during an on-boot warm-up window — NOT NED (EKF::update_altitude() does
 * the sign flip).
 */
void dps310_init(void);
