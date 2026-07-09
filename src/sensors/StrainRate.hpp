#pragma once
#include "ch.h"
#include "hal.h"

/*
 * Strain rate sensor — 4-axis, one reading per arm [FR, RL, FL, RR].
 *
 * Interface selection (compile-time):
 *   STRAIN_RATE_CAN  — CAN ID 0x69, 4×int16 little-endian, 8-byte frame (default)
 *   STRAIN_RATE_I2C  — I2C polling at 0x11, 400 kHz
 *
 * Override at the command line: -DSTRAIN_RATE_INTERFACE=STRAIN_RATE_I2C
 */

#define STRAIN_RATE_CAN 0
#define STRAIN_RATE_I2C 1

#ifndef STRAIN_RATE_INTERFACE
#define STRAIN_RATE_INTERFACE STRAIN_RATE_CAN
#endif

struct StrainRateRaw {
    int16_t val[4];  // one signed int16 per arm [FR, RL, FL, RR]
    bool    valid;   // true once at least one frame has arrived
};

extern mutex_t       strainRate_mtx;
extern StrainRateRaw g_strain_rate;

// Call after can_drv_init() / i2c_drv_init() in main.cpp.
// Registers the appropriate CAN callback or I2C poll function.
void strain_rate_init(void);
