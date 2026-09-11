#pragma once
#include "ch.h"
#include "hal.h"

/*
 * Shaft-angle encoder RPM sensor nodes — Feather M4 CAN Express + AS5047P,
 * one node per motor, for BDShot RPM cross-validation on the strain-gauge
 * drone. See Strain_CAN/Feather_Code/Feather_Code.ino for the sensor-side
 * firmware (NODE_ID selects which CAN address a given board broadcasts on).
 *
 * CAN IDs 0x70 (node 0), 0x71 (node 1), ... one per node, 8-byte frame,
 * little-endian (matches STM32's native byte order):
 *   bytes 0-3  float    rpm         filtered mechanical RPM, signed
 *   bytes 4-5  uint16_t angle_raw   last raw 14-bit shaft angle (0-16383/rev)
 *   byte  6    uint8_t  error_flag  AS5047P EF bit latched on last sample
 *   byte  7    (reserved)
 */

#define ENCODER_RPM_NUM_NODES   2
#define ENCODER_RPM_CAN_ID_BASE 0x70

struct EncoderRPMRaw {
    float    rpm;         // filtered mechanical RPM, signed by rotation direction
    uint16_t angle_raw;   // last raw 14-bit shaft angle (0-16383 counts / rev)
    uint8_t  error_flag;  // AS5047P EF bit latched on the last sample
    bool     valid;       // true once at least one frame has arrived from this node
};

extern mutex_t       encoderRpm_mtx;
extern EncoderRPMRaw g_encoder_rpm[ENCODER_RPM_NUM_NODES];

// Call after can_drv_init() in main.cpp. Registers one CAN callback per node
// (0x70, 0x71, ... up to ENCODER_RPM_NUM_NODES).
void encoder_rpm_init(void);
