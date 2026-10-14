#pragma once
#include <stdint.h>

/*
 * Persistent IMU calibration stored in STM32H743 Bank 2 sector 7
 * (0x081E0000, 128 KB), programmed directly via Bank 2 registers
 * because HAL_USE_EFL = FALSE.
 *
 * Layout: 128 bytes = 4 × 32-byte flash words.
 * Biases are in body frame (post-rotation-correction), subtracted from
 * IMU readings before the EKF predict step.
 */

struct CalibData {
    uint32_t magic;             // 0x42525943 ('BPRC')
    uint32_t version;           // 1
    float    gyro_bias[3][3];   // [imu][xyz]  rad/s
    float    accel_bias[3][3];  // [imu][xyz]  m/s²
    uint32_t crc32;             // CRC-32/ISO-HDLC over bytes [0 .. 80)
    uint8_t  _pad[44];          // pad to 128 bytes
};
static_assert(sizeof(CalibData) == 128, "CalibData must be 128 bytes");

// Read calibration from flash. Returns false and leaves |out| zeroed if
// no valid calibration is stored (bad magic or CRC mismatch). Always
// available — a flight build still needs to load and apply a calibration
// saved earlier from a debug build, it just can't write a new one itself.
bool cal_load(CalibData &out);

// Write path — only compiled in under BPRL_DEBUG (see CalFlash.cpp). Both
// briefly disable interrupts board-wide (ArduPilot's own STM32H7 flash
// driver does the same by default; see the comments at the definitions),
// which must never happen on a flight build regardless of whether anything
// would actually call it — hence gating the functions themselves, not just
// the USB commands that are their only real callers.
#ifdef BPRL_DEBUG
// Erase sector 7 then write |d| to flash.  Returns false on write error.
bool cal_save(const CalibData &d);

// Erase sector 7 (clears calibration).
void cal_clear();
#endif
