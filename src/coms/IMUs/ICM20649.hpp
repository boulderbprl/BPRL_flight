#pragma once
#include "hal.h"

/*
 * Driver for InvenSense ICM-20649 (6-DOF, SPI) — a pin/protocol-compatible
 * "high-g" sibling of the ICM-20948: same register map, but a wider
 * accelerometer range (±30 g vs ±16 g) and a different GYRO_CFG1 FS_SEL
 * encoding for the same ±2000 dps range. Confirmed present in the
 * "primary" IMU slot (SPI1, CS=PC2) on real Cube Blue hardware — WHOAMI
 * reads 0xE1, not ICM-20948's 0xEA, on two separate bench units. ArduPilot's
 * own CubeOrange/hwdef.inc already accounts for exactly this variant on the
 * same physical chip position (CHECK_IMU2_PRESENT $CHECK_ICM20649), so this
 * is the hardware's real, intended population, not a defect.
 *
 * One instance:
 *   imu1 — SPI1, CS = PC2  (primary)
 *
 * Call init() once from spi_drv_init() (inside SPIThread).
 * Call read() every SPIThread tick (1 kHz).
 */
class ICM20649 {
public:
    bool init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast);
    bool read(float accel_ms2[3], float gyro_rads[3]);

private:
    void    set_bank(uint8_t bank);
    uint8_t reg_read(uint8_t reg);
    void    reg_write(uint8_t reg, uint8_t val);
    void    burst_read(uint8_t reg, uint8_t *buf, size_t n);

    SPIDriver       *_spid     = nullptr;
    const SPIConfig *_cfg_init = nullptr;
    const SPIConfig *_cfg_fast = nullptr;
    uint8_t          _bank     = 0xFF;
    bool             _ready    = false;
    uint8_t          _whoami   = 0;

public:
    uint8_t whoami() const { return _whoami; }

private:
    uint8_t _txbuf[32] __attribute__((aligned(32)));
    uint8_t _rxbuf[32] __attribute__((aligned(32)));
};
