#pragma once
#include "hal.h"

/*
 * Driver for InvenSense ICM-20948 (9-DOF accel/gyro/mag, SPI).
 * On FMUv5x:
 *   IMU1 (primary):  SPI1 bus, CS = LINE_IMU1_CS (PC2)
 *   IMU2 (ext):      SPI4 bus, CS = LINE_IMU2_CS (PE4)
 *
 * Call init() once from SPIThread before the read loop.
 * Call read() every SPIThread tick (1 kHz).
 * Magnetometer is not read by this driver (handled separately via AK09916 slave).
 */
class ICM20948 {
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
    uint8_t          _bank     = 0xFF; // invalid; forces first set_bank() call
    bool             _ready    = false;

    uint8_t _txbuf[32] __attribute__((aligned(32)));
    uint8_t _rxbuf[32] __attribute__((aligned(32)));
};
