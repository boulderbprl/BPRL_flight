#pragma once
#include "hal.h"

/*
 * Driver for InvenSense ICM-20602 (6-DOF accel/gyro, SPI).
 * On FMUv5x: SPI4 bus, CS = LINE_IMU3_CS (PC13).
 *
 * Call init() once from SPIThread before the read loop.
 * Call read() every SPIThread tick (1 kHz).
 */
class ICM20602 {
public:
    bool init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast);
    bool read(float accel_ms2[3], float gyro_rads[3]);

private:
    uint8_t  reg_read(uint8_t reg);
    void     reg_write(uint8_t reg, uint8_t val);
    void     burst_read(uint8_t reg, uint8_t *buf, size_t n);

    SPIDriver       *_spid     = nullptr;
    const SPIConfig *_cfg_init = nullptr;
    const SPIConfig *_cfg_fast = nullptr;
    bool             _ready    = false;

    // 32-byte aligned so cacheBufferFlush/Invalidate operate on exactly one cache line
    uint8_t _txbuf[32] __attribute__((aligned(32)));
    uint8_t _rxbuf[32] __attribute__((aligned(32)));
};
