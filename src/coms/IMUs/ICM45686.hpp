#pragma once
#include "hal.h"

/*
 * Driver for TDK/InvenSense ICM-45686 (6-DOF accel/gyro, SPI Mode 0).
 * On CubeOrangePlus FMUv5x: SPI1, CS = PG1 (ICM45686_CS).   instance 2
 *
 * Uses FIFO-based data output (16 bytes per packet) at a ~3.2kHz fast-sampling
 * ODR — read() drains and averages every packet queued since the last call,
 * so the 1kHz caller gets a decimated/averaged sample, not an aliased one.
 * Call init() once from spi_drv_init() (inside SPIThread).
 * Call read() every SPIThread tick (1 kHz); returns false when FIFO is empty.
 */
class ICM45686 {
public:
    bool init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast);
    bool read(float accel_ms2[3], float gyro_rads[3]);
    uint8_t whoami() const { return _whoami; }

private:
    uint8_t reg_read(uint8_t reg);
    void    reg_write(uint8_t reg, uint8_t val);
    void    burst_read(uint8_t reg, uint8_t *buf, size_t n);

    // Indirect IPREG bank access (16-bit addressed config space used for the
    // gyro/accel anti-alias-filter enable bits, not directly memory-mapped).
    uint8_t ireg_read(uint16_t bank_addr, uint16_t reg);
    void    ireg_write(uint16_t bank_addr, uint16_t reg, uint8_t val);

    SPIDriver       *_spid     = nullptr;
    const SPIConfig *_cfg_init = nullptr;
    const SPIConfig *_cfg_fast = nullptr;
    bool             _ready    = false;
    uint8_t          _whoami   = 0;

    uint8_t _txbuf[32] __attribute__((aligned(32)));
    uint8_t _rxbuf[32] __attribute__((aligned(32)));
};
