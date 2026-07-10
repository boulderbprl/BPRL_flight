#pragma once
#include "hal.h"

/*
 * Driver for TE/Measurement Specialties MS5611 barometric pressure sensor
 * (SPI mode 3). On CubeOrangePlus: SPI1, CS = PD7 (BARO_CS), shares the bus
 * with imu1 (see src/coms/SPI.cpp).
 *
 * Unlike the IMU drivers, a full pressure+temperature sample can't be read
 * in one transaction — the chip needs a multi-millisecond ADC conversion
 * per channel. read() is therefore a small state machine: call it once per
 * SPIThread tick (1 kHz), it advances by at most one short SPI transaction
 * per call, and returns true only when a full compensated P+T pair (and
 * derived altitude) is ready — roughly every 6th call at OSR=1024.
 *
 * Call init() once from spi_drv_init() (inside SPIThread, tolerates
 * chThdSleepMilliseconds for reset + PROM read + warm-up).
 */
class MS5611 {
public:
    bool init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast);

    // Call every SPIThread tick (1 kHz). alt_m is POSITIVE UP, relative to
    // the reference altitude captured during init()'s warm-up phase — NOT
    // NED (the sign flip to NED-down happens in EKF::update_altitude()).
    // Returns false while warming up / mid-conversion; true once per
    // completed P+T pair.
    bool read(float &pressure_pa, float &temperature_c, float &alt_m);

private:
    enum class State : uint8_t {
        RESET, PROM_READ, CONVERT_D1, WAIT_D1, READ_D1,
        CONVERT_D2, WAIT_D2, READ_D2, COMPUTE
    };

    SPIDriver       *_spid     = nullptr;
    const SPIConfig *_cfg_init = nullptr;
    const SPIConfig *_cfg_fast = nullptr;
    bool             _ready    = false;

    uint16_t  _prom[6]  = {};   // C1..C6 factory calibration coefficients
    uint32_t  _d1_raw   = 0;    // raw pressure ADC (24-bit)
    uint32_t  _d2_raw   = 0;    // raw temperature ADC (24-bit)
    State     _state      = State::RESET;
    systime_t _wait_start  = 0;   // chVTGetSystemTime() at start of the current conversion wait

    static constexpr int WARMUP_SAMPLES = 8;
    bool  _ref_captured = false;
    int   _warmup_count = 0;
    float _warmup_sum_m = 0.0f;
    float _alt_ref_m    = 0.0f;

    uint8_t _txbuf[8] __attribute__((aligned(32)));
    uint8_t _rxbuf[8] __attribute__((aligned(32)));

    void _cmd(uint8_t cmd);
    void _read_bytes(uint8_t cmd, uint8_t *buf, size_t n);
    void _compensate(float &pressure_pa, float &temperature_c);
};
