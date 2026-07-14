#include "src/coms/IMUs/ICM42688.hpp"
#include <cstring>

static constexpr uint8_t REG_PWR_MGMT0     = 0x4E;
static constexpr uint8_t REG_GYRO_CONFIG0  = 0x4F;
static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
static constexpr uint8_t REG_INTF_CONFIG1  = 0x4D;  // stuck-gyro erratum workaround
static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x1F;  // burst: 6 accel + 6 gyro bytes
static constexpr uint8_t REG_WHO_AM_I      = 0x75;
static constexpr uint8_t REG_BANK_SEL      = 0x76;
static constexpr uint8_t WHOAMI_VALUE      = 0x47;

// Bank 1 — gyro anti-alias filter (AAF) static config
static constexpr uint8_t REG_GYRO_CONFIG_STATIC2 = 0x0B;  // AAF/notch enable bits
static constexpr uint8_t REG_GYRO_CONFIG_STATIC3 = 0x0C;  // GYRO_AAF_DELT
static constexpr uint8_t REG_GYRO_CONFIG_STATIC4 = 0x0D;  // GYRO_AAF_DELTSQR[7:0]
static constexpr uint8_t REG_GYRO_CONFIG_STATIC5 = 0x0E;  // GYRO_AAF_BITSHIFT | DELTSQR[11:8]

// Bank 2 — accel anti-alias filter (AAF) static config
static constexpr uint8_t REG_ACCEL_CONFIG_STATIC2 = 0x03;  // ACCEL_AAF_DELT | enable bit
static constexpr uint8_t REG_ACCEL_CONFIG_STATIC3 = 0x04;  // ACCEL_AAF_DELTSQR[7:0]
static constexpr uint8_t REG_ACCEL_CONFIG_STATIC4 = 0x05;  // ACCEL_AAF_BITSHIFT | DELTSQR[11:8]

// ±2000 dps, ±16 g  (FS_SEL = 0b000 for each → same as ICM-20602 at same range)
static constexpr float GYRO_SCALE  = (1.0f / 16.4f)  * (3.14159265f / 180.0f);
static constexpr float ACCEL_SCALE = (1.0f / 2048.0f) * 9.80665f;

bool ICM42688::init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast)
{
    _spid     = spid;
    _cfg_init = cfg_init;
    _cfg_fast = cfg_fast;
    _ready    = false;

    _whoami = reg_read(REG_WHO_AM_I);
    if (_whoami != WHOAMI_VALUE) { return false; }

    // Disable both sensors while reconfiguring (datasheet §12.9 recommended sequence)
    reg_write(REG_PWR_MGMT0, 0x00);

    // ±2000 dps, 1 kHz ODR  (FS_SEL[7:5]=000, ODR[3:0]=0110)
    reg_write(REG_GYRO_CONFIG0,  0x06);
    // ±16 g, 1 kHz ODR
    reg_write(REG_ACCEL_CONFIG0, 0x06);

    // Hardware anti-alias filter (AAF), matches ArduPilot's default 1 kHz-ODR
    // tuning: ~258 Hz gyro AAF, ~213 Hz accel AAF. Without this the chip's
    // AAF is left at its power-on default, and vibration energy above the
    // digital LPF cutoff aliases into the passband before any downstream
    // software filter ever sees it.
    const uint8_t aaf_enable = reg_read_bank(1, REG_GYRO_CONFIG_STATIC2);
    reg_write_bank(1, REG_GYRO_CONFIG_STATIC2, aaf_enable & ~0x03);      // enable AAF + notch
    reg_write_bank(1, REG_GYRO_CONFIG_STATIC3, 6);                       // GYRO_AAF_DELT
    reg_write_bank(1, REG_GYRO_CONFIG_STATIC4, 36 & 0xFF);               // GYRO_AAF_DELTSQR[7:0]
    reg_write_bank(1, REG_GYRO_CONFIG_STATIC5, ((10 << 4) & 0xF0) | ((36 >> 8) & 0x0F));

    reg_write_bank(2, REG_ACCEL_CONFIG_STATIC2, 5 << 1);                 // ACCEL_AAF_DELT + enable
    reg_write_bank(2, REG_ACCEL_CONFIG_STATIC3, 25 & 0xFF);              // ACCEL_AAF_DELTSQR[7:0]
    reg_write_bank(2, REG_ACCEL_CONFIG_STATIC4, ((10 << 4) & 0xF0) | ((25 >> 8) & 0x0F));

    // Stuck-gyro erratum: disable AFSR (angular-rate-dependent noise scaling),
    // which otherwise freezes gyro output for ~2 ms whenever the rate crosses
    // ~100 deg/s, producing a spurious DC bias burst.
    const uint8_t intf_cfg1 = reg_read(REG_INTF_CONFIG1);
    reg_write(REG_INTF_CONFIG1, (intf_cfg1 & 0x3F) | 0x40);

    // Wake both sensors in low-noise mode (bits [3:2]=11 gyro LN, [1:0]=11 accel LN)
    reg_write(REG_PWR_MGMT0, 0x0F);
    chThdSleepMilliseconds(20);  // accel needs 20 ms after enable

    _ready = true;
    return true;
}

bool ICM42688::read(float accel_ms2[3], float gyro_rads[3])
{
    if (!_ready) { return false; }

    // Direct register burst: ACCEL_DATA_X1..Z0 (0x1F-0x24) then GYRO_DATA_X1..Z0 (0x25-0x2A)
    uint8_t raw[12];
    burst_read(REG_ACCEL_XOUT_H, raw, 12);

    // Big-endian: _X1 (high byte) precedes _X0 (low byte)
    auto be16 = [](const uint8_t *p) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
    };

    accel_ms2[0] = be16(raw + 0)  * ACCEL_SCALE;
    accel_ms2[1] = be16(raw + 2)  * ACCEL_SCALE;
    accel_ms2[2] = be16(raw + 4)  * ACCEL_SCALE;
    gyro_rads[0] = be16(raw + 6)  * GYRO_SCALE;
    gyro_rads[1] = be16(raw + 8)  * GYRO_SCALE;
    gyro_rads[2] = be16(raw + 10) * GYRO_SCALE;
    return true;
}

uint8_t ICM42688::reg_read(uint8_t reg)
{
    _txbuf[0] = reg | 0x80;
    _txbuf[1] = 0xFF;
    cacheBufferFlush(_txbuf, 32);
    spiAcquireBus(_spid);
    spiStart(_spid, _cfg_init);
    spiSelect(_spid);
    spiExchange(_spid, 2, _txbuf, _rxbuf);
    spiUnselect(_spid);
    spiReleaseBus(_spid);
    cacheBufferInvalidate(_rxbuf, 32);
    return _rxbuf[1];
}

void ICM42688::reg_write(uint8_t reg, uint8_t val)
{
    _txbuf[0] = reg & 0x7F;
    _txbuf[1] = val;
    cacheBufferFlush(_txbuf, 32);
    spiAcquireBus(_spid);
    spiStart(_spid, _cfg_init);
    spiSelect(_spid);
    spiSend(_spid, 2, _txbuf);
    spiUnselect(_spid);
    spiReleaseBus(_spid);
}

void ICM42688::burst_read(uint8_t reg, uint8_t *buf, size_t n)
{
    _txbuf[0] = reg | 0x80;
    memset(_txbuf + 1, 0xFF, n);
    cacheBufferFlush(_txbuf, 32);
    spiAcquireBus(_spid);
    spiStart(_spid, _cfg_fast);
    spiSelect(_spid);
    spiExchange(_spid, n + 1, _txbuf, _rxbuf);
    spiUnselect(_spid);
    spiReleaseBus(_spid);
    cacheBufferInvalidate(_rxbuf, 32);
    memcpy(buf, _rxbuf + 1, n);
}

// Bank 1/2 static-config registers (AAF) share address space with bank-0
// registers — REG_BANK_SEL must be switched before and back to 0 after.
uint8_t ICM42688::reg_read_bank(uint8_t bank, uint8_t reg)
{
    reg_write(REG_BANK_SEL, bank);
    const uint8_t val = reg_read(reg);
    reg_write(REG_BANK_SEL, 0);
    return val;
}

void ICM42688::reg_write_bank(uint8_t bank, uint8_t reg, uint8_t val)
{
    reg_write(REG_BANK_SEL, bank);
    reg_write(reg, val);
    reg_write(REG_BANK_SEL, 0);
}
