#include "src/coms/IMUs/ICM20649.hpp"
#include <cstring>

// Register map — Bank 0 (identical layout to ICM-20948)
static constexpr uint8_t B0_WHO_AM_I      = 0x00;
static constexpr uint8_t B0_USER_CTRL     = 0x03;
static constexpr uint8_t B0_PWR_MGMT_1    = 0x06;
static constexpr uint8_t B0_ACCEL_XOUT_H  = 0x2D;

// Register map — Bank 2 (identical layout to ICM-20948)
static constexpr uint8_t B2_GYRO_SMPLRT   = 0x00;
static constexpr uint8_t B2_GYRO_CFG1     = 0x01;
static constexpr uint8_t B2_ACCEL_SMPL_1  = 0x10;
static constexpr uint8_t B2_ACCEL_SMPL_2  = 0x11;
static constexpr uint8_t B2_ACCEL_CFG     = 0x14;

static constexpr uint8_t REG_BANK_SEL     = 0x7F;

// Confirmed against ArduPilot's AP_InertialSensor_Invensensev2_registers.h:
// INV2_WHOAMI_ICM20649 = 0xE1 (ICM-20948's is 0xEA — do not confuse them).
static constexpr uint8_t WHOAMI_VALUE     = 0xE1;

// GYRO_CFG1 FS_SEL: ICM-20649 uses a DIFFERENT bit pattern than ICM-20948
// for the same ±2000 dps range (ArduPilot: BITS_GYRO_FS_2000DPS_20649 =
// 0x04, vs. BITS_GYRO_FS_2000DPS = 0x06 for plain ICM-20948) — this is not
// interchangeable with ICM20948.cpp's value.
static constexpr uint8_t GYRO_FS_2000DPS_20649 = 0x04;

// ±2000 dps → rad/s — same physical sensitivity as ICM-20948 once
// configured to the same range (only the config bits to get there differ).
// ±30 g → m/s²  (1024 LSB/g; ArduPilot AP_InertialSensor_Invensensev2.cpp:
// "20649 is setup for 30g full scale, 1024 LSB/g" — NOT ICM-20948's 2048
// LSB/g, even though both chips take the same raw ACCEL_CFG FS_SEL bits
// (0x06) to select their respective maximum range).
static constexpr float GYRO_SCALE  = (1.0f / 16.4f)  * (3.14159265f / 180.0f);
static constexpr float ACCEL_SCALE = (1.0f / 1024.0f) * 9.80665f;

void ICM20649::set_bank(uint8_t bank)
{
    if (bank == _bank) { return; }
    _txbuf[0] = REG_BANK_SEL & 0x7F;
    _txbuf[1] = static_cast<uint8_t>(bank << 4);
    cacheBufferFlush(_txbuf, 32);
    spiAcquireBus(_spid);
    spiStart(_spid, _cfg_init);
    spiSelect(_spid);
    spiSend(_spid, 2, _txbuf);
    spiUnselect(_spid);
    spiReleaseBus(_spid);
    _bank = bank;
}

bool ICM20649::init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast)
{
    _spid     = spid;
    _cfg_init = cfg_init;
    _cfg_fast = cfg_fast;
    _bank     = 0xFF;
    _ready    = false;

    set_bank(0);
    _whoami = reg_read(B0_WHO_AM_I);
    if (_whoami != WHOAMI_VALUE) { return false; }

    reg_write(B0_PWR_MGMT_1, 0xC1);   // reset + sleep
    chThdSleepMilliseconds(100);
    reg_write(B0_PWR_MGMT_1, 0x01);   // wake, auto clock
    chThdSleepMilliseconds(5);
    reg_write(B0_USER_CTRL,  0x10);   // SPI-only mode

    set_bank(2);
    reg_write(B2_GYRO_SMPLRT,  0x00);                         // 1.125 kHz ODR
    reg_write(B2_GYRO_CFG1,    GYRO_FS_2000DPS_20649 | 0x01); // ±2000 dps, DLPF on
    reg_write(B2_ACCEL_SMPL_1, 0x00);
    reg_write(B2_ACCEL_SMPL_2, 0x00); // 1.125 kHz ODR
    reg_write(B2_ACCEL_CFG,    0x19); // ±30 g (max range — same raw FS_SEL
                                       // bits as ICM-20948's ±16g setting;
                                       // see ACCEL_SCALE above), DLPF on

    set_bank(0);
    _ready = true;
    return true;
}

bool ICM20649::read(float accel_ms2[3], float gyro_rads[3])
{
    if (!_ready) { return false; }
    set_bank(0);

    uint8_t raw[12];
    burst_read(B0_ACCEL_XOUT_H, raw, 12);

    auto be16 = [](const uint8_t *p) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
    };

    accel_ms2[0] = be16(raw + 0) * ACCEL_SCALE;
    accel_ms2[1] = be16(raw + 2) * ACCEL_SCALE;
    accel_ms2[2] = be16(raw + 4) * ACCEL_SCALE;
    gyro_rads[0] = be16(raw + 6) * GYRO_SCALE;
    gyro_rads[1] = be16(raw + 8) * GYRO_SCALE;
    gyro_rads[2] = be16(raw + 10) * GYRO_SCALE;
    return true;
}

uint8_t ICM20649::reg_read(uint8_t reg)
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

void ICM20649::reg_write(uint8_t reg, uint8_t val)
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

void ICM20649::burst_read(uint8_t reg, uint8_t *buf, size_t n)
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
