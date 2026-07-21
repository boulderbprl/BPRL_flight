#include "src/coms/IMUs/ICM45686.hpp"
#include <cstring>

// Bank 0 registers
static constexpr uint8_t REG_WHO_AM_I      = 0x72;
static constexpr uint8_t WHOAMI_VALUE      = 0xE9;

static constexpr uint8_t REG_PWR_MGMT0     = 0x10;
static constexpr uint8_t REG_GYRO_CONFIG0  = 0x1C;  // [7:4]=FS_SEL, [3:0]=ODR
static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x1B;  // [7:4]=FS_SEL, [3:0]=ODR
static constexpr uint8_t REG_FIFO_CONFIG0  = 0x1D;
static constexpr uint8_t REG_FIFO_CONFIG2  = 0x20;  // flush control
static constexpr uint8_t REG_FIFO_CONFIG3  = 0x21;  // GYRO_EN | ACCEL_EN | FIFO_IF_EN
static constexpr uint8_t REG_FIFO_COUNTH   = 0x12;  // 2-byte record count (LE in register layout)
static constexpr uint8_t REG_FIFO_DATA     = 0x14;  // FIFO burst read register

// Indirect IPREG register access (16-bit addressed config banks)
static constexpr uint8_t REG_IREG_ADDRH    = 0x7C;
static constexpr uint8_t REG_IREG_ADDRL    = 0x7D;
static constexpr uint8_t REG_IREG_DATA     = 0x7E;
static constexpr uint8_t REG_MISC2         = 0x7F;  // bit0 = IREG_DATA ready

static constexpr uint16_t BANK_IPREG_SYS1_ADDR = 0xA400;  // GYRO_SRC_CTRL  lives at +0xA6
static constexpr uint16_t BANK_IPREG_SYS2_ADDR = 0xA500;  // ACCEL_SRC_CTRL lives at +0x7B

// ICM-45686 FS_SEL=1 → ±2000 dps / ±16 g (matches ICM-42688 at FS_SEL=0)
static constexpr float GYRO_SCALE  = (1.0f / 16.4f)  * (3.14159265f / 180.0f);
static constexpr float ACCEL_SCALE = (1.0f / 2048.0f) * 9.80665f;

// GYRO_CONFIG0 / ACCEL_CONFIG0: FS_SEL=1 (second entry = ±2000dps / ±16g), ODR=0x05 (~1600 Hz)
// TEMP BISECTION TEST (uncommitted): halfway point between the original
// 800Hz (0x06, no oversampling) and the full 3200Hz (0x04, ~4x oversampling)
// fast-sampling config. Noise reduction scales with 1/sqrt(N) averaged
// samples — 800->1600Hz (~1x -> ~1.6x samples/tick) captures roughly the
// first ~21% of the available noise reduction; the second doubling to
// 3200Hz only adds another ~23% on top for double the packet count and SPI
// cost (measured ~202us avg exec at 3200Hz vs ~12us un-oversampled) — this
// is meant to keep most of the benefit for a fraction of the SPIThread cost.
static constexpr uint8_t FS_ODR_CFG = (0x1 << 4) | 0x05;

bool ICM45686::init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast)
{
    _spid     = spid;
    _cfg_init = cfg_init;
    _cfg_fast = cfg_fast;
    _ready    = false;

    _whoami = reg_read(REG_WHO_AM_I);
    if (_whoami != WHOAMI_VALUE) { return false; }

    // Wake both sensors in low-noise mode (same bit layout as ICM-42688)
    reg_write(REG_PWR_MGMT0, 0x0F);
    chThdSleepMilliseconds(1);

    reg_write(REG_GYRO_CONFIG0,  FS_ODR_CFG);
    reg_write(REG_ACCEL_CONFIG0, FS_ODR_CFG);
    chThdSleepMilliseconds(20);  // accel needs 20 ms after enable

    // Enable the hardware interpolator + anti-alias filter on gyro and accel.
    // Without this the AAF path is bypassed and vibration energy above the
    // digital filter cutoff aliases straight into the sampled data before any
    // downstream software filter can act on it.
    uint8_t src_ctrl = ireg_read(BANK_IPREG_SYS1_ADDR, 0xA6);   // GYRO_SRC_CTRL [6:5]
    ireg_write(BANK_IPREG_SYS1_ADDR, 0xA6, (src_ctrl & ~(0x3 << 5)) | (0x2 << 5));

    src_ctrl = ireg_read(BANK_IPREG_SYS2_ADDR, 0x7B);           // ACCEL_SRC_CTRL [1:0]
    ireg_write(BANK_IPREG_SYS2_ADDR, 0x7B, (src_ctrl & ~0x3) | 0x2);

    // Flush FIFO, then configure streaming
    reg_write(REG_FIFO_CONFIG3, 0x00);   // disable FIFO
    reg_write(REG_FIFO_CONFIG0, 0x00);
    reg_write(REG_FIFO_CONFIG2, 0x80);   // flush command
    chThdSleepMilliseconds(1);
    reg_write(REG_FIFO_CONFIG2, 0x00);   // clear flush

    // Enable accel + gyro in FIFO (FIFO_IF_EN not set yet)
    reg_write(REG_FIFO_CONFIG3, 0x06);   // bit[2]=GYRO_EN, bit[1]=ACCEL_EN
    // Stop-on-full mode, 2K FIFO
    reg_write(REG_FIFO_CONFIG0, 0x87);   // (2<<6)|0x07
    // Now enable FIFO interface output
    reg_write(REG_FIFO_CONFIG3, 0x07);   // add bit[0]=FIFO_IF_EN

    _ready = true;
    return true;
}

bool ICM45686::read(float accel_ms2[3], float gyro_rads[3])
{
    if (!_ready) { return false; }

    // Read FIFO record count (2 bytes, LE: reg[0x12]=low byte, reg[0x13]=high byte)
    uint8_t cnt[2];
    burst_read(REG_FIFO_COUNTH, cnt, 2);
    uint16_t n = static_cast<uint16_t>(cnt[0]) | (static_cast<uint16_t>(cnt[1]) << 8);
    if (n == 0) { return false; }

    // At the ~3.2kHz fast-sampling ODR configured in init(), more than one
    // packet accumulates per 1kHz SPIThread tick. Drain and average every
    // queued packet (capped defensively) rather than a single sample, so the
    // stream gets genuine decimation-by-averaging instead of aliasing by
    // picking one sample out of several.
    static constexpr uint16_t MAX_PACKETS_PER_READ = 8;
    if (n > MAX_PACKETS_PER_READ) { n = MAX_PACKETS_PER_READ; }

    auto le16 = [](const uint8_t *p) -> int16_t {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
    };

    int32_t accel_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3]  = {0, 0, 0};
    uint16_t n_valid = 0;

    // Layout per packet: header(1) + accel[3]×int16_LE(6) + gyro[3]×int16_LE(6) + temp(1) + ts(2)
    for (uint16_t i = 0; i < n; ++i) {
        uint8_t pkt[16];
        burst_read(REG_FIFO_DATA, pkt, 16);

        // Validate header: bit[6]=ACCEL_EN and bit[5]=GYRO_EN must both be set
        if ((pkt[0] & 0x60) != 0x60) { continue; }

        accel_sum[0] += le16(pkt + 1);
        accel_sum[1] += le16(pkt + 3);
        accel_sum[2] += le16(pkt + 5);
        gyro_sum[0]  += le16(pkt + 7);
        gyro_sum[1]  += le16(pkt + 9);
        gyro_sum[2]  += le16(pkt + 11);
        ++n_valid;
    }

    if (n_valid == 0) { return false; }

    const float inv_n = 1.0f / static_cast<float>(n_valid);
    accel_ms2[0] = static_cast<float>(accel_sum[0]) * inv_n * ACCEL_SCALE;
    accel_ms2[1] = static_cast<float>(accel_sum[1]) * inv_n * ACCEL_SCALE;
    accel_ms2[2] = static_cast<float>(accel_sum[2]) * inv_n * ACCEL_SCALE;
    gyro_rads[0] = static_cast<float>(gyro_sum[0]) * inv_n * GYRO_SCALE;
    gyro_rads[1] = static_cast<float>(gyro_sum[1]) * inv_n * GYRO_SCALE;
    gyro_rads[2] = static_cast<float>(gyro_sum[2]) * inv_n * GYRO_SCALE;
    return true;
}

uint8_t ICM45686::reg_read(uint8_t reg)
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

void ICM45686::reg_write(uint8_t reg, uint8_t val)
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

void ICM45686::burst_read(uint8_t reg, uint8_t *buf, size_t n)
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

uint8_t ICM45686::ireg_read(uint16_t bank_addr, uint16_t reg)
{
    const uint16_t addr = bank_addr + reg;
    reg_write(REG_IREG_ADDRH, static_cast<uint8_t>(addr >> 8));
    reg_write(REG_IREG_ADDRL, static_cast<uint8_t>(addr & 0xFF));

    for (uint8_t i = 0; i < 10; ++i) {
        if (reg_read(REG_MISC2) & 0x01) break;
        chThdSleepMicroseconds(10);  // min 4us gap between IREG accesses
    }
    return reg_read(REG_IREG_DATA);
}

void ICM45686::ireg_write(uint16_t bank_addr, uint16_t reg, uint8_t val)
{
    const uint16_t addr = bank_addr + reg;
    reg_write(REG_IREG_ADDRH, static_cast<uint8_t>(addr >> 8));
    reg_write(REG_IREG_ADDRL, static_cast<uint8_t>(addr & 0xFF));
    reg_write(REG_IREG_DATA,  val);

    if (!(reg_read(REG_MISC2) & 0x01)) {
        chThdSleepMicroseconds(10);  // min 4us gap between IREG accesses
    }
}
