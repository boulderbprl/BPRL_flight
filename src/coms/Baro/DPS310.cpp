#include "src/coms/Baro/DPS310.hpp"
#include "src/coms/I2C.hpp"
#include "src/threads.hpp"
#include <cmath>

/*
 * Infineon DPS310 register map (relevant subset):
 *   0x00-0x02  PRS_B2/B1/B0     raw pressure, 24-bit signed, MSB first
 *   0x03-0x05  TMP_B2/B1/B0     raw temperature, 24-bit signed, MSB first
 *   0x06       PRS_CFG          [6:4]=rate [3:0]=oversample
 *   0x07       TMP_CFG          bit7=TMP_EXT (must match COEF_SRCE bit7),
 *                                [6:4]=rate [3:0]=oversample
 *   0x08       MEAS_CFG         bit7=COEF_RDY bit6=SENSOR_RDY bit5=TMP_RDY
 *                                bit4=PRS_RDY [2:0]=MEAS_CTRL
 *   0x0C       RESET            write 0x09 to soft-reset
 *   0x0D       PRODUCT_ID       expect 0x10
 *   0x10-0x21  COEF             18 bytes, c0/c1/c00/c10/c01/c11/c20/c21/c30
 *   0x28       COEF_SRCE        bit7 = which temp sensor calibration used
 *
 * Config below uses oversample=1x (no CFG_REG shift bits needed) at 8
 * measurements/sec for both P and T, in continuous background mode — noisier
 * than the 16x+ oversampled configs some drivers use, but avoids the extra
 * CFG_REG T_SHIFT/P_SHIFT bit-setting only needed above 8x, and 8 Hz is
 * comfortably above I2CThread's ability to notice a fresh sample (200 Hz
 * poll rate against ~125 ms between samples).
 */
static constexpr uint8_t I2C_ADDR      = 0x77;
static constexpr uint8_t REG_PRS_B2    = 0x00;  // 6-byte burst: PRS_B2..TMP_B0
static constexpr uint8_t REG_PRS_CFG   = 0x06;
static constexpr uint8_t REG_TMP_CFG   = 0x07;
static constexpr uint8_t REG_MEAS_CFG  = 0x08;
static constexpr uint8_t REG_RESET     = 0x0C;
static constexpr uint8_t REG_PRODUCT_ID= 0x0D;
static constexpr uint8_t REG_COEF      = 0x10;
static constexpr uint8_t REG_COEF_SRCE = 0x28;

static constexpr uint8_t RESET_CMD          = 0x09;
static constexpr uint8_t MEAS_CTRL_CONT_PT  = 0x07;  // continuous pressure+temperature
static constexpr uint8_t CFG_RATE8_OSR1     = 0x30;  // rate=8Hz(0b011<<4), oversample=1x(0b0000)
static constexpr uint8_t MEAS_CFG_RDY_MASK  = 0x30;  // PRS_RDY|TMP_RDY

static constexpr float KP_KT_OSR1 = 524288.0f;  // datasheet scale factor for oversample=1x

static constexpr float PRESSURE_MIN_PA = 30000.0f;
static constexpr float PRESSURE_MAX_PA = 110000.0f;
static constexpr int   WARMUP_SAMPLES  = 8;

// Sign-extended calibration coefficients, read once at init.
static int32_t s_c0, s_c1, s_c00, s_c10, s_c01, s_c11, s_c20, s_c21, s_c30;
static bool    s_ready = false;

static bool    s_ref_captured = false;
static int     s_warmup_count = 0;
static float   s_warmup_sum_m = 0.0f;
static float   s_alt_ref_m    = 0.0f;

static bool i2c_write_reg(uint8_t reg, uint8_t val)
{
    const uint8_t tx[2] = { reg, val };
    i2cAcquireBus(&I2CD2);
    msg_t status = i2cMasterTransmitTimeout(&I2CD2, I2C_ADDR, tx, 2, nullptr, 0, TIME_MS2I(5));
    i2cReleaseBus(&I2CD2);
    return status == MSG_OK;
}

static bool i2c_read_regs(uint8_t reg, uint8_t *buf, size_t n)
{
    i2cAcquireBus(&I2CD2);
    msg_t status = i2cMasterTransmitTimeout(&I2CD2, I2C_ADDR, &reg, 1, buf, n, TIME_MS2I(5));
    i2cReleaseBus(&I2CD2);
    return status == MSG_OK;
}

static int32_t sign_extend(uint32_t v, int bits)
{
    const uint32_t sign_bit = 1UL << (bits - 1);
    return (v & sign_bit) ? static_cast<int32_t>(v - (sign_bit << 1)) : static_cast<int32_t>(v);
}

static void decode_coefficients(const uint8_t *b)
{
    s_c0  = sign_extend((static_cast<uint32_t>(b[0]) << 4) | (b[1] >> 4), 12);
    s_c1  = sign_extend(((static_cast<uint32_t>(b[1]) & 0x0FU) << 8) | b[2], 12);
    s_c00 = sign_extend((static_cast<uint32_t>(b[3]) << 12) | (static_cast<uint32_t>(b[4]) << 4) | (b[5] >> 4), 20);
    s_c10 = sign_extend(((static_cast<uint32_t>(b[5]) & 0x0FU) << 16) | (static_cast<uint32_t>(b[6]) << 8) | b[7], 20);
    s_c01 = sign_extend((static_cast<uint32_t>(b[8])  << 8) | b[9],  16);
    s_c11 = sign_extend((static_cast<uint32_t>(b[10]) << 8) | b[11], 16);
    s_c20 = sign_extend((static_cast<uint32_t>(b[12]) << 8) | b[13], 16);
    s_c21 = sign_extend((static_cast<uint32_t>(b[14]) << 8) | b[15], 16);
    s_c30 = sign_extend((static_cast<uint32_t>(b[16]) << 8) | b[17], 16);
}

static void dps310_poll(void *ctx)
{
    (void)ctx;
    if (!s_ready) return;

    uint8_t meas_cfg;
    if (!i2c_read_regs(REG_MEAS_CFG, &meas_cfg, 1)) return;
    if ((meas_cfg & MEAS_CFG_RDY_MASK) != MEAS_CFG_RDY_MASK) return;  // no fresh P+T pair yet

    uint8_t raw[6];
    if (!i2c_read_regs(REG_PRS_B2, raw, 6)) return;

    const int32_t praw = sign_extend((static_cast<uint32_t>(raw[0]) << 16)
                                    | (static_cast<uint32_t>(raw[1]) << 8) | raw[2], 24);
    const int32_t traw = sign_extend((static_cast<uint32_t>(raw[3]) << 16)
                                    | (static_cast<uint32_t>(raw[4]) << 8) | raw[5], 24);

    const float p_sc = static_cast<float>(praw) / KP_KT_OSR1;
    const float t_sc = static_cast<float>(traw) / KP_KT_OSR1;

    // DPS310 datasheet compensation formula — result is Pa directly.
    const float pressure_pa = static_cast<float>(s_c00)
        + p_sc * (static_cast<float>(s_c10) + p_sc * (static_cast<float>(s_c20) + p_sc * static_cast<float>(s_c30)))
        + t_sc * static_cast<float>(s_c01)
        + t_sc * p_sc * (static_cast<float>(s_c11) + p_sc * static_cast<float>(s_c21));
    const float temperature_c = static_cast<float>(s_c0) * 0.5f + static_cast<float>(s_c1) * t_sc;

    if (pressure_pa < PRESSURE_MIN_PA || pressure_pa > PRESSURE_MAX_PA) return;  // corrupted read

    const float alt_raw_m = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.1903f));

    if (!s_ref_captured) {
        s_warmup_sum_m += alt_raw_m;
        if (++s_warmup_count >= WARMUP_SAMPLES) {
            s_alt_ref_m    = s_warmup_sum_m / static_cast<float>(WARMUP_SAMPLES);
            s_ref_captured = true;
        }
        return;  // don't report samples until the zero-reference is set
    }

    chMtxLock(&baro_mtx);
    g_baro.pressure_pa   = pressure_pa;
    g_baro.temperature_c = temperature_c;
    g_baro.alt_m         = alt_raw_m - s_alt_ref_m;
    g_baro.has_new       = true;
    g_baro.valid         = true;
    chMtxUnlock(&baro_mtx);
}

void dps310_init(void)
{
    if (!i2c_write_reg(REG_RESET, RESET_CMD)) return;
    chThdSleepMilliseconds(40);  // datasheet: sensor ready within ~40ms of reset

    uint8_t product_id = 0;
    if (!i2c_read_regs(REG_PRODUCT_ID, &product_id, 1)) return;
    if (product_id == 0x00 || product_id == 0xFF) return;  // nothing answered on the bus

    uint8_t coef[18];
    if (!i2c_read_regs(REG_COEF, coef, sizeof(coef))) return;
    decode_coefficients(coef);

    uint8_t coef_srce = 0;
    if (!i2c_read_regs(REG_COEF_SRCE, &coef_srce, 1)) return;
    const uint8_t tmp_ext = coef_srce & 0x80U;  // TMP_CFG bit7 must match COEF_SRCE bit7

    if (!i2c_write_reg(REG_PRS_CFG, CFG_RATE8_OSR1)) return;
    if (!i2c_write_reg(REG_TMP_CFG, static_cast<uint8_t>(CFG_RATE8_OSR1 | tmp_ext))) return;
    if (!i2c_write_reg(REG_MEAS_CFG, MEAS_CTRL_CONT_PT)) return;

    s_ready = true;
    bprl_i2c_register(I2C_ADDR, dps310_poll, nullptr);
}
