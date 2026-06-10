#include "src/coms/I2C.hpp"

/*
 * I2C2 config — 400 kHz Fast Mode.
 * Source clock: PCLK1 = 100 MHz (STM32_I2C123SEL_PCLK1, D2PPRE1=/2 from 200 MHz AHB).
 * PRESC=1 → t_PRESC=20 ns; SCLL=64 → t_LOW=1300 ns; SCLH=56 → t_HIGH=1140 ns → ~410 kHz.
 */
static const I2CConfig i2c_cfg = {
    .timingr = 0x10403840u,
    .cr1     = 0,
    .cr2     = 0,
};

struct I2CDevice {
    uint8_t     addr;
    I2CCallback poll;
    void       *ctx;
};
static I2CDevice i2c_table[MAX_I2C_DEVICES];
static int       num_i2c_devices = 0;

void bprl_i2c_register(uint8_t addr, I2CCallback poll_fn, void *ctx)
{
    if (num_i2c_devices < MAX_I2C_DEVICES) {
        i2c_table[num_i2c_devices++] = {addr, poll_fn, ctx};
    }
}

void i2c_poll_all(void)
{
    for (int i = 0; i < num_i2c_devices; i++) {
        i2c_table[i].poll(i2c_table[i].ctx);
    }
}

void i2c_drv_init(void)
{
    // Configure pins immediately before i2cStart() to avoid the STM32H7 errata
    // where early AF4 configuration can cause a spurious START and set BUSY.
    palSetPadMode(GPIOB, 10U,
                  PAL_MODE_ALTERNATE(4U) | PAL_STM32_OTYPE_OPENDRAIN |
                  PAL_STM32_OSPEED_MID2  | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOB, 11U,
                  PAL_MODE_ALTERNATE(4U) | PAL_STM32_OTYPE_OPENDRAIN |
                  PAL_STM32_OSPEED_MID2  | PAL_STM32_PUPDR_PULLUP);
    i2cStart(&I2CD2, &i2c_cfg);
}

void i2c_drv_reset(void)
{
    i2cStop(&I2CD2);
    i2cStart(&I2CD2, &i2c_cfg);
}
