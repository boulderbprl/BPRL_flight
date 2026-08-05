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

/*
 * Clock SCL up to 9 times as a plain GPIO to unstick any slave that was left
 * holding SDA low mid-byte (e.g. after a reset during a live transaction).
 * Without this the I2C peripheral sees SDA=0 on startup, sets BUSY, and SCL
 * never toggles — exactly the "clock never changes" symptom on a logic analyser.
 */
static void i2c_bus_recover(void)
{
    palSetPadMode(GPIOB, 10U, PAL_MODE_OUTPUT_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOB, 11U, PAL_MODE_INPUT            | PAL_STM32_PUPDR_PULLUP);
    palSetPad(GPIOB, 10U);  // SCL starts high

    for (int i = 0; i < 9; i++) {
        palClearPad(GPIOB, 10U);
        chThdSleepMicroseconds(10);
        palSetPad(GPIOB, 10U);
        chThdSleepMicroseconds(10);
        if (palReadPad(GPIOB, 11U))   // slave released SDA — bus is free
            break;
    }

    // Issue a STOP so any slave in a transaction returns to idle.
    palSetPadMode(GPIOB, 11U, PAL_MODE_OUTPUT_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);
    palClearPad(GPIOB, 10U);   // SCL low
    chThdSleepMicroseconds(10);
    palClearPad(GPIOB, 11U);   // SDA low (while SCL low is safe — not a START)
    chThdSleepMicroseconds(10);
    palSetPad(GPIOB, 10U);     // SCL high
    chThdSleepMicroseconds(10);
    palSetPad(GPIOB, 11U);     // SDA high while SCL high = STOP condition
    chThdSleepMicroseconds(10);
}

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
    i2c_bus_recover();
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
    i2c_bus_recover();
    palSetPadMode(GPIOB, 10U,
                  PAL_MODE_ALTERNATE(4U) | PAL_STM32_OTYPE_OPENDRAIN |
                  PAL_STM32_OSPEED_MID2  | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOB, 11U,
                  PAL_MODE_ALTERNATE(4U) | PAL_STM32_OTYPE_OPENDRAIN |
                  PAL_STM32_OSPEED_MID2  | PAL_STM32_PUPDR_PULLUP);
    i2cStart(&I2CD2, &i2c_cfg);
}
