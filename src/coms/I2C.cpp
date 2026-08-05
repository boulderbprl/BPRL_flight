#include "src/coms/I2C.hpp"
#include "src/usb_serial.hpp"
#include "chprintf.h"

/* DIAGNOSTIC: fine-grained checkpoints to find exactly where i2c_drv_init()
 * blocks — main.cpp's stage_print() before/after the whole call wasn't
 * granular enough. Remove once resolved. */
static void i2c_stage(const char *label)
{
    chprintf((BaseSequentialStream *)&SDU1, "BOOT,I2CSTAGE,%s\r\n", label);
    chThdSleepMilliseconds(20);
}

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
    i2c_stage("recover_enter");
    palSetPadMode(GPIOB, 10U, PAL_MODE_OUTPUT_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOB, 11U, PAL_MODE_INPUT            | PAL_STM32_PUPDR_PULLUP);
    palSetPad(GPIOB, 10U);  // SCL starts high
    i2c_stage("recover_padmode_done");

    for (int i = 0; i < 9; i++) {
        palClearPad(GPIOB, 10U);
        chThdSleepMicroseconds(10);
        palSetPad(GPIOB, 10U);
        chThdSleepMicroseconds(10);
        if (palReadPad(GPIOB, 11U))   // slave released SDA — bus is free
            break;
    }
    i2c_stage("recover_clockout_done");

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
    i2c_stage("recover_stop_done");
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
    /* DIAGNOSTIC: bisecting a hang inside i2cStart() — skipping the manual
     * bit-banged bus-recovery step to see if it's what leaves I2C2 in a bad
     * state. Restore once resolved. */
    // i2c_bus_recover();
    i2c_stage("bus_recover_SKIPPED");
    palSetPadMode(GPIOB, 10U,
                  PAL_MODE_ALTERNATE(4U) | PAL_STM32_OTYPE_OPENDRAIN |
                  PAL_STM32_OSPEED_MID2  | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOB, 11U,
                  PAL_MODE_ALTERNATE(4U) | PAL_STM32_OTYPE_OPENDRAIN |
                  PAL_STM32_OSPEED_MID2  | PAL_STM32_PUPDR_PULLUP);
    i2c_stage("alt_padmode_done");
    i2cStart(&I2CD2, &i2c_cfg);
    i2c_stage("i2cStart_returned");
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
