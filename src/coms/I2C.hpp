#pragma once
#include "hal.h"

/*
 * I2C bus driver — stub, ready for magnetometer / barometer.
 *
 * Adding devices:
 *   1. Write a poll function: void my_poll(void *ctx)
 *   2. In main(): bprl_i2c_register(MY_ADDR, my_poll, nullptr);
 *   I2CThread calls i2c_poll_all() at 100 Hz.
 *
 * TODO: enable HAL_USE_I2C in halconf.h, configure I2CD1 in mcuconf.h,
 *       call i2cStart(&I2CD1, &i2c_cfg) inside i2c_drv_init().
 */

#define MAX_I2C_DEVICES 8
typedef void (*I2CCallback)(void *ctx);

void bprl_i2c_register(uint8_t addr, I2CCallback poll_fn, void *ctx);
void i2c_poll_all(void);   // called by I2CThread each tick
void i2c_drv_init(void);   // hardware init + device registration
