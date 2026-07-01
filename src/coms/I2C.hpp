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
 * I2C2 on PB10 (SCL) / PB11 (SDA), AF4, 400 kHz.
 */

#define MAX_I2C_DEVICES 8
typedef void (*I2CCallback)(void *ctx);

void bprl_i2c_register(uint8_t addr, I2CCallback poll_fn, void *ctx);
void i2c_poll_all(void);   // called by I2CThread each tick
void i2c_drv_init(void);   // hardware init + device registration
void i2c_drv_reset(void);  // stop + restart I2CD2 after a timeout-induced LOCKED state
