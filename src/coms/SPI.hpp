#pragma once
#include "hal.h"
#include "src/coms/IMUs/ICM20948.hpp"
#include "src/coms/IMUs/ICM20602.hpp"

/*
 * SPI bus driver — FMUv5x on-board IMUs.
 *   imu1  ICM-20948 primary  SPI1  CS=PC2
 *   imu2  ICM-20948 ext      SPI4  CS=PE4
 *   imu3  ICM-20602          SPI4  CS=PC13
 *
 * spi_drv_init() must be called from inside SPIThread because
 * ICM init sequences use chThdSleepMilliseconds.
 */
extern ICM20948 imu1;
extern ICM20948 imu2;
extern ICM20602 imu3;

void spi_drv_init(void);
