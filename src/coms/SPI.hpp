#pragma once
#include "hal.h"
#include "src/coms/IMUs/ICM45686.hpp"
#include "src/coms/IMUs/ICM42688.hpp"
#include "src/coms/Baro/MS5611.hpp"

/*
 * SPI bus driver — FMUv5x on-board IMUs (CubeOrangePlus actual chips).
 *   imu1  ICM-45686   SPI1   CS=PG1   (ICM45686_CS)   — instance 2
 *   imu2  ICM-42688   SPI4   CS=PC15  (ACCEL_EXT_CS)  — instance 0
 *   imu3  ICM-42688   SPI4   CS=PC13  (GYRO_EXT_CS)   — instance 1
 *   baro1 MS5611      SPI1   CS=PD7   (BARO_CS)       — primary barometer
 *
 * spi_drv_init() must be called from inside SPIThread because
 * ICM/MS5611 init sequences use chThdSleepMilliseconds.
 */
extern ICM45686 imu1;
extern ICM45686 imu2;
extern ICM45686 imu3;
extern MS5611   baro1;

void spi_drv_init(void);
