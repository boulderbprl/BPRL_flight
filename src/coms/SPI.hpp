#pragma once
#include "hal.h"
#include "src/coms/Baro/MS5611.hpp"

#if defined(BPRL_BOARD_CUBEBLUE)
#include "src/coms/IMUs/ICM20948.hpp"
#include "src/coms/IMUs/ICM20602.hpp"
#else
#include "src/coms/IMUs/ICM45686.hpp"
#include "src/coms/IMUs/ICM42688.hpp"
#endif

/*
 * SPI bus driver — FMUv5x on-board IMUs, chip set selected by BOARD at
 * build time (see Makefile: BOARD=CubeBlueH7 defines BPRL_BOARD_CUBEBLUE,
 * BOARD=CubeOrangePlus defines BPRL_BOARD_CUBEORANGEPLUS).
 *
 * ── BPRL_BOARD_CUBEORANGEPLUS (default) ─────────────────────────────────
 * This board's three IMU slots are all populated with ICM-45686 (confirmed —
 * all three probe and initialize as ICM45686, matching their WHOAMI check):
 *   imu1  ICM-45686   SPI1   CS=PG1   (ICM45686_CS)   — instance 2
 *   imu2  ICM-45686   SPI4   CS=PC15  (ACCEL_EXT_CS)  — instance 0
 *   imu3  ICM-45686   SPI4   CS=PC13  (GYRO_EXT_CS)   — instance 1
 *   baro1 MS5611      SPI1   CS=PD7   (BARO_CS)       — primary barometer
 *
 * Other CubeOrangePlus hardware revisions instead populate imu2/imu3 with
 * ICM-42688 (1x ICM-45686 + 2x ICM-42688) rather than 3x ICM-45686 — the
 * ICM42688.hpp/.cpp driver exists to support that variant, not because it's
 * unused/legacy. It isn't instantiated here since this board doesn't have
 * that part on those slots.
 *
 * ── BPRL_BOARD_CUBEBLUE ──────────────────────────────────────────────────
 * Pin map below is from boards/CubeBlueH7/board.h's LINE_IMU*_CS (sourced
 * from ArduPilot's stock CubeOrange hwdef.inc reference schematic), NOT
 * yet WHOAMI-confirmed against a physical CubeBlueH7 unit the way the
 * CubeOrangePlus mapping above was. A wrong CS/chip pairing fails safe —
 * ICM45686/ICM20948/ICM20602 have distinct, non-colliding WHOAMI values, so
 * a mismatched driver just fails init() and that lane's g_imu[i].valid
 * stays false rather than fusing garbage — but confirm on the bench before
 * trusting attitude output for flight (e.g. watch $IMU telemetry for all
 * three lanes going valid=1 at power-on).
 *   imu1  ICM-20948   SPI1   CS=PC2   — primary
 *   imu2  ICM-20948   SPI4   CS=PE4   — external
 *   imu3  ICM-20602   SPI4   CS=PC13  — external
 *   baro1 MS5611      SPI1   CS=PD7  (same pin as CubeOrangePlus)
 * SPI mode: ICM-20948/ICM-20602 use MODE3 (CPOL=1, CPHA=1), matching
 * ArduPilot's hwdef.dat for this chip family (ICM-45686 uses MODE0).
 *
 * spi_drv_init() must be called from inside SPIThread because
 * ICM/MS5611 init sequences use chThdSleepMilliseconds.
 */
#if defined(BPRL_BOARD_CUBEBLUE)
extern ICM20948 imu1;
extern ICM20948 imu2;
extern ICM20602 imu3;
#else
extern ICM45686 imu1;
extern ICM45686 imu2;
extern ICM45686 imu3;
#endif
extern MS5611   baro1;

void spi_drv_init(void);
