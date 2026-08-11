#pragma once
#include "hal.h"

#if defined(BPRL_BOARD_ORQA)
#include "src/coms/IMUs/ICM42688.hpp"
#else
#include "src/coms/Baro/MS5611.hpp"
#if defined(BPRL_BOARD_CUBEBLUE)
#include "src/coms/IMUs/ICM20948.hpp"
#include "src/coms/IMUs/ICM20602.hpp"
#else
#include "src/coms/IMUs/ICM45686.hpp"
#include "src/coms/IMUs/ICM42688.hpp"
#endif
#endif

/*
 * SPI bus driver — on-board IMUs, chip set selected by DRONE at build time
 * (see Makefile: DRONE=Drone2 selects CubeBlueH7 and defines
 * BPRL_BOARD_CUBEBLUE, DRONE=Drone3 selects OrqaH7QuadCore and defines
 * BPRL_BOARD_ORQA, DRONE=Drone1 selects CubeOrangePlus and defines
 * BPRL_BOARD_CUBEORANGEPLUS — see configs/<DRONE>/config.mk).
 *
 * ── BPRL_BOARD_ORQA ──────────────────────────────────────────────────────
 * Only two IMU slots (not three like the Cube boards) — g_imu[2] is simply
 * never written and stays valid=false; StateManager already treats an
 * invalid lane as absent (see state_estimator/StateManager.cpp). No SPI
 * barometer — this board's DPS310 is I2C-only (src/coms/Baro/DPS310.hpp),
 * polled from I2CThread instead of SPIThread, so there is no baro1 here.
 * NOT bench-verified against a physical unit — confirm WHOAMI on both
 * lanes before trusting attitude output.
 *   imu1  ICM-42688   SPI1   CS=PA4   (GYRO1_CS)
 *   imu2  ICM-42688   SPI4   CS=PE11  (GYRO2_CS)
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
#if defined(BPRL_BOARD_ORQA)
extern ICM42688 imu1;
extern ICM42688 imu2;
#elif defined(BPRL_BOARD_CUBEBLUE)
extern ICM20948 imu1;
extern ICM20948 imu2;
extern ICM20602 imu3;
extern MS5611   baro1;
#else
extern ICM45686 imu1;
extern ICM45686 imu2;
extern ICM45686 imu3;
extern MS5611   baro1;
#endif

void spi_drv_init(void);
