#include "src/coms/SPI.hpp"

/*
 * Clock sources (from mcuconf.h / PLL config):
 *   SPI1 = PLL1_Q = 50 MHz   (STM32_SPI123SEL_PLL1_Q_CK, DIVQ=16)
 *   SPI4 = PCLK2  = 100 MHz  (STM32_SPI45SEL_PCLK2,  D2PPRE2=/2)
 *
 * SPIConfig v1 field order: {circular, end_cb, ssport, sspad, cfg1, cfg2,
 *                            dummytx, dummyrx}
 *
 * IMU chip set + CS pins are board-conditional — see SPI.hpp for the full
 * pin map and rationale. Baro is on the same pin/mode for both boards.
 */

#if defined(BPRL_BOARD_ORQA)

// ── IMU1: ICM-42688 — SPI1  CS=PA4  MODE3 (CPOL=1, CPHA=1) ───────────────────
// SPI1 = PLL1_Q = 50 MHz (same as the Cube boards — DIVM change for the 8 MHz
// HSE keeps every PLL output frequency identical, see cfg/mcuconf.h).
static const SPIConfig imu1_init = {
    false, nullptr, GPIOA, 4U,
    SPI_CFG1_MBR_DIV64 | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};
static const SPIConfig imu1_fast = {
    false, nullptr, GPIOA, 4U,
    SPI_CFG1_MBR_DIV8  | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};

// ── IMU2: ICM-42688 — SPI4  CS=PE11  MODE3 ────────────────────────────────────
// SPI4 = PCLK2 = 100 MHz (unchanged from the Cube boards).
static const SPIConfig imu2_init = {
    false, nullptr, GPIOE, 11U,
    SPI_CFG1_MBR_DIV128 | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};
static const SPIConfig imu2_fast = {
    false, nullptr, GPIOE, 11U,
    SPI_CFG1_MBR_DIV16  | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};

ICM42688 imu1;
ICM42688 imu2;

void spi_drv_init(void)
{
    imu1.init(&SPID1, &imu1_init, &imu1_fast);
    imu2.init(&SPID4, &imu2_init, &imu2_fast);
    // No SPI baro on this board — DPS310 is I2C, initialized/polled from
    // I2CThread (see src/coms/Baro/DPS310.hpp).
}

#elif defined(BPRL_BOARD_CUBEBLUE)

// ── IMU1: ICM-20649 — SPI1  CS=PC2  MODE3 (CPOL=1, CPHA=1) ───────────────────
// NOT ICM-20948 — WHOAMI-confirmed as ICM-20649 (0xE1) on real hardware on
// two separate units; see ICM20649.hpp for why. CS pin/bus/mode/divider are
// unaffected by this — only the chip class changed.
static const SPIConfig imu1_init = {
    false, nullptr, GPIOC, 2U,
    SPI_CFG1_MBR_DIV64 | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};
static const SPIConfig imu1_fast = {
    false, nullptr, GPIOC, 2U,
    SPI_CFG1_MBR_DIV8  | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};

// ── IMU2: ICM-20948 — SPI4  CS=PE4  MODE3 ─────────────────────────────────────
static const SPIConfig imu2_init = {
    false, nullptr, GPIOE, 4U,
    SPI_CFG1_MBR_DIV128 | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};
static const SPIConfig imu2_fast = {
    false, nullptr, GPIOE, 4U,
    SPI_CFG1_MBR_DIV16  | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};

// ── IMU3: ICM-20602 — SPI4  CS=PC13  MODE3 ────────────────────────────────────
static const SPIConfig imu3_init = {
    false, nullptr, GPIOC, 13U,
    SPI_CFG1_MBR_DIV128 | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};
static const SPIConfig imu3_fast = {
    false, nullptr, GPIOC, 13U,
    SPI_CFG1_MBR_DIV16  | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};

ICM20649 imu1;
ICM20948 imu2;
ICM20602 imu3;

#else  // BPRL_BOARD_CUBEORANGEPLUS

// ── IMU1: ICM-45686 — SPI1  CS=PG1  MODE0 ────────────────────────────────────
static const SPIConfig imu1_init = {
    false, nullptr, GPIOG, 1U,
    SPI_CFG1_MBR_DIV64 | SPI_CFG1_DSIZE_VALUE(7), 0, // no CPOL/CPHA = MODE0
    nullptr, nullptr
};
static const SPIConfig imu1_fast = {
    false, nullptr, GPIOG, 1U,
    SPI_CFG1_MBR_DIV8  | SPI_CFG1_DSIZE_VALUE(7), 0,
    nullptr, nullptr
};

// ── IMU2: probe as ICM-45686 — SPI4  CS=PC15  MODE0 ──────────────────────────
static const SPIConfig imu2_init = {
    false, nullptr, GPIOC, 15U,
    SPI_CFG1_MBR_DIV128 | SPI_CFG1_DSIZE_VALUE(7), 0, // MODE0
    nullptr, nullptr
};
static const SPIConfig imu2_fast = {
    false, nullptr, GPIOC, 15U,
    SPI_CFG1_MBR_DIV16  | SPI_CFG1_DSIZE_VALUE(7), 0,
    nullptr, nullptr
};

// ── IMU3: probe as ICM-45686 — SPI4  CS=PC13  MODE0 ──────────────────────────
static const SPIConfig imu3_init = {
    false, nullptr, GPIOC, 13U,
    SPI_CFG1_MBR_DIV128 | SPI_CFG1_DSIZE_VALUE(7), 0, // MODE0
    nullptr, nullptr
};
static const SPIConfig imu3_fast = {
    false, nullptr, GPIOC, 13U,
    SPI_CFG1_MBR_DIV16  | SPI_CFG1_DSIZE_VALUE(7), 0,
    nullptr, nullptr
};

ICM45686 imu1;
ICM45686 imu2;
ICM45686 imu3;

#endif

#if !defined(BPRL_BOARD_ORQA)
// ── BARO1: MS5611 — SPI1  CS=PD7  MODE3 (same on both Cube boards) ───────────
// Not compiled for BPRL_BOARD_ORQA — that board's DPS310 is I2C, see above.
static const SPIConfig baro1_init = {
    false, nullptr, GPIOD, 7U,
    SPI_CFG1_MBR_DIV64 | SPI_CFG1_DSIZE_VALUE(7),
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,   // MODE3
    nullptr, nullptr
};
static const SPIConfig baro1_fast = {
    false, nullptr, GPIOD, 7U,
    SPI_CFG1_MBR_DIV4 | SPI_CFG1_DSIZE_VALUE(7),   // 50MHz/4=12.5MHz (<20MHz max)
    SPI_CFG2_CPHA | SPI_CFG2_CPOL,
    nullptr, nullptr
};

MS5611   baro1;

void spi_drv_init(void)
{
    imu1.init(&SPID1, &imu1_init, &imu1_fast);
    imu2.init(&SPID4, &imu2_init, &imu2_fast);
    imu3.init(&SPID4, &imu3_init, &imu3_fast);
    baro1.init(&SPID1, &baro1_init, &baro1_fast);
}

#endif // !BPRL_BOARD_ORQA
