#include "src/coms/SPI.hpp"

/*
 * Clock sources (from mcuconf.h / PLL config):
 *   SPI1 = PLL1_Q = 50 MHz   (STM32_SPI123SEL_PLL1_Q_CK, DIVQ=16)
 *   SPI4 = PCLK2  = 100 MHz  (STM32_SPI45SEL_PCLK2,  D2PPRE2=/2)
 *
 * Init speed ~781 kHz for WHOAMI / register writes.
 * Fast speed 6.25 MHz for burst data reads.
 * Both ICMs: SPI Mode 3 (CPOL=1, CPHA=1), 8-bit frames.
 *
 * SPIConfig v1 field order: {circular, end_cb, ssport, sspad, cfg1, cfg2,
 *                            dummytx, dummyrx}
 */

// ── IMU1: ICM-20948 primary — SPI1 CS=PC2 ───────────────────────────────────
static const SPIConfig imu1_init = {
    false, nullptr, GPIOC, 2U,
    SPI_CFG1_MBR_DIV64  | SPI_CFG1_DSIZE_VALUE(7), SPI_CFG2_CPOL | SPI_CFG2_CPHA,
    nullptr, nullptr
};
static const SPIConfig imu1_fast = {
    false, nullptr, GPIOC, 2U,
    SPI_CFG1_MBR_DIV8   | SPI_CFG1_DSIZE_VALUE(7), SPI_CFG2_CPOL | SPI_CFG2_CPHA,
    nullptr, nullptr
};

// ── IMU2: ICM-20948 ext — SPI4 CS=PE4 ───────────────────────────────────────
static const SPIConfig imu2_init = {
    false, nullptr, GPIOE, 4U,
    SPI_CFG1_MBR_DIV128 | SPI_CFG1_DSIZE_VALUE(7), SPI_CFG2_CPOL | SPI_CFG2_CPHA,
    nullptr, nullptr
};
static const SPIConfig imu2_fast = {
    false, nullptr, GPIOE, 4U,
    SPI_CFG1_MBR_DIV16  | SPI_CFG1_DSIZE_VALUE(7), SPI_CFG2_CPOL | SPI_CFG2_CPHA,
    nullptr, nullptr
};

// ── IMU3: ICM-20602 — SPI4 CS=PC13 ──────────────────────────────────────────
static const SPIConfig imu3_init = {
    false, nullptr, GPIOC, 13U,
    SPI_CFG1_MBR_DIV128 | SPI_CFG1_DSIZE_VALUE(7), SPI_CFG2_CPOL | SPI_CFG2_CPHA,
    nullptr, nullptr
};
static const SPIConfig imu3_fast = {
    false, nullptr, GPIOC, 13U,
    SPI_CFG1_MBR_DIV16  | SPI_CFG1_DSIZE_VALUE(7), SPI_CFG2_CPOL | SPI_CFG2_CPHA,
    nullptr, nullptr
};

ICM20948 imu1;
ICM20948 imu2;
ICM20602 imu3;

void spi_drv_init(void)
{
    imu1.init(&SPID1, &imu1_init, &imu1_fast);
    imu2.init(&SPID4, &imu2_init, &imu2_fast);
    imu3.init(&SPID4, &imu3_init, &imu3_fast);
}
