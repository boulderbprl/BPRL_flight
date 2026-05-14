/*
 * boards/CubeBlueH7/board.c
 * CubeBlue H7 — STM32H753ZI board-level initialisation
 *
 * Called by ChibiOS halInit() after palInit(), before the scheduler starts.
 * Configures GPIO alternate functions and output levels for all peripherals
 * used by the firmware.
 */

#include "hal.h"
#include "board.h"

void boardInit(void)
{
    /* ── Sensor 3.3V rail ───────────────────────────────────────────────── */
    palSetPadMode(GPIOE, 3U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetLine(LINE_SENSOR_PWR_EN);

    /* ── Status LED (PB0, active high) ─────────────────────────────────── */
    palSetPadMode(GPIOB, 0U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
    palClearLine(LINE_LED_ACTIVITY);

    /* ── SPI1 — primary IMU bus (ICM-20948, PC2 CS) ─────────────────────
     * PA5=SCK, PA6=MISO, PA7=MOSI → AF5                                   */
    palSetPadMode(GPIOA, 5U, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOA, 6U, PAL_MODE_ALTERNATE(5) | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOA, 7U, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    /* CS idle-high before configuring as output */
    palSetPad(GPIOC, 2U);
    palSetPadMode(GPIOC, 2U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

    /* ── SPI4 — secondary IMU bus (ICM-20948 ext PE4, ICM-20602 PC13) ───
     * PE2=SCK, PE5=MISO, PE6=MOSI → AF5                                   */
    palSetPadMode(GPIOE, 2U, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOE, 5U, PAL_MODE_ALTERNATE(5) | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOE, 6U, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    palSetPad(GPIOE, 4U);
    palSetPadMode(GPIOE, 4U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetPad(GPIOC, 13U);
    palSetPadMode(GPIOC, 13U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    /* Barometer CS idle-high */
    palSetPad(GPIOD, 7U);
    palSetPadMode(GPIOD, 7U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

    /* ── USART3 — TELEM1 (PD8=TX, PD9=RX) → AF7 ───────────────────────── */
    palSetPadMode(GPIOD, 8U, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOD, 9U, PAL_MODE_ALTERNATE(7) | PAL_STM32_PUPDR_PULLUP);

    /* ── USART6 — SBUSo RC input (PC7=RX) → AF8 ────────────────────────── */
    palSetPadMode(GPIOC, 7U, PAL_MODE_ALTERNATE(8) | PAL_STM32_PUPDR_PULLUP);

    /* ── USB OTG_FS — micro USB (PA11=DM, PA12=DP) → AF10 ─────────────── */
    palSetPadMode(GPIOA, 11U, PAL_MODE_ALTERNATE(10) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOA, 12U, PAL_MODE_ALTERNATE(10) | PAL_STM32_OSPEED_HIGHEST);

    /* ── FDCAN1 (PH13=TX, PH14=RX) → AF9 ──────────────────────────────── */
    palSetPadMode(GPIOH, 13U, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOH, 14U, PAL_MODE_ALTERNATE(9) | PAL_STM32_PUPDR_PULLUP);

    /* ── SDMMC1 — microSD card → AF12 ──────────────────────────────────
     * D0=PC8, D1=PC9, D2=PC10, D3=PC11, CK=PC12, CMD=PD2
     * Data lines need pull-ups per SD spec.                             */
    palSetPadMode(GPIOC,  8U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOC,  9U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOC, 10U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOC, 11U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOC, 12U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOD,  2U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
}
