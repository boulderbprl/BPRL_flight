/*
 * boards/CubeOrangePlus/board.c
 * CubeOrange+ — STM32H743ZI board-level initialisation
 *
 * Identical PCB to CubeBlue H7 (FMUv5x); only the MCU variant differs.
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
    palSetPad(GPIOD, 7U);
    palSetPadMode(GPIOD, 7U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

    /* ── USART3 — debug / telemetry (PD8=TX, PD9=RX) → AF7 ────────────── */
    palSetPadMode(GPIOD, 8U, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOD, 9U, PAL_MODE_ALTERNATE(7) | PAL_STM32_PUPDR_PULLUP);

    /* ── FDCAN1 (PH13=TX, PH14=RX) → AF9 ──────────────────────────────── */
    palSetPadMode(GPIOH, 13U, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOH, 14U, PAL_MODE_ALTERNATE(9) | PAL_STM32_PUPDR_PULLUP);
}
