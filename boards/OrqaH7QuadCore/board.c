/*
 * boards/OrqaH7QuadCore/board.c
 * Orqa QuadCore H7 — STM32H743 board-level initialisation
 *
 * Called by ChibiOS halInit() after palInit(), before the scheduler starts.
 * Configures GPIO alternate functions and output levels for all peripherals
 * used by the firmware. Motor timer pins (TIM4/TIM2/TIM5/TIM3) are NOT
 * configured here — DShot.cpp owns those pins directly (same convention as
 * the Cube boards' TIM1/TIM4 motor pins, which DShot.cpp also self-configures).
 */

#include "hal.h"
#include "board.h"

void boardInit(void)
{
    /* ── Status LED (PA8, active LOW per hwdef "LED0 OUTPUT LOW") ───────── */
    palSetPadMode(GPIOA, 8U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
    palSetLine(LINE_LED_ACTIVITY);   /* idle HIGH = LED off (active LOW) */

    /* ── SPI1 — IMU1 bus (ICM-42688, PA4 CS) ─────────────────────────────
     * PA5=SCK, PA6=MISO, PA7=MOSI → AF5                                    */
    palSetPadMode(GPIOA, 5U, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOA, 6U, PAL_MODE_ALTERNATE(5) | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOA, 7U, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    /* CS idle-high before configuring as output */
    palSetPad(GPIOA, 4U);
    palSetPadMode(GPIOA, 4U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

    /* ── SPI4 — IMU2 bus (ICM-42688, PE11 CS) ────────────────────────────
     * PE12=SCK, PE13=MISO, PE14=MOSI → AF5                                 */
    palSetPadMode(GPIOE, 12U, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOE, 13U, PAL_MODE_ALTERNATE(5) | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOE, 14U, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    palSetPad(GPIOE, 11U);
    palSetPadMode(GPIOE, 11U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

    /* ── UART7 — MAVLink/telemetry radio (PE8=TX7, PE7=RX7) → AF7 ────────
     * The TX7/RX7 pads (ArduPilot's hwdef labels this port GPS) — where the
     * mocap telemetry radio is wired on this board; see BPRL_MAVLINK_SD in
     * src/coms/MAVLink.hpp. The T3/R3 pads (USART3, PD8/PD9) are unused.   */
    palSetPadMode(GPIOE, 8U, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOE, 7U, PAL_MODE_ALTERNATE(7) | PAL_STM32_PUPDR_PULLUP);

    /* ── USART6 — RC input, CRSF, full-duplex (PC6=TX6, PC7=RX6) → AF7 ───
     * Standard two-wire UART, no HDSEL needed — unlike the T3 pad this
     * replaced, TX6/RX6 are genuinely separate pins.                      */
    palSetPadMode(GPIOC, 6U, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOC, 7U, PAL_MODE_ALTERNATE(7) | PAL_STM32_PUPDR_PULLUP);

    /* ── USB OTG_FS — micro USB (PA11=DM, PA12=DP) → AF10 ───────────────── */
    palSetPadMode(GPIOA, 11U, PAL_MODE_ALTERNATE(10) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOA, 12U, PAL_MODE_ALTERNATE(10) | PAL_STM32_OSPEED_HIGHEST);

    /* ── FDCAN1 (PB8=RX, PB9=TX) → AF9 ───────────────────────────────────
     * Different pins to the Cube boards (PD0/PD1) — same peripheral/AF.    */
    palSetPadMode(GPIOB, 8U, PAL_MODE_ALTERNATE(9) | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOB, 9U, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);

    /* ── SDMMC1 — microSD card → AF12 ──────────────────────────────────
     * D0=PC8, D1=PC9, D2=PC10, D3=PC11, CK=PC12, CMD=PD2
     * Identical pinout to the Cube boards.                                */
    palSetPadMode(GPIOC,  8U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOC,  9U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOC, 10U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOC, 11U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOC, 12U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOD,  2U, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
}
