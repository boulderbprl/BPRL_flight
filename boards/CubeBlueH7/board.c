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
    /*
     * ── AUX output power enable (MUST come first) ─────────────────────
     *
     * PA8 = nVDD_5V_PERIPH_EN (active LOW).
     *   Powers the level-shifter ICs on AUX 1-6 output path.
     *   PCB pull-up → reset default HIGH = rail OFF = no signal to motors.
     *
     * PB4 = PWM_VOLT_SEL (HIGH = 3.3 V mode).
     *   Releases the level-shifter output-enable.
     *
     * ArduPilot hwdef/CubeOrange/hwdef.inc:
     *   PA8 nVDD_5V_PERIPH_EN OUTPUT LOW
     *   PB4 PWM_VOLT_SEL      OUTPUT HIGH
     */
    palSetPadMode(GPIOA, 8U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
    palClearLine(LINE_PERIPH_5V_EN);   /* LOW = 5 V peripheral rail ON */

    palSetPadMode(GPIOB, 4U, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
    palSetLine(LINE_PWM_VOLT_SEL);     /* HIGH = 3.3 V PWM output level */

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

    /* ── USART2 — TELEM1 (PD5=TX, PD6=RX) → AF7  [CRSF radio] ─────────── */
    palSetPadMode(GPIOD, 5U, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOD, 6U, PAL_MODE_ALTERNATE(7) | PAL_STM32_PUPDR_PULLUP);

    /* ── USART3 — TELEM2 (PD8=TX, PD9=RX) → AF7  [future sensor] ───────── */
    palSetPadMode(GPIOD, 8U, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOD, 9U, PAL_MODE_ALTERNATE(7) | PAL_STM32_PUPDR_PULLUP);

    /* PC7 (SBUSo/USART6) intentionally left unconfigured — SBUS disabled.
     * RC input is CRSF on TELEM1 (USART2, PD5/PD6). */

    /* ── USB OTG_FS — micro USB (PA11=DM, PA12=DP) → AF10 ─────────────── */
    palSetPadMode(GPIOA, 11U, PAL_MODE_ALTERNATE(10) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOA, 12U, PAL_MODE_ALTERNATE(10) | PAL_STM32_OSPEED_HIGHEST);

    /* ── FDCAN1 (PD1=TX, PD0=RX) → AF9 ────────────────────────────────── */
    palSetPadMode(GPIOD, 1U, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOD, 0U, PAL_MODE_ALTERNATE(9) | PAL_STM32_PUPDR_PULLUP);

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
