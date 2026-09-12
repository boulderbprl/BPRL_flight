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

/*
 * __early_init() is a weak hook the reset handler calls (crt0_v7m.S) before
 * .data/.bss init and before main() — ChibiOS's own STM32H7 reference board
 * files (and ArduPilot's) override it to call stm32_clock_init() here.
 * This project's board.c files never did: the default empty stub in
 * third_party/ChibiOS/.../crt1.c was used instead, so stm32_clock_init()
 * was never called at all, on any board, ever — the PLL/VOS/prescaler tree
 * in cfg/mcuconf.h was pure dead code (confirmed: no reference to it
 * anywhere in the actual link; USE_LINK_GC drops it). The MCU has been
 * running this whole time on whatever the POR reset default clock leaves
 * it at (HSI, no PLL, ~64 MHz), not the 400 MHz configured in mcuconf.h.
 *
 * CAUTION: fixing this changes every real-time constant in the firmware —
 * control loop rate, filter time constants, DShot bit timing, tuned PID
 * gains — on whichever board it's applied to. Do NOT port this to
 * CubeOrangePlus/OrqaH7QuadCore without deliberate, bench-tested
 * re-verification; it is currently applied to CubeBlueH7 only, which has
 * no working flight tuning to disturb.
 */
void __early_init(void)
{
    stm32_clock_init();

    /* Ensure ITCM and DTCM are enabled — the PX4/ArduPilot ChibiOS bootloader
     * on this board (confirmed: tools/flash_upload.py speaks its protocol,
     * "BL rev=5") can leave them disabled before jumping to the app.
     * ArduPilot's own board.c does exactly this, with this exact comment,
     * for exactly this reason (hwdef/common/board.c __early_init). Our
     * linker script (STM32H743xI.ld) places 128 KB of RAM directly in DTCM
     * at 0x20000000 ("ram5"), so if DTCM is left disabled/indeterminate by
     * the bootloader, that's the RAM this firmware's stack, .data and .bss
     * actually live in — plausibly the root cause of every "works
     * sometimes, not others, even with byte-identical firmware" symptom
     * seen bringing this board up. Never done anywhere in this codebase
     * before this. */
    SCB->ITCMCR |= 1U;   /* ITCM enable */
    SCB->DTCMCR |= 1U;   /* DTCM enable */
}

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

    /* ── SPI1 — primary IMU bus (ICM-20649, PC2 CS) ─────────────────────
     * PA5=SCK, PA6=MISO, PA7=MOSI → AF5
     * WHOAMI-confirmed as ICM-20649, not ICM-20948 as originally assumed
     * (see src/coms/IMUs/ICM20649.hpp).                                   */
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
    /* PA9 = VBUS sense, tied internally to the OTG_FS peripheral's hardware
     * VBUS comparator regardless of GPIO function — ChibiOS's OTG driver
     * never touches GCCFG's VBUS-sensing bits itself, so it depends on this
     * pin being wired up for the comparator to read a real level. Left
     * unconfigured (floating, reset default) it can read a bogus/unstable
     * VBUS state: enough for enumeration to sometimes limp through on a
     * lucky transient, not enough to sustain real bulk data transfer.
     * ArduPilot's hwdef.inc for this exact board: "PA9 VBUS INPUT OPENDRAIN". */
    palSetPadMode(GPIOA, 9U, PAL_MODE_INPUT);

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
