/*
 * boards/OrqaH7QuadCore/board.h
 * Orqa FC 3030 / QuadCore H7 — STM32H743xx, betaflight-style FPV racing hardware
 *
 * Pin assignments transcribed from ArduPilot's OrqaH7QuadCore hwdef.dat
 * (libraries/AP_HAL_ChibiOS/hwdef/OrqaH7QuadCore/hwdef.dat, upstream
 * ArduPilot/ardupilot master — not present in this repo's vendored
 * ardupilot/ clone, fetched from GitHub directly). NOT bench-verified
 * against a physical unit yet — confirm CS/WHOAMI pairing and motor
 * spin direction before flight.
 *
 * Unlike the Cube boards (FMUv5x carrier), this board has no AUX 5V
 * peripheral rail / PWM_VOLT_SEL — motor/servo pads are driven directly.
 * No built-in compass. Barometer is I2C (DPS310), not SPI.
 */

#ifndef BOARD_H
#define BOARD_H

/* ── Board identity ────────────────────────────────────────────────────────── */
#define BOARD_NAME              "Orqa QuadCore H7"
#define BOARD_MCU               "STM32H743"

/* ── External crystal ──────────────────────────────────────────────────────── */
/* 8 MHz, not 24 MHz like the Cube boards — cfg/mcuconf.h's PLL DIVM values are
 * conditionalized on BPRL_BOARD_ORQA to keep the same 8 MHz PLL reference
 * (HSE/DIVM) and therefore the same VCO/output frequencies. This define is
 * informational only; cfg/mcuconf.h's own STM32_HSECLK is what the PLL math
 * actually uses (see cfg/mcuconf.h's BPRL_BOARD_ORQA branch). */
#define STM32_LSECLK             0U           /* no 32kHz crystal fitted */
#define STM32_LSEDRV             (3U << 3U)
#define STM32_HSECLK             8000000U

/* ── Status LED ─────────────────────────────────────────────────────────────── */
/* hwdef: "PA8 LED0 OUTPUT LOW GPIO(90)" — active LOW, only LED0 wired up here
 * (board also has LED1/PA10 and LED2/PD11; not used by this firmware yet). */
#define LINE_LED_ACTIVITY       PAL_LINE(GPIOA, 8U)

/* ── SPI1 — IMU1 bus (ICM-42688) ───────────────────────────────────────────── */
#define LINE_SPI1_SCK           PAL_LINE(GPIOA, 5U)
#define LINE_SPI1_MISO          PAL_LINE(GPIOA, 6U)
#define LINE_SPI1_MOSI          PAL_LINE(GPIOA, 7U)
#define LINE_IMU1_CS            PAL_LINE(GPIOA, 4U)    /* GYRO1_CS */

/* ── SPI4 — IMU2 bus (ICM-42688) ───────────────────────────────────────────── */
#define LINE_SPI4_SCK           PAL_LINE(GPIOE, 12U)
#define LINE_SPI4_MISO          PAL_LINE(GPIOE, 13U)
#define LINE_SPI4_MOSI          PAL_LINE(GPIOE, 14U)
#define LINE_IMU2_CS            PAL_LINE(GPIOE, 11U)   /* GYRO2_CS */

/* No SPI baro on this board — DPS310 is I2C2 (see I2C buses below). */

/* ── FDCAN1 ─────────────────────────────────────────────────────────────────── */
/* Different pins to the Cube boards (PD0/PD1) — same FDCAN1 peripheral. */
#define LINE_CAN1_RX            PAL_LINE(GPIOB, 8U)
#define LINE_CAN1_TX            PAL_LINE(GPIOB, 9U)

/* ── SDMMC1 (SD card) ───────────────────────────────────────────────────────── */
/* Identical pinout to the Cube boards — same STM32H7 SDMMC1 AF12 mapping. */
#define LINE_SDMMC1_D0          PAL_LINE(GPIOC, 8U)
#define LINE_SDMMC1_D1          PAL_LINE(GPIOC, 9U)
#define LINE_SDMMC1_D2          PAL_LINE(GPIOC, 10U)
#define LINE_SDMMC1_D3          PAL_LINE(GPIOC, 11U)
#define LINE_SDMMC1_CK          PAL_LINE(GPIOC, 12U)
#define LINE_SDMMC1_CMD         PAL_LINE(GPIOD, 2U)

/* ── I2C buses ──────────────────────────────────────────────────────────────── */
/* I2C2 pins happen to be identical to the Cube boards (PB10/PB11) — src/coms/I2C.cpp's
 * hardcoded PB10/PB11 setup is reused unmodified for the DPS310 barometer. */
#define LINE_I2C1_SCL           PAL_LINE(GPIOB, 6U)
#define LINE_I2C1_SDA           PAL_LINE(GPIOB, 7U)
#define LINE_I2C2_SCL           PAL_LINE(GPIOB, 10U)
#define LINE_I2C2_SDA           PAL_LINE(GPIOB, 11U)

/* ── Motor outputs (BIDIR-capable DShot pins, one per timer) ─────────────────
 * hwdef exposes 8 motor pads across TIM4/TIM2/TIM5/TIM3; only the 4 marked
 * BIDIR in hwdef.dat are used (X-quad, 4 motors) — one channel per timer,
 * unlike the Cube boards' TIM1 (3 channels sharing one burst) + TIM4 (1
 * channel) split. See src/coms/DShot.cpp's BPRL_BOARD_ORQA branch. */
#define LINE_MOTOR0              PAL_LINE(GPIOD, 12U)   /* TIM4_CH1 */
#define LINE_MOTOR1              PAL_LINE(GPIOA, 1U)    /* TIM2_CH2 */
#define LINE_MOTOR2              PAL_LINE(GPIOA, 2U)    /* TIM5_CH3 */
#define LINE_MOTOR3              PAL_LINE(GPIOB, 1U)    /* TIM3_CH4 */

/* ── RC input — CRSF, full-duplex on USART6 (TX6/RX6 pads) ──────────────────
 * Not the hwdef's stock RC pad (that's half-duplex USART3/T3, default
 * protocol GHST) — moved to TX6/RX6 instead: easier to wire, standard
 * two-pin UART (no HDSEL), and leaves USART3 free for MAVLink/telemetry
 * (src/coms/MAVLink.cpp is unconditionally SD3 on every board — sharing it
 * with RC input on this board would have silently collided the two). */
#define LINE_RC_INPUT_TX          PAL_LINE(GPIOC, 6U)    /* USART6_TX */
#define LINE_RC_INPUT_RX          PAL_LINE(GPIOC, 7U)    /* USART6_RX */

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
