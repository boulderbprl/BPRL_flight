/*
 * boards/CubeBlueH7/board.h
 * CubeBlue H7 — STM32H753ZI, FMUv5x hardware
 *
 * Pin assignments are identical to CubeOrange+ (same PCB design).
 * Only the MCU variant differs: H753 vs H743.
 *
 * Reference: ProfiCNC/Hex CubeOrange schematics + ArduPilot hwdef/CubeOrange/hwdef.inc
 */

#ifndef BOARD_H
#define BOARD_H

/* ── Board identity ────────────────────────────────────────────────────────── */
#define BOARD_NAME              "CubeBlue H7"
#define BOARD_MCU               "STM32H753ZI"

/* ── External crystal ──────────────────────────────────────────────────────── */
#define STM32_LSECLK            32768U
#define STM32_LSEDRV            (3U << 3U)
#define STM32_HSECLK            24000000U    /* 24 MHz on all Cube boards */

/* ── Status LEDs ───────────────────────────────────────────────────────────── */
#define LINE_LED_ACTIVITY       PAL_LINE(GPIOB, 0U)    /* Amber activity LED */

/* ── Debug / telemetry UART (USART3) ───────────────────────────────────────── */
/* Connected to the Telem1 / USB-serial converter port */
#define LINE_UART3_TX           PAL_LINE(GPIOD, 8U)
#define LINE_UART3_RX           PAL_LINE(GPIOD, 9U)

/* ── Sensor power enable ────────────────────────────────────────────────────── */
/* Hold HIGH to power the 3.3V sensor rail (IMUs, baro) */
#define LINE_SENSOR_PWR_EN      PAL_LINE(GPIOE, 3U)

/* ── SPI1 — primary IMU bus (ICM-20948) ────────────────────────────────────── */
#define LINE_SPI1_SCK           PAL_LINE(GPIOA, 5U)
#define LINE_SPI1_MISO          PAL_LINE(GPIOA, 6U)
#define LINE_SPI1_MOSI          PAL_LINE(GPIOA, 7U)
#define LINE_IMU1_CS            PAL_LINE(GPIOC, 2U)    /* ICM-20948 primary */

/* ── SPI4 — secondary IMU / ext sensor bus ─────────────────────────────────── */
#define LINE_SPI4_SCK           PAL_LINE(GPIOE, 2U)
#define LINE_SPI4_MISO          PAL_LINE(GPIOE, 5U)
#define LINE_SPI4_MOSI          PAL_LINE(GPIOE, 6U)
#define LINE_IMU2_CS            PAL_LINE(GPIOE, 4U)    /* ICM-20948 ext */
#define LINE_IMU3_CS            PAL_LINE(GPIOC, 13U)   /* ICM-20602 ext */
#define LINE_BARO_CS            PAL_LINE(GPIOD, 7U)    /* MS5611 primary (SPI1) */
#define LINE_BARO_EXT_CS        PAL_LINE(GPIOC, 14U)   /* MS5611 ext (SPI4) */

/* ── IMU data-ready interrupts ─────────────────────────────────────────────── */
#define LINE_IMU1_DRDY          PAL_LINE(GPIOD, 15U)
#define LINE_IMU2_DRDY          PAL_LINE(GPIOD, 10U)

/* ── FDCAN1 ─────────────────────────────────────────────────────────────────── */
#define LINE_CAN1_TX            PAL_LINE(GPIOH, 13U)
#define LINE_CAN1_RX            PAL_LINE(GPIOH, 14U)

/* ── FDCAN2 ─────────────────────────────────────────────────────────────────── */
#define LINE_CAN2_TX            PAL_LINE(GPIOB, 13U)
#define LINE_CAN2_RX            PAL_LINE(GPIOB, 12U)

/* ── SDMMC1 (SD card) ───────────────────────────────────────────────────────── */
#define LINE_SDMMC1_D0          PAL_LINE(GPIOC, 8U)
#define LINE_SDMMC1_D1          PAL_LINE(GPIOC, 9U)
#define LINE_SDMMC1_D2          PAL_LINE(GPIOC, 10U)
#define LINE_SDMMC1_D3          PAL_LINE(GPIOC, 11U)
#define LINE_SDMMC1_CK          PAL_LINE(GPIOC, 12U)
#define LINE_SDMMC1_CMD         PAL_LINE(GPIOD, 2U)

/* ── I2C buses ──────────────────────────────────────────────────────────────── */
#define LINE_I2C1_SCL           PAL_LINE(GPIOB, 8U)
#define LINE_I2C1_SDA           PAL_LINE(GPIOB, 9U)
#define LINE_I2C2_SCL           PAL_LINE(GPIOB, 10U)
#define LINE_I2C2_SDA           PAL_LINE(GPIOB, 11U)

/* ── PWM / servo output (TIMx) ──────────────────────────────────────────────── */
/* FMU outputs: TIM1_CH1..4 on PE9, PE11, PE13, PE14 */
#define LINE_FMU_CH1            PAL_LINE(GPIOE, 9U)
#define LINE_FMU_CH2            PAL_LINE(GPIOE, 11U)
#define LINE_FMU_CH3            PAL_LINE(GPIOE, 13U)
#define LINE_FMU_CH4            PAL_LINE(GPIOE, 14U)

/* ── RC input (USART6 for SBUS / TIM8 for PPM) ──────────────────────────────── */
/* SBUS (inverted UART): USART6 RX on PC7 */
#define LINE_RC_INPUT           PAL_LINE(GPIOC, 7U)

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
