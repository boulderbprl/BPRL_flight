/*
 * board.h - CubeBlue H7 (STM32H753, hardware-identical to CubeOrange)
 *
 * This file defines the board-level hardware configuration:
 * GPIO pin modes, alternate functions, and board identification.
 *
 * GPIO pin definitions use the PAL (Port Abstraction Layer) macros.
 * For each pin you need: mode, output type, speed, pull, alternate function.
 *
 * Modes:     PAL_MODE_INPUT, PAL_MODE_INPUT_PULLUP, PAL_MODE_INPUT_PULLDOWN,
 *            PAL_MODE_OUTPUT_PUSHPULL, PAL_MODE_OUTPUT_OPENDRAIN,
 *            PAL_MODE_ALTERNATE(n), PAL_MODE_ANALOG, PAL_MODE_UNCONNECTED
 *
 * Reference: CubeOrange schematics (ProfiCNC/Hex)
 *            ArduPilot hwdef/CubeOrange/hwdef.dat
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Board identification                                                       */
/*===========================================================================*/

#define BOARD_NAME              "CubeBlue H7"
#define BOARD_MCU               "STM32H753ZI"

/*===========================================================================*/
/* Board frequencies                                                          */
/* The Cube Orange/Blue uses a 24 MHz external crystal (HSE).                */
/* The Nucleo demo uses 8 MHz - this is a critical difference.               */
/*===========================================================================*/

#define STM32_LSECLK            32768U
#define STM32_LSEDRV            (3U << 3U)
#define STM32_HSECLK            24000000U   /* 24 MHz crystal on Cube */

/*===========================================================================*/
/* Board-specific GPIO definitions                                            */
/*                                                                            */
/* Expand this section as you bring up peripherals.                          */
/* Use the CubeOrange schematic / hwdef.dat as your pin reference.           */
/*                                                                            */
/* Example format:                                                            */
/*   #define LINE_LED1           PAL_LINE(GPIOA, 15U)                        */
/*===========================================================================*/

/* ---- Status/activity LED ------------------------------------------------ */
/* CubeOrange: LED_ACTIVITY = PB0, LED_BUZZER = PA5                          */
/* Verify against your specific board revision schematic.                    */
#define LINE_LED_ACTIVITY       PAL_LINE(GPIOB, 0U)

/* ---- UART1 (Telem1) ------------------------------------------------------ */
/* PA9 = TX, PA10 = RX                                                       */
#define LINE_UART1_TX           PAL_LINE(GPIOA, 9U)
#define LINE_UART1_RX           PAL_LINE(GPIOA, 10U)

/*
 * Add further pin definitions here as you bring up each peripheral.
 * Keep this file as the single source of truth for pin assignments.
 */

/*===========================================================================*/
/* ChibiOS PAL setup                                                         */
/*                                                                            */
/* This macro table is consumed by board.c / palInit().                      */
/* For each GPIO port (A..K on H753) provide a pal_gpio_config_t entry.     */
/* Unconnected/unused pins should be set to PAL_MODE_UNCONNECTED or          */
/* PAL_MODE_INPUT_PULLDOWN to avoid floating inputs.                         */
/*===========================================================================*/

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
