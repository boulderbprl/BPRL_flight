#pragma once
#include "hal.h"

/*
 * RC radio input — currently a stub.
 *
 * PWM path (future):
 *   Enable HAL_USE_ICU in halconf.h, configure TIM8 for capture on
 *   LINE_RC_INPUT, then fill in radio_input_update() to read ICU widths.
 *
 * SBUS path (future):
 *   Configure USART6 (inverted UART, 100000 baud, 8E2), set
 *   radio_input_update() to decode the 25-byte SBUS frame.
 *   Change RadioThread period to TIME_MS2I(14) in main.cpp.
 */
void  radio_input_init(void);
void  radio_input_update(void);

float radio_thr(void);    // throttle  [0, 1]
float radio_roll(void);   // roll      [-1, 1]
float radio_pitch(void);  // pitch     [-1, 1]
float radio_yaw(void);    // yaw rate  [-1, 1]
bool  radio_armed(void);
