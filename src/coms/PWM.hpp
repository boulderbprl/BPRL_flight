#pragma once
#include "hal.h"
#include <cstdint>

/*
 * Motor output. Three protocols, selected at compile time via
 * MOTOR_PROTOCOL — see each one's driver file for the full story:
 *   MOTOR_PROTO_DSHOT (default): DShot600 bidirectional, Cube carrier
 *     boards' AUX2-5 (DShot.hpp) or Orqa's MOT1-4 (same file).
 *   MOTOR_PROTO_PWM: Standard servo PWM via TIM1/TIM4, Cube carrier
 *     boards' AUX2-5 (PWM.cpp).
 *   MOTOR_PROTO_IOMCU: Standard servo PWM via the carrier board's IO
 *     co-processor, MAIN 1-4 (IOMCU.hpp) — for boards where AUX is
 *     unreliable (bench-confirmed on CubeBlueH7, including under stock
 *     ArduPilot firmware, ruling out this project's firmware as the cause).
 *
 * motor_output_write() accepts a 0–1000 normalized throttle per motor:
 *   0        → disarmed (DShot 0 / PWM 1000 µs)
 *   1–1000   → 0.1%–100% throttle
 */
#define MOTOR_PROTO_DSHOT  0
#define MOTOR_PROTO_PWM    1
#define MOTOR_PROTO_IOMCU  2

#ifndef MOTOR_PROTOCOL
#define MOTOR_PROTOCOL MOTOR_PROTO_DSHOT
#endif

void motor_output_init(void);
void motor_output_write(const int32_t val[4]);  // val: 0=disarm, 1–1000=throttle ×0.1%
