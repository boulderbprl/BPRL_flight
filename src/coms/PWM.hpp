#pragma once
#include "hal.h"
#include <cstdint>

/*
 * Motor output — FMU CH1-4.
 *
 * motor_output_write() accepts a 0–1000 normalized throttle per motor:
 *   0        → disarmed (DShot 0 / PWM 1000 µs)
 *   1–1000   → 0.1%–100% throttle
 *
 * Protocol is selected at compile time via MOTOR_PROTOCOL:
 *   MOTOR_PROTO_DSHOT (default): DShot600 bidirectional
 *   MOTOR_PROTO_PWM:             Standard servo PWM via TIM1
 */
#define MOTOR_PROTO_DSHOT  0
#define MOTOR_PROTO_PWM    1

#ifndef MOTOR_PROTOCOL
#define MOTOR_PROTOCOL MOTOR_PROTO_DSHOT
#endif

void motor_output_init(void);
void motor_output_write(const int32_t val[4]);  // val: 0=disarm, 1–1000=throttle ×0.1%
