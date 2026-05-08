#pragma once
#include "hal.h"
#include <cstdint>

/*
 * Motor PWM output — FMU CH1-4 via TIM1.
 *
 * TODO: enable HAL_USE_PWM in halconf.h, configure TIM1 in mcuconf.h,
 *       then fill in motor_output_init() and motor_output_write().
 *
 * To switch to DShot: replace the body of motor_output_write() only.
 * ControlThread and MotorMixer are unaffected.
 *
 * PWM range: 1000 µs (idle / disarmed) … 1950 µs (full throttle).
 */
void motor_output_init(void);
void motor_output_write(const int32_t pwm_us[4]);
