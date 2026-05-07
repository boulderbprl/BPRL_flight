#pragma once
#include "PID.hpp"

/*
 * Cascade attitude controller - ported from FreeRTOS/Tiva controllers.c
 *
 * Structure (same as ArduPilot's AC_AttitudeControl cascade pattern):
 *   Outer loop: attitude error [rad]  → angular rate target [rad/s]
 *   Inner loop: rate error [rad/s]    → normalised torque output [-1, 1]
 *
 * Input/output conventions:
 *   state[0..2]: roll, pitch, yaw in radians
 *   state[3..5]: p, q, r body-frame angular rates in rad/s
 *   input[1..3]: roll_tgt, pitch_tgt, yaw_rate_tgt normalised [-1, 1]
 *   input[0]:    thrust normalised [0, 1]
 *   out_cmds[3]: normalised torque [roll, pitch, yaw] in [-1, 1]
 *
 * The [-1, 1] normalisation matches the original FreeRTOS code.
 * MotorMixer scales these to PWM microseconds.
 */
class AttitudeController {
public:
    AttitudeController();

    // Run one full cascade update.
    void update(const float state[], const float input[], float out_cmds[3]);

    // Throttle with exponential curve + angle-of-attack boost.
    float compute_throttle(const float state[], const float input[]) const;

    void reset_all();

private:
    PID _roll_att;    // outer loop
    PID _pitch_att;
    PID _roll_rate;   // inner loop
    PID _pitch_rate;
    PID _yaw_rate;

    static constexpr float YAW_GAIN = 1.5f;
    static constexpr float THR_MID  = 0.45f;
};
