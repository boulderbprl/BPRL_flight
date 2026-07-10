#pragma once
#include "PID.hpp"

/*
 * Cascade attitude controller (PID) — outer attitude loop + inner rate loop.
 *
 * Outer loop: attitude error [rad]  → angular rate target [rad/s]
 * Inner loop: rate error [rad/s]    → normalised torque output [-1, 1]
 *
 * Conventions:
 *   state[0..2]  roll, pitch, yaw in radians
 *   state[3..5]  p, q, r body-frame rates in rad/s
 *   input[1..3]  roll_tgt, pitch_tgt, yaw_rate_tgt [-1, 1]
 *   out_cmds[3]  normalised torque [roll, pitch, yaw] in [-1, 1]
 */
class AttitudePID {
public:
    AttitudePID();

    void update(const float state[], const float input[], float out_cmds[3]);
    void reset_all();

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;
    PID _pitch_rate;
    PID _yaw_rate;

    static constexpr float YAW_STICK_GAIN = 2.5f;
};
