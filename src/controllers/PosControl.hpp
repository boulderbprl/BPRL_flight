#pragma once
#include "PID.hpp"

/*
 * Cascade position controller (PosControl) — position target → lean angles + climb rate.
 *
 * Outer loop:  NE/D position  [m]    → NE/D velocity target [m/s]
 * Inner NE:    NE velocity    [m/s]  → roll/pitch commands  [rad]
 *
 * Conventions:
 *   state[0..2]  N, E, D in inertial frame [m]
 *   state[3..5]  roll, pitch, yaw body-frame [rad]
 *   state[6..8]  vN, vE, vD inertial frame [m/s]
 *   pos_tgt[3]   N, E, D position targets [m]
 *   vel_tgt[3]   N, E, D velocity targets [m/s]  (output of NED_update)
 *   att_cmds[2]  [roll_tgt, pitch_tgt] [rad]      (output of NE_rate_update)
 */
class PosControl {
public:
    PosControl();

    void NED_update(const float state[], const float pos_tgt[3], float vel_tgt[3]);
    void NE_rate_update(const float state[], const float vel_NE_tgt[2], float att_cmds[2]);

    void reset_all();

private:
    void compute_lean_angles(float yaw_rad, float accel_N_tgt, float accel_E_tgt,
                             float &roll_tgt, float &pitch_tgt);

    PID _pos_N;
    PID _pos_E;
    PID _pos_D;
    PID _vel_N;
    PID _vel_E;

    static constexpr float MAX_VEL_NE   = 5.0f;
    static constexpr float MAX_VEL_D    = 3.0f;
    static constexpr float MAX_LEAN_deg = 30.0f;
    static constexpr float GRAVITY_MSS  = 9.80665f;
};
