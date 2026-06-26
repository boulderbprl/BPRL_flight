#pragma once
#include "PID.hpp"

/*
 * Cascade position controller (PosControl) — Target pos rate -> desired angles + throttle.
 *
 * Outer loop:  NE/D position  [m]  → NE/D rate (vel) [m/s]
 * Inner loop: NE rate (vel) [m/s]  → NE/ accel [m/s^2] → lean angle compute (roll_cmd, pitch_cmd)[rad]
 * Inner loop: D rate (vel) [m/s]  → throttle [0,1]
 *
 * Conventions:
 *   state[0..2]  N,E,D in inertial frame in m
 *   state[3..5]  roll,pitch,yaw body-frame positions in rad
  *   state[6..8]  vN,vE,vD in inertial frame in m/s
 *   vel_target[0..2]    N,E targets in m
 *   vel_D_target   vel_D_tgt in m
 *   att_cmd[0..2]  targets [roll, pitch]in rad
 *   thr_cmd throttle in [0, 1]
 */
class PosControl {
public:
    PosControl();

    void  NED_update(const float state[], const float pos_tgt[3], float vel_cmds[3]);
    void  NE_rate_update(const float state[], const float vel_tgt[2], float att_cmds[2]);
    void  D_rate_update(const float state[], const float vel_D_tgt, const float thr_cmd);
    void  compute_lean_angles(const float yaw_rad, const float accel_N_tgt, 
                    const float accel_E_tgt, float roll_tgt, const float pitch_tgt);
    

    float compute_throttle(const float state[], const float input[]) const;
    
    void  reset_all();

private:
    PID _pos_N;
    PID _pos_E;
    PID _pos_D;
    PID _vel_N;
    PID _vel_E;
    PID _vel_D;

   
    static constexpr float THR_MID  = 0.4f;

    static constexpr float MAX_VEL_NE  = 5.0f;
    static constexpr float MAX_VEL_D  = 3.0f;
    static constexpr float MAX_LEAN_deg = 30.0f;

    static constexpr float GRAVITY_MSS = 9.80665f;


};
