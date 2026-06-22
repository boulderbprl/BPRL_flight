#pragma once
#include "PID.hpp"
#include "Unmixer.hpp"

/*
 * INDI (Incremental Nonlinear Dynamic Inversion) attitude controller.
 *
 * Roll and pitch use INDI; yaw and thrust fall back to standard rate PID
 * (same as SLCPID).
 *
 * Control loop per axis (roll shown, pitch symmetric):
 *
 *   Outer loop:  angle_error [rad] → rate_tgt [rad/s]  (P only)
 *   Inner loop:  rate_error [rad/s] → accel_cmd [rad/s²]  (PID)
 *   INDI step:
 *     delta_accel  = accel_cmd - state[P_DOT]          (rad/s²)
 *     delta_torque = delta_accel * INDI_GAIN            (N·m)
 *     total_torque = current_torque_Nm + delta_torque   (N·m)
 *     out_cmds[0]  = unmixer.normalize_torque(total_torque)  ([-1,1])
 *
 * Conventions:
 *   euler[3]         roll, pitch, yaw (rad)
 *   state_full[19]   full EKF state (StateIdx::P_DOT=16, Q_DOT=17 used here)
 *   input[]          [thrust, roll_tgt, pitch_tgt, yaw_rate, flight_mode]
 *   current_torque[2] [roll_Nm, pitch_Nm] from Unmixer
 *   out_cmds[3]      normalised torque [roll, pitch, yaw] in [-1, 1]
 */
class INDI {
public:
    INDI();

    void  update(const float euler[3], const float state_full[],
                 const float input[], const float current_torque[2],
                 const Unmixer &unmixer, float out_cmds[3]);
    float compute_throttle(const float euler[3], const float input[]) const;
    void  reset_all();

    // N·m per rad/s² — physical meaning is the airframe moment of inertia (Ixx, Iyy).
    // Update once airframe inertia is characterised.
    static constexpr float INDI_GAIN = 0.01f;

private:
    PID _roll_att;    // angle → rate  (P only, same Kp as SLCPID)
    PID _pitch_att;
    PID _roll_rate;   // rate → accel_cmd (rad/s²)
    PID _pitch_rate;
    PID _yaw_rate;    // rate → torque_norm (same as SLCPID)

    static constexpr float YAW_GAIN = 1.5f;
    static constexpr float THR_MID  = 0.4f;
};
