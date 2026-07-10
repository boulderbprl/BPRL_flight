#pragma once
#include "PID.hpp"
#include "Unmixer.hpp"

/*
 * INDI (Incremental Nonlinear Dynamic Inversion) attitude controller.
 *
 * Roll and pitch use INDI; yaw falls back to standard rate PID.
 *
 * Control loop per axis (roll shown, pitch symmetric):
 *
 *   Outer loop:  angle_error [rad] → rate_tgt [rad/s]  (P only)
 *   Inner loop:  rate_error [rad/s] → accel_cmd [rad/s²]  (PID)
 *   INDI step:
 *     delta_torque = (accel_cmd - measured_accel) * INDI_GAIN
 *     total_torque = current_torque_Nm + delta_torque
 *     out_cmds[i]  = unmixer.normalize_torque(total_torque)
 *
 * Conventions:
 *   euler[3]          roll, pitch, yaw (rad)
 *   state_full[19]    full EKF state (StateIdx::P_DOT, Q_DOT used here)
 *   input[]           [thrust, roll_tgt, pitch_tgt, yaw_rate, flight_mode]
 *   current_torque[2] [roll_Nm, pitch_Nm] from Unmixer
 *   out_cmds[3]       normalised torque [roll, pitch, yaw] in [-1, 1]
 *   delta_torque[2]   [delta_roll_Nm, delta_pitch_Nm] incremental INDI correction (for logging)
 *   accel_cmd[2]      [accel_cmd_roll, accel_cmd_pitch] rad/s², rate-PID output fed to the INDI step (for logging)
 */
class AttitudeINDI {
public:
    AttitudeINDI();

    void update(const float euler[3], const float state_full[],
                const float input[], const float current_torque[2],
                const Unmixer &unmixer, float out_cmds[3],
                float delta_torque[2], float accel_cmd[2]);
    void reset_all();

    // G(x)^-1 : N·m per rad/s² — airframe moment of inertia (Ixx, Iyy) * gain (1 for now)
    static constexpr float INDI_GAIN_ROLL = 0.0023f;
    static constexpr float INDI_GAIN_PITCH = 0.0033f;

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;
    PID _pitch_rate;
    PID _yaw_rate;

    static constexpr float YAW_GAIN = 1.5f;
};
