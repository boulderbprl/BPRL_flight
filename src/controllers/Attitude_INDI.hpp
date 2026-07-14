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
 * Yaw has no angle loop of its own (unlike roll/pitch), so nothing
 * previously corrected a rate estimate that isn't exactly mean-zero
 * (residual gyro-bias error, control asymmetry) — heading could walk
 * indefinitely under a nominally "zero rate" command. _yaw_hold adds a
 * small heading-lock trim on top of the rate PID: while the stick is
 * centered, _yaw_target stays fixed and a P correction pulls current
 * heading back to it; while actively yawing, _yaw_target tracks the
 * current heading so there's no stored error to fight when the stick
 * recentres. Heading comes from euler[2], which is the state estimator's
 * IMX5-fused yaw — accurate enough to close this loop.
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
    static constexpr float INDI_GAIN_ROLL = 0.001f;
    static constexpr float INDI_GAIN_PITCH = 0.0015f;

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;
    PID _pitch_rate;
    PID _yaw_rate;
    PID _yaw_hold;   // heading-lock trim: heading error [rad] -> corrective rate [rad/s]

    float _yaw_target;        // held heading target [rad]
    bool  _yaw_target_valid;  // false until first update() captures a target

    static constexpr float YAW_GAIN            = 1.5f;
    static constexpr float YAW_STICK_DEADBAND   = 0.10f;  // normalised stick [-1,1], matches FlightStateMachine::STICK_DEADBAND
    static constexpr float YAW_HOLD_MAX_RATE    = 0.3f;   // rad/s cap on the heading-hold trim — needs flight tuning
};
