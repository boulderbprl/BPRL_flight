#pragma once
#include "PID.hpp"
#include "Unmixer.hpp"
#include "AttitudeController.hpp"
#include "configs/DroneConfig.hpp"

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
 *
 * delta_torque/accel_cmd (shadow-logging diagnostics, [roll, pitch]) are no
 * longer separate update() out-params — that would break the shared
 * AttitudeController signature every controller in FlightStateMachine's list
 * now uses. update() stores them internally; get_diag() reads them back.
 */
class AttitudeINDI : public AttitudeController {
public:
    explicit AttitudeINDI(const AttitudeIndiGains &g);

    void update(const float euler[3], const float state_full[],
                const float input[], const float current_torque[2],
                const Unmixer &unmixer, float out_cmds[3]) override;
    void reset_all() override;

    // [delta_roll_Nm, delta_pitch_Nm] incremental INDI correction, and
    // [accel_cmd_roll, accel_cmd_pitch] rad/s² rate-PID output fed to the
    // INDI step — both from the most recent update() call, for logging.
    void get_diag(float delta_torque[2], float accel_cmd[2]) const
    {
        delta_torque[0] = _delta_torque[0];
        delta_torque[1] = _delta_torque[1];
        accel_cmd[0]    = _accel_cmd[0];
        accel_cmd[1]    = _accel_cmd[1];
    }

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;
    PID _pitch_rate;
    PID _yaw_rate;
    PID _yaw_hold;   // heading-lock trim: heading error [rad] -> corrective rate [rad/s]

    // G(x)^-1 : N·m per rad/s² — airframe moment of inertia (Ixx, Iyy) * gain (1 for now)
    float _indi_gain_roll;   // from DroneConfig; was INDI_GAIN_ROLL
    float _indi_gain_pitch;  // from DroneConfig; was INDI_GAIN_PITCH
    float _yaw_gain;         // from DroneConfig; was YAW_GAIN

    float _yaw_target;        // held heading target [rad]
    bool  _yaw_target_valid;  // false until first update() captures a target

    float _delta_torque[2] = {};  // see get_diag()
    float _accel_cmd[2]    = {};

    static constexpr float YAW_STICK_DEADBAND   = 0.10f;  // normalised stick [-1,1], matches FlightStateMachine::STICK_DEADBAND
    static constexpr float YAW_HOLD_MAX_RATE    = 0.3f;   // rad/s cap on the heading-hold trim — needs flight tuning
};
