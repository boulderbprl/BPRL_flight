#pragma once
#include "src/controllers/extras/PID.hpp"
#include "src/math/math.hpp"
#include "AttitudeController.hpp"
#include "configs/DroneConfig.hpp"

/*
 * Three-stage cascade attitude controller — outer attitude loop + rate loop
 * + inner PI loop closed on measured angular acceleration. Ported from the
 * BPRL ArduPilot fork (AC_AttitudeControl_Multi::rate_controller_run_dt +
 * ArduCopter/Attitude.cpp, Strain_rate branch).
 *
 * Outer loop:  attitude error [rad]   -> angular rate target [rad/s]       (P)
 * Rate loop:   rate error [rad/s]     -> accel target [normalised cmd]     (PID — ArduPilot's stock rate PID)
 * Inner loop:  accel error [norm cmd] -> normalised torque out [-1, 1]     (PI, kd=0)
 *
 * As in ArduPilot, the rate loop's output is the same normalised command
 * AttitudePID would send the mixer, and is used directly as the inner loop's
 * "acceleration" target. The measured p_dot/q_dot are brought into those
 * units by dividing by accel_norm (ArduPilot hardcodes 80 rad/s² = 1.0).
 *
 * Yaw has no inner accel loop (mirrors the ArduPilot source) — same rate PID +
 * heading-lock trim as AttitudePID/AttitudeINDI;
 *
 * Conventions (own 6-element internal state, built from euler[]/state_full[]
 * inside update() to match the shared AttitudeController interface):
 *   state6[0..2]  roll, pitch, yaw in radians
 *   state6[3..5]  p, q, r body-frame rates in rad/s
 *   input[1..3]   roll_tgt, pitch_tgt, yaw_rate_tgt [-1, 1]
 *   out_cmds[3]   normalised torque [roll, pitch, yaw] in [-1, 1]
 */
class AttitudePIDPI : public AttitudeController {
public:
    explicit AttitudePIDPI(const AttitudePidPiGains &g);

    void update(const float euler[3], const float state_full[], const float input[],
                const float current_torque[2], const Unmixer &unmixer,
                float out_cmds[3]) override;
    void reset_all() override;
    void yaw_frame_reset(float delta_rad) override { _yaw_target = wrap_pi(_yaw_target + delta_rad); }
    void set_mixer_limits(const MixerLimits &limits) override { _mixer_limits = limits; }

    // Last update()'s roll/pitch cascade, for PIDP logging. axis 0 = roll, 1 = pitch.
    float rate_tgt(int axis)   const { return _rate_tgt[axis]; }    // rad/s       angle loop output
    float accel_tgt(int axis)  const { return _accel_tgt[axis]; }   // norm. cmd   rate loop output = inner PI target
    float accel_meas(int axis) const { return _accel_meas[axis]; }  // norm. cmd   p_dot/q_dot / accel_norm
    float accel_i(int axis)    const { return axis == 0 ? _roll_accel.integrator() : _pitch_accel.integrator(); }

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;    // rate error -> accel target (normalised command units)
    PID _pitch_rate;
    PID _roll_accel;   // inner loop: accel error -> normalised torque (PI, kd=0)
    PID _pitch_accel;
    PID _yaw_rate;
    PID _yaw_hold;     // heading-lock trim: heading error [rad] -> corrective rate [rad/s]

    float _yaw_stick_gain;     // from DroneConfig; mirrors AttitudePidGains::yaw_stick_gain
    float _accel_norm;         // rad/s² per unit normalised command, from DroneConfig
    float _yaw_target;        // held heading target [rad]
    bool  _yaw_target_valid;  // false until first update() captures a target

    MixerLimits _mixer_limits;  // rate-loop anti-windup, as ArduPilot passes _motors.limit to its rate PIDs

    float _rate_tgt[2]   = {};
    float _accel_tgt[2]  = {};
    float _accel_meas[2] = {};

    static constexpr float YAW_STICK_DEADBAND = 0.10f;  // normalised stick [-1,1], matches PosControl::STICK_DEADBAND
    static constexpr float YAW_HOLD_MAX_RATE  = 0.3f;   // rad/s cap on the heading-hold trim — needs flight tuning
};
