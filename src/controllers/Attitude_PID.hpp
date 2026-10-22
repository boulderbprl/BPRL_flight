#pragma once
#include "PID.hpp"
#include "src/math/math.hpp"
#include "AttitudeController.hpp"
#include "configs/DroneConfig.hpp"
#include <cstring>

/*
 * Cascade attitude controller (PID) — outer attitude loop + inner rate loop.
 * Implements AttitudeController — see that header for the shared update()
 * signature and how FlightStateMachine's controller list drives it.
 *
 * Outer loop: attitude error [rad]  → angular rate target [rad/s]
 * Inner loop: rate error [rad/s]    → normalised torque output [-1, 1]
 *
 * Yaw has no angle loop of its own (unlike roll/pitch), so nothing
 * previously corrected a rate estimate that isn't exactly mean-zero —
 * heading could walk indefinitely under a nominally "zero rate" command.
 * _yaw_hold adds a small heading-lock trim on top of the rate PID: while
 * the stick is centered, _yaw_target stays fixed and a P correction pulls
 * current heading (state[2]) back to it; while actively yawing, _yaw_target
 * tracks the current heading so there's no stored error to fight when the
 * stick recentres. Mirrors the same trim in AttitudeINDI.
 *
 * Conventions (own 6-element internal state, built from euler[]/state_full[]
 * inside update() to match the shared AttitudeController interface):
 *   state6[0..2]  roll, pitch, yaw in radians
 *   state6[3..5]  p, q, r body-frame rates in rad/s
 *   input[1..2]   roll_tgt, pitch_tgt lean angles [rad] (FlightStateMachine
 *                 scales the stick by stick_max_angle_deg)
 *   input[3]      yaw stick [-1, 1]
 *   out_cmds[3]   normalised torque [roll, pitch, yaw] in [-1, 1]
 *
 * current_torque/unmixer (from the shared interface) are unused here — this
 * controller doesn't need RPM-derived torque feedback, unlike AttitudeINDI.
 */
class AttitudePID : public AttitudeController {
public:
    explicit AttitudePID(const AttitudePidGains &g);

    void update(const float euler[3], const float state_full[], const float input[],
                const float current_torque[2], const Unmixer &unmixer,
                float out_cmds[3]) override;
    void reset_all() override;
    void yaw_frame_reset(float delta_rad) override { _yaw_target = wrap_pi(_yaw_target + delta_rad); }
    void set_mixer_limits(const MixerLimits &limits) override { _mixer_limits = limits; }

    // CTUN log, from the last update(): diag[10] = [roll_tgt, roll_err,
    // roll_rate_tgt, roll_rate_err, pitch_tgt, pitch_err, pitch_rate_tgt,
    // pitch_rate_err, roll_rate_i, pitch_rate_i]. Angles rad, rates rad/s,
    // errors target - measured; *_rate_tgt is the angle loop's output before
    // the rate PID's own target filter; *_rate_i is the rate PID integrator
    // (normalised output units).
    void get_ctun_diag(float diag[10]) const { memcpy(diag, _ctun_diag, sizeof(_ctun_diag)); }

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;
    PID _pitch_rate;
    PID _yaw_rate;
    PID _yaw_hold;   // heading-lock trim: heading error [rad] -> corrective rate [rad/s]

    float _yaw_stick_gain;     // from DroneConfig; was YAW_STICK_GAIN
    float _yaw_target;        // held heading target [rad]
    bool  _yaw_target_valid;  // false until first update() captures a target

    // Rate-integrator anti-windup: an axis counts as saturated if the mixer
    // couldn't deliver it last tick, or if this controller's own [-1, 1]
    // output clamp clipped it last tick.
    MixerLimits _mixer_limits;
    bool        _out_clipped[3] = { false, false, false };

    float _ctun_diag[10] = {};   // see get_ctun_diag()

    static constexpr float YAW_STICK_DEADBAND = 0.10f;  // normalised stick [-1,1], matches PosControl::STICK_DEADBAND
    static constexpr float YAW_HOLD_MAX_RATE  = 0.3f;   // rad/s cap on the heading-hold trim — needs flight tuning
};
