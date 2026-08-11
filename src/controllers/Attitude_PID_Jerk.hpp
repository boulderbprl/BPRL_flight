#pragma once
#include "PID.hpp"

/*
 * Cascade attitude controller (PID) — outer attitude loop + inner rate loop,
 * plus a roll-axis jerk damping term. Variant of AttitudePID (see
 * Attitude_PID.hpp/.cpp) selectable in flight via the channel-7 switch's 3rd
 * position (see FlightStateMachine::_use_jerk) — always runs and logs
 * (LOG_MSG_PIDJ) regardless of which position is selected, same shadow
 * pattern as AttitudeINDI.
 *
 * Outer loop: attitude error [rad]  → angular rate target [rad/s]
 * Inner loop: rate error [rad/s]    → normalised torque output [-1, 1]
 *   roll axis only: normalised torque -= _roll_jerk.update(0.0f, roll_jerk),
 *   a PI damping term against the fitted roll-jerk estimate (see
 *   src/sensors/JerkFit.hpp). Reuses PID.hpp as a jerk-error-to-zero
 *   controller (target 0, measurement = roll_jerk) purely for its dt-aware
 *   P+I and anti-windup — filt_target_hz/filt_error_hz are both disabled so
 *   the signal feeds back unfiltered, per the fit already being filtered
 *   upstream. This is the innermost correction on the roll axis: nothing
 *   runs after it except the final [-1,1] clamp.
 *
 * Yaw has no angle loop of its own (unlike roll/pitch), so nothing
 * previously corrected a rate estimate that isn't exactly mean-zero —
 * heading could walk indefinitely under a nominally "zero rate" command.
 * _yaw_hold adds a small heading-lock trim on top of the rate PID: while
 * the stick is centered, _yaw_target stays fixed and a P correction pulls
 * current heading (state[2]) back to it; while actively yawing, _yaw_target
 * tracks the current heading so there's no stored error to fight when the
 * stick recentres. Mirrors the same trim in AttitudePID / AttitudeINDI.
 *
 * Conventions:
 *   state[0..2]  roll, pitch, yaw in radians
 *   state[3..5]  p, q, r body-frame rates in rad/s
 *   input[1..3]  roll_tgt, pitch_tgt, yaw_rate_tgt [-1, 1]
 *   roll_jerk    fitted roll jerk estimate [rad/s^3] (JerkFit.hpp, "Pdd")
 *   out_cmds[3]  normalised torque [roll, pitch, yaw] in [-1, 1]
 *   roll_rate_tgt  outer roll-loop angular rate target [rad/s] (diagnostic out)
 */
class AttitudePIDJerk {
public:
    AttitudePIDJerk();

    void update(const float state[], const float input[], float roll_jerk,
                float out_cmds[3], float &roll_rate_tgt);
    void reset_all();

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;
    PID _pitch_rate;
    PID _yaw_rate;
    PID _yaw_hold;   // heading-lock trim: heading error [rad] -> corrective rate [rad/s]
    PID _roll_jerk;  // PI-on-jerk damping term: bias-corrected roll_jerk [rad/s^3] -> corrective torque [-1,1]

    float _yaw_target;        // held heading target [rad]
    bool  _yaw_target_valid;  // false until first update() captures a target

    static constexpr float YAW_STICK_GAIN     = 3.0f;
    static constexpr float YAW_STICK_DEADBAND = 0.10f;  // normalised stick [-1,1], matches FlightStateMachine::STICK_DEADBAND
    static constexpr float YAW_HOLD_MAX_RATE  = 0.3f;   // rad/s cap on the heading-hold trim — needs flight tuning
};
