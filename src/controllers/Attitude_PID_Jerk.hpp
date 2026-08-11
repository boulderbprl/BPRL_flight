#pragma once
#include "PID.hpp"
#include "AttitudeController.hpp"

/*
 * Cascade attitude controller (PID) — outer attitude loop + inner rate loop,
 * plus a roll-axis jerk damping term. Variant of AttitudePID (see
 * Attitude_PID.hpp/.cpp), reachable via _controllers[] if the drone's config
 * enables it (DroneConfig::ControllersConfig::jerk_enabled) — selectable in
 * flight via the channel-7 switch, same shadow-when-not-selected pattern as
 * AttitudeINDI/AttitudePIDPI (runs and logs — LOG_MSG_PIDJ — every tick it's
 * in the list, regardless of which list index is actually selected).
 *
 * Outer loop: attitude error [rad]  → angular rate target [rad/s]
 * Inner loop, roll axis: _roll_jerk.update(roll_rate_tgt, roll_jerk) — reuses
 *   PID.hpp as a target-tracking controller directly on the fitted roll-jerk
 *   signal (see src/sensors/JerkFit.hpp) rather than on measured roll rate;
 *   its output is the final [-1,1]-clamped torque command (WIP — see the
 *   commented-out state[3]-based _roll_rate alternative in the .cpp).
 * Inner loop, pitch axis: plain rate PID, same as AttitudePID.
 *
 * Yaw has no angle loop of its own (unlike roll/pitch), so nothing
 * previously corrected a rate estimate that isn't exactly mean-zero —
 * heading could walk indefinitely under a nominally "zero rate" command.
 * _yaw_hold adds a small heading-lock trim on top of the rate PID: while
 * the stick is centered, _yaw_target stays fixed and a P correction pulls
 * current heading back to it; while actively yawing, _yaw_target tracks the
 * current heading so there's no stored error to fight when the stick
 * recentres. Mirrors the same trim in AttitudePID / AttitudeINDI.
 *
 * roll_jerk (the fitted roll-jerk estimate, JerkFit.hpp "Pdd") isn't part of
 * the shared AttitudeController::update() signature — set_roll_jerk() is
 * called once per tick by FlightStateMachine::run_attitude() (same pattern
 * as AttitudeINDI::set_indi_active()) before the generic update() dispatch,
 * so update() itself just reads the most recently set value.
 *
 * Conventions (own 6-element internal state, built from euler[]/state_full[]
 * inside update() to match the shared AttitudeController interface):
 *   state6[0..2]  roll, pitch, yaw in radians
 *   state6[3..5]  p, q, r body-frame rates in rad/s
 *   input[1..3]   roll_tgt, pitch_tgt, yaw_rate_tgt [-1, 1]
 *   out_cmds[3]   normalised torque [roll, pitch, yaw] in [-1, 1]
 *
 * current_torque/unmixer (from the shared interface) are unused here, same
 * as AttitudePID — this controller doesn't need RPM-derived torque feedback.
 */
class AttitudePIDJerk : public AttitudeController {
public:
    AttitudePIDJerk();

    void update(const float euler[3], const float state_full[], const float input[],
                const float current_torque[2], const Unmixer &unmixer,
                float out_cmds[3]) override;
    void reset_all() override;

    // Set once per tick by FlightStateMachine before the controller-list
    // dispatch — see class comment.
    void set_roll_jerk(float roll_jerk) { _roll_jerk_input = roll_jerk; }

    // Outer roll-loop angular rate target [rad/s] from the most recent
    // update() — diagnostic only, see FlightStateMachine::get_jerk_diag().
    float roll_rate_tgt() const { return _roll_rate_tgt; }

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;
    PID _pitch_rate;
    PID _yaw_rate;
    PID _yaw_hold;   // heading-lock trim: heading error [rad] -> corrective rate [rad/s]
    PID _roll_jerk;  // roll-jerk-tracking term: fitted roll_jerk [rad/s^3] -> corrective torque [-1,1]

    float _yaw_target;        // held heading target [rad]
    bool  _yaw_target_valid;  // false until first update() captures a target

    float _roll_jerk_input = 0.0f;  // most recent set_roll_jerk() value
    float _roll_rate_tgt   = 0.0f;  // most recent update()'s outer roll-loop target — see roll_rate_tgt()

    static constexpr float YAW_STICK_GAIN     = 3.0f;
    static constexpr float YAW_STICK_DEADBAND = 0.10f;  // normalised stick [-1,1], matches FlightStateMachine::STICK_DEADBAND
    static constexpr float YAW_HOLD_MAX_RATE  = 0.3f;   // rad/s cap on the heading-hold trim — needs flight tuning
};
