#pragma once

class Unmixer;

/*
 * AttitudeController — common interface for FlightStateMachine's
 * config-driven controller list (see code_rework.md Section 3.4).
 *
 * PID is always list index 0 and is the default; any additional controller
 * (INDI today) that a drone's config enables runs every tick alongside it
 * (shadow mode) regardless of which one is actually selected — see
 * FlightStateMachine::run_attitude().
 *
 * euler[3]:            roll, pitch, yaw (rad)
 * state_full[19]:      full EKF state (StateIdx::*)
 * input[]:             InputIdx::* (thrust, roll/pitch/yaw targets, flight_mode, control switch)
 * current_torque[2]:   [roll_Nm, pitch_Nm] from Unmixer — only INDI-shaped
 *                       controllers need this; PID-only controllers ignore it.
 * out_cmds[3]:         normalised torque [roll, pitch, yaw] in [-1, 1]
 */
class AttitudeController {
public:
    virtual void update(const float euler[3], const float state_full[],
                         const float input[], const float current_torque[2],
                         const Unmixer &unmixer, float out_cmds[3]) = 0;
    virtual void reset_all() = 0;

    // The estimator's yaw just stepped by delta_rad with no physical
    // rotation (StateManager::consume_yaw_reset()). Shift the held heading
    // target by the same amount — ArduPilot's inertial_frame_reset().
    virtual void yaw_frame_reset(float delta_rad) = 0;

    // POS_HOLD's heading controller (HeadingControl) supplies the yaw-rate
    // target directly. While enabled, a controller uses rate_rad_s as its
    // yaw-rate loop target in place of its own stick rate + heading-lock
    // trim, and keeps its held heading on the current one so there is no
    // stored error when this is switched off again. FlightStateMachine sets
    // this every tick; STABILIZE and ALT_HOLD always pass enabled = false.
    void set_external_yaw_rate(bool enabled, float rate_rad_s)
    {
        _ext_yaw_rate_enabled = enabled;
        _ext_yaw_rate         = rate_rad_s;
    }

protected:
    // Deliberately non-virtual and protected, not public+virtual: every
    // AttitudeController is a FlightStateMachine member with static storage
    // duration, never heap-allocated or deleted through a base pointer (see
    // FlightStateMachine::_controllers[]) — protected blocks that misuse at
    // compile time rather than needing a real virtual destructor. This also
    // keeps the class trivially destructible, which matters on this
    // embedded target: its minimal C++ runtime doesn't link operator
    // delete/__cxa_atexit, both of which a virtual destructor here would
    // pull in.
    ~AttitudeController() = default;

    bool  _ext_yaw_rate_enabled = false;   // see set_external_yaw_rate()
    float _ext_yaw_rate         = 0.0f;    // rad/s
};
