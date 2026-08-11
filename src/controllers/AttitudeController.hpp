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
};
