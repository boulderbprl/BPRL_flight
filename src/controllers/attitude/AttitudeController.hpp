#pragma once
#include "src/controllers/extras/MotorMixer.hpp"
#include <cmath>

class Unmixer;

/*
 * AttitudeController — common interface for FlightStateMachine's
 * config-driven controller list.
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

    // Which axes the mixer saturated on last tick (MotorMixer::limits()),
    // for integrator anti-windup. Only the controller actually flying gets
    // the real flags — see FlightStateMachine::set_mixer_limits(). No-op
    // for controllers that don't use it.
    virtual void set_mixer_limits(const MixerLimits & /*limits*/) {}

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

    // Euler-angle rate targets → body rate targets, in place (ArduPilot
    // AC_AttitudeControl::euler_rate_to_ang_vel). The angle loops produce
    // roll/pitch Euler rates and the yaw stick / HeadingControl a heading
    // rate, but the rate loops close on body p/q/r. Feeding the heading rate
    // straight in as body r rotates a tilted vehicle about its own (tilted)
    // z axis, which drags roll and pitch with it (pitch at -r·sinφ, roll at
    // r·tanθ) — on Drone3 that was the roll/pitch motion on every yaw
    // (LOG0020, +1.5° hover pitch trim) and a ~15° pitch error in a fast
    // banked circle (q = ψ̇·sinφ ≈ 1.8 rad/s at 2.9 rad/s, 38°). Identity in
    // level flight.
    //   p = φ̇ − ψ̇·sinθ
    //   q = θ̇·cosφ + ψ̇·sinφ·cosθ
    //   r = −θ̇·sinφ + ψ̇·cosφ·cosθ
    static void euler_rate_to_body(const float euler[3], float &roll_rate, float &pitch_rate, float &yaw_rate)
    {
        const float sr = sinf(euler[0]), cr = cosf(euler[0]);
        const float sp = sinf(euler[1]), cp = cosf(euler[1]);
        const float er = roll_rate, ep = pitch_rate, ey = yaw_rate;
        roll_rate  = er - ey * sp;
        pitch_rate = cr * ep + sr * cp * ey;
        yaw_rate   = -sr * ep + cr * cp * ey;
    }

    bool  _ext_yaw_rate_enabled = false;   // see set_external_yaw_rate()
    float _ext_yaw_rate         = 0.0f;    // rad/s
};
