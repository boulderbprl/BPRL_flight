#include "Attitude_PID.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include <cmath>

static PID make_pid(const PidGains &g)
{
    return PID(g.kp, g.ki, g.kd, g.imax, g.filt_target_hz, g.filt_error_hz, g.filt_d_hz);
}

AttitudePID::AttitudePID(const AttitudePidGains &g)
    : _roll_att  (make_pid(g.roll_att))
    , _pitch_att (make_pid(g.pitch_att))
    , _roll_rate (make_pid(g.roll_rate))
    , _pitch_rate(make_pid(g.pitch_rate))
    , _yaw_rate  (make_pid(g.yaw_rate))
    , _yaw_hold  (make_pid(g.yaw_hold))
    , _yaw_stick_gain(g.yaw_stick_gain)
    , _yaw_target(0.0f)
    , _yaw_target_valid(false)
{}

void AttitudePID::update(const float euler[3], const float state_full[], const float input[],
                         const float /*current_torque*/[2], const Unmixer & /*unmixer*/,
                         float out_cmds[3])
{
    // This controller doesn't need RPM-derived torque feedback (unlike
    // AttitudeINDI), so it builds its own 6-element state locally rather
    // than taking current_torque/unmixer from the shared interface.
    const float state6[6] = {
        euler[0], euler[1], euler[2],
        state_full[StateIdx::P], state_full[StateIdx::Q], state_full[StateIdx::R]
    };

    const float roll_rate_tgt  = _roll_att.update(input[1], state6[0]);
    const float pitch_rate_tgt = _pitch_att.update(input[2], state6[1]);

    out_cmds[0] = constrain_float(_roll_rate.update(roll_rate_tgt,   state6[3]), -1.0f, 1.0f);
    out_cmds[1] = constrain_float(_pitch_rate.update(pitch_rate_tgt, state6[4]), -1.0f, 1.0f);

    // ── Yaw: rate PID + heading-lock trim
    const float yaw_now = state6[2];
    if (fabsf(input[3]) > YAW_STICK_DEADBAND || !_yaw_target_valid) {
        _yaw_target       = yaw_now;
        _yaw_target_valid = true;
    }
    const float yaw_err       = wrap_pi(yaw_now - _yaw_target);
    const float yaw_hold_rate = constrain_float(_yaw_hold.update(0.0f, yaw_err),
                                                 -YAW_HOLD_MAX_RATE, YAW_HOLD_MAX_RATE);

    out_cmds[2] = constrain_float(_yaw_rate.update(_yaw_stick_gain * input[3] + yaw_hold_rate, state6[5]), -1.0f, 1.0f);
}

void AttitudePID::reset_all()
{
    _roll_att.reset();  _pitch_att.reset();
    _roll_rate.reset(); _pitch_rate.reset();
    _yaw_rate.reset();  _yaw_hold.reset();
    _yaw_target_valid = false;
}
