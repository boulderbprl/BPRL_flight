#include "Attitude_PID.hpp"
#include "src/math/math.hpp"
#include <cmath>

AttitudePID::AttitudePID()
    : _roll_att  (4.00f, 0.00f, 0.000f, 0.5f, 0.0f, 0.0f, 30.0f)
    , _pitch_att (4.00f, 0.00f, 0.000f, 0.5f, 0.0f, 0.0f, 30.0f)
    , _roll_rate (0.08f, 0.05f, 0.002f, 0.5f, 20.0f, 0.0f, 30.0f)
    , _pitch_rate(0.09f, 0.06f, 0.002f, 0.5f, 20.0f, 0.0f, 30.0f)
    , _yaw_rate  (0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f)
    , _yaw_hold  (0.60f, 0.050f, 0.000f, 0.3f, 0.0f, 0.0f, 30.0f)
    , _yaw_target(0.0f)
    , _yaw_target_valid(false)
{}

void AttitudePID::update(const float state[], const float input[],
                         float out_cmds[3])
{
    const float roll_rate_tgt  = _roll_att.update(input[1], state[0]);
    const float pitch_rate_tgt = _pitch_att.update(input[2], state[1]);

    out_cmds[0] = constrain_float(_roll_rate.update(roll_rate_tgt,   state[3]), -1.0f, 1.0f);
    out_cmds[1] = constrain_float(_pitch_rate.update(pitch_rate_tgt, state[4]), -1.0f, 1.0f);

    // ── Yaw: rate PID + heading-lock trim 
    const float yaw_now = state[2];
    if (fabsf(input[3]) > YAW_STICK_DEADBAND || !_yaw_target_valid) {
        _yaw_target       = yaw_now;
        _yaw_target_valid = true;
    }
    const float yaw_err       = wrap_pi(yaw_now - _yaw_target);
    const float yaw_hold_rate = constrain_float(_yaw_hold.update(0.0f, yaw_err),
                                                 -YAW_HOLD_MAX_RATE, YAW_HOLD_MAX_RATE);

    out_cmds[2] = constrain_float(_yaw_rate.update(YAW_STICK_GAIN * input[3] + yaw_hold_rate, state[5]), -1.0f, 1.0f);
}

void AttitudePID::reset_all()
{
    _roll_att.reset();  _pitch_att.reset();
    _roll_rate.reset(); _pitch_rate.reset();
    _yaw_rate.reset();  _yaw_hold.reset();
    _yaw_target_valid = false;
}
