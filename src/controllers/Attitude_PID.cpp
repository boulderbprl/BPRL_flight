#include "Attitude_PID.hpp"
#include "src/math/math.hpp"

AttitudePID::AttitudePID()
    : _roll_att  (3.50f, 0.00f, 0.000f, 0.5f, 30.0f)
    , _pitch_att (3.50f, 0.00f, 0.000f, 0.5f, 30.0f)
    , _roll_rate (0.09f, 0.075f, 0.001f, 0.5f, 30.0f)
    , _pitch_rate(0.13f, 0.100f, 0.002f, 0.5f, 30.0f)
    , _yaw_rate  (0.18f, 0.018f, 0.000f, 0.5f, 5.0f)
{}

void AttitudePID::update(const float state[], const float input[],
                         float out_cmds[3])
{
    const float roll_rate_tgt  = _roll_att.update(input[1] - state[0]);
    const float pitch_rate_tgt = _pitch_att.update(input[2] - state[1]);

    out_cmds[0] = constrain_float(_roll_rate.update(roll_rate_tgt      - state[3]), -1.0f, 1.0f);
    out_cmds[1] = constrain_float(_pitch_rate.update(pitch_rate_tgt    - state[4]), -1.0f, 1.0f);
    out_cmds[2] = constrain_float(_yaw_rate.update(YAW_STICK_GAIN * input[3] - state[5]), -1.0f, 1.0f);
}

void AttitudePID::reset_all()
{
    _roll_att.reset();  _pitch_att.reset();
    _roll_rate.reset(); _pitch_rate.reset();
    _yaw_rate.reset();
}
