#include "HeadingControl.hpp"
#include "src/math/math.hpp"
#include "src/threads.hpp"   // CONTROL_DT_S
#include <cmath>

float HeadingControl::step(float yaw_now, float rate_des)
{
    const float dt = CONTROL_DT_S;

    // Start from where the nose is pointing, at rest.
    if (!_valid) {
        _target_cmd = yaw_now;
        _target     = yaw_now;
        _rate_tgt   = 0.0f;
        _rate_pred  = 0.0f;
        _valid      = true;
    }

    _rate_tgt  += constrain_float(rate_des - _rate_tgt, -_g.max_accel * dt, _g.max_accel * dt);
    _rate_pred += (dt / (_g.lag_s + dt)) * (_rate_tgt - _rate_pred);
    _target_cmd = wrap_pi(_target_cmd + _rate_tgt  * dt);
    _target     = wrap_pi(_target     + _rate_pred * dt);

    // Leash: don't let the target run away from a vehicle that can't follow
    // (yaw-saturated, on the ground), which would wind up the heading error.
    float err = wrap_pi(_target - yaw_now);
    if (fabsf(err) > LEASH) {
        const float pulled = constrain_float(err, -LEASH, LEASH);
        _target_cmd = wrap_pi(_target_cmd + (pulled - err));
        _target     = wrap_pi(yaw_now + pulled);
        err         = pulled;
    }

    _rate_cmd = _rate_tgt + constrain_float(_g.kp * err, -_g.max_rate, _g.max_rate);
    return _rate_cmd;
}

float HeadingControl::update_rate(float yaw_now, float rate_req)
{
    return step(yaw_now, rate_req);
}

float HeadingControl::update_heading(float yaw_now, float yaw_des, float rate_ff)
{
    // Approach rate toward yaw_des: proportional close in, sqrt profile
    // (constant deceleration) further out, capped at max_rate. The sqrt
    // profile uses half of max_accel so the slew limit in step() can always
    // keep up with it.
    //
    // Steered on _target_cmd, the un-lagged target. The lagged one (which
    // the vehicle follows) trails it by rate × lag_s while turning, so aim
    // that far ahead of a moving yaw_des to end up on it.
    const float aim = yaw_des + rate_ff * _g.lag_s;
    const float err = _valid ? wrap_pi(aim - _target_cmd) : wrap_pi(aim - yaw_now);
    const float mag = fabsf(err);
    float approach  = fminf(SHAPE_KP * mag, sqrtf(_g.max_accel * mag));
    approach        = fminf(approach, _g.max_rate);
    return step(yaw_now, rate_ff + (err < 0.0f ? -approach : approach));
}

void HeadingControl::yaw_frame_reset(float delta_rad)
{
    _target_cmd = wrap_pi(_target_cmd + delta_rad);
    _target     = wrap_pi(_target + delta_rad);
}
