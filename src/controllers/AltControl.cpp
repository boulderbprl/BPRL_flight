#include "AltControl.hpp"
#include "src/math/math.hpp"
#include "src/threads.hpp"   // CONTROL_DT_S
#include <cmath>
#include <algorithm>

AltControl::AltControl(const AltControlGains &g)
    // TODO: retune now that this loop commands throttle directly instead of
    // an intermediate accel target — gains below are a conservative starting
    // point, not a validated tune.
    : _pos_pid(g.pos_D.kp, g.pos_D.ki, g.pos_D.kd, g.pos_D.imax,
               g.pos_D.filt_target_hz, g.pos_D.filt_error_hz, g.pos_D.filt_d_hz)
    , _climb_rate_pid(g.climb_rate.kp, g.climb_rate.ki, g.climb_rate.kd, g.climb_rate.imax,
                       g.climb_rate.filt_target_hz, g.climb_rate.filt_error_hz, g.climb_rate.filt_d_hz)
{}

float AltControl::compute_throttle(float roll, float pitch, float thr_in) const
{
    const float expo    = -(THR_MID - 0.5f) / 0.375f;
    const float thr_exp = thr_in * (1.0f - expo)
                          + expo * thr_in * thr_in * thr_in;
    const float boost   = 1.0f / std::min(cosf(roll), cosf(pitch));
    return constrain_float(thr_exp * boost, 0.0f, 1.0f);
}

float AltControl::stick_to_climb_rate(float pilot_thr) const
{
    const float centered = pilot_thr - 0.5f;  // [-0.5, +0.5]
    const float half     = 0.5f - DEADBAND;
    if (centered > DEADBAND) {
        // stick above centre → climb (negative W / negative vD)
        return -((centered - DEADBAND) / half) * MAX_CLIMB_RATE;
    } else if (centered < -DEADBAND) {
        // stick below centre → descend (positive W / positive vD)
        return -((centered + DEADBAND) / half) * MAX_CLIMB_RATE;
    }
    return 0.0f;
}

float AltControl::alt_hold(float pilot_thr, float cur_D, float vD)
{
    const float climb_stick = stick_to_climb_rate(pilot_thr);

    if (!_alt_tgt_valid) {
        _alt_tgt_D     = cur_D;   // seed on the first call after a reset
        _alt_tgt_valid = true;
    }
    _alt_tgt_D += climb_stick * CONTROL_DT_S;
    // Leash: don't let the target run away from a vehicle that can't follow
    // (on the ground, thrust-saturated), which would wind up the position error.
    _alt_tgt_D = constrain_float(_alt_tgt_D, cur_D - ALT_LEASH_M, cur_D + ALT_LEASH_M);

    _rate_tgt = constrain_float(climb_stick + _pos_pid.update(_alt_tgt_D, cur_D),
                                -MAX_CLIMB_RATE, MAX_CLIMB_RATE);

    // Positive delta_thr = want to accelerate downward → reduce throttle
    const float delta_thr = _climb_rate_pid.update(_rate_tgt, vD);
    return constrain_float(THR_MID - delta_thr, 0.0f, 1.0f);
}

void AltControl::reset_all()
{
    _pos_pid.reset();
    _climb_rate_pid.reset();
    _alt_tgt_valid = false;
    _rate_tgt      = 0.0f;
}
