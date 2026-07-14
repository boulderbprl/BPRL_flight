#include "AltControl.hpp"
#include "src/math/math.hpp"
#include <cmath>
#include <algorithm>

AltControl::AltControl()
    // TODO: retune now that this loop commands throttle directly instead of
    // an intermediate accel target — gains below are a conservative starting
    // point, not a validated tune.
    : _climb_rate_pid(0.15f, 0.05f, 0.0f, 0.3f, /*filt_target_hz=*/0.0f, /*filt_error_hz=*/5.0f, /*filt_d_hz=*/20.0f)
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

float AltControl::run_loop(float rate_tgt, float vD)
{
    // Positive delta_thr = want to accelerate downward → reduce throttle
    const float delta_thr = _climb_rate_pid.update(rate_tgt, vD);
    return constrain_float(THR_MID - delta_thr, 0.0f, 1.0f);
}

float AltControl::alt_hold_from_stick(float pilot_thr, float vD)
{
    return run_loop(stick_to_climb_rate(pilot_thr), vD);
}

float AltControl::alt_hold_from_rate(float rate_tgt, float vD)
{
    return run_loop(rate_tgt, vD);
}

void AltControl::reset_all()
{
    _climb_rate_pid.reset();
}
