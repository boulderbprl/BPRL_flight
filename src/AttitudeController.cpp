#include "AttitudeController.hpp"
#include <cmath>
#include <algorithm>

static inline float constrain_float(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Gains are the tuned values from the FreeRTOS/Tiva flight tests.
// Outer loop (attitude → rate): P-only (Kp=4.5, no I/D).
// Inner loop (rate → torque):
//   Roll/Pitch: Kp=0.11, Ki=0.09, Kd=0.003, Imax=0.5, fcut=30 Hz
//   Yaw:        Kp=0.10, Ki=0.02, Kd=0,     Imax=0.5
AttitudeController::AttitudeController()
    : _roll_att  (4.50f, 0.00f, 0.000f, 0.5f, 30.0f)
    , _pitch_att (4.50f, 0.00f, 0.000f, 0.5f, 30.0f)
    , _roll_rate (0.11f, 0.09f, 0.003f, 0.5f, 30.0f)
    , _pitch_rate(0.11f, 0.09f, 0.003f, 0.5f, 30.0f)
    , _yaw_rate  (0.10f, 0.02f, 0.000f, 0.5f, 30.0f)
{}

void AttitudeController::update(const float state[], const float input[],
                                float out_cmds[3])
{
    // Outer loop: attitude error → rate targets
    const float roll_rate_tgt  = _roll_att.update(input[1] - state[0]);
    const float pitch_rate_tgt = _pitch_att.update(input[2] - state[1]);

    // Inner loop: rate error → torque commands
    out_cmds[0] = _roll_rate.update(roll_rate_tgt         - state[3]);
    out_cmds[1] = _pitch_rate.update(pitch_rate_tgt       - state[4]);
    out_cmds[2] = _yaw_rate.update(YAW_GAIN * input[3]    - state[5]);
}

float AttitudeController::compute_throttle(const float state[],
                                           const float input[]) const
{
    const float thr_in = input[0];
    const float expo   = -(THR_MID - 0.5f) / 0.375f;
    const float thr_exp = thr_in * (1.0f - expo)
                          + expo * thr_in * thr_in * thr_in;
    // Angle-of-attack boost
    const float boost = 1.0f / std::min(cosf(state[0]), cosf(state[1]));
    return constrain_float(thr_exp * boost, 0.0f, 1.0f);
}

void AttitudeController::reset_all()
{
    _roll_att.reset();  _pitch_att.reset();
    _roll_rate.reset(); _pitch_rate.reset();
    _yaw_rate.reset();
}
