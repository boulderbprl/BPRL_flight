#include "Attitude_PID_Jerk.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include <cmath>

AttitudePIDJerk::AttitudePIDJerk()
    : _roll_att  (4.00f, 0.00f, 0.000f, 0.5f, 0.0f, 0.0f, 30.0f)
    , _pitch_att (4.00f, 0.00f, 0.000f, 0.5f, 0.0f, 0.0f, 30.0f)
    , _roll_rate (0.08f, 0.05f, 0.002f, 0.5f, 20.0f, 0.0f, 30.0f)
    , _pitch_rate(0.09f, 0.06f, 0.002f, 0.5f, 20.0f, 0.0f, 30.0f)
    , _yaw_rate  (0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f)
    , _yaw_hold  (0.60f, 0.050f, 0.000f, 0.3f, 0.0f, 0.0f, 30.0f)
    // kp=0.001, ki=0.001 — first-cut small values, same order as each other.
    // imax=0.05 caps the jerk integrator's contribution to at most 5% of
    // full torque authority even fully wound up. kd=0 (no D-on-jerk yet).
    // filt_error_hz=40 — estimate_jerk() (JerkFit.hpp) is an unfiltered
    // linear-regression readout of raw strain channels, so knock down strain
    // noise before it hits P/I. Kept high enough (>> the ~few-Hz band a
    // twice-differentiated, filtered p_dot would resolve) to preserve the
    // fast jerk-band response that's the reason to use strain over gyro here.
    , _roll_jerk (0.001f, 0.0005f, 0.0f, 0.2f, 0.0f, 40.0f, 20.0f)
    , _yaw_target(0.0f)
    , _yaw_target_valid(false)
{}

void AttitudePIDJerk::update(const float euler[3], const float state_full[], const float input[],
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

    _roll_rate_tgt = 73.3342f*_roll_att.update(input[1], state6[0]);
    const float pitch_rate_tgt = _pitch_att.update(input[2], state6[1]);

    const float roll_torque = _roll_jerk.update(_roll_rate_tgt, _roll_jerk_input);
    // const float roll_torque = _roll_rate.update(_roll_rate_tgt, state6[3]) - jerk_correction;
    out_cmds[0] = constrain_float(roll_torque, -1.0f, 1.0f);
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

    out_cmds[2] = constrain_float(_yaw_rate.update(YAW_STICK_GAIN * input[3] + yaw_hold_rate, state6[5]), -1.0f, 1.0f);
}

void AttitudePIDJerk::reset_all()
{
    _roll_att.reset();  _pitch_att.reset();
    _roll_rate.reset(); _pitch_rate.reset();
    _yaw_rate.reset();  _yaw_hold.reset();
    _roll_jerk.reset();
    _yaw_target_valid = false;
}
