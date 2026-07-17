#include "Attitude_INDI.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include <cmath>

AttitudeINDI::AttitudeINDI()
    : _roll_att  (3.50f, 0.00f, 0.000f, 0.5f, 0.0f, 0.0f, 30.0f)
    , _pitch_att (3.50f, 0.00f, 0.000f, 0.5f, 0.0f, 0.0f, 30.0f)
    , _roll_rate (4.5f, 0.00f, 0.6f, 15.0f, 0.0f, 0.0f, 30.0f)
    , _pitch_rate(4.5f, 0.00f, 0.6f, 15.0f, 0.0f, 0.0f, 30.0f)
    , _yaw_rate  (0.065f, 0.02f, 0.000f, 0.5f, 0.0f, 0.0f, 30.0f)
    , _yaw_hold  (0.60f, 0.05f, 0.000f, 0.2f, 0.0f, 0.0f, 30.0f)
    , _yaw_target(0.0f)
    , _yaw_target_valid(false)
{}

void AttitudeINDI::update(const float euler[3], const float state_full[],
                          const float input[], const float current_torque[2],
                          const Unmixer &unmixer, float out_cmds[3],
                          float delta_torque[2], float accel_cmd[2])
{
    // ── Outer loop: angle error → rate target ─────────────────────────────
    const float roll_rate_tgt  = _roll_att.update(input[1], euler[0]);
    const float pitch_rate_tgt = _pitch_att.update(input[2], euler[1]);

    const float p = state_full[StateIdx::P];
    const float q = state_full[StateIdx::Q];
    const float r = state_full[StateIdx::R];

    // ── Inner loop: rate error → commanded angular acceleration (rad/s²) ──
    const float accel_cmd_roll  = _roll_rate.update(roll_rate_tgt,  p);
    const float accel_cmd_pitch = _pitch_rate.update(pitch_rate_tgt, q);

    accel_cmd[0] = accel_cmd_roll;
    accel_cmd[1] = accel_cmd_pitch;

    // ── INDI step: incremental torque from acceleration error ──────────────
    const float p_dot_meas = state_full[StateIdx::P_DOT]; 
    const float q_dot_meas = state_full[StateIdx::Q_DOT];

    const float delta_torque_roll  = (accel_cmd_roll  - p_dot_meas) * INDI_GAIN_ROLL;
    const float delta_torque_pitch = (accel_cmd_pitch - q_dot_meas) * INDI_GAIN_PITCH;

    delta_torque[0] = delta_torque_roll;
    delta_torque[1] = delta_torque_pitch;

    out_cmds[0] = unmixer.normalize_torque(current_torque[0] + delta_torque_roll);
    out_cmds[1] = unmixer.normalize_torque(current_torque[1] + delta_torque_pitch);

    // ── Yaw: rate PID + heading-lock trim ──────────────────────────────────
    // Roll/pitch self-correct drift via their angle loop; yaw has none, so
    // add a small corrective rate from heading error whenever the stick is
    // centered. While actively yawing, keep _yaw_target tracking the current
    // heading so there's no stored error to fight when the stick recentres.
    const float yaw_now = euler[2];
    if (fabsf(input[3]) > YAW_STICK_DEADBAND || !_yaw_target_valid) {
        _yaw_target       = yaw_now;
        _yaw_target_valid = true;
    }
    const float yaw_err       = wrap_pi(yaw_now - _yaw_target);
    const float yaw_hold_rate = constrain_float(_yaw_hold.update(0.0f, yaw_err),
                                                 -YAW_HOLD_MAX_RATE, YAW_HOLD_MAX_RATE);

    out_cmds[2] = _yaw_rate.update(YAW_GAIN * input[3] + yaw_hold_rate, r);
}

void AttitudeINDI::reset_all()
{
    _roll_att.reset();  _pitch_att.reset();
    _roll_rate.reset(); _pitch_rate.reset();
    _yaw_rate.reset();  _yaw_hold.reset();
    _yaw_target_valid = false;
}
