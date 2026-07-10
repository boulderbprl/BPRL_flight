#include "Attitude_INDI.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"

AttitudeINDI::AttitudeINDI()
    : _roll_att  (3.50f, 0.00f, 0.000f, 0.5f, 30.0f)
    , _pitch_att (3.50f, 0.00f, 0.000f, 0.5f, 30.0f)
    , _roll_rate (6.50f, 0.00f, 0.65f, 0.5f, 30.0f)
    , _pitch_rate(6.50f, 0.00f, 0.65f, 0.5f, 30.0f)
    , _yaw_rate  (0.065f, 0.02f, 0.000f, 0.5f, 30.0f)
{}

void AttitudeINDI::update(const float euler[3], const float state_full[],
                          const float input[], const float current_torque[2],
                          const Unmixer &unmixer, float out_cmds[3],
                          float delta_torque[2], float accel_cmd[2])
{
    // ── Outer loop: angle error → rate target ─────────────────────────────
    const float roll_rate_tgt  = _roll_att.update(input[1] - euler[0]);
    const float pitch_rate_tgt = _pitch_att.update(input[2] - euler[1]);

    const float p = state_full[StateIdx::P];
    const float q = state_full[StateIdx::Q];
    const float r = state_full[StateIdx::R];

    // ── Inner loop: rate error → commanded angular acceleration (rad/s²) ──
    const float accel_cmd_roll  = _roll_rate.update(roll_rate_tgt  - p);
    const float accel_cmd_pitch = _pitch_rate.update(pitch_rate_tgt - q);

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

    // ── Yaw: standard rate PID ─────────────────────────────────────────────
    out_cmds[2] = _yaw_rate.update(YAW_GAIN * input[3] - r);
}

void AttitudeINDI::reset_all()
{
    _roll_att.reset();  _pitch_att.reset();
    _roll_rate.reset(); _pitch_rate.reset();
    _yaw_rate.reset();
}
