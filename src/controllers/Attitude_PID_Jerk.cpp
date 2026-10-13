#include "Attitude_PID_Jerk.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include <cmath>

// Roll inertia estimate [kg*m^2] used only to scale the accel-stage gains —
// matches g1_seed_roll in the drone configs. See class comment.
static constexpr float ROLL_INERTIA = 0.006f;

AttitudePIDJerk::AttitudePIDJerk()
    : _roll_att  (4.00f, 0.00f, 0.000f, 0.5f, 0.0f, 0.0f, 30.0f)
    , _pitch_att (4.00f, 0.00f, 0.000f, 0.5f, 0.0f, 0.0f, 30.0f)
    // Rate loop ("SLC") outputs an accel target, same role/gains as
    // AttitudeINDI's / AttitudePIDPI's rate loop.
    , _roll_rate (6.50f, 0.20f, 0.000f, 10.0f, 30.0f, 0.0f, 30.0f)
    // Accel stage PI: Kp = I*kappa (kappa = 1), Ki = I*omega_a (omega_a = 30
    // rad/s). The integrator is the held roll torque, so imax = 1.2 N*m
    // (~95% of T_max) — see class comment.
    , _roll_accel(ROLL_INERTIA * 1.0f, ROLL_INERTIA * 30.0f, 0.0f, 1.2f, 0.0f, 0.0f, 30.0f)
    , _pitch_rate(0.09f, 0.06f, 0.002f, 0.5f, 20.0f, 0.0f, 30.0f)
    , _yaw_rate  (0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f)
    , _yaw_hold  (0.60f, 0.050f, 0.000f, 0.3f, 0.0f, 0.0f, 30.0f)
    , _yaw_target(0.0f)
    , _yaw_target_valid(false)
{}

void AttitudePIDJerk::update(const float euler[3], const float state_full[], const float input[],
                             const float /*current_torque*/[2], const Unmixer &unmixer,
                             float out_cmds[3])
{
    const float p     = state_full[StateIdx::P];
    const float q     = state_full[StateIdx::Q];
    const float r     = state_full[StateIdx::R];
    const float p_dot = state_full[StateIdx::P_DOT];

    // ── Roll: angle → rate → accel PI + jerk D → torque ────────────────────
    _roll_rate_tgt  = _roll_att.update(input[1], euler[0]);
    _roll_accel_tgt = _roll_rate.update(_roll_rate_tgt, p);

    _roll_torque_pi = _roll_accel.update(_roll_accel_tgt, p_dot);

    // Diagnostic jerk target (logged only, not used below).
    const float jerk_tgt_raw = _prev_accel_tgt_valid
                             ? (_roll_accel_tgt - _prev_accel_tgt) / CTRL_DT_S : 0.0f;
    _prev_accel_tgt       = _roll_accel_tgt;
    _prev_accel_tgt_valid = true;
    _roll_jerk_tgt = lowpass2p(jerk_tgt_raw, _jerk_tgt_lpf_state, JERK_D_LPF_HZ, CTRL_DT_S);

    // Jerk D on measurement (jerk_tgt = 0): the D term alone gets the 20 Hz
    // D lowpass; the accel PI is unaffected.
    const float jerk_d_in = lowpass2p(_roll_jerk_input, _jerk_d_lpf_state, JERK_D_LPF_HZ, CTRL_DT_S);
    _roll_torque_d  = -ROLL_JERK_KD * jerk_d_in;

    _roll_torque_nm = constrain_float(_roll_torque_pi + _roll_torque_d,
                                      -unmixer.max_torque_nm(), unmixer.max_torque_nm());
    out_cmds[0] = unmixer.normalize_torque(_roll_torque_nm);

    // ── Pitch: plain cascade, same as AttitudePID ──────────────────────────
    const float pitch_rate_tgt = _pitch_att.update(input[2], euler[1]);
    out_cmds[1] = constrain_float(_pitch_rate.update(pitch_rate_tgt, q), -1.0f, 1.0f);

    // ── Yaw: rate PID + heading-lock trim
    const float yaw_now = euler[2];
    if (fabsf(input[3]) > YAW_STICK_DEADBAND || !_yaw_target_valid) {
        _yaw_target       = yaw_now;
        _yaw_target_valid = true;
    }
    const float yaw_err       = wrap_pi(yaw_now - _yaw_target);
    const float yaw_hold_rate = constrain_float(_yaw_hold.update(0.0f, yaw_err),
                                                 -YAW_HOLD_MAX_RATE, YAW_HOLD_MAX_RATE);

    out_cmds[2] = constrain_float(_yaw_rate.update(YAW_STICK_GAIN * input[3] + yaw_hold_rate, r), -1.0f, 1.0f);
}

void AttitudePIDJerk::reset_all()
{
    _roll_att.reset();   _pitch_att.reset();
    _roll_rate.reset();  _roll_accel.reset();
    _pitch_rate.reset();
    _yaw_rate.reset();   _yaw_hold.reset();
    _jerk_d_lpf_state     = Biquad2pState();
    _jerk_tgt_lpf_state   = Biquad2pState();
    _prev_accel_tgt_valid = false;
    _yaw_target_valid = false;
}

void AttitudePIDJerk::prepare_handover(float current_roll_torque_nm)
{
    // _roll_accel is deliberately not reset: PID::reset() + the first update
    // after it returns P-only (no integrator), which would drop the seeded
    // trim torque for a tick. Only its integrator is overwritten.
    _roll_att.reset();   _pitch_att.reset();
    _roll_rate.reset();
    _pitch_rate.reset();
    _yaw_rate.reset();   _yaw_hold.reset();
    _yaw_target_valid = false;
    _prev_accel_tgt_valid = false;   // rate-loop reset steps accel_tgt — don't log it as a jerk spike
    _roll_accel.set_integrator(current_roll_torque_nm);
}
