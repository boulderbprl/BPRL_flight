#include "Attitude_INDI_Jerk.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include <cmath>

static PID make_pid(const PidGains &g)
{
    return PID(g.kp, g.ki, g.kd, g.imax, g.filt_target_hz, g.filt_error_hz, g.filt_d_hz);
}

AttitudeINDIJerk::AttitudeINDIJerk(const AttitudeIndiJerkGains &g)
    : _roll_att  (make_pid(g.roll_att))
    , _pitch_att (make_pid(g.pitch_att))
    , _roll_rate (make_pid(g.roll_rate))
    , _pitch_rate(make_pid(g.pitch_rate))
    , _roll_accel(make_pid(g.roll_accel))
    , _yaw_rate  (make_pid(g.yaw_rate))
    , _yaw_hold  (make_pid(g.yaw_hold))
    , _g1_roll(g.g1_seed_roll)
    , _g1_pitch(g.g1_seed_pitch)
    , _g1_seed_roll(g.g1_seed_roll)
    , _g1_seed_pitch(g.g1_seed_pitch)
    , _g2_roll(g.g2_seed_roll)
    , _g2_seed_roll(g.g2_seed_roll)
    , _kappa_roll(g.indi_output_gain_roll)
    , _kappa_pitch(g.indi_output_gain_pitch)
    , _kappa2_roll(g.jerk_output_gain_roll)
    , _mu_pid(g.nlms_mu_pid)
    , _mu_indi(g.nlms_mu_indi)
    , _yaw_gain(g.yaw_gain)
    , _yaw_target(0.0f)
    , _yaw_target_valid(false)
    , _jerk_cmd_notch_coeffs(notch_coeffs(JERK_CMD_NOTCH_CENTER_HZ, JERK_CMD_NOTCH_BW_HZ, NLMS_DT_S))  // TEMP, see class comment on JERK_CMD_NOTCH_CENTER_HZ
{}

void AttitudeINDIJerk::update(const float euler[3], const float state_full[],
                              const float input[], const float current_torque[2],
                              const Unmixer &unmixer, float out_cmds[3])
{
    // ── Outer loop: angle error → rate target ─────────────────────────────
    const float roll_rate_tgt  = _roll_att.update(input[1], euler[0]);
    const float pitch_rate_tgt = _pitch_att.update(input[2], euler[1]);

    const float p = state_full[StateIdx::P];
    const float q = state_full[StateIdx::Q];
    const float r = state_full[StateIdx::R];

    // ── Rate loop: rate error → commanded angular acceleration (rad/s²) ────
    const float accel_cmd_roll  = _roll_rate.update(roll_rate_tgt,  p);
    const float accel_cmd_pitch = _pitch_rate.update(pitch_rate_tgt, q);

    _accel_cmd[0] = accel_cmd_roll;
    _accel_cmd[1] = accel_cmd_pitch;

    const float p_dot_meas = state_full[StateIdx::P_DOT];
    const float q_dot_meas = state_full[StateIdx::Q_DOT];

    // ── Roll: PD-style INDI step — accel term ("P") + jerk term ("D") ───────
    // Uses G1_hat/G2_hat from the *previous* tick's NLMS update, same
    // convention as pitch below — refreshed for next tick further down.
    // The "P" term is exactly AttitudeINDI's own roll correction (accel
    // error * kappa_roll * G1_hat); "D" is the additional jerk-error
    // correction (jerk error * kappa2_roll * G2_hat) layered on top, same
    // relationship a PID's D term has to its P term one derivative up.
    // roll_jerk (_roll_jerk_input) is used raw here — no digital filtering
    // anywhere in this class touches it, see class comment.
    _jerk_cmd_roll = _roll_accel.update(accel_cmd_roll, p_dot_meas);
    _jerk_cmd_roll = lowpass2p(_jerk_cmd_roll, _jerk_cmd_filt_state, JERK_CMD_FILT_HZ, NLMS_DT_S);
    // TEMP: see JERK_CMD_NOTCH_CENTER_HZ in the header — delete this one
    // line (plus its two members/two constants) once the 7.2 Hz parasitic
    // oscillation is fixed structurally.
    _jerk_cmd_roll = notch_apply(_jerk_cmd_roll, _jerk_cmd_notch_state, _jerk_cmd_notch_coeffs);
    const float delta_torque_roll_p = (accel_cmd_roll  - p_dot_meas)       * _kappa_roll  * _g1_roll;
    const float delta_torque_roll_d = (_jerk_cmd_roll   - _roll_jerk_input) * _kappa2_roll * _g2_roll;
    const float delta_torque_roll   = delta_torque_roll_p + delta_torque_roll_d;

    // ── Pitch: unmodified accel-level INDI step (identical to AttitudeINDI) ─
    const float delta_torque_pitch = (accel_cmd_pitch - q_dot_meas) * _kappa_pitch * _g1_pitch;

    _delta_torque[0] = delta_torque_roll;
    _delta_torque[1] = delta_torque_pitch;

    out_cmds[0] = unmixer.normalize_torque(current_torque[0] + delta_torque_roll);
    out_cmds[1] = unmixer.normalize_torque(current_torque[1] + delta_torque_pitch);

    // ── Live G(x) adaptation: NLMS update of G1 (both axes) and G2 (roll),
    // decimated — see class comment and AttitudeINDI's identical mechanism.
    ++_nlms_tick_count;
    const bool do_nlms_step = (_nlms_tick_count >= NLMS_DECIMATION);
    if (do_nlms_step) {
        _nlms_tick_count = 0;
    }

    _nlms_update_axis(current_torque[0], p_dot_meas, _g1_roll, _g1_seed_roll,
                       _tau_filt_state[0], _tau_extra_filt[0],
                       _prev_tau_f[0], _prev_omegadot_f[0], do_nlms_step);
    _nlms_update_axis(current_torque[1], q_dot_meas, _g1_pitch, _g1_seed_pitch,
                       _tau_filt_state[1], _tau_extra_filt[1],
                       _prev_tau_f[1], _prev_omegadot_f[1], do_nlms_step);
    // G2: same current_torque[0] driving signal as G1_roll above, paired
    // against roll_jerk instead of p_dot_meas — independent filter/prev
    // state (index 2) so it doesn't share a Delta window with G1_roll's.
    _nlms_update_axis(current_torque[0], _roll_jerk_input, _g2_roll, _g2_seed_roll,
                       _tau_filt_state[2], _tau_extra_filt[2],
                       _prev_tau_f[2], _prev_omegadot_f[2], do_nlms_step);
    if (do_nlms_step) {
        _nlms_initialized = true;
    }

    // ── Yaw: rate PID + heading-lock trim ──────────────────────────────────
    const float yaw_now = euler[2];
    if (fabsf(input[3]) > YAW_STICK_DEADBAND || !_yaw_target_valid) {
        _yaw_target       = yaw_now;
        _yaw_target_valid = true;
    }
    const float yaw_err       = wrap_pi(yaw_now - _yaw_target);
    const float yaw_hold_rate = constrain_float(_yaw_hold.update(0.0f, yaw_err),
                                                 -YAW_HOLD_MAX_RATE, YAW_HOLD_MAX_RATE);

    out_cmds[2] = _yaw_rate.update(_yaw_gain * input[3] + yaw_hold_rate, r);
}

void AttitudeINDIJerk::reset_all()
{
    _roll_att.reset();   _pitch_att.reset();
    _roll_rate.reset();  _pitch_rate.reset();
    _roll_accel.reset();
    _yaw_rate.reset();   _yaw_hold.reset();
    _yaw_target_valid = false;
    _jerk_cmd_filt_state = Biquad2pState();
    _jerk_cmd_notch_state = Biquad2pState();  // TEMP, see JERK_CMD_NOTCH_CENTER_HZ

    // Reset only the NLMS differencing/filter memory — G1_hat/G2_hat
    // themselves are deliberately left alone, same rationale as AttitudeINDI
    // (learned estimate persists across arm cycles within a boot, bounded at
    // all times by the drift clamp against its seed).
    for (int i = 0; i < 3; ++i) {
        _tau_filt_state[i]  = Biquad2pState();
        _tau_extra_filt[i]  = 0.0f;
        _prev_tau_f[i]      = 0.0f;
        _prev_omegadot_f[i] = 0.0f;
    }
    _nlms_initialized = false;
    _nlms_tick_count  = 0;
}

void AttitudeINDIJerk::_nlms_update_axis(float tau_now, float omegadot_now, float &g1, float seed,
                                         Biquad2pState &filt_state, float &extra_filt,
                                         float &prev_tau_f, float &prev_omegadot_f, bool do_step)
{
    const float stage2p = lowpass2p(tau_now, filt_state, STATEMGR_LP_PQRDOT_HZ, NLMS_DT_S);
    const float alpha_extra = lowpass_alpha(STATEMGR_LP_PQRDOT_EXTRA_HZ, NLMS_DT_S);
    extra_filt = lowpass(stage2p, extra_filt, alpha_extra);
    const float tau_f = extra_filt;

    if (!do_step) {
        return;
    }

    if (_nlms_initialized) {
        const float delta_tau_f      = tau_f - prev_tau_f;
        const float delta_omegadot_f = omegadot_now - prev_omegadot_f;

        if (fabsf(delta_tau_f) >= NLMS_EXCITATION_MIN_NM) {
            const float mu = _indi_active ? _mu_indi : _mu_pid;
            const float e  = delta_omegadot_f - g1 * delta_tau_f;
            float step = mu * e * delta_tau_f / (delta_tau_f * delta_tau_f + NLMS_EPS);

            const float max_step = NLMS_MAX_STEP_FRAC * fabsf(seed);
            step = constrain_float(step, -max_step, max_step);

            const float lo = seed - NLMS_MAX_DRIFT_FRAC * fabsf(seed);
            const float hi = seed + NLMS_MAX_DRIFT_FRAC * fabsf(seed);
            g1 = constrain_float(g1 + step, lo, hi);
        }
    }

    prev_tau_f      = tau_f;
    prev_omegadot_f = omegadot_now;
}
