#include "Attitude_JINDI.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include <cmath>

static PID make_pid(const PidGains &g)
{
    return PID(g.kp, g.ki, g.kd, g.imax, g.filt_target_hz, g.filt_error_hz, g.filt_d_hz);
}

AttitudeJINDI::AttitudeJINDI(const AttitudeJindiGains &g)
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
    , _kappa_pitch(g.indi_output_gain_pitch)
    , _h_roll(g.h_roll)
    , _alpha(g.alpha)
    , _mu_pid(g.nlms_mu_pid)
    , _mu_indi(g.nlms_mu_indi)
    , _yaw_gain(g.yaw_gain)
    , _yaw_target(0.0f)
    , _yaw_target_valid(false)
    , _jerk_err_notch_coeffs(notch_coeffs(JERK_ERR_NOTCH_CENTER_HZ, JERK_ERR_NOTCH_BW_HZ, NLMS_DT_S))
{}

void AttitudeJINDI::update(const float euler[3], const float state_full[],
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

    // ── Roll: two independent full-torque-reference paths, then blend ──────
    _jerk_cmd_roll = _roll_accel.update(accel_cmd_roll, p_dot_meas);
    // _jerk_cmd_roll = lowpass2p(_jerk_cmd_roll, _jerk_cmd_filt_state, JERK_CMD_FILT_HZ, NLMS_DT_S);

    // Accel path — instant-actuation assumption, anchored to the plant's
    // actual/measured torque. Identical in form to AttitudeINDI's own roll
    // term; no kappa (see class comment).
    _tau_accel_roll = current_torque[0] + (accel_cmd_roll - p_dot_meas) * _g1_roll;

    // Jerk path — actuator-dynamics assumption, anchored to TC (the actual
    // combined torque commanded last tick, see class comment): the actuator
    // only ever sees one signal, the blend, so that's what it's physically
    // lagging behind — not an isolated per-path value. (G*H)^-1 ==
    // g1_roll/h_roll, the same G1 the accel path uses.
    //
    // The error itself is notched at 9-11 Hz before use (see
    // JERK_ERR_NOTCH_CENTER_HZ) — a confirmed ~10.2 Hz structural mode that
    // amplified badly enough at alpha=0.7 to strain the motors and burst a
    // battery. roll_jerk_input/jerk_cmd_roll stay unfiltered/as-filtered-
    // above for logging; only this error, which actually drives torque,
    // gets the notch.
    // Measured jerk lowpassed at JERK_MEAS_LPF_HZ (own filter state — see header).
    _jerk_meas_lpf = lowpass(_roll_jerk_input, _jerk_meas_lpf, lowpass_alpha(JERK_MEAS_LPF_HZ, NLMS_DT_S));
    const float roll_jerk_filt = _jerk_meas_lpf;

    const float jerk_err_raw = _jerk_cmd_roll - roll_jerk_filt;
    // const float jerk_err = notch_apply(jerk_err_raw, _jerk_err_notch_state, _jerk_err_notch_coeffs);
    // Single-pole lowpass on the error (JERK_ERR_LPF_HZ, see header).
    _jerk_err_lpf = lowpass(jerk_err_raw, _jerk_err_lpf, lowpass_alpha(JERK_ERR_LPF_HZ, NLMS_DT_S));
    const float jerk_err = _jerk_err_lpf;
    _tau_jerk_roll = _tau_cmd_jerk_prev_roll + (_g1_roll / _h_roll) * jerk_err * 0.13;
    // Clamp to the same physical torque limit normalize_torque() divides by
    // — a persistent jerk_cmd/roll_jerk bias would otherwise integrate
    // without bound, and this keeps the jerk path's contribution to the
    // blend below from ever exceeding what the airframe could actually
    // produce.
    const float max_torque_nm = unmixer.max_torque_nm();
    _tau_jerk_roll = constrain_float(_tau_jerk_roll, -max_torque_nm, max_torque_nm);

    _tau_roll = _alpha * _tau_accel_roll + (1.0f - _alpha) * _tau_jerk_roll;

    // TC: anchor next tick's jerk-path increment to the combined command
    // just computed (clamped to the same physical limit, since tau_accel_roll
    // itself isn't bounded pre-normalize_torque()) — see class comment.
    _tau_cmd_jerk_prev_roll = constrain_float(_tau_roll, -max_torque_nm, max_torque_nm);

    // ── Pitch: unmodified accel-level INDI step (identical to AttitudeINDI) ─
    const float delta_torque_pitch = (accel_cmd_pitch - q_dot_meas) * _kappa_pitch * _g1_pitch;

    out_cmds[0] = unmixer.normalize_torque(_tau_roll);
    out_cmds[1] = unmixer.normalize_torque(current_torque[1] + delta_torque_pitch);

    // ── Live G1 adaptation: roll + pitch only, decimated — identical
    // mechanism to AttitudeINDI's (no separate G2 — see class comment).
    // mu is configured at 0 for now (AttitudeJindiGains::nlms_mu_pid/indi).
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

void AttitudeJINDI::reset_all()
{
    _roll_att.reset();   _pitch_att.reset();
    _roll_rate.reset();  _pitch_rate.reset();
    _roll_accel.reset();
    _yaw_rate.reset();   _yaw_hold.reset();
    _yaw_target_valid = false;
    _jerk_cmd_filt_state    = Biquad2pState();
    _jerk_err_notch_state   = Biquad2pState();
    _jerk_err_lpf           = 0.0f;
    _jerk_meas_lpf          = 0.0f;
    _tau_cmd_jerk_prev_roll = 0.0f;

    // Reset only the NLMS differencing/filter memory — G1_hat itself is
    // deliberately left alone, same rationale as AttitudeINDI/AttitudeINDIJerk
    // (learned estimate persists across arm cycles within a boot, bounded at
    // all times by the drift clamp against its seed).
    for (int i = 0; i < 2; ++i) {
        _tau_filt_state[i]  = Biquad2pState();
        _tau_extra_filt[i]  = 0.0f;
        _prev_tau_f[i]      = 0.0f;
        _prev_omegadot_f[i] = 0.0f;
    }
    _nlms_initialized = false;
    _nlms_tick_count  = 0;
}

void AttitudeJINDI::_nlms_update_axis(float tau_now, float omegadot_now, float &g1, float seed,
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
