#include "Attitude_INDI.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include <cmath>

static PID make_pid(const PidGains &g)
{
    return PID(g.kp, g.ki, g.kd, g.imax, g.filt_target_hz, g.filt_error_hz, g.filt_d_hz);
}

AttitudeINDI::AttitudeINDI(const AttitudeIndiGains &g)
    : _roll_att  (make_pid(g.roll_att))
    , _pitch_att (make_pid(g.pitch_att))
    , _roll_rate (make_pid(g.roll_rate))
    , _pitch_rate(make_pid(g.pitch_rate))
    , _yaw_rate  (make_pid(g.yaw_rate))
    , _yaw_hold  (make_pid(g.yaw_hold))
    , _g1_roll(g.g1_seed_roll)
    , _g1_pitch(g.g1_seed_pitch)
    , _g1_seed_roll(g.g1_seed_roll)
    , _g1_seed_pitch(g.g1_seed_pitch)
    , _kappa_roll(g.indi_output_gain_roll)
    , _kappa_pitch(g.indi_output_gain_pitch)
    , _mu_pid(g.nlms_mu_pid)
    , _mu_indi(g.nlms_mu_indi)
    , _yaw_gain(g.yaw_gain)
    , _yaw_target(0.0f)
    , _yaw_target_valid(false)
{}

void AttitudeINDI::update(const float euler[3], const float state_full[],
                          const float input[], const float current_torque[2],
                          const Unmixer &unmixer, float out_cmds[3])
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

    _accel_cmd[0] = accel_cmd_roll;
    _accel_cmd[1] = accel_cmd_pitch;

    // ── INDI step: incremental torque from acceleration error ──────────────
    // Uses G1_hat from the *previous* tick's NLMS update — no separate
    // initialization pass; the live estimate is handed straight to the
    // control law. G1_hat is refreshed for next tick further below.
    const float p_dot_meas = state_full[StateIdx::P_DOT];
    const float q_dot_meas = state_full[StateIdx::Q_DOT];

    const float delta_torque_roll  = (accel_cmd_roll  - p_dot_meas) * _kappa_roll  * _g1_roll;
    const float delta_torque_pitch = (accel_cmd_pitch - q_dot_meas) * _kappa_pitch * _g1_pitch;

    _delta_torque[0] = delta_torque_roll;
    _delta_torque[1] = delta_torque_pitch;

    out_cmds[0] = unmixer.normalize_torque(current_torque[0] + delta_torque_roll);
    out_cmds[1] = unmixer.normalize_torque(current_torque[1] + delta_torque_pitch);

    // ── Live G(x) adaptation: NLMS update of G1_hat, decimated ─────────────
    // Called every tick INDI is enabled in the drone's config (per
    // FlightStateMachine's shadow-mode dispatch) regardless of _indi_active
    // — only mu (via _nlms_update_axis) depends on which controller is
    // actually driving out_cmds. The tau_f/omegadot_f filters inside
    // _nlms_update_axis still get fed every tick; only the NLMS regressor
    // Delta and step itself are formed once every NLMS_DECIMATION ticks —
    // see the comment on NLMS_DECIMATION in Attitude_INDI.hpp and
    // src/controllers/README.md's AttitudeINDI "Live G(x) adaptation" section.
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

    out_cmds[2] = _yaw_rate.update(_yaw_gain * input[3] + yaw_hold_rate, r);
}

void AttitudeINDI::reset_all()
{
    _roll_att.reset();  _pitch_att.reset();
    _roll_rate.reset(); _pitch_rate.reset();
    _yaw_rate.reset();  _yaw_hold.reset();
    _yaw_target_valid = false;

    // Reset only the NLMS differencing/filter memory, so a stale
    // Delta_tau_f/Delta_Omega_dot_f pair is never computed across a disarm
    // gap. G1_hat itself (_g1_roll/_g1_pitch) is deliberately left alone —
    // the learned estimate persists across arm cycles within a boot, bounded
    // at all times by the drift clamp against its seed.
    _tau_filt_state[0]  = Biquad2pState();
    _tau_filt_state[1]  = Biquad2pState();
    _tau_extra_filt[0]  = 0.0f;
    _tau_extra_filt[1]  = 0.0f;
    _prev_tau_f[0]      = 0.0f;
    _prev_tau_f[1]      = 0.0f;
    _prev_omegadot_f[0] = 0.0f;
    _prev_omegadot_f[1] = 0.0f;
    _nlms_initialized   = false;
    _nlms_tick_count    = 0;
}

// One NLMS update step for a single axis — see the class-level comment in
// Attitude_INDI.hpp and src/controllers/README.md's AttitudeINDI "Live G(x) adaptation" section.
// do_step selects whether this call forms the decimated Delta_tau_f/
// Delta_Omega_dot_f regressor and attempts a step (every NLMS_DECIMATION
// ticks) or just advances the tau_f filter state (every other tick).
void AttitudeINDI::_nlms_update_axis(float tau_now, float omegadot_now, float &g1, float seed,
                                      Biquad2pState &filt_state, float &extra_filt,
                                      float &prev_tau_f, float &prev_omegadot_f, bool do_step)
{
    // tau_f: current_torque through the same filter chain (type/order/
    // cutoff) StateManager applies to p_dot/q_dot — delay-matched regressor
    // input, independent of current_torque's own use,
    // unfiltered, as the increment-law baseline above. Runs every tick
    // regardless of do_step so it keeps its designed 400 Hz-sampled cutoff.
    const float stage2p = lowpass2p(tau_now, filt_state, STATEMGR_LP_PQRDOT_HZ, NLMS_DT_S);
    const float alpha_extra = lowpass_alpha(STATEMGR_LP_PQRDOT_EXTRA_HZ, NLMS_DT_S);
    extra_filt = lowpass(stage2p, extra_filt, alpha_extra);
    const float tau_f = extra_filt;

    if (!do_step) {
        return;   // decimated: only the regressor/step below are skipped, not the filter above
    }

    if (_nlms_initialized) {
        const float delta_tau_f      = tau_f - prev_tau_f;
        const float delta_omegadot_f = omegadot_now - prev_omegadot_f;

        // Excitation gating: near-hover/no-input periods
        // produce an uninformative regression that mostly fits noise. Now
        // evaluated over the decimated NLMS_DECIMATION-tick window rather
        // than a single 2.5 ms tick.
        if (fabsf(delta_tau_f) >= NLMS_EXCITATION_MIN_NM) {
            const float mu = _indi_active ? _mu_indi : _mu_pid;
            const float e  = delta_omegadot_f - g1 * delta_tau_f;
            float step = mu * e * delta_tau_f / (delta_tau_f * delta_tau_f + NLMS_EPS);

            // Safety clamps: bound the per-step change
            // and the total drift from the offline seed — a large jump
            // indicates a transient glitch that should saturate, not
            // propagate.
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
