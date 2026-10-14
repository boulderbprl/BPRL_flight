#pragma once
#include "PID.hpp"
#include "Unmixer.hpp"
#include "AttitudeController.hpp"
#include "configs/DroneConfig.hpp"
#include "src/math/math.hpp"
#include "src/state_estimator/StateManager.hpp"   // STATEMGR_LP_PQRDOT_HZ / _EXTRA_HZ — shared with the tau_f matched filter below

/*
 * JINDI — like AttitudeINDIJerk, roll_jerk (the strain-fit jerk estimate,
 * see src/sensors/JerkFit.hpp) gives roll a second control path beyond the
 * usual accel-level INDI. The formulation is different, though:
 * AttitudeINDIJerk treats the jerk term as a "D" correction layered onto the
 * accel-level ("P") increment (delta = P*kappa*G1 + D*kappa2*G2, both
 * anchored to the same current_torque measurement). JINDI instead assumes
 * the jerk path has its own actuator dynamics — torque isn't applied
 * instantly, unlike the accel path's instant-actuation assumption — so each
 * path computes its own full torque reference independently, and the two
 * are blended with a convex combination instead of summed increments.
 *
 * Roll, two independent paths, each producing a full torque reference
 * (N·m), not an increment:
 *
 *   Accel path (instant-actuation assumption, identical in form to
 *   AttitudeINDI's own roll term — anchored to the plant's actual/measured
 *   torque from Unmixer, since instant actuation means "what it's doing
 *   now" IS "what we last commanded"):
 *     tau_accel = current_torque_Nm[0] + (accel_cmd - p_dot_meas) * G1_hat
 *
 *   Jerk path (actuator-dynamics assumption — the increment is anchored to
 *   TC, the actual combined torque commanded last tick, not the plant's
 *   actual/measured torque: the actuator only ever sees one signal (the
 *   alpha-blend below), so that's what it's physically lagging behind, not
 *   an isolated per-path value):
 *     tau_jerk[k+1] = TC[k] + (G*H)^-1 * (jerk_cmd - roll_jerk)
 *   G here is the SAME live-adapted G1_hat the accel path uses — one
 *   physical system, one effectiveness estimate, not a second independent
 *   "G2" — and H is a fixed per-drone actuator-dynamics constant
 *   (DroneConfig::AttitudeJindiGains::h_roll), so in code:
 *     (G*H)^-1 == G1_hat_roll / h_roll
 *   (G1_hat is already stored as the ready-to-use N·m-per-rad/s² multiplier,
 *   i.e. it's already "G^-1" in the continuous-math sense above — dividing
 *   by H folds in the actuator-dynamics term directly.)
 *
 *   Combine (convex combination — no kappa/kappa2 output gains at all; see
 *   DroneConfig::AttitudeJindiGains::alpha):
 *     tau_roll = alpha * tau_accel + (1 - alpha) * tau_jerk
 *     out_cmds[0] = unmixer.normalize_torque(tau_roll)
 *   TC in the jerk path's formula above is exactly this tau_roll, clamped to
 *   ±unmixer.max_torque_nm() and carried forward as next tick's TC[k] (see
 *   combined_torque_roll()/_tau_cmd_jerk_prev_roll) — i.e. the jerk path's
 *   own increment feeds back through the SAME blended value it just helped
 *   produce, not a value it tracks independently of the accel path.
 *
 * G1_hat (both roll and pitch) is the exact same live NLMS estimator
 * AttitudeINDI uses, identified against the real measured current_torque —
 * JINDI has no separate "G2" estimator at all, unlike AttitudeINDIJerk,
 * since both of roll's paths share one G. Adaptation rate (mu) is
 * configured at 0 for now (not yet characterized) — the machinery stays
 * wired, same convention as every other not-yet-tuned gain in this
 * codebase, so it can be turned on later without a code change.
 *
 * Pitch and yaw: unchanged from AttitudeINDIJerk/AttitudeINDI — plain
 * accel-level INDI for pitch (its own kappa, DroneConfig::AttitudeJindiGains
 * ::indi_output_gain_pitch — there's no jerk sensor on pitch, so nothing to
 * convex-combine there), rate-PID + heading-lock trim for yaw.
 *
 * roll_jerk isn't part of the shared AttitudeController::update() signature
 * — set_roll_jerk() is called once per tick by FlightStateMachine::
 * run_attitude() before the generic update() dispatch, same pattern as
 * AttitudeINDIJerk::set_roll_jerk().
 *
 * Conventions:
 *   euler[3]          roll, pitch, yaw (rad)
 *   state_full[19]    full EKF state (StateIdx::P_DOT, Q_DOT used here)
 *   input[]           [thrust, roll_tgt, pitch_tgt, yaw_rate, flight_mode]
 *   current_torque[2] [roll_Nm, pitch_Nm] from Unmixer
 *   out_cmds[3]       normalised torque [roll, pitch, yaw] in [-1, 1]
 */
class AttitudeJINDI : public AttitudeController {
public:
    explicit AttitudeJINDI(const AttitudeJindiGains &g);

    void update(const float euler[3], const float state_full[],
                const float input[], const float current_torque[2],
                const Unmixer &unmixer, float out_cmds[3]) override;
    void reset_all() override;

    // Set once per tick by FlightStateMachine before the controller-list
    // dispatch — see class comment.
    void set_roll_jerk(float roll_jerk) { _roll_jerk_input = roll_jerk; }

    // [tau_accel_roll, tau_jerk_roll]: roll's two independent path torque
    // references (N·m), pre-blend — see class comment. accel_cmd[2] =
    // [accel_cmd_roll, accel_cmd_pitch], same shape/meaning as
    // AttitudeINDIJerk::get_diag()'s second output.
    void get_diag(float tau_paths_roll[2], float accel_cmd[2]) const
    {
        tau_paths_roll[0] = _tau_accel_roll;
        tau_paths_roll[1] = _tau_jerk_roll;
        accel_cmd[0]      = _accel_cmd[0];
        accel_cmd[1]      = _accel_cmd[1];
    }

    // [G1_hat_roll, G1_hat_pitch] — live NLMS accel-level effectiveness
    // estimate, shared by both of roll's paths (no separate G2) — same
    // meaning/units as AttitudeINDI::get_g1().
    void get_g1(float g1[2]) const
    {
        g1[0] = _g1_roll;
        g1[1] = _g1_pitch;
    }

    // Roll's alpha-blended combined torque reference (N·m), pre-normalise —
    // tau_roll = alpha*tau_accel_roll + (1-alpha)*tau_jerk_roll, the value
    // passed to unmixer.normalize_torque() to produce out_cmds[0]. Logged
    // alongside the two pre-blend paths (get_diag()) so the blend itself is
    // directly visible without recomputing it from alpha_roll() by hand.
    float combined_torque_roll() const { return _tau_roll; }

    // jerk_cmd_roll from the most recent update() — the target fed to the
    // jerk path's increment, for logging/comparison against the actual
    // measured roll_jerk. Same meaning as AttitudeINDIJerk::jerk_cmd_roll().
    float jerk_cmd_roll() const { return _jerk_cmd_roll; }

    // Convex-combination weight, roll — static per-drone config value
    // (DroneConfig::AttitudeJindiGains::alpha). For logging: since it never
    // changes at runtime, this repeats the same value every tick — same
    // rationale as AttitudeINDIJerk::kappa2_roll().
    float alpha_roll() const { return _alpha; }

    // Measured roll jerk after the JERK_MEAS_LPF_HZ lowpass — the value the
    // jerk error actually uses (most recent update()). Diagnostic only.
    float roll_jerk_filt() const { return _jerk_meas_lpf; }

    // Tells the live NLMS estimator whether this controller is the one
    // currently driving out_cmds (vs. running in shadow) — same mechanism
    // as AttitudeINDI::set_indi_active().
    void set_indi_active(bool active) { _indi_active = active; }

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;
    PID _pitch_rate;
    PID _roll_accel;  // roll-only: accel error [rad/s²] -> jerk target [rad/s³]
    PID _yaw_rate;
    PID _yaw_hold;    // heading-lock trim: heading error [rad] -> corrective rate [rad/s]

    // G1_hat: N·m per rad/s² — shared by both of roll's paths, independent
    // for pitch. See class comment on why there's no "G2" here.
    float _g1_roll;
    float _g1_pitch;
    float _g1_seed_roll;
    float _g1_seed_pitch;

    float _kappa_pitch;  // pitch-only output gain — roll has none, see class comment
    float _h_roll;       // H: roll's jerk-path actuator-dynamics constant
    float _alpha;        // convex-combination weight, roll

    float _mu_pid;    // NLMS adaptation rate while this controller is in shadow
    float _mu_indi;   // NLMS adaptation rate while this controller is active
    float _yaw_gain;

    float _yaw_target;
    bool  _yaw_target_valid;

    float _roll_jerk_input = 0.0f;  // most recent set_roll_jerk() value, used raw/unfiltered everywhere EXCEPT the notched error below
    float _jerk_cmd_roll   = 0.0f;  // most recent update()'s jerk target, post-JERK_CMD_FILT_HZ filter — see jerk_cmd_roll()
    Biquad2pState _jerk_cmd_filt_state;  // 2nd-order Butterworth state for _jerk_cmd_roll, see update()

    // ── 8-15 Hz notch on the jerk path's error term ──────────────────────────
    // A confirmed-real ~10.2 Hz structural mode shows up in both roll_jerk
    // (strain sensor) and the IMU (which feeds jerk_cmd_roll through the
    // rate/accel cascade) — notching jerk_cmd_roll alone wouldn't suppress
    // it, since roll_jerk carries it too. Applied to the error
    // (jerk_cmd_roll - roll_jerk_input) directly rather than to either side
    // separately: notch filtering is linear, so notch(A)-notch(B) ==
    // notch(A-B), and this way roll_jerk_input/jerk_cmd_roll both stay
    // exactly as logged/used elsewhere (raw / JERK_CMD_FILT_HZ-only) — only
    // what actually drives the jerk path's increment is protected. Found
    // after alpha=0.7 amplified this mode enough to strain the motors and
    // burst a battery — this is load-bearing, not a nice-to-have.
    static constexpr float JERK_ERR_NOTCH_CENTER_HZ = 11.5f;
    static constexpr float JERK_ERR_NOTCH_BW_HZ     = 7.0f;   // 8-15 Hz
    NotchCoeffs   _jerk_err_notch_coeffs;
    Biquad2pState _jerk_err_notch_state;

    // ── Lowpass on the jerk path's error ─────────────────────────────────────
    // Single-pole (1st-order IIR via lowpass()/lowpass_alpha()). In LOG0046
    // the jerk path contributed ~3.5× the accel path's 40-50 Hz torque — the
    // same arm-mode loop that limit-cycled the old PIDJerk.
    static constexpr float JERK_ERR_LPF_HZ = 35.0f;
    float _jerk_err_lpf = 0.0f;   // single-pole filter state = previous output

    // Single-pole lowpass on the measured jerk before it enters the
    // jerk error. Its own state (not shared with any other filter), and
    // applied to a local copy so _roll_jerk_input stays the raw set_roll_jerk()
    // value.
    static constexpr float JERK_MEAS_LPF_HZ = 20.0f;
    float _jerk_meas_lpf = 0.0f;   // single-pole filter state = previous output

    // TC — the actual combined torque commanded last tick (_tau_roll,
    // clamped to ±unmixer.max_torque_nm()), carried forward as the jerk
    // path's increment base point for next tick (see class comment: the
    // actuator only ever sees the blend, so that's what it's physically
    // lagging behind — not current_torque[0], the accel path's
    // actual-torque base point, and not an isolated per-path value either).
    // Zeroed on reset_all(). Note: at alpha=1, tau_roll==tau_accel_roll
    // every tick, so this register tracks the accel path (plus one tick's
    // jerk increment) rather than accumulating independently — it no longer
    // reveals the jerk path's own unmasked trajectory in shadow the way an
    // isolated accumulator would.
    float _tau_cmd_jerk_prev_roll = 0.0f;

    float _tau_accel_roll = 0.0f;  // see get_diag()
    float _tau_jerk_roll  = 0.0f;
    float _tau_roll       = 0.0f;  // see combined_torque_roll()
    float _accel_cmd[2]   = {};

    bool _indi_active = false;

    // ── Live NLMS G1 estimator state — roll + pitch only, identical
    // mechanism/shape to AttitudeINDI's own (no third "G2" instance — see
    // class comment).
    Biquad2pState _tau_filt_state[2];
    float         _tau_extra_filt[2] = {};

    float _prev_tau_f[2]        = {};
    float _prev_omegadot_f[2]   = {};
    bool  _nlms_initialized     = false;

    int _nlms_tick_count = 0;

    static constexpr float NLMS_DT_S = 0.0025f;   // 400 Hz ControlThread
    static constexpr int   NLMS_DECIMATION = 8;   // see AttitudeINDI's identical constant

    static constexpr float JERK_CMD_FILT_HZ = 30.0f;  // 2nd-order Butterworth cutoff applied to jerk_cmd_roll in update()

    static constexpr float NLMS_EPS               = 1.0e-6f;
    static constexpr float NLMS_EXCITATION_MIN_NM  = 0.02f;
    static constexpr float NLMS_MAX_STEP_FRAC      = 0.10f;
    static constexpr float NLMS_MAX_DRIFT_FRAC     = 0.50f;

    static constexpr float YAW_STICK_DEADBAND = 0.10f;
    static constexpr float YAW_HOLD_MAX_RATE  = 0.3f;

    // One NLMS update step for a single estimator — identical algorithm to
    // AttitudeINDI::_nlms_update_axis()/AttitudeINDIJerk's.
    void _nlms_update_axis(float tau_now, float omegadot_now, float &g1, float seed,
                            Biquad2pState &filt_state, float &extra_filt,
                            float &prev_tau_f, float &prev_omegadot_f, bool do_step);
};
