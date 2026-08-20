#pragma once
#include "PID.hpp"
#include "Unmixer.hpp"
#include "AttitudeController.hpp"
#include "configs/DroneConfig.hpp"
#include "src/math/math.hpp"
#include "src/state_estimator/StateManager.hpp"   // STATEMGR_LP_PQRDOT_HZ / _EXTRA_HZ — shared with the tau_f matched filter below

/*
 * INDIJ — Incremental Nonlinear Dynamic Inversion, one derivative level
 * deeper on roll than AttitudeINDI: roll_jerk (the strain-fit jerk estimate,
 * see src/sensors/JerkFit.hpp) is the INDI inner loop for roll, in place of
 * a second angular-acceleration estimate. Pitch and yaw are otherwise
 * identical to AttitudeINDI (their own independent accel-level INDI /
 * rate-PID+heading-hold — this class doesn't share state with an
 * AttitudeINDI instance, it duplicates that structure so this file stays
 * self-contained and AttitudeINDI itself stays untouched).
 *
 * Control loop, roll (pitch is the unmodified two-stage AttitudeINDI cascade
 * — see that class's comment):
 *
 *   Outer loop:  angle_error [rad]     → rate_tgt [rad/s]        (P)
 *   Rate loop:   rate_error [rad/s]    → accel_cmd [rad/s²]      (PID)
 *   Accel loop:  accel_error [rad/s²]  → jerk_cmd [rad/s³]       (PID)  ── new
 *   jerk_cmd is then run through a 10 Hz 2nd-order Butterworth (JERK_CMD_FILT_HZ)
 *   before use — both logged and fed to the INDI step below, so what you see
 *   in jerk_cmd_roll() is exactly what the D-term used, not a pre-filter value.
 *   roll_jerk (the measurement, from JerkFit.hpp's estimate_jerk()) is used
 *   raw/unfiltered — deliberately: JerkFit.hpp's whole premise is a fast,
 *   low-latency jerk signal, and no digital filtering is applied to it or to
 *   anything derived from it anywhere in this class.
 *   INDI step (PD-style — "D" one derivative level up from the usual INDI "P"):
 *     delta_torque = (accel_cmd - p_dot_meas) * kappa_roll  * G1_hat    ("P" — identical in form to AttitudeINDI's own roll term)
 *                  + (jerk_cmd  - roll_jerk)  * kappa2_roll * G2_hat    ("D" — additional jerk-error correction; jerk_cmd filtered, roll_jerk raw)
 *     total_torque = current_torque_Nm + delta_torque
 *     out_cmds[0]  = unmixer.normalize_torque(total_torque)
 *
 * roll_jerk isn't part of the shared AttitudeController::update() signature
 * — set_roll_jerk() is called once per tick by FlightStateMachine::
 * run_attitude() (same pattern as AttitudeINDI::set_indi_active()) before
 * the generic update() dispatch, so update() itself just reads the most
 * recently set value.
 *
 * G1_hat (roll's accel-level effectiveness — the "P" term above) and G2_hat
 * (jerk-level effectiveness — the "D" term) are both live NLMS estimates,
 * adapted by the exact same _nlms_update_axis() machinery AttitudeINDI
 * uses — G2's regressor just pairs the same filtered current_torque signal
 * against roll_jerk instead of against measured angular acceleration,
 * rather than differentiating torque a second time (which would
 * reintroduce the double-differentiation noise problem strain-based jerk
 * sensing exists to avoid). See src/controllers/README.md's AttitudeINDI
 * "Live G(x) adaptation" section — everything there applies here one level
 * deeper, for both G1 and G2.
 *
 * Yaw: identical rate-PID + heading-lock trim to AttitudeINDI/AttitudePID.
 *
 * Conventions:
 *   euler[3]          roll, pitch, yaw (rad)
 *   state_full[19]    full EKF state (StateIdx::P_DOT, Q_DOT used here)
 *   input[]           [thrust, roll_tgt, pitch_tgt, yaw_rate, flight_mode]
 *   current_torque[2] [roll_Nm, pitch_Nm] from Unmixer
 *   out_cmds[3]       normalised torque [roll, pitch, yaw] in [-1, 1]
 */
class AttitudeINDIJerk : public AttitudeController {
public:
    explicit AttitudeINDIJerk(const AttitudeIndiJerkGains &g);

    void update(const float euler[3], const float state_full[],
                const float input[], const float current_torque[2],
                const Unmixer &unmixer, float out_cmds[3]) override;
    void reset_all() override;

    // Set once per tick by FlightStateMachine before the controller-list
    // dispatch — see class comment. Stored and used as-is, no filtering —
    // see class comment on why roll_jerk stays raw throughout this class.
    void set_roll_jerk(float roll_jerk) { _roll_jerk_input = roll_jerk; }

    // [delta_roll_Nm, delta_pitch_Nm]: roll's combined P+D INDI correction
    // (the one actually used) and pitch's accel-level INDI correction.
    // [accel_cmd_roll, accel_cmd_pitch] rad/s²: roll's rate-PID output (also
    // the target fed into the jerk stage) and pitch's, same shape as
    // AttitudeINDI::get_diag().
    void get_diag(float delta_torque[2], float accel_cmd[2]) const
    {
        delta_torque[0] = _delta_torque[0];
        delta_torque[1] = _delta_torque[1];
        accel_cmd[0]    = _accel_cmd[0];
        accel_cmd[1]    = _accel_cmd[1];
    }

    // [G1_hat_roll, G1_hat_pitch] — live NLMS accel-level effectiveness
    // estimate, same meaning/units as AttitudeINDI::get_g1().
    void get_g1(float g1[2]) const
    {
        g1[0] = _g1_roll;
        g1[1] = _g1_pitch;
    }

    // G2_hat_roll — live NLMS jerk-level effectiveness estimate (roll only).
    float get_g2() const { return _g2_roll; }

    // jerk_cmd_roll from the most recent update() — the target fed to the
    // INDI-jerk step, for logging/comparison against roll_jerk (JKFT.Pdd).
    // This is the *filtered* value (JERK_CMD_FILT_HZ, see update()) — what's
    // logged and what the D-term actually uses are the same number.
    float jerk_cmd_roll() const { return _jerk_cmd_roll; }

    // "D" term output gain, roll — static per-drone config value (DroneConfig
    // ::AttitudeIndiJerkGains::jerk_output_gain_roll), not live-adapted like
    // g2_roll. For logging: since it never changes at runtime, this repeats
    // the same value every tick — included so a flight's log is
    // self-documenting without cross-referencing the firmware build.
    float kappa2_roll() const { return _kappa2_roll; }

    // Tells the live NLMS estimators whether this controller is the one
    // currently driving out_cmds (vs. running in shadow), so they can pick
    // the mode-dependent adaptation rate mu — same as AttitudeINDI::set_indi_active().
    void set_indi_active(bool active) { _indi_active = active; }

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;
    PID _pitch_rate;
    PID _roll_accel;  // roll-only: accel error [rad/s²] -> jerk target [rad/s³]
    PID _yaw_rate;
    PID _yaw_hold;    // heading-lock trim: heading error [rad] -> corrective rate [rad/s]

    // G1_hat: N·m per rad/s² — accel-level, both axes. Roll's is the "P"
    // term (see class comment); pitch's drives out_cmds[1] alone, same as
    // AttitudeINDI.
    float _g1_roll;
    float _g1_pitch;
    float _g1_seed_roll;
    float _g1_seed_pitch;
    // G2_hat: jerk-level, roll only — the "D" term added on top of G1's
    // contribution for roll (see class comment).
    float _g2_roll;
    float _g2_seed_roll;

    float _kappa_roll;    // accel-level ("P") output gain, roll
    float _kappa_pitch;   // accel-level output gain (pitch)
    float _kappa2_roll;   // jerk-level ("D") output gain, roll

    float _mu_pid;    // NLMS adaptation rate while this controller is in shadow — shared by G1 and G2
    float _mu_indi;   // NLMS adaptation rate while this controller is active  — shared by G1 and G2
    float _yaw_gain;

    float _yaw_target;
    bool  _yaw_target_valid;

    float _roll_jerk_input = 0.0f;  // most recent set_roll_jerk() value, used raw/unfiltered throughout
    float _jerk_cmd_roll   = 0.0f;  // most recent update()'s jerk target, post-JERK_CMD_FILT_HZ filter (and TEMP notch, see below) — see jerk_cmd_roll()
    Biquad2pState _jerk_cmd_filt_state;  // 2nd-order Butterworth state for _jerk_cmd_roll, see update()

    // ── TEMP: 6.2-8.2 Hz notch on jerk_cmd_roll ──────────────────────────────
    // Kills a parasitic ~7.2 Hz oscillation found in flight testing
    // (2026-08-12) that the P/Butterworth filters above don't touch (both
    // their cutoffs sit above 7.2 Hz). Stopgap pending a structural fix —
    // delete this whole block (both constants, both members, and the single
    // notch_apply() call in update()) once that lands.
    static constexpr float JERK_CMD_NOTCH_CENTER_HZ = 7.2f;
    static constexpr float JERK_CMD_NOTCH_BW_HZ     = 2.0f;
    NotchCoeffs   _jerk_cmd_notch_coeffs;  // precomputed once at construction — center/bandwidth are fixed, not RPM-tracked like StateManager's notch
    Biquad2pState _jerk_cmd_notch_state;

    float _delta_torque[2] = {};  // see get_diag()
    float _accel_cmd[2]    = {};

    // ── Live NLMS G(x) estimator state — mirrors AttitudeINDI exactly, plus
    // an independent third instance (index 2) for G2's roll-only estimator.
    bool _indi_active = false;

    Biquad2pState _tau_filt_state[3];   // [0]=roll G1, [1]=pitch G1, [2]=roll G2
    float         _tau_extra_filt[3] = {};

    float _prev_tau_f[3]        = {};
    float _prev_omegadot_f[3]   = {};  // [2] holds prev roll_jerk, not prev omegadot — see _nlms_update_axis
    bool  _nlms_initialized     = false;

    int _nlms_tick_count = 0;

    static constexpr float NLMS_DT_S = 0.0025f;   // 400 Hz ControlThread
    static constexpr int   NLMS_DECIMATION = 8;   // see AttitudeINDI's identical constant

    static constexpr float JERK_CMD_FILT_HZ = 5.0f;  // 2nd-order Butterworth cutoff applied to jerk_cmd_roll in update()

    static constexpr float NLMS_EPS               = 1.0e-6f;
    static constexpr float NLMS_EXCITATION_MIN_NM  = 0.02f;
    static constexpr float NLMS_MAX_STEP_FRAC      = 0.10f;
    static constexpr float NLMS_MAX_DRIFT_FRAC     = 0.50f;

    static constexpr float YAW_STICK_DEADBAND = 0.10f;
    static constexpr float YAW_HOLD_MAX_RATE  = 0.3f;

    // One NLMS update step for a single estimator — identical algorithm to
    // AttitudeINDI::_nlms_update_axis(), just also called a third time for
    // G2 with roll_jerk in place of omegadot_now (see class comment on why
    // that's the regressor pairing used instead of a second-differenced
    // torque signal).
    void _nlms_update_axis(float tau_now, float omegadot_now, float &g1, float seed,
                            Biquad2pState &filt_state, float &extra_filt,
                            float &prev_tau_f, float &prev_omegadot_f, bool do_step);
};
