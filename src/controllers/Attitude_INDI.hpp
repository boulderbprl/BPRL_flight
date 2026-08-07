#pragma once
#include "PID.hpp"
#include "Unmixer.hpp"
#include "AttitudeController.hpp"
#include "configs/DroneConfig.hpp"
#include "src/math/math.hpp"
#include "src/state_estimator/StateManager.hpp"   // STATEMGR_LP_PQRDOT_HZ / _EXTRA_HZ — shared with the tau_f matched filter below

/*
 * INDI (Incremental Nonlinear Dynamic Inversion) attitude controller.
 *
 * Roll and pitch use INDI; yaw falls back to standard rate PID.
 *
 * Control loop per axis (roll shown, pitch symmetric):
 *
 *   Outer loop:  angle_error [rad] → rate_tgt [rad/s]  (P only)
 *   Inner loop:  rate_error [rad/s] → accel_cmd [rad/s²]  (PID)
 *   INDI step:
 *     delta_torque = (accel_cmd - measured_accel) * kappa * G1_hat
 *     total_torque = current_torque_Nm + delta_torque
 *     out_cmds[i]  = unmixer.normalize_torque(total_torque)
 *
 * G1_hat (~1/Ixx, 1/Iyy) is a live NLMS estimate seeded from the offline-
 * identified config value and continuously adapted (see the NLMS block in
 * update() and indi_adaptive_G_controller_spec.md at the repo root). kappa
 * is a separate, static, per-axis output-authority gain — decoupled from the
 * physical-effectiveness estimate so tuning one never silently retunes the
 * other. update() is called every tick this controller is enabled in the
 * drone's config (per FlightStateMachine's shadow-mode dispatch), regardless
 * of whether INDI is the currently active controller — but the NLMS
 * regressor/step itself is decimated (NLMS_DECIMATION ticks) rather than
 * formed every tick, since the regressor is filtered well below the 400 Hz
 * tick rate (see the comment on NLMS_DECIMATION below). Only the adaptation
 * rate mu changes with mode (see set_indi_active()).
 *
 * Yaw has no angle loop of its own (unlike roll/pitch), so nothing
 * previously corrected a rate estimate that isn't exactly mean-zero
 * (residual gyro-bias error, control asymmetry) — heading could walk
 * indefinitely under a nominally "zero rate" command. _yaw_hold adds a
 * small heading-lock trim on top of the rate PID: while the stick is
 * centered, _yaw_target stays fixed and a P correction pulls current
 * heading back to it; while actively yawing, _yaw_target tracks the
 * current heading so there's no stored error to fight when the stick
 * recentres. Heading comes from euler[2], which is the state estimator's
 * IMX5-fused yaw — accurate enough to close this loop.
 *
 * Conventions:
 *   euler[3]          roll, pitch, yaw (rad)
 *   state_full[19]    full EKF state (StateIdx::P_DOT, Q_DOT used here)
 *   input[]           [thrust, roll_tgt, pitch_tgt, yaw_rate, flight_mode]
 *   current_torque[2] [roll_Nm, pitch_Nm] from Unmixer
 *   out_cmds[3]       normalised torque [roll, pitch, yaw] in [-1, 1]
 *
 * delta_torque/accel_cmd (shadow-logging diagnostics, [roll, pitch]) are no
 * longer separate update() out-params — that would break the shared
 * AttitudeController signature every controller in FlightStateMachine's list
 * now uses. update() stores them internally; get_diag() reads them back.
 */
class AttitudeINDI : public AttitudeController {
public:
    explicit AttitudeINDI(const AttitudeIndiGains &g);

    void update(const float euler[3], const float state_full[],
                const float input[], const float current_torque[2],
                const Unmixer &unmixer, float out_cmds[3]) override;
    void reset_all() override;

    // [delta_roll_Nm, delta_pitch_Nm] incremental INDI correction, and
    // [accel_cmd_roll, accel_cmd_pitch] rad/s² rate-PID output fed to the
    // INDI step — both from the most recent update() call, for logging.
    void get_diag(float delta_torque[2], float accel_cmd[2]) const
    {
        delta_torque[0] = _delta_torque[0];
        delta_torque[1] = _delta_torque[1];
        accel_cmd[0]    = _accel_cmd[0];
        accel_cmd[1]    = _accel_cmd[1];
    }

    // [G1_hat_roll, G1_hat_pitch] — the live NLMS-adapted effectiveness
    // estimate (N·m per rad/s²), for logging/comparison against the
    // offline-identified seed (DroneConfig::AttitudeIndiGains::g1_seed_roll/pitch).
    void get_g1(float g1[2]) const
    {
        g1[0] = _g1_roll;
        g1[1] = _g1_pitch;
    }

    // Tells the live NLMS estimator whether INDI is the controller currently
    // driving out_cmds (vs. running in shadow behind PID/PID+PI), so it can
    // pick the mode-dependent adaptation rate mu (spec section 5.2). Called
    // once per tick by FlightStateMachine::run_attitude(), independent of
    // the AttitudeController::update() interface (INDI-only, like get_diag()).
    void set_indi_active(bool active) { _indi_active = active; }

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;
    PID _pitch_rate;
    PID _yaw_rate;
    PID _yaw_hold;   // heading-lock trim: heading error [rad] -> corrective rate [rad/s]

    // G1_hat^-1 : N·m per rad/s² — live NLMS estimate of airframe effectiveness
    // (~1/Ixx, 1/Iyy), seeded from _g1_seed_roll/_g1_seed_pitch and adapted
    // every update() call (see the NLMS block in Attitude_INDI.cpp).
    float _g1_roll;
    float _g1_pitch;
    // Offline-identified seed values (from DroneConfig) — the NLMS drift
    // clamp (NLMS_MAX_DRIFT_FRAC) bounds _g1_roll/_g1_pitch relative to these,
    // never to zero or to each other.
    float _g1_seed_roll;
    float _g1_seed_pitch;
    // kappa: decoupled output-authority gain, static per-axis, tuned
    // independently of the physical G1_hat estimate. from DroneConfig; was
    // the implicit "gain of 1" folded into the old indi_gain_roll/pitch.
    float _kappa_roll;
    float _kappa_pitch;
    // NLMS adaptation rate, mode-selected each update() call by _indi_active.
    float _mu_pid;    // from DroneConfig::nlms_mu_pid  — INDI in shadow
    float _mu_indi;   // from DroneConfig::nlms_mu_indi — INDI active (trickle)
    float _yaw_gain;         // from DroneConfig; was YAW_GAIN

    float _yaw_target;        // held heading target [rad]
    bool  _yaw_target_valid;  // false until first update() captures a target

    float _delta_torque[2] = {};  // see get_diag()
    float _accel_cmd[2]    = {};

    // ── Live NLMS G(x) estimator state ──────────────────────────────────────
    bool _indi_active = false;   // set via set_indi_active(); selects _mu_pid vs _mu_indi

    // tau_f: current_torque passed through the same filter chain (type/
    // order/cutoff) StateManager applies to p_dot/q_dot, so the NLMS
    // regressor and Omega_dot_f are delay-matched (spec section 3.2). Reuses
    // STATEMGR_LP_PQRDOT_HZ/_EXTRA_HZ from StateManager.hpp rather than
    // redefining them, so the two filters can't silently drift apart.
    Biquad2pState _tau_filt_state[2];   // [roll, pitch] 2nd-order stage
    float         _tau_extra_filt[2] = {};  // [roll, pitch] optional 1st-order stage memory

    // Previous-step filtered values, for the incremental NLMS regressor
    // (Delta_tau_f, Delta_Omega_dot_f) — see spec section 5.1. Only refreshed
    // every NLMS_DECIMATION ticks (see below), so this is a Delta across the
    // decimated window, not a single 400 Hz tick.
    float _prev_tau_f[2]        = {};  // [roll, pitch]
    float _prev_omegadot_f[2]   = {};  // [roll, pitch]
    bool  _nlms_initialized     = false;  // false until the first post-reset decimated step seeds _prev_*

    // Counts ticks since the last decimated NLMS step; shared by both axes
    // since they're always evaluated together in the same update() call.
    int _nlms_tick_count = 0;

    // Fixed control-loop period — matches Unmixer::RPM_FILT_DT_S's existing
    // precedent of a fixed rather than measured dt for this filter chain.
    static constexpr float NLMS_DT_S = 0.0025f;   // 400 Hz ControlThread

    // The tau_f/omegadot_f low-pass filters below still run every tick (they
    // need a continuous 400 Hz feed to hold their designed cutoff), but the
    // NLMS Delta_tau_f/Delta_Omega_dot_f regressor is only formed and stepped
    // once every NLMS_DECIMATION ticks. A per-tick (2.5 ms) Delta of a signal
    // filtered at STATEMGR_LP_PQRDOT_HZ/_EXTRA_HZ (20/15 Hz) is dominated by
    // filter ripple/measurement noise rather than real excitation — flight
    // data showed doublet excitation visible at 50 Hz (20 ms) resolution but
    // not at 400 Hz (2.5 ms) resolution. 8 ticks = 20 ms, roughly matching
    // that filter bandwidth and the 50 Hz log rate — needs bench/flight
    // tuning like the constants below.
    static constexpr int NLMS_DECIMATION = 8;

    // NLMS safety margins (spec section 5.3/5.4) — adaptation-safety
    // constants, not physical per-drone identification data, so they start
    // as flight-tuning constants here rather than DroneConfig fields (same
    // convention as YAW_HOLD_MAX_RATE below); promote to DroneConfig later
    // only if the two airframes need different values.
    static constexpr float NLMS_EPS                 = 1.0e-6f;  // division-blowup guard near zero excitation
    static constexpr float NLMS_EXCITATION_MIN_NM    = 0.02f;   // |Delta_tau_f| (over the decimated window) below this: freeze the update — was tuned against per-tick deltas, needs re-checking against the new window
    static constexpr float NLMS_MAX_STEP_FRAC        = 0.10f;   // max |G1_hat step| per update, as a fraction of the seed — needs bench tuning
    static constexpr float NLMS_MAX_DRIFT_FRAC       = 0.50f;   // max total drift of G1_hat from its seed — needs bench tuning

    static constexpr float YAW_STICK_DEADBAND   = 0.10f;  // normalised stick [-1,1], matches FlightStateMachine::STICK_DEADBAND
    static constexpr float YAW_HOLD_MAX_RATE    = 0.3f;   // rad/s cap on the heading-hold trim — needs flight tuning

    // One NLMS update step for a single axis. tau_now/omegadot_now are this
    // tick's raw current_torque[axis] and measured p_dot/q_dot; g1 is the
    // live estimate to update in place; seed is its drift-clamp reference;
    // filt_state/extra_filt/prev_tau_f/prev_omegadot_f are that axis's
    // filter/differencing memory.
    void _nlms_update_axis(float tau_now, float omegadot_now, float &g1, float seed,
                            Biquad2pState &filt_state, float &extra_filt,
                            float &prev_tau_f, float &prev_omegadot_f, bool do_step);
};
