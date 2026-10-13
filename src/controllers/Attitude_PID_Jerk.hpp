#pragma once
#include "PID.hpp"
#include "Unmixer.hpp"
#include "AttitudeController.hpp"
#include "src/math/math.hpp"

/*
 * Cascade attitude controller in the AttitudePIDPI mould (angle → rate →
 * accel → torque), with a jerk "D" term added to roll's accel→torque stage
 * using the strain-fit roll jerk (src/sensors/JerkFit.hpp) as the measured
 * derivative of acceleration. Pitch and yaw are identical to AttitudePID.
 *
 * Reachable via FlightStateMachine's controller list when the drone's
 * config enables it (ControllersConfig::pid_jerk_enabled) — selectable on
 * the channel-7 switch, and like every listed controller it runs and logs
 * (LOG_MSG_JPID) every tick whether or not it's selected.
 *
 * History: the previous version closed an innermost loop *on* jerk (accel
 * error → jerk target → jerk PI → torque). Torque→jerk has no high-frequency
 * roll-off (jerk ~ d(torque)/dt / I), so that loop's gain stayed ~flat out
 * to the 40-60 Hz arm mode and it limit-cycled there (LOG0038, LOG0042 —
 * measured loop gain ≈ 1.0 at 41.5 Hz). It also couldn't make actual jerk
 * follow its command (loop gain ≈ ki/I ≈ 2 at every frequency). This
 * version closes the loop on acceleration and uses jerk only as damping.
 *
 * Roll, every stage in physical units:
 *
 *   Angle:  angle_err [rad]    → rate_tgt  [rad/s]   (P)
 *   Rate:   rate_err  [rad/s]  → accel_tgt [rad/s²]  (PI, "SLC" — same role/gains as AttitudeINDI's rate loop)
 *   Accel:  torque [N·m] = Kp·(accel_tgt − P_DOT) + Ki·∫(accel_tgt − P_DOT)   (PI, _roll_accel)
 *                        + Kd·(jerk_tgt − jerk_meas)                          (jerk "D")
 *   out_cmds[0] = unmixer.normalize_torque(torque)
 *
 * The jerk "D" term:
 *   - jerk_tgt = 0 ("D on measurement", the ArduPilot convention): the term
 *     is just −Kd·jerk_meas. Disturbances enter through the measurement, so
 *     gust rejection is the same as with a differentiated target, without
 *     derivative kick on stick inputs.
 *   - Why it helps: Kp·a + Kd·jerk ≈ Kp·a(t + τ), τ = Kd/Kp — the accel stage
 *     reacts to acceleration τ seconds earlier. P_DOT is gyro-differentiated
 *     then lowpassed (2nd-order 20 Hz + 1st-order 15 Hz, ~22 ms lag); the
 *     strain jerk arrives in ~5 ms, so the D term claws back part of that
 *     lag. Steady/slow disturbances are held by the accel integrator — jerk
 *     is zero once a disturbance stops changing.
 *   - Kd is capped by the 40-60 Hz loop through the airframe: Kd × D-filter
 *     gain × |torque→jerk| there (~15,800 rad/s³ per N·m at 41.5 Hz, LOG0042)
 *     must stay well under 1. The measured jerk grows with frequency (largest
 *     band 40-50 Hz, LOG0044), so the D term is lowpassed (2nd-order 20 Hz,
 *     the usual D filter) — that cuts ~-13 dB at 41.5 Hz and lets Kd be 4×
 *     larger for the same 40-60 Hz margin. Kd = 8e-5: ~0.27 there before the
 *     Teensy notch, ~0.16 with the Q 1.0 notch (unfiltered Kd = 2e-5 was
 *     0.31 / 0.19). In the useful band the D term is ~0.25× the accel P term
 *     at 3 Hz (+78° lead) and ~0.42× at 5 Hz (+70°) — vs 0.06 / 0.10 before.
 *   - The D term (only) is lowpassed; there is no Cube-side notch on it.
 *
 * Accel PI gains (rigid body, I ≈ g1_seed_roll ≈ 0.006 kg·m²): Kp = I·κ with
 * κ = 1, Ki = I·ω_a with ω_a = 30 rad/s, i.e. loop gain ≈ (κ·s + ω_a)/s at
 * low frequency → crossover ≈ 30 rad/s (~5 Hz), ~4.6× above the rate loop
 * (6.5 rad/s), PI zero at ω_a/κ = 30 rad/s for phase margin against P_DOT
 * lag + motor lag. AttitudePIDPI's ported 0.7/0.8 (normalised torque per
 * rad/s²) are not used — in this firmware's units that's ~150× inertia.
 * NONE of this is bench/flight tuned.
 *
 * The accel integrator IS the held roll torque (trim, sustained gusts), so
 * imax = 1.2 N·m (~95% of T_max ≈ 1.27 N·m) — a small imax caps
 * low-frequency roll authority (LOG0042: 0.3 N·m couldn't recover a ~30°
 * roll).
 *
 * Shadow wind-up / handover: unlike INDI/JINDI, nothing here is anchored to
 * measured torque, so while another controller flies, this one's
 * integrators wind up against a plant it isn't driving. FlightStateMachine
 * calls prepare_handover() on the edge into this controller: angle/rate/
 * pitch/yaw loops are reset, and the accel integrator is seeded with the
 * measured roll torque so the first command matches what the motors are
 * already producing.
 *
 * Yaw has no angle loop of its own (unlike roll/pitch), so nothing
 * previously corrected a rate estimate that isn't exactly mean-zero —
 * heading could walk indefinitely under a nominally "zero rate" command.
 * _yaw_hold adds a small heading-lock trim on top of the rate PID: while
 * the stick is centered, _yaw_target stays fixed and a P correction pulls
 * current heading back to it; while actively yawing, _yaw_target tracks the
 * current heading so there's no stored error to fight when the stick
 * recentres. Mirrors the same trim in AttitudePID / AttitudeINDI.
 *
 * roll_jerk isn't part of the shared AttitudeController::update()
 * signature — set_roll_jerk() must be called once per tick before the
 * generic update() dispatch (same pattern as AttitudeINDIJerk), so update()
 * itself just reads the most recently set value.
 *
 * Conventions:
 *   euler[3]          roll, pitch, yaw (rad)
 *   state_full[19]    full EKF state (StateIdx::P, Q, R, P_DOT used here)
 *   input[1..3]       roll_tgt, pitch_tgt, yaw_rate_tgt [-1, 1]
 *   out_cmds[3]       normalised torque [roll, pitch, yaw] in [-1, 1]
 *
 * current_torque (from the shared interface) is unused outside the handover
 * — unmixer is used only to normalise N·m → [-1, 1].
 */
class AttitudePIDJerk : public AttitudeController {
public:
    AttitudePIDJerk();

    void update(const float euler[3], const float state_full[], const float input[],
                const float current_torque[2], const Unmixer &unmixer,
                float out_cmds[3]) override;
    void reset_all() override;

    // Set once per tick before the controller-list dispatch — see class comment.
    void set_roll_jerk(float roll_jerk) { _roll_jerk_input = roll_jerk; }

    // Bumpless handover — see class comment. current_roll_torque_nm is the
    // Unmixer's RPM-derived roll torque.
    void prepare_handover(float current_roll_torque_nm);

    // Roll intermediate signals from the most recent update() — diagnostics only.
    float roll_rate_tgt()   const { return _roll_rate_tgt; }
    float roll_accel_tgt()  const { return _roll_accel_tgt; }
    float roll_jerk_tgt()   const { return _roll_jerk_tgt; }    // d/dt(accel_tgt), 20 Hz LP — diagnostic only, see below
    float roll_torque_pi()  const { return _roll_torque_pi; }   // accel P+I part [N·m]
    float roll_torque_d()   const { return _roll_torque_d; }    // jerk D part [N·m]
    float roll_torque_nm()  const { return _roll_torque_nm; }   // total, clamped [N·m]

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;   // rate error [rad/s]    -> accel target [rad/s^2]
    PID _roll_accel;  // accel error [rad/s^2] -> roll torque [N*m] (PI; the jerk D is added outside)
    PID _pitch_rate;
    PID _yaw_rate;
    PID _yaw_hold;    // heading-lock trim: heading error [rad] -> corrective rate [rad/s]

    float _yaw_target;        // held heading target [rad]
    bool  _yaw_target_valid;  // false until first update() captures a target

    float _roll_jerk_input = 0.0f;  // most recent set_roll_jerk() value
    float _roll_rate_tgt   = 0.0f;
    float _roll_accel_tgt  = 0.0f;
    float _roll_torque_pi  = 0.0f;
    float _roll_torque_d   = 0.0f;
    float _roll_torque_nm  = 0.0f;

    // Jerk implied by the accel target, d/dt(accel_tgt) lowpassed at
    // JERK_D_LPF_HZ (same as the D term). Logged only — the D term itself uses
    // jerk_tgt = 0 (D on measurement), so this is for comparing commanded vs
    // measured jerk in the logs, not part of the control law.
    float         _roll_jerk_tgt       = 0.0f;
    float         _prev_accel_tgt      = 0.0f;
    bool          _prev_accel_tgt_valid = false;
    Biquad2pState _jerk_tgt_lpf_state;

    // Jerk "D" gain [N·m per rad/s³] and its lowpass — see class comment for
    // the 40-60 Hz cap. Kd was 2e-5 unfiltered; LOG0044 showed it doing ~10%
    // of the accel P term at 5 Hz with most of its output at 40-50 Hz.
    static constexpr float ROLL_JERK_KD    = 8.0e-5f;
    static constexpr float JERK_D_LPF_HZ   = 20.0f;   // 2nd-order Butterworth
    Biquad2pState _jerk_d_lpf_state;

    // No Cube-side filtering on the measured jerk beyond the D lowpass above
    // — the strain channels are already notched and lowpassed on the Teensy.
    // (A ~10.6 Hz notch on the D term was removed; the ~10 Hz roll mode is
    // now visible to the D term.)

    static constexpr float CTRL_DT_S          = 0.0025f;  // 400 Hz ControlThread

    static constexpr float YAW_STICK_GAIN     = 3.0f;
    static constexpr float YAW_STICK_DEADBAND = 0.10f;  // normalised stick [-1,1], matches FlightStateMachine::STICK_DEADBAND
    static constexpr float YAW_HOLD_MAX_RATE  = 0.3f;   // rad/s cap on the heading-hold trim — needs flight tuning
};
