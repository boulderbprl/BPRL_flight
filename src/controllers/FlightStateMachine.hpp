#pragma once
#include <cstdint>
#include <cstring>
#include "Attitude_PID.hpp"
#include "Attitude_INDI_Jerk.hpp"
#include "Attitude_JINDI.hpp"
#include "Attitude_PID_Jerk.hpp"
#include "Attitude_INDI.hpp"
#include "AttitudeController.hpp"
#include "AltControl.hpp"
#include "PosControl.hpp"
#include "Unmixer.hpp"
#include "configs/DroneConfig.hpp"

enum class FlightMode  { STABILIZE, ALT_HOLD, POS_HOLD };
enum class FlightPhase { DISARMED, GROUND_IDLE, ACTIVE };

/*
 * FlightStateMachine — selects and drives the active flight mode.
 *
 * Called at 400 Hz from ControlThread.  Manages three phases:
 *   DISARMED:    arm switch low → motors off, all controllers reset.
 *   GROUND_IDLE: just armed (or just landed) → controller outputs are
 *                discarded outright and motors held at the mixer's idle
 *                floor, regardless of what the cascades compute. Requires a
 *                sustained, deliberate stick push past a mode-dependent
 *                threshold (debounced) to advance to ACTIVE, then ramps
 *                thrust up over SPOOL_UP_TICKS rather than stepping to full
 *                authority. Mirrors ArduPilot's GROUND_IDLE spool state.
 *   ACTIVE:      armed and spooled up → full controller running. Drops back
 *                to GROUND_IDLE if commanded thrust and vertical speed both
 *                stay low for LANDED_DEBOUNCE_TICKS ("landed").
 *
 * Flight mode from input[InputIdx::FLIGHT_MODE] (3-position switch):
 *   < -0.33  → STABILIZE  (attitude + throttle passthrough)
 *   -0.33..0.33 → ALT_HOLD (attitude + altitude hold cascade)
 *   > +0.33  → POS_HOLD   (position hold + attitude + altitude hold)
 *
 * TEMP (CTUN tuning, see CTUN_POSHOLD_SHADOW): while true, POS_HOLD actually
 * flies hands-off STABILIZE — the pos-hold NE cascade still runs every tick
 * but only in shadow, feeding get_ctun_diag() for the CTUN log. Flip that
 * flag to false to restore closed-loop pos-hold flight.
 *
 * Every controller in the drone's config list runs every tick (shadow mode —
 * see code_rework.md Section 3.4), so a controller not currently selected
 * stays live and its output is always directly comparable to whichever one
 * actually flew (see get_indi_diag() / get_indij_diag() / get_jindi_diag()
 * for INDI's/Jerk's/JINDI's shadow diagnostics specifically). Only the
 * controller at _active_index drives out_cmds. PID is always list index 0
 * and is the default; a drone's config (DroneConfig::controllers) decides
 * whether INDI/Jerk/JINDI are in the list. "Jerk" is AttitudeINDIJerk
 * (Attitude_INDI_Jerk.hpp) — the same accel-level INDI as AttitudeINDI for
 * pitch/yaw, but roll_jerk (the strain-fit jerk estimate) is a "D"-style
 * correction layered onto roll's accel-level INDI increment. "JINDI" is
 * AttitudeJINDI (Attitude_JINDI.hpp) — a different formulation of the same
 * idea: roll_jerk drives an independent second torque-reference path
 * (actuator-dynamics-aware), convex-combined with the accel path rather
 * than summed as P+D (see that class's comment). "PIDJerk" is
 * AttitudePIDJerk (Attitude_PID_Jerk.hpp) — no INDI at all: a plain
 * angle→rate→accel→jerk PID cascade on roll, the jerk loop's integrator
 * holding the torque. All three are enabled/selected via
 * ControllersConfig::jerk_enabled/jindi_enabled/pid_jerk_enabled, the
 * same mechanism INDI uses; a drone should enable at most 2 of {INDI, Jerk,
 * JINDI, PIDJerk} alongside PID so the 3-position switch maps cleanly
 * (see set_active_controller()). List order is fixed: PID, INDI, Jerk,
 * PIDJerk, JINDI (whichever are enabled) — Drone1 runs PIDJerk+JINDI, so
 * low/mid/high = PID/PIDJerk/JINDI.
 *
 * _active_index is set every tick by ControlThread from the radio's
 * controller-select switch (channel 7) via set_active_controller() — it is
 * not a flight-mode-linked choice, so the pilot can flip attitude
 * controllers in any of STABILIZE/ALT_HOLD/POS_HOLD without a mode change.
 * Channel 7 is a 3-position switch; see set_active_controller()'s mapping
 * from raw switch position to list index.
 *
 * roll_jerk (the fitted roll-jerk estimate, see src/sensors/JerkFit.hpp)
 * isn't part of the shared AttitudeController::update() signature — it's
 * supplied by the caller each tick and handed to AttitudeINDIJerk/
 * AttitudeJINDI/AttitudePIDJerk via set_roll_jerk() right before the controller-list
 * dispatch, whether or not either is currently the active (motor-driving)
 * controller.
 *
 * Output (out_cmds[3], thrust_out) feeds the unchanged MotorMixer.
 */
class FlightStateMachine {
public:
    explicit FlightStateMachine(const DroneConfig &cfg);

    // state_full[19]: full EKF state vector (StateIdx::*)
    // euler[3]:       [roll, pitch, yaw] rad
    // input[5]:       [thrust, roll_tgt, pitch_tgt, yaw_rate, flight_mode]
    // armed:          arm switch state from radio
    // rpm[4]:         per-motor mechanical RPM [FR, RL, FL, RR] (for INDI unmixer)
    // roll_jerk:      fitted roll jerk estimate [rad/s^3] (JerkFit.hpp), fed to AttitudeINDIJerk (see set_roll_jerk())
    // out_cmds[3]:    normalised torque [roll, pitch, yaw] → MotorMixer
    // thrust_out:     throttle [0, 1] → MotorMixer
    void update(const float state_full[], const float euler[3],
                const float input[], bool armed,
                const uint32_t rpm[4], float roll_jerk,
                float out_cmds[3], float &thrust_out);

    // Maps the raw 3-position RC switch value (0/1/2, low/mid/high) to a
    // controller-list index.
    //   _num_controllers <= 2 (today's PID/PID+INDI drones): unchanged
    //     legacy mapping — only the highest position ever selects something
    //     other than PID (list index 0, the default); a drone whose config
    //     doesn't enable a second controller has _num_controllers == 1, so
    //     the highest position is a no-op and stays on PID.
    //   _num_controllers == 3 (PID + any 2 of {INDI, Jerk, JINDI} enabled,
    //     e.g. Drone1's INDI+JINDI): the switch has exactly as many
    //     positions as there are controllers, so each position selects its
    //     same-numbered list index directly. A 4th controller (all of
    //     INDI+Jerk+JINDI enabled at once) would still clamp to this same
    //     logic — see the radio_switch_pos clamp below — but nothing
    //     reachable past list index 2 on a physically 3-position switch;
    //     don't enable all three on one drone.
    //
    // PID (list index 0) is the only controller whose output isn't anchored
    // to the real current_torque measurement (unlike INDI/Jerk/JINDI, all of
    // which seed from it every tick — see Attitude_INDI.cpp /
    // Attitude_INDI_Jerk.cpp / Attitude_JINDI.cpp), so its shadow-mode integrators can drift
    // arbitrarily far from whatever's actually being commanded while
    // another controller flies. Reset it on the edge into PID so it starts
    // from a clean P-only response instead of handing the motors a stale,
    // possibly wound-up integrator.
    void set_active_controller(int radio_switch_pos)
    {
        const int prev_index = _active_index;

        if (_num_controllers >= 3) {
            _active_index = (radio_switch_pos < 0) ? 0
                           : (radio_switch_pos >= _num_controllers) ? _num_controllers - 1
                           : radio_switch_pos;
        } else {
            _active_index = (radio_switch_pos >= 2 && _num_controllers > 1) ? 1 : 0;
        }

        if (_active_index == 0 && prev_index != 0) {
            _pid.reset_all();
        }

        // AttitudePIDJerk isn't anchored to current_torque either — same
        // shadow wind-up problem as PID. Its handover needs the measured
        // torque, which only run_attitude() has, so just flag it here.
        if (_active_index == _pid_jerk_index && prev_index != _pid_jerk_index) {
            _pid_jerk_handover_pending = true;
        }
    }

    // Resolved controller-list index actually driving out_cmds (may differ
    // from the raw radio switch position if the drone's config doesn't
    // enable a second controller) — for $TEL/logging.
    int active_index() const { return _active_index; }

    FlightPhase phase() const { return _phase; }
    FlightMode  mode()  const { return _mode;  }

    // diag[10]: [unmix_roll, unmix_pitch, delta_roll, delta_pitch, cmd_roll, cmd_pitch, accel_cmd_roll, accel_cmd_pitch, g1_hat_roll, g1_hat_pitch]
    // Always populated from AttitudeINDI, regardless of _active_index (shadow-mode logging).
    // g1_hat_roll/pitch are the live NLMS-adapted effectiveness estimate (see AttitudeINDI::get_g1()).
    void get_indi_diag(float diag[10]) const { memcpy(diag, _indi_diag, sizeof(_indi_diag)); }

    // diag[12]: [unmix_roll, unmix_pitch, delta_roll, delta_pitch, cmd_roll, cmd_pitch, accel_cmd_roll, accel_cmd_pitch, g1_hat_roll, g2_hat_roll, jerk_cmd_roll, kappa2_roll]
    // Same shape as get_indi_diag() (plus jerk_cmd_roll, kappa2_roll) — see
    // AttitudeINDIJerk::get_diag()/get_g1()/get_g2()/jerk_cmd_roll()/kappa2_roll().
    // Roll's delta_roll/cmd_roll are the combined P (g1_hat_roll-based) + D
    // (g2_hat_roll-based) correction/output actually used; jerk_cmd_roll is
    // the "D" term's target, for comparing against the actual measured
    // roll_jerk (JKFT.Pdd in the log); kappa2_roll is the static config gain
    // multiplying that D term (constant per flight, logged for reference).
    // Populated when the drone's config enables Jerk
    // (ControllersConfig::jerk_enabled), zeroed otherwise. May or may not be
    // the controller actually driving out_cmds; see active_index().
    void get_indij_diag(float diag[12]) const { memcpy(diag, _indij_diag, sizeof(_indij_diag)); }

    // diag[13]: [unmix_roll, unmix_pitch, tau_accel_roll, tau_jerk_roll, cmd_roll, cmd_pitch, accel_cmd_roll, accel_cmd_pitch, g1_hat_roll, g1_hat_pitch, jerk_cmd_roll, alpha_roll, tau_roll]
    // Same shape as get_indij_diag(), plus tau_roll — see AttitudeJINDI::
    // get_diag()/get_g1()/jerk_cmd_roll()/alpha_roll()/combined_torque_roll().
    // tau_accel_roll/tau_jerk_roll are roll's two independent path torque
    // references (N·m), pre-blend; tau_roll is the alpha-blended combination
    // of those two, pre-normalise (N·m); cmd_roll is tau_roll already
    // normalised to [-1,1] — the output actually used (same as AttitudeJINDI's
    // own out_cmds[0], not a separate quantity); alpha_roll is the static
    // per-drone blend weight (constant per flight, logged for reference).
    // Populated when the drone's config enables JINDI
    // (ControllersConfig::jindi_enabled), zeroed otherwise. May or may not
    // be the controller actually driving out_cmds; see active_index().
    void get_jindi_diag(float diag[13]) const { memcpy(diag, _jindi_diag, sizeof(_jindi_diag)); }

    // diag[9]: [rate_tgt, accel_tgt, jerk_tgt, torque_pi_nm, torque_d_nm, torque_roll_nm, cmd_roll, cmd_pitch, cmd_yaw]
    // AttitudePIDJerk's roll cascade targets, the accel-PI and jerk-D torque
    // contributions, then the final commands — see LogMsgJPID. (Measured
    // P / P_DOT / jerk are logged in ATT / JKFT.)
    // Populated when the drone's config enables PIDJerk
    // (ControllersConfig::pid_jerk_enabled), zeroed otherwise. May or may not
    // be the controller actually driving out_cmds; see active_index().
    void get_pidj_diag(float diag[9]) const { memcpy(diag, _pidj_diag, sizeof(_pidj_diag)); }

    // TEMP (CTUN tuning): diag[12] = [pos_n_tgt, pos_n_err, pos_e_tgt, pos_e_err,
    // vel_n_tgt, vel_n_err, vel_e_tgt, vel_e_err, roll_tgt, pitch_tgt,
    // climb_rate_tgt, climb_rate_err].
    // Populated by mode_pos_hold's outer pos + inner vel + climb-rate loops
    // every tick that mode is POS_HOLD, regardless of CTUN_POSHOLD_SHADOW.
    void get_ctun_diag(float diag[12]) const { memcpy(diag, _ctun_diag, sizeof(_ctun_diag)); }

    void reset_all();

private:
    // ── Mode dispatch helpers ────────────────────────────────────────────────
    void run_attitude(const float euler[], const float state_full[],
                      const float input[], const uint32_t rpm[],
                      float roll_jerk, float out_cmds[3]);

    void mode_stabilize(const float euler[], const float state_full[],
                        const float input[], const uint32_t rpm[],
                        float roll_jerk, float out_cmds[3], float &thrust_out);

    void mode_alt_hold(const float euler[], const float state_full[],
                       const float input[], const uint32_t rpm[],
                       float roll_jerk, float out_cmds[3], float &thrust_out);

    void mode_pos_hold(const float euler[], const float state_full[],
                       const float input[], const uint32_t rpm[],
                       float roll_jerk, float out_cmds[3], float &thrust_out);

    // ── PosHold pilot-blend state ────────────────────────────────────────────
    enum class PHAxisMode { PILOT, BRAKE, HOLD, RETURNING };
    PHAxisMode _ph_N = PHAxisMode::PILOT;
    PHAxisMode _ph_E = PHAxisMode::PILOT;
    bool  _hold_pos_valid   = false;
    float _hold_pos[3]      = {};
    float _blend_lean_N     = 0.0f;
    float _blend_lean_E     = 0.0f;
    uint32_t _blend_ticks_N = 0;
    uint32_t _blend_ticks_E = 0;

    static constexpr uint32_t BLEND_TICKS    = 200;   // ~0.5 s at 400 Hz
    static constexpr float    BRAKE_VEL_THR  = 0.20f; // m/s
    static constexpr float    STICK_DEADBAND = 0.10f; // normalised [-1,1]
    static constexpr float    MAX_VEL_NE     = 5.0f;  // m/s

    // TEMP (CTUN tuning): while true, POS_HOLD flies hands-off like STABILIZE
    // (direct stick → attitude/throttle) and the pos-hold NE cascade runs
    // shadow-only, populating _ctun_diag / get_ctun_diag() for CTUN logging.
    // Flip to false to restore closed-loop pos-hold flight.
    static constexpr bool CTUN_POSHOLD_SHADOW = true;

    // ── Ground-idle state machine ────────────────────────────────────────────
    uint32_t _takeoff_debounce_ticks = 0;
    uint32_t _landed_debounce_ticks  = 0;
    uint32_t _spool_ticks            = 0;   // ticks since GROUND_IDLE→ACTIVE, for spool-up ramp

    // Takeoff-intent threshold is mode-dependent: STABILIZE's stick IS the direct
    // thrust command (compute_throttle expo curve, no deadband), so any deliberate
    // raise off idle signals intent to fly. ALT_HOLD/POS_HOLD's stick is a signed
    // climb-rate command centered on a hold-altitude deadband, so intent-to-fly
    // means crossing past that center.
    static constexpr float    TAKEOFF_THR_THRESHOLD_STABILIZE = 0.10f;
    static constexpr float    TAKEOFF_THR_THRESHOLD_HOLD      = 0.10f;  // ALT_HOLD / POS_HOLD — TEMP: change back to 0.5 before re-enabling closed-loop POS_HOLD (see CTUN_POSHOLD_SHADOW)
    static constexpr uint32_t TAKEOFF_DEBOUNCE_TICKS = 100;   // 0.25 s @ 400 Hz sustained push
    static constexpr float    LANDED_THR_THRESHOLD   = 0.15f; // commanded thrust considered "at rest"
    static constexpr float    LANDED_VEL_THRESHOLD   = 0.2f;  // m/s, vertical speed considered "at rest"
    static constexpr uint32_t LANDED_DEBOUNCE_TICKS  = 400;   // 1.0 s @ 400 Hz sustained before re-idling
    static constexpr uint32_t SPOOL_UP_TICKS         = 200;   // 0.5 s @ 400 Hz linear ramp out of idle

    // ── State ────────────────────────────────────────────────────────────────
    FlightPhase _phase    = FlightPhase::DISARMED;
    FlightMode  _mode     = FlightMode::STABILIZE;

    // [unmix_roll, unmix_pitch, delta_roll, delta_pitch, cmd_roll, cmd_pitch, accel_cmd_roll, accel_cmd_pitch, g1_hat_roll, g1_hat_pitch] — see get_indi_diag()
    float _indi_diag[10] = {};

    // TEMP (CTUN tuning) — see get_ctun_diag()
    float _ctun_diag[12] = {};

    // [unmix_roll, unmix_pitch, delta_roll, delta_pitch, cmd_roll, cmd_pitch, accel_cmd_roll, accel_cmd_pitch, g1_hat_roll, g2_hat_roll, jerk_cmd_roll, kappa2_roll] — see get_indij_diag()
    float _indij_diag[12] = {};

    // [unmix_roll, unmix_pitch, tau_accel_roll, tau_jerk_roll, cmd_roll, cmd_pitch, accel_cmd_roll, accel_cmd_pitch, g1_hat_roll, g1_hat_pitch, jerk_cmd_roll, alpha_roll, tau_roll] — see get_jindi_diag()
    float _jindi_diag[13] = {};

    // [rate_tgt, accel_tgt, jerk_tgt, torque_pi_nm, torque_d_nm, torque_roll_nm, cmd_roll, cmd_pitch, cmd_yaw] — see get_pidj_diag()
    float _pidj_diag[9] = {};

    AttitudePID     _pid;       // always present, always list index 0 (the default)
    AttitudeINDIJerk _indi_jerk; // storage always exists (no heap allocation); only
    AttitudeJINDI   _jindi;     // reachable via _controllers[] if the drone's
    AttitudePIDJerk _pid_jerk;  //   (same — see constructor)
    AttitudeINDI    _indi;      // config enables it (see constructor)
    AltControl      _alt;
    PosControl      _pos;
    Unmixer         _unmixer;

    // ── Config-driven attitude-controller list (see code_rework.md 3.4) ──────
    static constexpr int MAX_ATTITUDE_CONTROLLERS = 5;  // PID + INDI + Jerk + JINDI + PIDJerk today
    AttitudeController *_controllers[MAX_ATTITUDE_CONTROLLERS];
    int _num_controllers;
    int _active_index;   // generalizes the old _use_indi bool
    int _indi_index;     // INDI's list index if cfg.controllers.indi_enabled, else -1 (see constructor)
    int _jerk_index;     // Jerk's list index if cfg.controllers.jerk_enabled, else -1 (see constructor)
    int _jindi_index;    // JINDI's list index if cfg.controllers.jindi_enabled, else -1 (see constructor)
    int _pid_jerk_index; // PIDJerk's list index if cfg.controllers.pid_jerk_enabled, else -1 (see constructor)
    bool _pid_jerk_handover_pending = false;  // set on the switch edge into PIDJerk, consumed in run_attitude()
};
