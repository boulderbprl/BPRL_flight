#pragma once
#include <cstdint>
#include <cstring>
#include "Attitude_PID.hpp"
#include "Attitude_INDI.hpp"
#include "Attitude_PID_PI.hpp"
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
 * actually flew (see get_indi_diag() for INDI's shadow diagnostics
 * specifically). Only the controller at _active_index drives out_cmds.
 * PID is always list index 0 and is the default; a drone's config
 * (DroneConfig::controllers) decides whether INDI is in the list at all.
 *
 * _active_index is set every tick by ControlThread from the radio's
 * controller-select switch (channel 7) via set_active_controller() — it is
 * not a flight-mode-linked choice, so the pilot can flip attitude
 * controllers in any of STABILIZE/ALT_HOLD/POS_HOLD without a mode change.
 * Channel 7 is a 3-position switch; see set_active_controller()'s mapping
 * from raw switch position to list index.
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
    // out_cmds[3]:    normalised torque [roll, pitch, yaw] → MotorMixer
    // thrust_out:     throttle [0, 1] → MotorMixer
    void update(const float state_full[], const float euler[3],
                const float input[], bool armed,
                const uint32_t rpm[4],
                float out_cmds[3], float &thrust_out);

    // Maps the raw 3-position RC switch value (0/1/2, low/mid/high) to a
    // controller-list index.
    //   _num_controllers <= 2 (today's PID/PID+INDI drones): unchanged
    //     legacy mapping — only the highest position ever selects something
    //     other than PID (list index 0, the default); a drone whose config
    //     doesn't enable a second controller has _num_controllers == 1, so
    //     the highest position is a no-op and stays on PID.
    //   _num_controllers == 3 (PID + INDI + PID+PI all enabled): the switch
    //     has exactly as many positions as there are controllers, so each
    //     position selects its same-numbered list index directly.
    void set_active_controller(int radio_switch_pos)
    {
        if (_num_controllers >= 3) {
            _active_index = (radio_switch_pos < 0) ? 0
                           : (radio_switch_pos >= _num_controllers) ? _num_controllers - 1
                           : radio_switch_pos;
        } else {
            _active_index = (radio_switch_pos >= 2 && _num_controllers > 1) ? 1 : 0;
        }
    }

    // Resolved controller-list index actually driving out_cmds (may differ
    // from the raw radio switch position if the drone's config doesn't
    // enable a second controller) — for $TEL/logging.
    int active_index() const { return _active_index; }

    FlightPhase phase() const { return _phase; }
    FlightMode  mode()  const { return _mode;  }

    // diag[8]: [unmix_roll, unmix_pitch, delta_roll, delta_pitch, cmd_roll, cmd_pitch, accel_cmd_roll, accel_cmd_pitch]
    // Always populated from AttitudeINDI, regardless of _active_index (shadow-mode logging).
    void get_indi_diag(float diag[8]) const { memcpy(diag, _indi_diag, sizeof(_indi_diag)); }

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
                      const float input[], const uint32_t rpm[], float out_cmds[3]);

    void mode_stabilize(const float euler[], const float state_full[],
                        const float input[], const uint32_t rpm[],
                        float out_cmds[3], float &thrust_out);

    void mode_alt_hold(const float euler[], const float state_full[],
                       const float input[], const uint32_t rpm[],
                       float out_cmds[3], float &thrust_out);

    void mode_pos_hold(const float euler[], const float state_full[],
                       const float input[], const uint32_t rpm[],
                       float out_cmds[3], float &thrust_out);

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

    // [unmix_roll, unmix_pitch, delta_roll, delta_pitch, cmd_roll, cmd_pitch, accel_cmd_roll, accel_cmd_pitch] — see get_indi_diag()
    float _indi_diag[8] = {};

    // TEMP (CTUN tuning) — see get_ctun_diag()
    float _ctun_diag[12] = {};

    AttitudePID   _pid;      // always present, always list index 0 (the default)
    AttitudeINDI  _indi;     // storage always exists (no heap allocation); only
    AttitudePIDPI _pid_pi;   // reachable via _controllers[] if the drone's
                             // config enables it (see constructor)
    AltControl   _alt;
    PosControl   _pos;
    Unmixer      _unmixer;

    // ── Config-driven attitude-controller list (see code_rework.md 3.4) ──────
    static constexpr int MAX_ATTITUDE_CONTROLLERS = 3;  // PID + INDI + PID+PI today
    AttitudeController *_controllers[MAX_ATTITUDE_CONTROLLERS];
    int _num_controllers;
    int _active_index;   // generalizes the old _use_indi bool
    int _indi_index;     // INDI's list index if cfg.controllers.indi_enabled, else -1 (see constructor)
};
