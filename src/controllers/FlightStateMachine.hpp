#pragma once
#include <cstdint>
#include <cstring>
#include "Attitude_PID.hpp"
#include "Attitude_INDI.hpp"
#include "AltControl.hpp"
#include "PosControl.hpp"
#include "Unmixer.hpp"

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
 * Both AttitudePID and AttitudeINDI run every tick (shadow mode), so INDI's
 * internal controller state stays live and its output is always directly
 * comparable to whichever one actually flew (see get_indi_diag()). Only the
 * controller selected by _use_indi (default: false = PID) drives out_cmds:
 *   false → AttitudePID cascade
 *   true  → AttitudeINDI (incremental NDI; roll/pitch INDI, yaw PID)
 *
 * Output (out_cmds[3], thrust_out) feeds the unchanged MotorMixer.
 */
class FlightStateMachine {
public:
    FlightStateMachine();

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

    void set_use_indi(bool use_indi) { _use_indi = use_indi; }

    FlightPhase phase() const { return _phase; }
    FlightMode  mode()  const { return _mode;  }

    // diag[6]: [unmix_roll, unmix_pitch, delta_roll, delta_pitch, cmd_roll, cmd_pitch]
    // Always populated from AttitudeINDI, regardless of _use_indi (shadow-mode logging).
    void get_indi_diag(float diag[6]) const { memcpy(diag, _indi_diag, sizeof(_indi_diag)); }

    void reset_all();

private:
    // ── Mode dispatch helpers ────────────────────────────────────────────────
    void run_attitude(const float ctrl_state6[], const float euler[],
                      const float state_full[], const float input[],
                      const uint32_t rpm[], float out_cmds[3]);

    void mode_stabilize(const float ctrl_state6[], const float euler[],
                        const float state_full[], const float input[],
                        const uint32_t rpm[],
                        float out_cmds[3], float &thrust_out);

    void mode_alt_hold(const float ctrl_state6[], const float euler[],
                       const float state_full[], const float input[],
                       const uint32_t rpm[],
                       float out_cmds[3], float &thrust_out);

    void mode_pos_hold(const float ctrl_state6[], const float euler[],
                       const float state_full[], const float input[],
                       const uint32_t rpm[],
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
    static constexpr float    TAKEOFF_THR_THRESHOLD_HOLD      = 0.50f;  // ALT_HOLD / POS_HOLD
    static constexpr uint32_t TAKEOFF_DEBOUNCE_TICKS = 100;   // 0.25 s @ 400 Hz sustained push
    static constexpr float    LANDED_THR_THRESHOLD   = 0.15f; // commanded thrust considered "at rest"
    static constexpr float    LANDED_VEL_THRESHOLD   = 0.2f;  // m/s, vertical speed considered "at rest"
    static constexpr uint32_t LANDED_DEBOUNCE_TICKS  = 400;   // 1.0 s @ 400 Hz sustained before re-idling
    static constexpr uint32_t SPOOL_UP_TICKS         = 200;   // 0.5 s @ 400 Hz linear ramp out of idle

    // ── State ────────────────────────────────────────────────────────────────
    FlightPhase _phase    = FlightPhase::DISARMED;
    FlightMode  _mode     = FlightMode::STABILIZE;
    bool        _use_indi = false;

    // [unmix_roll, unmix_pitch, delta_roll, delta_pitch, cmd_roll, cmd_pitch] — see get_indi_diag()
    float _indi_diag[6] = {};

    AttitudePID  _pid;
    AttitudeINDI _indi;
    AltControl   _alt;
    PosControl   _pos;
    Unmixer      _unmixer;
};
