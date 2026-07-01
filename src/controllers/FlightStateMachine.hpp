#pragma once
#include <cstdint>
#include "Attitude_PID.hpp"
#include "Attitude_INDI.hpp"
#include "AltControl.hpp"
#include "PosControl.hpp"
#include "Unmixer.hpp"

enum class FlightMode  { STABILIZE, ALT_HOLD, POS_HOLD };
enum class FlightPhase { DISARMED, ACTIVE };

/*
 * FlightStateMachine — selects and drives the active flight mode.
 *
 * Called at 400 Hz from ControlThread.  Manages two phases:
 *   DISARMED: arm switch low → motors off, all controllers reset.
 *   ACTIVE:   armed → full controller running; zero throttle idles props.
 *
 * Flight mode from input[InputIdx::FLIGHT_MODE] (3-position switch):
 *   < -0.33  → STABILIZE  (attitude + throttle passthrough)
 *   -0.33..0.33 → ALT_HOLD (attitude + altitude hold cascade)
 *   > +0.33  → POS_HOLD   (position hold + attitude + altitude hold)
 *
 * Attitude controller selected by _use_indi (default: false = PID):
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

    // ── State ────────────────────────────────────────────────────────────────
    FlightPhase _phase    = FlightPhase::DISARMED;
    FlightMode  _mode     = FlightMode::STABILIZE;
    bool        _use_indi = false;

    AttitudePID  _pid;
    AttitudeINDI _indi;
    AltControl   _alt;
    PosControl   _pos;
    Unmixer      _unmixer;
};
