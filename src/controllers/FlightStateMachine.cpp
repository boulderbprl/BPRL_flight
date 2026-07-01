#include "FlightStateMachine.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

FlightStateMachine::FlightStateMachine()
    : _phase(FlightPhase::DISARMED)
    , _mode(FlightMode::STABILIZE)
    , _use_indi(false)
{}

void FlightStateMachine::reset_all()
{
    _pid.reset_all();
    _indi.reset_all();
    _alt.reset_all();
    _pos.reset_all();
    _ph_N = PHAxisMode::PILOT;
    _ph_E = PHAxisMode::PILOT;
    memset(_hold_pos, 0, sizeof(_hold_pos));
    _hold_pos_valid = false;
    _blend_lean_N = _blend_lean_E = 0.0f;
    _blend_ticks_N = _blend_ticks_E = 0;
}

// ── Attitude dispatcher ──────────────────────────────────────────────────────

void FlightStateMachine::run_attitude(const float ctrl_state6[],
                                      const float euler[],
                                      const float state_full[],
                                      const float input[],
                                      const uint32_t rpm[],
                                      float out_cmds[3])
{
    if (_use_indi) {
        float current_torque[2];
        _unmixer.compute(rpm, current_torque);
        _indi.update(euler, state_full, input, current_torque, _unmixer, out_cmds);
    } else {
        _pid.update(ctrl_state6, input, out_cmds);
    }
}

// ── Mode: STABILIZE ─────────────────────────────────────────────────────────

void FlightStateMachine::mode_stabilize(const float ctrl_state6[],
                                        const float euler[],
                                        const float state_full[],
                                        const float input[],
                                        const uint32_t rpm[],
                                        float out_cmds[3], float &thrust_out)
{
    run_attitude(ctrl_state6, euler, state_full, input, rpm, out_cmds);
    thrust_out = _alt.compute_throttle(euler[0], euler[1], input[InputIdx::THRUST]);
}

// ── Mode: ALT_HOLD ──────────────────────────────────────────────────────────

void FlightStateMachine::mode_alt_hold(const float ctrl_state6[],
                                       const float euler[],
                                       const float state_full[],
                                       const float input[],
                                       const uint32_t rpm[],
                                       float out_cmds[3], float &thrust_out)
{
    run_attitude(ctrl_state6, euler, state_full, input, rpm, out_cmds);

    const float vD     = state_full[StateIdx::W];
    const float vD_dot = state_full[StateIdx::W_DOT];
    thrust_out = _alt.alt_hold_from_stick(input[InputIdx::THRUST], vD, vD_dot);
}

// ── Mode: POS_HOLD ──────────────────────────────────────────────────────────
//
// Per-axis pilot-blend state machine (PILOT→BRAKE→HOLD→RETURNING).
// N and E axes run independently.  D axis always runs AltControl cascade.

void FlightStateMachine::mode_pos_hold(const float ctrl_state6[],
                                       const float euler[],
                                       const float state_full[],
                                       const float input[],
                                       const uint32_t rpm[],
                                       float out_cmds[3], float &thrust_out)
{
    // ── Body→NED velocity rotation ───────────────────────────────────────────
    const float rol = euler[0], pit = euler[1], yaw = euler[2];
    const float U   = state_full[StateIdx::U];
    const float V   = state_full[StateIdx::V];
    const float W   = state_full[StateIdx::W];

    const float vN = U*cosf(pit)*cosf(yaw)
                   + V*(sinf(rol)*sinf(pit)*cosf(yaw) - cosf(rol)*sinf(yaw))
                   + W*(cosf(rol)*sinf(pit)*cosf(yaw) + sinf(rol)*sinf(yaw));
    const float vE = U*cosf(pit)*sinf(yaw)
                   + V*(sinf(rol)*sinf(pit)*sinf(yaw) + cosf(rol)*cosf(yaw))
                   + W*(cosf(rol)*sinf(pit)*sinf(yaw) - sinf(rol)*cosf(yaw));
    const float vD = -U*sinf(pit) + V*sinf(rol)*cosf(pit) + W*cosf(rol)*cosf(pit);

    const float pos_state9[9] = {
        state_full[StateIdx::X],
        state_full[StateIdx::Y],
        state_full[StateIdx::Z_POS],
        euler[0], euler[1], euler[2],
        vN, vE, vD
    };

    const float cur_N = pos_state9[0];
    const float cur_E = pos_state9[1];
    const float cur_D = pos_state9[2];

    // ── Stick mapping for NE pilot override ──────────────────────────────────
    const float stick_N = -input[InputIdx::PITCH_TGT] * MAX_VEL_NE;
    const float stick_E =  input[InputIdx::ROLL_TGT]  * MAX_VEL_NE;
    const bool  sN_act  = fabsf(input[InputIdx::PITCH_TGT]) > STICK_DEADBAND;
    const bool  sE_act  = fabsf(input[InputIdx::ROLL_TGT])  > STICK_DEADBAND;

    // ── Per-axis N state machine ─────────────────────────────────────────────
    float vel_N_tgt   = 0.0f;
    bool  hold_N_pos  = false;   // true → use _hold_pos[0] in NED_update

    switch (_ph_N) {
        case PHAxisMode::PILOT:
            vel_N_tgt = stick_N;
            if (!sN_act) _ph_N = PHAxisMode::BRAKE;
            break;

        case PHAxisMode::BRAKE:
            vel_N_tgt = 0.0f;
            if (sN_act) {
                _ph_N = PHAxisMode::PILOT;
            } else if (fabsf(vN) < BRAKE_VEL_THR) {
                _hold_pos[0] = cur_N;
                _ph_N = PHAxisMode::HOLD;
            }
            break;

        case PHAxisMode::HOLD:
            hold_N_pos = true;
            if (sN_act) {
                _ph_N = PHAxisMode::RETURNING;
                _blend_ticks_N = BLEND_TICKS;
                // _blend_lean_N will be set after NED_update runs below
            }
            break;

        case PHAxisMode::RETURNING:
            if (_blend_ticks_N > 0) {
                const float t = static_cast<float>(_blend_ticks_N) / BLEND_TICKS;
                vel_N_tgt = t * _blend_lean_N + (1.0f - t) * stick_N;
                --_blend_ticks_N;
            } else {
                _ph_N = PHAxisMode::PILOT;
                vel_N_tgt = stick_N;
            }
            break;
    }

    // ── Per-axis E state machine ─────────────────────────────────────────────
    float vel_E_tgt  = 0.0f;
    bool  hold_E_pos = false;

    switch (_ph_E) {
        case PHAxisMode::PILOT:
            vel_E_tgt = stick_E;
            if (!sE_act) _ph_E = PHAxisMode::BRAKE;
            break;

        case PHAxisMode::BRAKE:
            vel_E_tgt = 0.0f;
            if (sE_act) {
                _ph_E = PHAxisMode::PILOT;
            } else if (fabsf(vE) < BRAKE_VEL_THR) {
                _hold_pos[1] = cur_E;
                _ph_E = PHAxisMode::HOLD;
            }
            break;

        case PHAxisMode::HOLD:
            hold_E_pos = true;
            if (sE_act) {
                _ph_E = PHAxisMode::RETURNING;
                _blend_ticks_E = BLEND_TICKS;
            }
            break;

        case PHAxisMode::RETURNING:
            if (_blend_ticks_E > 0) {
                const float t = static_cast<float>(_blend_ticks_E) / BLEND_TICKS;
                vel_E_tgt = t * _blend_lean_E + (1.0f - t) * stick_E;
                --_blend_ticks_E;
            } else {
                _ph_E = PHAxisMode::PILOT;
                vel_E_tgt = stick_E;
            }
            break;
    }

    // ── D hold: latch position on first full hold, then keep ─────────────────
    if (!_hold_pos_valid) {
        _hold_pos[2] = cur_D;
        // Only commit once both horizontal axes have settled to HOLD
        if (_ph_N == PHAxisMode::HOLD && _ph_E == PHAxisMode::HOLD) {
            _hold_pos_valid = true;
        }
    }

    // ── Single NED_update call with appropriate position targets ─────────────
    const float pos_tgt[3] = {
        hold_N_pos ? _hold_pos[0] : cur_N,   // PILOT/BRAKE/RETURNING → error = 0
        hold_E_pos ? _hold_pos[1] : cur_E,
        _hold_pos[2]
    };
    float vel_tgt[3];
    _pos.NED_update(pos_state9, pos_tgt, vel_tgt);

    // Override N and E with state-machine velocities for non-HOLD axes.
    // HOLD axes use NED_update output.  After NED_update snapshot for blend.
    // Snapshot controller velocity on the tick we first enter RETURNING
    // (hold_N/E_pos still true that tick, so NED_update ran with hold target).
    if (_ph_N == PHAxisMode::RETURNING && _blend_ticks_N == BLEND_TICKS) {
        _blend_lean_N = vel_tgt[0];
    }
    if (_ph_E == PHAxisMode::RETURNING && _blend_ticks_E == BLEND_TICKS) {
        _blend_lean_E = vel_tgt[1];
    }

    if (!hold_N_pos) vel_tgt[0] = vel_N_tgt;
    if (!hold_E_pos) vel_tgt[1] = vel_E_tgt;

    // ── NE lean angle commands ────────────────────────────────────────────────
    const float vel_NE[2] = { vel_tgt[0], vel_tgt[1] };
    float att_cmds[2];
    _pos.NE_rate_update(pos_state9, vel_NE, att_cmds);

    // ── Attitude controller with position-derived lean targets ────────────────
    float pos_input[InputIdx::N_INPUTS];
    memcpy(pos_input, input, sizeof(pos_input));
    pos_input[InputIdx::ROLL_TGT]  = att_cmds[0];
    pos_input[InputIdx::PITCH_TGT] = att_cmds[1];
    run_attitude(ctrl_state6, euler, state_full, pos_input, rpm, out_cmds);

    // ── Altitude: inner loops only (climb rate from NED_update D) ─────────────
    const float vD_dot = state_full[StateIdx::W_DOT];
    thrust_out = _alt.alt_hold_from_rate(vel_tgt[2], vD, vD_dot);
}

// ── Main update ──────────────────────────────────────────────────────────────

void FlightStateMachine::update(const float state_full[], const float euler[3],
                                const float input[], bool armed,
                                const uint32_t rpm[4],
                                float out_cmds[3], float &thrust_out)
{
    // ── Flight mode selection ────────────────────────────────────────────────
    const float fm = input[InputIdx::FLIGHT_MODE];
    FlightMode new_mode;
    if      (fm < -0.33f) new_mode = FlightMode::STABILIZE;
    else if (fm >  0.33f) new_mode = FlightMode::POS_HOLD;
    else                  new_mode = FlightMode::ALT_HOLD;

    if (new_mode != _mode) {
        _mode = new_mode;
        reset_all();
    }

    // ── Phase transitions ────────────────────────────────────────────────────
    if (!armed) {
        if (_phase != FlightPhase::DISARMED) {
            _phase = FlightPhase::DISARMED;
            reset_all();
        }
        out_cmds[0] = out_cmds[1] = out_cmds[2] = 0.0f;
        thrust_out = 0.0f;
        return;
    }
    if (_phase == FlightPhase::DISARMED) {
        _phase = FlightPhase::ACTIVE;
    }

    // ── Build 6-element attitude state [roll,pitch,yaw, p,q,r] ──────────────
    const float ctrl_state6[6] = {
        euler[0], euler[1], euler[2],
        state_full[StateIdx::P],
        state_full[StateIdx::Q],
        state_full[StateIdx::R]
    };

    // ── Dispatch ─────────────────────────────────────────────────────────────
    switch (_mode) {
        case FlightMode::STABILIZE:
            mode_stabilize(ctrl_state6, euler, state_full, input, rpm, out_cmds, thrust_out);
            break;
        case FlightMode::ALT_HOLD:
            mode_alt_hold(ctrl_state6, euler, state_full, input, rpm, out_cmds, thrust_out);
            break;
        case FlightMode::POS_HOLD:
            mode_pos_hold(ctrl_state6, euler, state_full, input, rpm, out_cmds, thrust_out);
            break;
    }
}
