#include "FlightStateMachine.hpp"
#include "src/FlightState.hpp"
#include <cstring>

FlightStateMachine::FlightStateMachine()
    : _phase(FlightPhase::DISARMED)
    , _mode(FlightMode::STABILIZE)
{}

void FlightStateMachine::reset_all()
{
    _slc.reset_all();
    _indi.reset_all();
}

void FlightStateMachine::update(const float state_full[], const float euler[3],
                                const float input[], bool armed,
                                const uint32_t rpm[4],
                                float out_cmds[3], float &thrust_out)
{
    // ── Flight mode selection from RC switch ─────────────────────────────
    const FlightMode new_mode = (input[InputIdx::FLIGHT_MODE] < 0.0f)
                                ? FlightMode::STABILIZE
                                : FlightMode::INDI;
    if (new_mode != _mode) {
        _mode = new_mode;
        reset_all();
    }

    // ── Phase transitions ────────────────────────────────────────────────
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
        _phase = FlightPhase::IDLE;
    }

    const float thr = input[InputIdx::THRUST];
    if (_phase == FlightPhase::IDLE && thr > ACTIVE_THR_THRESH) {
        _phase = FlightPhase::ACTIVE;
    }
    if (_phase == FlightPhase::ACTIVE && thr <= ACTIVE_THR_THRESH) {
        _phase = FlightPhase::IDLE;
        reset_all();
    }

    // ── IDLE: keep controllers clean, zero torques (mixer outputs PWM_IDLE) ─
    if (_phase == FlightPhase::IDLE) {
        reset_all();
        out_cmds[0] = out_cmds[1] = out_cmds[2] = 0.0f;
        thrust_out = 0.0f;
        return;
    }

    // ── ACTIVE: run selected controller ──────────────────────────────────
    // Build 6-element state array expected by SLCPID (roll,pitch,yaw,p,q,r).
    float ctrl_state6[6] = {
        euler[0], euler[1], euler[2],
        state_full[StateIdx::P],
        state_full[StateIdx::Q],
        state_full[StateIdx::R]
    };

    if (_mode == FlightMode::STABILIZE) {
        _slc.update(ctrl_state6, input, out_cmds);
        thrust_out = _slc.compute_throttle(ctrl_state6, input);
    } else {
        float current_torque[2];
        _unmixer.compute(rpm, current_torque);
        _indi.update(euler, state_full, input, current_torque, _unmixer, out_cmds);
        thrust_out = _indi.compute_throttle(euler, input);
    }
}
