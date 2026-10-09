#include "FlightStateMachine.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

FlightStateMachine::FlightStateMachine(const DroneConfig &cfg)
    : _phase(FlightPhase::DISARMED)
    , _mode(FlightMode::STABILIZE)
    , _pid(cfg.pid)
    , _indi(cfg.indi)
    , _pid_pi(cfg.pid_pi)
    , _alt(cfg.alt)
    , _pos(cfg.pos)
    , _heading(cfg.heading)
    , _yaw_stick_rate(cfg.heading.stick_rate)
    , _unmixer(cfg.unmixer)
    , _num_controllers(0)
    , _active_index(0)
    , _indi_index(-1)
{
    _controllers[_num_controllers++] = &_pid;   // always present, always index 0
    if (cfg.controllers.indi_enabled) {
        _indi_index = _num_controllers;
        _controllers[_num_controllers++] = &_indi;
    }
    if (cfg.controllers.pid_pi_enabled) {
        _controllers[_num_controllers++] = &_pid_pi;
    }
}

void FlightStateMachine::reset_all()
{
    _pid.reset_all();
    _indi.reset_all();
    _pid_pi.reset_all();
    _alt.reset_all();
    _pos.reset_all();
    _heading.reset();
    _traj.cancel();   // the origin and floor are kept
}

// ── Attitude dispatcher ──────────────────────────────────────────────────────

void FlightStateMachine::run_attitude(const float euler[],
                                      const float state_full[],
                                      const float input[],
                                      const uint32_t rpm[],
                                      float out_cmds[3])
{
    // Every controller in this drone's config list runs (shadow mode) so a
    // controller not currently selected stays live and comparable in the
    // logs regardless of which one actually flies.
    float current_torque[2];
    _unmixer.compute(rpm, current_torque);

    // Tell the live NLMS estimator whether INDI is the controller actually
    // driving out_cmds this tick, so it can select its mode-dependent
    // adaptation rate mu. The estimator itself still runs every tick below
    // regardless of this flag (continuous shadow estimation) — see
    // Attitude_INDI's Live G(x) adaptation write-up in
    // src/controllers/README.md.
    if (_indi_index >= 0) {
        _indi.set_indi_active(_active_index == _indi_index);
    }

    float cmds[MAX_ATTITUDE_CONTROLLERS][3];
    for (int i = 0; i < _num_controllers; ++i) {
        _controllers[i]->update(euler, state_full, input, current_torque, _unmixer, cmds[i]);
    }

    // _indi_index is INDI's list index when the drone's config enables it
    // (see constructor), -1 otherwise — no longer assumable as "1" now that
    // list index 1 may instead be PID+PI on a drone with INDI disabled.
    if (_indi_index >= 0) {
        float delta_torque[2], accel_cmd[2], g1[2];
        _indi.get_diag(delta_torque, accel_cmd);
        _indi.get_g1(g1);
        _indi_diag[0] = current_torque[0];
        _indi_diag[1] = current_torque[1];
        _indi_diag[2] = delta_torque[0];
        _indi_diag[3] = delta_torque[1];
        _indi_diag[4] = cmds[_indi_index][0];
        _indi_diag[5] = cmds[_indi_index][1];
        _indi_diag[6] = accel_cmd[0];
        _indi_diag[7] = accel_cmd[1];
        _indi_diag[8] = g1[0];
        _indi_diag[9] = g1[1];
    } else {
        memset(_indi_diag, 0, sizeof(_indi_diag));
    }

    const float *active = cmds[_active_index];
    out_cmds[0] = active[0];
    out_cmds[1] = active[1];
    out_cmds[2] = active[2];
}

// ── Mode: STABILIZE ─────────────────────────────────────────────────────────

void FlightStateMachine::mode_stabilize(const float euler[],
                                        const float state_full[],
                                        const float input[],
                                        const uint32_t rpm[],
                                        float out_cmds[3], float &thrust_out)
{
    run_attitude(euler, state_full, input, rpm, out_cmds);
    thrust_out = _alt.compute_throttle(euler[0], euler[1], input[InputIdx::THRUST]);
}

// ── Mode: ALT_HOLD ──────────────────────────────────────────────────────────

// D component of the body→NED velocity rotation (positive = descending).
static float ned_vel_D(const float euler[], const float state_full[])
{
    const float rol = euler[0], pit = euler[1];
    return -state_full[StateIdx::U]*sinf(pit)
         +  state_full[StateIdx::V]*sinf(rol)*cosf(pit)
         +  state_full[StateIdx::W]*cosf(rol)*cosf(pit);
}

void FlightStateMachine::mode_alt_hold(const float euler[],
                                       const float state_full[],
                                       const float input[],
                                       const uint32_t rpm[],
                                       float out_cmds[3], float &thrust_out)
{
    run_attitude(euler, state_full, input, rpm, out_cmds);

    thrust_out = _alt.alt_hold(input[InputIdx::THRUST], state_full[StateIdx::Z_POS],
                               ned_vel_D(euler, state_full));
}

// ── Mode: POS_HOLD ──────────────────────────────────────────────────────────
//
// Sticks (default): N/E is PosControl::update() (sticks → accel → velocity →
// position targets, see PosControl.hpp), D is AltControl::alt_hold() exactly
// as in ALT_HOLD, and the yaw stick turns HeadingControl's heading target.
//
// Trajectory (TrajectoryTracker active): the same three controllers track
// the tracker's reference instead, through their track()/update_heading()
// entry points. Because they are the same controllers, with the same
// integrators and targets, going back to the sticks is seamless.

float FlightStateMachine::yaw_stick_to_rate(float stick) const
{
    const float deadband = PosControl::STICK_DEADBAND;
    const float mag      = fabsf(stick);
    if (mag <= deadband) return 0.0f;
    const float rate = _yaw_stick_rate * constrain_float((mag - deadband) / (1.0f - deadband), 0.0f, 1.0f);
    return stick < 0.0f ? -rate : rate;
}

void FlightStateMachine::mode_pos_hold(const float euler[],
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
    const float vD = ned_vel_D(euler, state_full);

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

    // ── Trajectory reference, if one is running ──────────────────────────────
    TrajRef ref;
    const bool traj_active = !CTUN_POSHOLD_SHADOW &&
        _traj.update(&pos_state9[0], &pos_state9[6], yaw,
                     input[InputIdx::ROLL_TGT], input[InputIdx::PITCH_TGT],
                     input[InputIdx::YAW_RATE], input[InputIdx::THRUST], ref);

    // ── N/E → lean angle commands, D → throttle, heading → yaw rate ──────────
    float att_cmds[2];
    float alt_thrust;
    float yaw_rate_cmd;
    if (traj_active) {
        _pos.track(pos_state9, ref.pos, ref.vel, ref.acc, att_cmds);
        alt_thrust   = _alt.track(ref.pos[2], ref.vel[2], cur_D, vD);
        yaw_rate_cmd = _heading.update_heading(yaw, ref.yaw, ref.yaw_rate);
    } else {
        // Sticks: forward/right → N/E, throttle → climb rate (same
        // controller as ALT_HOLD), yaw → turn rate.
        _pos.update(pos_state9, -input[InputIdx::PITCH_TGT], input[InputIdx::ROLL_TGT], att_cmds);
        alt_thrust = _alt.alt_hold(input[InputIdx::THRUST], cur_D, vD);
        // Skipped while POS_HOLD is flying as STABILIZE, so the heading
        // target doesn't integrate against a vehicle that isn't following it.
        yaw_rate_cmd = CTUN_POSHOLD_SHADOW ? 0.0f
                     : _heading.update_rate(yaw, yaw_stick_to_rate(input[InputIdx::YAW_RATE]));
    }
    const float pos_tgt[2] = { _pos.pos_tgt(0), _pos.pos_tgt(1) };
    const float vel_tgt[2] = { _pos.vel_tgt(0), _pos.vel_tgt(1) };

    _traj_diag[0] = (float)_traj.state();
    _traj_diag[1] = traj_active ? ref.pos[0] : 0.0f;
    _traj_diag[2] = traj_active ? ref.pos[1] : 0.0f;
    _traj_diag[3] = traj_active ? ref.pos[2] : 0.0f;
    _traj_diag[4] = _heading.target();
    _traj_diag[5] = yaw_rate_cmd;
    _traj_diag[6] = traj_active ? _traj.height_offset() : 0.0f;
    _traj_diag[7] = traj_active ? _traj.path_speed()    : 0.0f;

    // ── CTUN shadow diagnostics: outer pos-loop + inner vel-loop targets/errors
    // and the lean angle pos-hold would have sent to the attitude controller.
    // Populated every tick regardless of CTUN_POSHOLD_SHADOW.
    _ctun_diag[0] = pos_tgt[0];          // pos_n_tgt
    _ctun_diag[1] = pos_tgt[0] - cur_N;  // pos_n_err
    _ctun_diag[2] = pos_tgt[1];          // pos_e_tgt
    _ctun_diag[3] = pos_tgt[1] - cur_E;  // pos_e_err
    _ctun_diag[4] = vel_tgt[0];          // vel_n_tgt
    _ctun_diag[5] = vel_tgt[0] - vN;     // vel_n_err
    _ctun_diag[6] = vel_tgt[1];          // vel_e_tgt
    _ctun_diag[7] = vel_tgt[1] - vE;     // vel_e_err
    _ctun_diag[8]  = att_cmds[0];        // roll_tgt  (shadow)
    _ctun_diag[9]  = att_cmds[1];        // pitch_tgt (shadow)
    _ctun_diag[10] = _alt.climb_rate_tgt();       // climb_rate_tgt (shadow while CTUN_POSHOLD_SHADOW is set)
    _ctun_diag[11] = _alt.climb_rate_tgt() - vD;  // climb_rate_err

    if (CTUN_POSHOLD_SHADOW) {
        // TEMP (CTUN tuning): fly hands-off STABILIZE — direct stick to
        // attitude and throttle — while the pos-hold cascade above runs
        // shadow-only. See CTUN_POSHOLD_SHADOW.
        run_attitude(euler, state_full, input, rpm, out_cmds);
        thrust_out = _alt.compute_throttle(euler[0], euler[1], input[InputIdx::THRUST]);
        return;
    }

    // ── Attitude controller with position-derived lean targets and the
    // heading controller's yaw rate ────────────────────────────────────────
    float pos_input[InputIdx::N_INPUTS];
    memcpy(pos_input, input, sizeof(pos_input));
    pos_input[InputIdx::ROLL_TGT]  = att_cmds[0];
    pos_input[InputIdx::PITCH_TGT] = att_cmds[1];
    for (int i = 0; i < _num_controllers; ++i)
        _controllers[i]->set_external_yaw_rate(true, yaw_rate_cmd);
    run_attitude(euler, state_full, pos_input, rpm, out_cmds);

    thrust_out = alt_thrust;
}

// ── Main update ──────────────────────────────────────────────────────────────

void FlightStateMachine::update(const float state_full[], const float euler[3],
                                const float input[], bool armed, bool pos_valid,
                                const uint32_t rpm[4],
                                float out_cmds[3], float &thrust_out)
{
    _pos_valid = pos_valid;
    memset(_traj_diag, 0, sizeof(_traj_diag));   // refilled by mode_pos_hold() when it runs

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
        _takeoff_debounce_ticks = _landed_debounce_ticks = _spool_ticks = 0;
        return;
    }
    if (_phase == FlightPhase::DISARMED) {
        _phase = FlightPhase::GROUND_IDLE;   // arm → ground idle, never straight to ACTIVE
        reset_all();
        _takeoff_debounce_ticks = 0;
    }

    // ── Ground-idle gating: require a sustained, deliberate stick push past a
    // mode-dependent threshold before leaving GROUND_IDLE. Mirrors ArduPilot's
    // spool-state machine: controller output is discarded outright while
    // grounded, so nothing computed by the cascades can reach the motors.
    const float thr_stick = input[InputIdx::THRUST];
    if (_phase == FlightPhase::GROUND_IDLE) {
        const float takeoff_threshold = (_mode == FlightMode::STABILIZE)
            ? TAKEOFF_THR_THRESHOLD_STABILIZE : TAKEOFF_THR_THRESHOLD_HOLD;
        if (thr_stick > takeoff_threshold) {
            if (++_takeoff_debounce_ticks >= TAKEOFF_DEBOUNCE_TICKS) {
                _phase = FlightPhase::ACTIVE;
                _spool_ticks = 0;
                // Where the vehicle is sitting now is the floor no
                // trajectory may go below (see TrajectoryTracker.hpp).
                if (pos_valid) _traj.set_floor(state_full[StateIdx::Z_POS]);
                else           _traj.clear_floor();
            }
        } else {
            _takeoff_debounce_ticks = 0;
        }
    }

    if (_phase == FlightPhase::GROUND_IDLE) {
        reset_all();
        out_cmds[0] = out_cmds[1] = out_cmds[2] = 0.0f;
        thrust_out = 0.0f;
        return;
    }

    // ── A trajectory only runs in POS_HOLD on a live position. (Disarming,
    // ground idle and a mode change have already cancelled it through
    // reset_all().)
    if (!pos_valid) _traj.cancel();

    // ── Yaw: each attitude controller's own rate loop + heading-lock trim,
    // unless mode_pos_hold() hands them HeadingControl's rate below.
    for (int i = 0; i < _num_controllers; ++i)
        _controllers[i]->set_external_yaw_rate(false, 0.0f);

    // ── Dispatch ─────────────────────────────────────────────────────────────
    switch (_mode) {
        case FlightMode::STABILIZE:
            mode_stabilize(euler, state_full, input, rpm, out_cmds, thrust_out);
            break;
        case FlightMode::ALT_HOLD:
            mode_alt_hold(euler, state_full, input, rpm, out_cmds, thrust_out);
            break;
        case FlightMode::POS_HOLD:
            mode_pos_hold(euler, state_full, input, rpm, out_cmds, thrust_out);
            break;
    }

    // ── Spool-up ramp: linearly ramp thrust over the first SPOOL_UP_TICKS
    // after leaving GROUND_IDLE, rather than stepping straight to whatever
    // the cascade commands (mirrors ArduPilot's MOT_SPOOL_TIME).
    const bool still_spooling = _spool_ticks < SPOOL_UP_TICKS;
    if (still_spooling) {
        thrust_out *= (float)_spool_ticks / (float)SPOOL_UP_TICKS;
        ++_spool_ticks;
    }

    // ── Landed detection: sustained low commanded thrust + low vertical speed
    // drops back to GROUND_IDLE so a future cascade bug can't idle-wind-up
    // while sitting on the ground. Suppressed during the spool-up ramp itself:
    // the ramp deliberately holds thrust_out near zero right after leaving
    // GROUND_IDLE, which would otherwise look identical to "landed" and bounce
    // the phase straight back before the vehicle ever reaches full authority.
    const float vD = state_full[StateIdx::W];
    if (!still_spooling && thrust_out < LANDED_THR_THRESHOLD && fabsf(vD) < LANDED_VEL_THRESHOLD) {
        if (++_landed_debounce_ticks >= LANDED_DEBOUNCE_TICKS) {
            _phase = FlightPhase::GROUND_IDLE;
            _takeoff_debounce_ticks = 0;
        }
    } else {
        _landed_debounce_ticks = 0;
    }
}

// ── Trajectory commands ──────────────────────────────────────────────────────

TrajResult FlightStateMachine::traj_command(const TrajCommand &cmd, const float state_full[],
                                            const float euler[3])
{
    const float pos[3] = { state_full[StateIdx::X], state_full[StateIdx::Y], state_full[StateIdx::Z_POS] };

    switch (cmd.type) {
        case TrajCmdType::STOP:
            _traj.cancel();
            return TrajResult::ACCEPTED;

        case TrajCmdType::SET_ORIGIN:
            if (!_pos_valid) return TrajResult::NOT_READY;
            return _traj.set_origin(pos);

        case TrajCmdType::POINT:
        case TrajCmdType::CIRCLE: {
            const bool flying_pos_hold = (_mode == FlightMode::POS_HOLD) && !CTUN_POSHOLD_SHADOW &&
                                         (_phase == FlightPhase::ACTIVE) && _pos_valid;
            if (!_traj.origin_set()) return TrajResult::NO_ORIGIN;
            if (!flying_pos_hold)    return TrajResult::NOT_READY;
            // Start from what POS_HOLD is holding right now (its targets,
            // not the measured position), so taking over moves nothing.
            const float hold[3] = { _pos.pos_tgt(0), _pos.pos_tgt(1), _alt.alt_tgt() };
            return (cmd.type == TrajCmdType::POINT) ? _traj.start_point(cmd.p, hold, euler[2])
                                                    : _traj.start_circle(cmd.p, hold, euler[2]);
        }
    }
    return TrajResult::BAD_PARAM;
}
