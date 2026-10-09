#include "PosControl.hpp"
#include "src/math/math.hpp"
#include "src/threads.hpp"   // CONTROL_DT_S
#include <cmath>

static PID make_pid(const PidGains &g)
{
    return PID(g.kp, g.ki, g.kd, g.imax, g.filt_target_hz, g.filt_error_hz, g.filt_d_hz);
}

PosControl::PosControl(const PosControlGains &g)
    : _pos_N(make_pid(g.pos_N))
    , _pos_E(make_pid(g.pos_E))
    , _vel_N(make_pid(g.vel_N))
    , _vel_E(make_pid(g.vel_E))
{}

// Normalised stick → commanded acceleration [m/s²]. Zero inside the deadband,
// rescaled outside it so there is no step at the deadband edge.
float PosControl::stick_to_accel(float stick)
{
    const float mag = fabsf(stick);
    if (mag <= STICK_DEADBAND) return 0.0f;
    const float scaled = constrain_float((mag - STICK_DEADBAND) / (1.0f - STICK_DEADBAND), 0.0f, 1.0f);
    const float accel  = GRAVITY_MSS * tanf(scaled * degreesToRadians(MAX_STICK_LEAN_deg));
    return stick < 0.0f ? -accel : accel;
}

void PosControl::update(const float state[], float stick_fwd, float stick_right,
                        float att_cmds[2])
{
    const float dt      = CONTROL_DT_S;
    const float pos[2]  = { state[0], state[1] };
    const float vel[2]  = { state[6], state[7] };
    const float yaw_rad = state[5];

    // Start from where the vehicle is and how fast it is going, so engaging
    // the controller is a smooth brake to a stop rather than a jump.
    if (!_tgt_valid) {
        for (int i = 0; i < 2; ++i) {
            _pos_des[i]     = pos[i];
            _vel_des[i]     = vel[i];
            _accel_pilot[i] = 0.0f;
            _accel_pred[i]  = 0.0f;
        }
        _brake_accel   = 0.0f;
        _brake_delay_s = 0.0f;
        _tgt_valid     = true;
    }

    // ── Sticks → commanded acceleration in N/E ───────────────────────────────
    const float accel_fwd   = stick_to_accel(stick_fwd);
    const float accel_right = stick_to_accel(stick_right);
    const bool  sticks_centred = (accel_fwd == 0.0f && accel_right == 0.0f);
    const float cy = STICKS_BODY_FRAME ? cosf(yaw_rad) : 1.0f;
    const float sy = STICKS_BODY_FRAME ? sinf(yaw_rad) : 0.0f;
    const float accel_cmd[2] = { cy * accel_fwd - sy * accel_right,
                                 sy * accel_fwd + cy * accel_right };

    // Jerk limit: the vehicle can't change lean instantly, so don't let the
    // acceleration the targets are built from do so either.
    {
        const float d[2]     = { accel_cmd[0] - _accel_pilot[0], accel_cmd[1] - _accel_pilot[1] };
        const float len      = sqrtf(d[0] * d[0] + d[1] * d[1]);
        const float max_step = STICK_JERK_MAX * dt;
        const float scale    = (len > max_step) ? max_step / len : 1.0f;
        _accel_pilot[0] += d[0] * scale;
        _accel_pilot[1] += d[1] * scale;
    }

    // ── Velocity target: commanded accel (stick − brake), drag ───────────────
    if (sticks_centred) {
        if (_brake_delay_s < BRAKE_DELAY_S) _brake_delay_s += dt;
    } else {
        _brake_delay_s = 0.0f;
    }

    // Deceleration along −vel_des: drag + brake.
    float decel_vec[2] = { 0.0f, 0.0f };
    const float speed = sqrtf(_vel_des[0] * _vel_des[0] + _vel_des[1] * _vel_des[1]);
    if (speed > 1.0e-6f) {
        const float dir[2] = { _vel_des[0] / speed, _vel_des[1] / speed };

        // Drag proportional to speed: a held stick settles at a steady speed
        // (MAX_SPEED at full stick) instead of accelerating forever.
        const float drag = STICK_ACCEL_MAX * speed / MAX_SPEED;

        // Braking once the sticks have been centred for BRAKE_DELAY_S.
        const float brake_cmd = (sticks_centred && _brake_delay_s >= BRAKE_DELAY_S)
                              ? fminf(BRAKE_GAIN * speed, BRAKE_ACCEL_MAX) : 0.0f;
        _brake_accel += constrain_float(brake_cmd - _brake_accel,
                                        -BRAKE_JERK_MAX * dt, BRAKE_JERK_MAX * dt);
        decel_vec[0] = dir[0] * (drag + _brake_accel);
        decel_vec[1] = dir[1] * (drag + _brake_accel);
    } else {
        _brake_accel = 0.0f;
    }

    // Commanded acceleration: what is fed forward to the lean angle. Drag is
    // included (ArduPilot leaves it out of its feed-forward) because the
    // flight logs show almost no real aerodynamic drag at these speeds, so
    // leaving it out made the vehicle out-accelerate its own target.
    const float accel_cmd_ff[2] = { _accel_pilot[0] - decel_vec[0],
                                    _accel_pilot[1] - decel_vec[1] };

    // Predicted acceleration: the commanded one through a first-order lag
    // matching the attitude loop's response. The targets are integrated from
    // this, not from the command — the vehicle's lean (and so its real
    // acceleration) trails the command by ATT_LAG_S, and a target built from
    // the un-lagged command gets ahead of the vehicle by ATT_LAG_S × (change
    // in accel) in velocity on every stick push and release, which the
    // feedback then "corrects" into an overshoot. Same role as ArduPilot's
    // _predicted_accel (AC_Loiter::set_pilot_desired_acceleration()).
    {
        const float a = dt / (ATT_LAG_S + dt);
        _accel_pred[0] += a * (accel_cmd_ff[0] - _accel_pred[0]);
        _accel_pred[1] += a * (accel_cmd_ff[1] - _accel_pred[1]);
    }
    _vel_des[0] += _accel_pred[0] * dt;
    _vel_des[1] += _accel_pred[1] * dt;

    // ── Position target: integrate the velocity target ───────────────────────
    _pos_des[0] += _vel_des[0] * dt;
    _pos_des[1] += _vel_des[1] * dt;

    // Leash: don't let the target run away from a vehicle that can't follow
    // (lean-limited, on the ground), which would wind up the position error.
    {
        const float e[2] = { _pos_des[0] - pos[0], _pos_des[1] - pos[1] };
        const float len  = sqrtf(e[0] * e[0] + e[1] * e[1]);
        if (len > POS_LEASH_M) {
            _pos_des[0] = pos[0] + e[0] * (POS_LEASH_M / len);
            _pos_des[1] = pos[1] + e[1] * (POS_LEASH_M / len);
        }
    }

    feedback(state, accel_cmd_ff, att_cmds);
}

void PosControl::track(const float state[], const float pos_des[2], const float vel_des[2],
                       const float accel_ff[2], float att_cmds[2])
{
    // Take the reference over as the stick generator's own state, so that
    // when update() next runs (trajectory cancelled, pilot back on the
    // sticks) it carries on from here: same targets, and a stick/drag
    // acceleration that reproduces accel_ff, so the feed-forward — and the
    // lean angle with it — doesn't step. The jerk limit and drag/brake in
    // update() then take the vehicle from the trajectory's motion to
    // whatever the sticks ask for.
    for (int i = 0; i < 2; ++i) {
        _pos_des[i]     = pos_des[i];
        _vel_des[i]     = vel_des[i];
        _accel_pred[i]  = accel_ff[i];
        _accel_pilot[i] = accel_ff[i] + STICK_ACCEL_MAX * vel_des[i] / MAX_SPEED;   // + drag, which update() subtracts again
    }
    _brake_accel   = 0.0f;
    _brake_delay_s = 0.0f;
    _tgt_valid     = true;

    feedback(state, accel_ff, att_cmds);
}

void PosControl::feedback(const float state[], const float accel_ff[2], float att_cmds[2])
{
    const float pos[2]  = { state[0], state[1] };
    const float vel[2]  = { state[6], state[7] };
    const float yaw_rad = state[5];

    // ── Feedback: position P → velocity PID, with feed-forward at each stage ─
    _vel_tgt[0] = _vel_des[0] + constrain_float(_pos_N.update(_pos_des[0], pos[0]), -VEL_CORR_MAX, VEL_CORR_MAX);
    _vel_tgt[1] = _vel_des[1] + constrain_float(_pos_E.update(_pos_des[1], pos[1]), -VEL_CORR_MAX, VEL_CORR_MAX);

    const float accel_N_fb_raw = _vel_N.update(_vel_tgt[0], vel[0]);
    const float accel_E_fb_raw = _vel_E.update(_vel_tgt[1], vel[1]);

    // Filter the feedback accel in the inertial N/E frame (before the yaw
    // rotation + atan2 in compute_lean_angles) so the smoothing is
    // yaw-invariant. The feed-forward bypasses it so the stick response
    // doesn't pick up the filter's lag.
    // Fixed dt, not measured — see CONTROL_DT_S in src/threads.hpp.
    const float accel_N_fb = lowpass2p(accel_N_fb_raw, _accel_N_filt, ACCEL_FILT_HZ, CONTROL_DT_S);
    const float accel_E_fb = lowpass2p(accel_E_fb_raw, _accel_E_filt, ACCEL_FILT_HZ, CONTROL_DT_S);

    const float accel_N_tgt = accel_ff[0] + accel_N_fb;
    const float accel_E_tgt = accel_ff[1] + accel_E_fb;

    float roll_tgt  = 0.0f;
    float pitch_tgt = 0.0f;
    compute_lean_angles(yaw_rad, accel_N_tgt, accel_E_tgt, roll_tgt, pitch_tgt);

    att_cmds[0] = constrain_float(roll_tgt,  -degreesToRadians(MAX_LEAN_deg),
                                               degreesToRadians(MAX_LEAN_deg));
    att_cmds[1] = constrain_float(pitch_tgt, -degreesToRadians(MAX_LEAN_deg),
                                               degreesToRadians(MAX_LEAN_deg));
}

void PosControl::compute_lean_angles(float yaw_rad, float accel_N_tgt,
                                     float accel_E_tgt,
                                     float &roll_tgt, float &pitch_tgt)
{
    const float accel_N_body =  cosf(yaw_rad) * accel_N_tgt + sinf(yaw_rad) * accel_E_tgt;
    const float accel_E_body = -sinf(yaw_rad) * accel_N_tgt + cosf(yaw_rad) * accel_E_tgt;

    pitch_tgt = atan2f(-accel_N_body, GRAVITY_MSS);
    roll_tgt  = atan2f( accel_E_body * cosf(pitch_tgt), GRAVITY_MSS);
}

void PosControl::reset_all()
{
    _pos_N.reset(); _pos_E.reset();
    _vel_N.reset(); _vel_E.reset();

    _accel_N_filt = Biquad2pState();
    _accel_E_filt = Biquad2pState();
    _tgt_valid    = false;
}
