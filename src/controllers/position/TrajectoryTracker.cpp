#include "TrajectoryTracker.hpp"
#include "AltControl.hpp"
#include "PosControl.hpp"
#include "src/math/math.hpp"
#include "src/threads.hpp"   // CONTROL_DT_S
#include <cmath>

static constexpr float TWO_PI = 6.28318530718f;

// NaN ("not given") → fallback.
static float param_or(float value, float fallback)
{
    return std::isnan(value) ? fallback : value;
}

TrajResult TrajectoryTracker::set_origin(const float pos[3])
{
    if (active()) return TrajResult::NOT_READY;
    _origin[0]  = pos[0];
    _origin[1]  = pos[1];
    _origin[2]  = pos[2];
    _origin_set = true;
    return TrajResult::ACCEPTED;
}

// Common start: the reference begins where it already is (a trajectory is
// running) or at POS_HOLD's own targets (it isn't), so the target never jumps.
void TrajectoryTracker::begin(const float hold[3])
{
    if (active()) {
        _ref_D += _height_offset;
    } else {
        _ref_ne[0] = hold[0];
        _ref_ne[1] = hold[1];
        _ref_D     = hold[2];
        _vspeed    = 0.0f;
    }
    _speed           = 0.0f;
    _speed_pred      = 0.0f;
    _height_offset   = 0.0f;
    _theta_rate_prev = 0.0f;
}

// Commanded D offset → altitude goal. Not given → the current altitude.
// False if the offset is out of range or the goal is below the lowest
// altitude a trajectory may use — the command is then refused outright
// rather than flown at a different height from the one asked for.
bool TrajectoryTracker::base_altitude(float d_offset, const float hold[3], float &goal_D) const
{
    if (std::isnan(d_offset)) {
        goal_D = hold[2];
    } else {
        if (fabsf(d_offset) > OFFSET_MAX_M) return false;   // also catches ±inf
        goal_D = _origin[2] + d_offset;
    }
    return goal_D <= _floor_D - FLOOR_MARGIN_M;
}

TrajResult TrajectoryTracker::start_point(const float p[7], const float hold[3], float yaw)
{
    if (!_origin_set)  return TrajResult::NO_ORIGIN;
    if (!_floor_valid) return TrajResult::NOT_READY;

    const float n = param_or(p[0], 0.0f);
    const float e = param_or(p[1], 0.0f);
    float goal_D;
    if (!(fabsf(n) <= OFFSET_MAX_M) || !(fabsf(e) <= OFFSET_MAX_M) || !base_altitude(p[2], hold, goal_D))
        return TrajResult::BAD_PARAM;

    begin(hold);
    _goal_ne[0] = _origin[0] + n;
    _goal_ne[1] = _origin[1] + e;
    _goal_D     = goal_D;
    _hold_yaw   = yaw;
    _is_circle  = false;
    _state      = TrajState::MOVE;
    return TrajResult::ACCEPTED;
}

TrajResult TrajectoryTracker::start_circle(const float p[7], const float hold[3], float /*yaw*/)
{
    if (!_origin_set)  return TrajResult::NO_ORIGIN;
    if (!_floor_valid) return TrajResult::NOT_READY;

    const float r     = p[0];
    const float n     = param_or(p[1], 0.0f);
    const float e     = param_or(p[2], 0.0f);
    const float focus = param_or(p[4], 0.0f);
    const float c     = fabsf(focus);
    float goal_D;
    // Written so that a NaN or infinite value fails every test.
    if (!(r >= RADIUS_MIN_M) || !(r + c <= RADIUS_MAX_M) ||
        !(fabsf(n) <= OFFSET_MAX_M) || !(fabsf(e) <= OFFSET_MAX_M) || !base_altitude(p[3], hold, goal_D))
        return TrajResult::BAD_PARAM;

    const float speed = param_or(p[6], 0.0f);

    begin(hold);
    _centre[0] = _origin[0] + n;
    _centre[1] = _origin[1] + e;
    const float semi_major = r + c;
    const float semi_minor = sqrtf(r * (r + 2.0f * c));
    _rN    = (focus >= 0.0f) ? semi_major : semi_minor;
    _rE    = (focus >= 0.0f) ? semi_minor : semi_major;
    _dir   = (param_or(p[5], 1.0f) < 0.0f) ? -1.0f : 1.0f;
    _v_cmd = (speed > 0.0f) ? fminf(speed, V_MAX) : V_DEFAULT;

    // Join the path at its nearest point to where the reference is now.
    float best = -1.0f;
    for (int i = 0; i < NEAREST_STEPS; ++i) {
        const float     th = (TWO_PI * (float)i) / (float)NEAREST_STEPS;
        const PathPoint pt = path_at(th);
        const float     dn = pt.pos[0] - _ref_ne[0];
        const float     de = pt.pos[1] - _ref_ne[1];
        const float     d2 = dn * dn + de * de;
        if (best < 0.0f || d2 < best) {
            best   = d2;
            _theta = th;
        }
    }
    const PathPoint entry = path_at(_theta);
    _goal_ne[0] = entry.pos[0];
    _goal_ne[1] = entry.pos[1];
    _goal_D     = goal_D;
    _hold_yaw   = atan2f(_dir * entry.d1[1], _dir * entry.d1[0]);
    _is_circle  = true;
    _state      = TrajState::MOVE;
    return TrajResult::ACCEPTED;
}

TrajectoryTracker::PathPoint TrajectoryTracker::path_at(float theta) const
{
    const float s = sinf(theta), c = cosf(theta);
    PathPoint pt;
    pt.pos[0] = _centre[0] + _rN * c;
    pt.pos[1] = _centre[1] + _rE * s;
    pt.d1[0]  = -_rN * s;
    pt.d1[1]  =  _rE * c;
    pt.d2[0]  = -_rN * c;
    pt.d2[1]  = -_rE * s;
    pt.len    = sqrtf(pt.d1[0] * pt.d1[0] + pt.d1[1] * pt.d1[1]);
    pt.kappa  = (_rN * _rE) / (pt.len * pt.len * pt.len);
    return pt;
}

// Fastest the path may be flown at this point: the commanded speed, cut
// back where the turn would need too much lean or too fast a yaw.
float TrajectoryTracker::speed_limit(const PathPoint &pt) const
{
    return fminf(_v_cmd, fminf(sqrtf(A_LAT_MAX / pt.kappa), YAW_RATE_MAX / pt.kappa));
}

bool TrajectoryTracker::update(const float pos[3], const float vel[3], float yaw,
                               float stick_roll, float stick_pitch, float stick_yaw, float stick_thr,
                               TrajRef &ref)
{
    if (!active()) return false;

    // Any deliberate roll/pitch/yaw input hands the vehicle back to the pilot.
    if (fabsf(stick_roll)  > PosControl::STICK_DEADBAND ||
        fabsf(stick_pitch) > PosControl::STICK_DEADBAND ||
        fabsf(stick_yaw)   > PosControl::STICK_DEADBAND) {
        cancel();
        return false;
    }

    // Progress scale: 1 while the vehicle is keeping up, 0 once it is
    // ERR_PAUSE_M behind.
    const float err_n = _ref_ne[0] - pos[0];
    const float err_e = _ref_ne[1] - pos[1];
    const float err   = sqrtf(err_n * err_n + err_e * err_e);
    const float scale = constrain_float((ERR_PAUSE_M - err) / (ERR_PAUSE_M - ERR_SLOW_M), 0.0f, 1.0f);

    if (_state == TrajState::ORBIT) {
        update_orbit(scale, ref);
    } else {
        update_move(scale, ref);

        const bool ref_arrived = (_ref_ne[0] == _goal_ne[0] && _ref_ne[1] == _goal_ne[1]);
        if (ref_arrived && !_is_circle) {
            _state = TrajState::HOLD;
        } else if (ref_arrived) {
            // Start the orbit only once the vehicle itself is at the start
            // of the path, settled, and pointing along it.
            const float speed = sqrtf(vel[0] * vel[0] + vel[1] * vel[1]);
            if (err < ARRIVE_POS_M && speed < ARRIVE_VEL_MS &&
                fabsf(wrap_pi(_hold_yaw - yaw)) < ARRIVE_YAW_RAD) {
                _state           = TrajState::ORBIT;
                _speed           = 0.0f;
                _speed_pred      = 0.0f;
                _theta_rate_prev = 0.0f;
            }
        }
    }

    update_vertical(stick_thr, ref);
    return true;
}

// Straight line toward _goal_ne: speed limited to V_MOVE, slowing on a
// constant-deceleration (sqrt) profile into the goal, with speed changes
// limited to A_TAN_MAX.
//
// The reference itself moves at _speed_pred — the commanded speed through a
// first-order lag matching the attitude loop — while the commanded
// acceleration is fed forward un-lagged. A lean commanded now only produces
// its acceleration ATT_LAG_S later, so a reference built from the un-lagged
// speed gets ahead of the vehicle on every speed change; the feedback then
// makes up the gap by overspeeding, and the vehicle arrives too fast and
// overshoots (seen in flight: 0.8 m/s against a 0.5 m/s reference). Same
// reasoning as PosControl's _accel_pred for the sticks.
void TrajectoryTracker::update_move(float scale, TrajRef &ref)
{
    const float dt   = CONTROL_DT_S;
    const float d[2] = { _goal_ne[0] - _ref_ne[0], _goal_ne[1] - _ref_ne[1] };
    const float dist = sqrtf(d[0] * d[0] + d[1] * d[1]);

    ref.vel[0] = ref.vel[1] = 0.0f;
    ref.acc[0] = ref.acc[1] = 0.0f;

    if (dist > 0.0f) {
        const float v_des = fminf(V_MOVE, fminf(sqrtf(2.0f * A_TAN_MAX * dist), MOVE_KP * dist));
        const float prev  = _speed;
        _speed += constrain_float(v_des - _speed, -A_TAN_MAX * dt, A_TAN_MAX * dt);

        _speed_pred += (dt / (PosControl::ATT_LAG_S + dt)) * (_speed - _speed_pred);

        const float step = _speed_pred * scale * dt;
        if (step >= dist || dist < 1.0e-3f) {
            _ref_ne[0]  = _goal_ne[0];
            _ref_ne[1]  = _goal_ne[1];
            _speed      = 0.0f;
            _speed_pred = 0.0f;
        } else {
            const float dir[2] = { d[0] / dist, d[1] / dist };
            const float accel  = (_speed - prev) / dt;
            _ref_ne[0] += dir[0] * step;
            _ref_ne[1] += dir[1] * step;
            ref.vel[0]  = dir[0] * _speed_pred * scale;
            ref.vel[1]  = dir[1] * _speed_pred * scale;
            ref.acc[0]  = dir[0] * accel * scale;
            ref.acc[1]  = dir[1] * accel * scale;
        }
    }

    ref.pos[0]   = _ref_ne[0];
    ref.pos[1]   = _ref_ne[1];
    ref.yaw      = _hold_yaw;
    ref.yaw_rate = 0.0f;
}

void TrajectoryTracker::update_orbit(float scale, TrajRef &ref)
{
    const float dt = CONTROL_DT_S;
    PathPoint   pt = path_at(_theta);

    // How fast is it safe to be going here? No faster than this point's own
    // limit, and slow enough to brake (at A_TAN_MAX) to the limit of every
    // point within stopping distance ahead — which is what slows the
    // vehicle down before the tight ends of an ellipse rather than in them.
    float v_allow = speed_limit(pt);
    {
        const float ds = fmaxf(0.02f, (_speed * _speed) / (2.0f * A_TAN_MAX) / (float)LOOKAHEAD_STEPS);
        float       th = _theta;
        float       len = pt.len;
        for (int i = 1; i <= LOOKAHEAD_STEPS; ++i) {
            th += _dir * ds / len;
            const PathPoint ahead = path_at(th);
            const float     v_lim = speed_limit(ahead);
            v_allow = fminf(v_allow, sqrtf(v_lim * v_lim + 2.0f * A_TAN_MAX * ds * (float)i));
            len     = ahead.len;
        }
    }
    _speed += constrain_float(v_allow - _speed, -A_TAN_MAX * dt, A_TAN_MAX * dt);

    // Advance along the path.
    const float s_dot      = _speed * scale;
    const float theta_rate = _dir * s_dot / pt.len;
    _theta = wrap_pi(_theta + theta_rate * dt);
    pt     = path_at(_theta);

    const float tangent[2] = { _dir * pt.d1[0] / pt.len, _dir * pt.d1[1] / pt.len };
    _ref_ne[0] = pt.pos[0];
    _ref_ne[1] = pt.pos[1];
    ref.pos[0] = pt.pos[0];
    ref.pos[1] = pt.pos[1];
    ref.vel[0] = tangent[0] * s_dot;
    ref.vel[1] = tangent[1] * s_dot;

    // Nose along the path; it turns at speed × curvature.
    ref.yaw      = atan2f(tangent[1], tangent[0]);
    ref.yaw_rate = _dir * s_dot * pt.kappa;

    // Acceleration feed-forward: p''·θ'² (turning) + p'·θ'' (speeding up /
    // slowing down). Evaluated where the vehicle will be one attitude-loop
    // lag from now, since that is when a lean commanded now takes effect.
    const float theta_acc_max = A_TAN_MAX / pt.len;
    const float theta_acc     = constrain_float((theta_rate - _theta_rate_prev) / dt, -theta_acc_max, theta_acc_max);
    _theta_rate_prev = theta_rate;

    const PathPoint lead = path_at(_theta + theta_rate * PosControl::ATT_LAG_S);
    ref.acc[0] = lead.d2[0] * theta_rate * theta_rate + lead.d1[0] * theta_acc;
    ref.acc[1] = lead.d2[1] * theta_rate * theta_rate + lead.d1[1] * theta_acc;
}

// Altitude: _ref_D moves to the commanded D at up to V_VERT_MAX, and the
// throttle stick's climb rate moves an offset on top of it. A command whose
// D is below the floor limit was refused before it got here; the clamp at
// the end is for the stick, which can only be limited, not refused.
void TrajectoryTracker::update_vertical(float stick_thr, TrajRef &ref)
{
    const float dt = CONTROL_DT_S;

    const float d    = _goal_D - _ref_D;
    const float dist = fabsf(d);
    const float v_des = fminf(V_VERT_MAX, fminf(sqrtf(2.0f * A_VERT_MAX * dist), MOVE_KP * dist));
    _vspeed += constrain_float((d < 0.0f ? -v_des : v_des) - _vspeed, -A_VERT_MAX * dt, A_VERT_MAX * dt);
    if (dist < 1.0e-3f && fabsf(_vspeed) < 0.01f) {
        _ref_D  = _goal_D;
        _vspeed = 0.0f;
    } else {
        _ref_D += _vspeed * dt;
    }

    const float climb = AltControl::stick_to_climb_rate(stick_thr);   // m/s, positive = descending
    _height_offset += climb * dt;

    ref.pos[2] = _ref_D + _height_offset;
    ref.vel[2] = _vspeed + climb;

    const float lowest = _floor_D - FLOOR_MARGIN_M;
    if (ref.pos[2] > lowest) {
        // Take the excess back out of the pilot's offset (as far as it is a
        // lowering offset) so the stick isn't left with a dead zone to wind
        // back through before the vehicle climbs again.
        _height_offset -= fminf(ref.pos[2] - lowest, fmaxf(_height_offset, 0.0f));
        ref.pos[2] = lowest;
        ref.vel[2] = fminf(ref.vel[2], 0.0f);
    }
}
