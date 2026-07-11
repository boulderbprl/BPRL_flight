#include "StateManager.hpp"
#include "src/math/math.hpp"
#include <cfloat>

static constexpr float GRAVITY = 9.80665f;

// State index aliases mirroring EKF's internal enum (N=16)
static constexpr int iX=0,  iY=1,  iZ=2;
static constexpr int iU=3,  iV=4,  iW=5;
static constexpr int iQ0=6, iQ1=7, iQ2=8, iQ3=9;
static constexpr int iBax=10, iBay=11, iBaz=12;
static constexpr int iBgx=13, iBgy=14, iBgz=15;

/* ══════════════════════════════════════════════════════════════════════════
 * Construction / initialisation
 * ══════════════════════════════════════════════════════════════════════════ */

StateManager::StateManager()
    : _primary(0), _initialized(false),
      _blended_p(0.0f), _blended_q(0.0f), _blended_r(0.0f),
      _blended_x(0.0f), _blended_y(0.0f), _blended_z(0.0f),
      _blended_u(0.0f), _blended_v(0.0f), _blended_w(0.0f),
      _blended_ud(0.0f), _blended_vd(0.0f), _blended_wd(0.0f),
      _prev_p(0.0f), _prev_q(0.0f), _prev_r(0.0f),
      _pdot_filt(0.0f), _qdot_filt(0.0f), _rdot_filt(0.0f),
      _p_filt(0.0f), _q_filt(0.0f), _r_filt(0.0f),
      _ud_filt(0.0f), _vd_filt(0.0f), _wd_filt(0.0f),
      _lane_p{}, _lane_q{}, _lane_r{}
{}

void StateManager::init()
{
    for (int i = 0; i < NUM_LANES; ++i)
        _lanes[i].init(i);

    _primary      = 0;
    _blended_p    = _blended_q    = _blended_r    = 0.0f;
    _blended_x    = _blended_y    = _blended_z    = 0.0f;
    _blended_u    = _blended_v    = _blended_w    = 0.0f;
    _blended_ud   = _blended_vd   = _blended_wd   = 0.0f;
    _prev_p       = _prev_q       = _prev_r       = 0.0f;
    _pdot_filt    = _qdot_filt    = _rdot_filt    = 0.0f;
    _p_filt       = _q_filt       = _r_filt       = 0.0f;
    _ud_filt      = _vd_filt      = _wd_filt      = 0.0f;
    _ud_filt_state = _vd_filt_state = _wd_filt_state = Biquad2pState{};

    for (int i = 0; i < NUM_LANES; ++i)
        _lane_p[i] = _lane_q[i] = _lane_r[i] = 0.0f;

    _initialized = true;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Main update — called at 500 Hz from StateEstThread
 * ══════════════════════════════════════════════════════════════════════════ */

void StateManager::update(float dt, const IMURaw imu[3], const CANIMURaw& can_imu, const MocapRaw& mocap, const BaroRaw& baro)
{
    if (!_initialized) return;

    // ── 1. Predict: each lane uses its own IMU ─────────────────────────────
    for (int i = 0; i < NUM_LANES; ++i) {
        if (imu[i].valid)
            _lanes[i].predict(dt, imu[i].accel, imu[i].gyro);
    }

    // ── 1.5. Gravity-vector attitude + accel-bias update (gated on |a| ≈ g) ─
    for (int i = 0; i < NUM_LANES; ++i) {
        if (imu[i].valid)
            _lanes[i].update_gravity(imu[i].accel, R_GRAVITY);
    }

    // ── 2. IMX5 quaternion update on all lanes (200 Hz, asynchronous) ─────
    // IMX5 quaternion is body→NED (verified: Euler angles match physical attitude).
    if (can_imu.valid && can_imu.has_new_quat) {
        Quat q_meas = { can_imu.q0, can_imu.q1, can_imu.q2, can_imu.q3 };
        for (int i = 0; i < NUM_LANES; ++i)
            _lanes[i].update_quaternion(q_meas, R_QUAT);
    }

    // ── 3. Select primary lane ────────────────────────────────────────────
    _primary = _select_primary();

    // ── 4. Compute innovation-norm weights ────────────────────────────────
    float w[NUM_LANES] = {}, w_sum = 0.0f;
    for (int i = 0; i < NUM_LANES; ++i) {
        if (!_lanes[i].is_valid()) continue;
        w[i] = 1.0f / (1e-4f + _lanes[i].innovation_norm());
        w_sum += w[i];
    }
    if (w_sum > 1e-10f) {
        for (int i = 0; i < NUM_LANES; ++i) w[i] /= w_sum;
    } else {
        w[_primary] = 1.0f;
    }

    // ── 5. Mocap position + velocity fusion (all lanes, when connected) ───
    // Gated independently: VISION_POSITION_ESTIMATE and VISION_SPEED_ESTIMATE
    // are separate MAVLink messages that need not arrive together or at the
    // same rate. Fusing update_ned_vel() off a has_new_pos-only tick would
    // re-fuse a stale mocap.vx/vy/vz into u/v on every position packet.
    if (mocap.valid && mocap.has_new_pos) {
        const float xyz[3] = { mocap.x, mocap.y, mocap.z };
        for (int i = 0; i < NUM_LANES; ++i) {
            if (!_lanes[i].is_valid()) continue;
            _lanes[i].update_position(xyz, R_MOCAP_POS);
        }
    }
    if (mocap.valid && mocap.has_new_vel) {
        const float vel[3] = { mocap.vx, mocap.vy, mocap.vz };
        for (int i = 0; i < NUM_LANES; ++i) {
            if (!_lanes[i].is_valid()) continue;
            _lanes[i].update_ned_vel(vel, R_MOCAP_VEL);
        }
    }

    // ── 5.6. Barometric altitude fusion (all lanes, when available) ────────
    // Gated on has_new the same way mocap pos/vel are — avoids re-fusing a
    // stale sample on ticks where SPIThread hasn't completed a new P+T pair
    // (baro updates at ~100+ Hz into a 500 Hz EKF loop).
    //
    // Suppressed while mocap is connected — mocap directly measures a more
    // accurate absolute position, and baro's boot-time zero reference isn't
    // reconciled with mocap's world origin, so fusing both would fight rather
    // than average. Falls back to baro automatically on mocap disconnect.
    if (baro.valid && baro.has_new && !mocap.valid) {
        for (int i = 0; i < NUM_LANES; ++i) {
            if (!_lanes[i].is_valid()) continue;
            _lanes[i].update_altitude(baro.alt_m, R_BARO_POS);
        }
    }

    // ── 5.5. Soft-blend X/Y/Z, U/V/W across lanes (same weights as p/q/r) ──
    // Unlike quaternion, plain vectors have no antipodal issue, so there is
    // no reason to hard-select these from a single _primary lane — doing so
    // let independent per-IMU integration drift show up as a discontinuous
    // jump in u/v every time _select_primary() picked a different lane.
    _blended_x = _blended_y = _blended_z = 0.0f;
    _blended_u = _blended_v = _blended_w = 0.0f;
    for (int i = 0; i < NUM_LANES; ++i) {
        if (w[i] < 1e-15f || !_lanes[i].is_valid()) continue;
        const float* st = _lanes[i].state();
        _blended_x += w[i] * st[iX];
        _blended_y += w[i] * st[iY];
        _blended_z += w[i] * st[iZ];
        _blended_u += w[i] * st[iU];
        _blended_v += w[i] * st[iV];
        _blended_w += w[i] * st[iW];
    }

    // ── 6. Soft-blend p/q/r: bias-corrected gyros + IMX5 blend ──
    _blended_p = _blended_q = _blended_r = 0.0f;
    for (int i = 0; i < NUM_LANES; ++i) {
        if (!imu[i].valid || !_lanes[i].is_valid()) {
            _lane_p[i] = _lane_q[i] = _lane_r[i] = 0.0f;
            continue;
        }
        const float* st = _lanes[i].state();
        const float lp = imu[i].gyro[0] - st[iBgx];
        const float lq = imu[i].gyro[1] - st[iBgy];
        const float lr = imu[i].gyro[2] - st[iBgz];
        _lane_p[i] = lp;
        _lane_q[i] = lq;
        _lane_r[i] = lr;
        if (w[i] < 1e-15f) continue;
        _blended_p += w[i] * lp;
        _blended_q += w[i] * lq;
        _blended_r += w[i] * lr;
    }
    if (can_imu.valid) {
        // IMX5 p/q/r blending
        _blended_p = (1.0f-STATEMGR_IMX5_RATE_WEIGHT)*_blended_p + STATEMGR_IMX5_RATE_WEIGHT*can_imu.p;
        _blended_q = (1.0f-STATEMGR_IMX5_RATE_WEIGHT)*_blended_q + STATEMGR_IMX5_RATE_WEIGHT*can_imu.q;
        _blended_r = (1.0f-STATEMGR_IMX5_RATE_WEIGHT)*_blended_r + STATEMGR_IMX5_RATE_WEIGHT*can_imu.r;
    }

    // ── 7. Blend uvw_dot: gravity+Coriolis-corrected IMU accel per lane ───
    _blended_ud = _blended_vd = _blended_wd = 0.0f;
    for (int i = 0; i < NUM_LANES; ++i) {
        if (w[i] < 1e-15f || !imu[i].valid) continue;
        const float* st = _lanes[i].state();

        Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
        float R[3][3];
        quat_to_rot_body2ned(q, R);

        // Gravity in body frame: g_body = R_b2n^T * [0,0,g] = R_b2n row 2 * g
        const float g_body[3] = { R[2][0]*GRAVITY, R[2][1]*GRAVITY, R[2][2]*GRAVITY };

        // Per-lane bias-corrected gyro rates for Coriolis
        const float pqr_l[3] = {
            imu[i].gyro[0] - st[iBgx],
            imu[i].gyro[1] - st[iBgy],
            imu[i].gyro[2] - st[iBgz]
        };
        const float vel_l[3] = { st[iU], st[iV], st[iW] };
        float oxv[3];
        cross3(pqr_l, vel_l, oxv);

        // NED z-down: a_true = accel + g_body - ω×v (sensor reads -g at hover)
        _blended_ud += w[i] * (imu[i].accel[0] - st[iBax] + g_body[0] - oxv[0]);
        _blended_vd += w[i] * (imu[i].accel[1] - st[iBay] + g_body[1] - oxv[1]);
        _blended_wd += w[i] * (imu[i].accel[2] - st[iBaz] + g_body[2] - oxv[2]);
    }

    // 2nd-order (Butterworth) lowpass filter uvw_dot — matches ArduPilot's
    // INS-level LowPassFilter2p on raw accelerometer samples, steeper
    // roll-off than the 1st-order lowpass() used elsewhere in this file.
    _ud_filt = lowpass2p(_blended_ud, _ud_filt_state, STATEMGR_LP_UVWDOT_HZ, dt);
    _vd_filt = lowpass2p(_blended_vd, _vd_filt_state, STATEMGR_LP_UVWDOT_HZ, dt);
    _wd_filt = lowpass2p(_blended_wd, _wd_filt_state, STATEMGR_LP_UVWDOT_HZ, dt);

    // ── 8. Angular acceleration: differentiate blended rates + lowpass ─────
    const float alpha_pqr_dot = lowpass_alpha(STATEMGR_LP_PQRDOT_HZ, dt);
    _pdot_filt = lowpass(derivative(_blended_p, _prev_p, dt), _pdot_filt, alpha_pqr_dot);
    _qdot_filt = lowpass(derivative(_blended_q, _prev_q, dt), _qdot_filt, alpha_pqr_dot);
    _rdot_filt = lowpass(derivative(_blended_r, _prev_r, dt), _rdot_filt, alpha_pqr_dot);

    _prev_p = _blended_p;
    _prev_q = _blended_q;
    _prev_r = _blended_r;

    // ── 9. Lowpass the blended rates themselves before they reach the rate
    // PID — rejects vibration-band noise that would otherwise pass through unfiltered.
    // Yaw (r) gets a heavier cutoff than roll/pitch (p/q).
    const float alpha_pq_out = lowpass_alpha(STATEMGR_LP_PQ_HZ, dt);
    const float alpha_r_out  = lowpass_alpha(STATEMGR_LP_R_HZ, dt);
    _p_filt = lowpass(_blended_p, _p_filt, alpha_pq_out);
    _q_filt = lowpass(_blended_q, _q_filt, alpha_pq_out);
    _r_filt = lowpass(_blended_r, _r_filt, alpha_r_out);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Lane selection
 * ══════════════════════════════════════════════════════════════════════════ */

int StateManager::_select_primary() const
{
    int best = _primary;
    float best_score = FLT_MAX;

    for (int i = 0; i < NUM_LANES; ++i) {
        if (!_lanes[i].is_valid()) continue;
        float score = _lanes[i].innovation_norm();
        if (score < best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Full 19-element state output
 * Maps 16-state EKF lanes onto the StateIdx ordering and inserts derived states.
 * ══════════════════════════════════════════════════════════════════════════ */

void StateManager::get_state(float out[StateIdx::N]) const
{
    static_assert(StateIdx::N == 19, "state size mismatch");

    const float* ekf = _lanes[_primary].state();  // EKF::N = 16 elements

    // Position (NED) — soft-blended across lanes (see _blended_x/y/z)
    out[StateIdx::X]     = _blended_x;
    out[StateIdx::Y]     = _blended_y;
    out[StateIdx::Z_POS] = _blended_z;

    // Body velocity — soft-blended across lanes (see _blended_u/v/w)
    out[StateIdx::U] = _blended_u;
    out[StateIdx::V] = _blended_v;
    out[StateIdx::W] = _blended_w;

    // Body acceleration (blended + lowpass filtered) → out[6–8]
    out[StateIdx::U_DOT] = _ud_filt;
    out[StateIdx::V_DOT] = _vd_filt;
    out[StateIdx::W_DOT] = _wd_filt;

    // Quaternion Body→NED — EKF[6–9] → out[9–12]
    out[StateIdx::Q0] = ekf[iQ0];
    out[StateIdx::Q1] = ekf[iQ1];
    out[StateIdx::Q2] = ekf[iQ2];
    out[StateIdx::Q3] = ekf[iQ3];

    // Angular rates (soft-blended + lowpass filtered, body frame) → out[13–15]
    out[StateIdx::P] = _p_filt;
    out[StateIdx::Q] = _q_filt;
    out[StateIdx::R] = _r_filt;

    // Angular acceleration (differentiated + lowpass filtered) → out[16–18]
    out[StateIdx::P_DOT] = _pdot_filt;
    out[StateIdx::Q_DOT] = _qdot_filt;
    out[StateIdx::R_DOT] = _rdot_filt;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Individual state accessors
 * ══════════════════════════════════════════════════════════════════════════ */

float StateManager::roll() const
{
    const float* st = _lanes[_primary].state();
    Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
    float ro, pi, ya;
    quat_to_euler(q, ro, pi, ya);
    return ro;
}

float StateManager::pitch() const
{
    const float* st = _lanes[_primary].state();
    Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
    float ro, pi, ya;
    quat_to_euler(q, ro, pi, ya);
    return pi;
}

float StateManager::yaw() const
{
    const float* st = _lanes[_primary].state();
    Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
    float ro, pi, ya;
    quat_to_euler(q, ro, pi, ya);
    return ya;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Per-lane accessors (called by StateEstThread only)
 * ══════════════════════════════════════════════════════════════════════════ */

void StateManager::get_lane_euler(int lane, float& roll, float& pitch, float& yaw) const
{
    if (lane < 0 || lane >= NUM_LANES || !_lanes[lane].is_valid()) {
        roll = pitch = yaw = 0.0f;
        return;
    }
    const float* st = _lanes[lane].state();
    Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
    float ro, pi, ya;
    quat_to_euler(q, ro, pi, ya);
    roll  = ro;
    pitch = pi;
    yaw   = ya;
}

void StateManager::get_lane_pqr(int lane, float& p, float& q, float& r) const
{
    if (lane < 0 || lane >= NUM_LANES) { p = q = r = 0.0f; return; }
    p = _lane_p[lane];
    q = _lane_q[lane];
    r = _lane_r[lane];
}

