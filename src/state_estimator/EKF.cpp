#include "EKF.hpp"
#include "src/math/math.hpp"
#include <cmath>
#include <cstring>

/* ══════════════════════════════════════════════════════════════════════════
 * Construction / initialisation
 * ══════════════════════════════════════════════════════════════════════════ */

EKF::EKF()
    : _imu_idx(0), _initialized(false), _innov_norm(0.0f), _vibe_filt(0.0f)
{
    memset(_x, 0, sizeof(_x));
    memset(_P, 0, sizeof(_P));
    memset(_Q, 0, sizeof(_Q));
    memset(_F, 0, sizeof(_F));
    memset(_wA, 0, sizeof(_wA));
    memset(_wB, 0, sizeof(_wB));
}

void EKF::init(int imu_index)
{
    _imu_idx     = imu_index;
    _initialized = false;
    _innov_norm  = 0.0f;
    _vibe_filt   = 0.0f;

    memset(_x, 0, sizeof(_x));
    memset(_P, 0, sizeof(_P));
    memset(_Q, 0, sizeof(_Q));

    // Identity quaternion — level attitude
    _x[iQ0] = 1.0f;

    // Initial covariance — moderate uncertainty on all states
    for (int i = 0; i < N; ++i) {
        float p0;
        if      (i <= iZ)   p0 = P0_POS;
        else if (i <= iW)   p0 = P0_VEL;
        else if (i <= iQ3)  p0 = P0_QUAT;
        else if (i <= iBaz) p0 = P0_BIAS_A;
        else                p0 = P0_BIAS_G;
        _P[i][i] = p0;
    }

    // Process noise — diagonal
    for (int i = iX;   i <= iZ;   ++i) _Q[i][i] = Q_POS;
    for (int i = iU;   i <= iW;   ++i) _Q[i][i] = Q_VEL;
    for (int i = iQ0;  i <= iQ3;  ++i) _Q[i][i] = Q_QUAT;
    for (int i = iBax; i <= iBaz; ++i) _Q[i][i] = Q_BIAS_A;
    for (int i = iBgx; i <= iBgz; ++i) _Q[i][i] = Q_BIAS_G;

    _initialized = true;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Predict step
 * ══════════════════════════════════════════════════════════════════════════ */

void EKF::predict(float dt, const float accel[3], const float gyro[3])
{
    if (!_initialized) return;

    // ── Bias-corrected IMU inputs ─────────────────────────────────────────
    float accel_corr[3] = { accel[0]-_x[iBax], accel[1]-_x[iBay], accel[2]-_x[iBaz] };
    float gyro_corr[3]  = { gyro[0] -_x[iBgx], gyro[1] -_x[iBgy], gyro[2] -_x[iBgz] };

    // ── Build rotation matrix R_body2ned from current quaternion ──────────
    float R[3][3];
    quat_to_rot_body2ned(_get_quat(), R);

    // ── Gravity vector in body frame: g_body = R_b2n^T * [0, 0, g] ───────
    // NED z-down: g_NED = [0,0,+g]. g_body[i] = R_b2n[2][i] * g
    float g_body[3];
    g_body[0] = R[2][0] * GRAVITY;
    g_body[1] = R[2][1] * GRAVITY;
    g_body[2] = R[2][2] * GRAVITY;

    // ── Coriolis coupling: ω × v (body frame) ────────────────────────────
    const float vel[3] = { _x[iU], _x[iV], _x[iW] };
    float omega_x_v[3];
    cross3(gyro_corr, vel, omega_x_v);

    // NED z-down: sensor reads -g at hover, so a_true = accel + g_body - ω×v
    float a_true[3];
    for (int i = 0; i < 3; ++i)
        a_true[i] = accel_corr[i] + g_body[i] - omega_x_v[i];

    // ── Propagate position: X += R_b2n * v_body * dt ─────────────────────
    for (int i = 0; i < 3; ++i)
        _x[iX + i] += (R[i][0]*_x[iU] + R[i][1]*_x[iV] + R[i][2]*_x[iW]) * dt;

    // ── Propagate body velocity: v += a_true * dt ─────────────────────────
    for (int i = 0; i < 3; ++i)
        _x[iU + i] += a_true[i] * dt;

    // ── Propagate quaternion: q += 0.5 * q ⊗ {0, gx, gy, gz} * dt ─────────
    // First-order integration using bias-corrected gyro rates.
    Quat q_cur = _get_quat();
    Quat dq    = { 1.0f, 0.5f*gyro_corr[0]*dt, 0.5f*gyro_corr[1]*dt, 0.5f*gyro_corr[2]*dt };
    Quat q_new = quat_norm(quat_mul(q_cur, dq));
    _x[iQ0] = q_new.w;
    _x[iQ1] = q_new.x;
    _x[iQ2] = q_new.y;
    _x[iQ3] = q_new.z;

    // ── Covariance propagation: P = F*P*F^T + Q ──────────────────────────
    _build_F(dt, gyro_corr);
    mat_mul<N>(_F, _P, _wA);        // wA = F * P
    mat_trans<N>(_F, _wB);           // wB = F^T
    mat_mul<N>(_wA, _wB, _P);       // P  = F * P * F^T
    mat_add<N>(_P, _Q, _P);          // P += Q
}

/* ══════════════════════════════════════════════════════════════════════════
 * Build F Jacobian (discrete, F ≈ I + Fc*dt)
 * ══════════════════════════════════════════════════════════════════════════ */

void EKF::_build_F(float dt, const float gyro[3])
{
    memset(_F, 0, sizeof(_F));

    // Diagonal = identity
    for (int i = 0; i < N; ++i) _F[i][i] = 1.0f;

    // ── ∂Ẋ/∂(u,v,w) = R_b2n * dt ─────────────────────────────────────────
    float R[3][3];
    quat_to_rot_body2ned(_get_quat(), R);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            _F[iX + i][iU + j] = R[i][j] * dt;

    // ── ∂Ẋ/∂q — analytical partials of (R_b2n * v_body) w.r.t. quaternion ──
    const float u = _x[iU], v = _x[iV], w = _x[iW];
    const float qw = _x[iQ0], qx = _x[iQ1], qy = _x[iQ2], qz = _x[iQ3];

    // ∂(R_b2n * v)/∂qw  (column 0 of the 3×4 sub-block)
    _F[iX][iQ0] = dt * 2.0f*( qy*w - qz*v);
    _F[iY][iQ0] = dt * 2.0f*( qz*u - qx*w);
    _F[iZ][iQ0] = dt * 2.0f*( qx*v - qy*u);

    // ∂(R_b2n * v)/∂qx
    _F[iX][iQ1] = dt * 2.0f*( qy*v + qz*w);
    _F[iY][iQ1] = dt * 2.0f*( qy*u - 2.0f*qx*v - qw*w);
    _F[iZ][iQ1] = dt * 2.0f*( qz*u + qw*v - 2.0f*qx*w);

    // ∂(R_b2n * v)/∂qy
    _F[iX][iQ2] = dt * 2.0f*(-2.0f*qy*u + qx*v + qw*w);
    _F[iY][iQ2] = dt * 2.0f*( qx*u + qz*w);
    _F[iZ][iQ2] = dt * 2.0f*(-qw*u + qz*v - 2.0f*qy*w);

    // ∂(R_b2n * v)/∂qz
    _F[iX][iQ3] = dt * 2.0f*(-2.0f*qz*u - qw*v + qy*w);
    _F[iY][iQ3] = dt * 2.0f*( qw*u - 2.0f*qz*v + qy*w);
    _F[iZ][iQ3] = dt * 2.0f*( qx*u + qy*v);

    // ── ∂q̇/∂q — quaternion kinematics using bias-corrected gyro ─────────
    // dq/dt = 0.5 * Omega_mat * q   →  ∂q̇/∂q = 0.5 * Omega_mat * dt
    // Omega_mat (4×4) for gyro_corr = {gx, gy, gz}:
    //    [ 0  -gx  -gy  -gz ]
    //    [ gx   0   gz  -gy ]
    //    [ gy  -gz   0   gx ]
    //    [ gz   gy  -gx   0 ]
    const float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    const float h = 0.5f * dt;
    _F[iQ0][iQ1] += -gx*h; _F[iQ0][iQ2] += -gy*h; _F[iQ0][iQ3] += -gz*h;
    _F[iQ1][iQ0] +=  gx*h; _F[iQ1][iQ2] +=  gz*h;  _F[iQ1][iQ3] += -gy*h;
    _F[iQ2][iQ0] +=  gy*h; _F[iQ2][iQ1] += -gz*h;  _F[iQ2][iQ3] +=  gx*h;
    _F[iQ3][iQ0] +=  gz*h; _F[iQ3][iQ1] +=  gy*h;  _F[iQ3][iQ2] += -gx*h;

    // ── ∂(u̇,v̇,ẇ)/∂(ba_x,ba_y,ba_z) = -I * dt ───────────────────────────
    // v_dot uses accel_corr = accel - ba, so ∂v_dot/∂ba = -I
    for (int k = 0; k < 3; ++k) _F[iU + k][iBax + k] = -dt;

    // ── ∂q̇/∂(bg_x,bg_y,bg_z) = -0.5 * dt * Xi(q) ────────────────────────
    // gyro_corr = gyro - bg, so ∂(dq/dt)/∂bg = -0.5 * Xi(q)
    // Xi(q) = [[-qx,-qy,-qz], [qw,-qz, qy], [qz, qw,-qx], [-qy, qx, qw]]
    _F[iQ0][iBgx] =  h*qx;  _F[iQ0][iBgy] =  h*qy;  _F[iQ0][iBgz] =  h*qz;
    _F[iQ1][iBgx] = -h*qw;  _F[iQ1][iBgy] =  h*qz;  _F[iQ1][iBgz] = -h*qy;
    _F[iQ2][iBgx] = -h*qz;  _F[iQ2][iBgy] = -h*qw;  _F[iQ2][iBgz] =  h*qx;
    _F[iQ3][iBgx] =  h*qy;  _F[iQ3][iBgy] = -h*qx;  _F[iQ3][iBgz] = -h*qw;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Measurement updates
 * ══════════════════════════════════════════════════════════════════════════ */

void EKF::update_quaternion(const Quat& q_meas_in, float R_var)
{
    if (!_initialized) return;

    Quat q_pred = _get_quat();
    Quat q_meas = q_meas_in;

    // Antipodal fix: ensure shorter-arc path
    if (quat_dot(q_meas, q_pred) < 0.0f) {
        q_meas.w = -q_meas.w;
        q_meas.x = -q_meas.x;
        q_meas.y = -q_meas.y;
        q_meas.z = -q_meas.z;
    }

    // H (4×N): identity block at quaternion columns 6–9
    float H[4][N] = {};
    H[0][iQ0] = 1.0f;
    H[1][iQ1] = 1.0f;
    H[2][iQ2] = 1.0f;
    H[3][iQ3] = 1.0f;

    const float R_diag[4] = { R_var, R_var, R_var, R_var };
    const float innov[4]  = {
        q_meas.w - q_pred.w,
        q_meas.x - q_pred.x,
        q_meas.y - q_pred.y,
        q_meas.z - q_pred.z
    };

    // Update innovation norm (exponential smoothing, τ ≈ 10 updates).
    // Computed unconditionally, even on samples the gate below rejects —
    // a lane that's diverging enough to fail the gate should still score
    // worse in StateManager::_select_primary()'s lane-health comparison,
    // not have its disagreement hidden by skipping the update.
    float imag = innov[0]*innov[0] + innov[1]*innov[1]
               + innov[2]*innov[2] + innov[3]*innov[3];
    _innov_norm = INNOV_SMOOTH * _innov_norm + (1.0f - INNOV_SMOOTH) * imag;

    // ── Chi-squared innovation gate ────────────────────────────────────────
    // See QUAT_CHI2_GATE's comment. H is diagonal (row i observes iQ0+i
    // directly), so S_ii is just that state's P diagonal + R_var.
    {
        float innov_sq_sum = 0.0f;
        float S_sum        = 0.0f;
        for (int i = 0; i < 4; ++i) {
            S_sum        += _P[iQ0+i][iQ0+i] + R_var;
            innov_sq_sum += innov[i] * innov[i];
        }
        const float gate_sq = QUAT_CHI2_GATE * QUAT_CHI2_GATE;
        if (S_sum < 1e-10f || innov_sq_sum > S_sum * gate_sq) return;
    }

    _update(4, H, R_diag, innov);
    _normalize_quat();
}

void EKF::update_gravity(const float accel[3], float R_var)
{
    if (!_initialized) return;

    // ── Hard outer gate: reject clearly bad IMU samples (e.g. >3g) ────────
    float norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (norm > GRAV_HARD_GATE || norm < 0.1f) return;

    // ── Vibration filter: IIR estimate of (|a|-g)² ────────────────────────
    // Mirrors ArduPilot's velDotNEDfilt = velDotNED*α + filt*(1-α).
    // Time constant τ ≈ 1/GRAV_VIBE_ALPHA steps ≈ 40 updates = 0.1 s at 400 Hz.
    float dev = norm - GRAVITY;
    _vibe_filt = GRAV_VIBE_ALPHA * (dev * dev) + (1.0f - GRAV_VIBE_ALPHA) * _vibe_filt;

    // ── Adaptive measurement noise: R_eff = R_base + R_VIBE * vibe_rms² ───
    // Mirrors ArduPilot's sq(gpsNEVelVarAccScale * accNavMag) additive term.
    // Under heavy vibration _vibe_filt grows and the update is trusted less,
    // but is NOT blocked outright — only the chi-sq gate can fully reject it.
    float R_eff = R_var + GRAV_R_VIBE * _vibe_filt;

    // ── Direction-only tilt update ──────────────────────────────────────────
    // Mirrors ArduPilot DCM's drift_correction(): normalize the measured
    // specific-force vector to unit length before comparing it to the
    // (already-unit-length, by construction) predicted "down" direction, so
    // any deviation of |accel| from g — sensor scale error, or genuinely
    // just noise in the window average — contributes nothing to the tilt
    // correction. Bias is still subtracted from the measurement first (a
    // biased sensor's *direction* reading is still wrong), but this update's
    // Jacobian has no bias columns: the tilt correction depends only on
    // quaternion, never touches ba_x/ba_y/ba_z. NOTE: normalizing does NOT
    // by itself reject a genuine horizontal linear-acceleration disturbance
    // (e.g. sliding the vehicle on a bench) — that changes the *direction*
    // of specific force for real, which any gravity-vector tilt reference
    // (ArduPilot's included) will partially misread as tilt. That class of
    // error is instead handled by STATEMGR_GRAVACC_AVG_S (window-cancels
    // symmetric push/pull transients) and GRAV_MAX_CORR_RAD below (bounds
    // how much of it can land in one call) — see their comments.
    const float accel_corr[3] = {
        accel[0] - _x[iBax],
        accel[1] - _x[iBay],
        accel[2] - _x[iBaz]
    };
    const float accel_corr_norm = sqrtf(accel_corr[0]*accel_corr[0]
                                       + accel_corr[1]*accel_corr[1]
                                       + accel_corr[2]*accel_corr[2]);
    if (accel_corr_norm < 0.1f) return;  // guard divide-by-zero (hard gate above should prevent this)
    const float inv_an = 1.0f / accel_corr_norm;
    const float accel_dir[3] = {
        accel_corr[0] * inv_an, accel_corr[1] * inv_an, accel_corr[2] * inv_an
    };

    float R_rot[3][3];
    quat_to_rot_body2ned(_get_quat(), R_rot);
    // NED z-down: predicted sensor reading direction is -g_body/|g_body|,
    // and R_rot's rows are already unit vectors, so no division needed.
    const float g_dir_pred[3] = { -R_rot[2][0], -R_rot[2][1], -R_rot[2][2] };

    const float innov_dir[3] = {
        accel_dir[0] - g_dir_pred[0],
        accel_dir[1] - g_dir_pred[1],
        accel_dir[2] - g_dir_pred[2]
    };

    float H[3][N] = {};
    const float qw = _x[iQ0], qx = _x[iQ1], qy = _x[iQ2], qz = _x[iQ3];

    // ∂(-g_body/g)/∂q — same partials as before, minus the *g factor now
    // that the predicted vector is unit-length rather than magnitude-g.
    H[0][iQ0] =  2*qy;  H[0][iQ1] = -2*qz;  H[0][iQ2] =  2*qw;  H[0][iQ3] = -2*qx;
    H[1][iQ0] = -2*qx;  H[1][iQ1] = -2*qw;  H[1][iQ2] = -2*qz;  H[1][iQ3] = -2*qy;
    H[2][iQ0] =  0.0f;  H[2][iQ1] =  4*qx;  H[2][iQ2] =  4*qy;  H[2][iQ3] =  0.0f;

    // R for this dimensionless (unit-vector) innovation space. R_eff below is
    // in (m/s²)²; first-order, d(v/|v|) ≈ dv/|v| when |v| ≈ GRAVITY, so an
    // equivalent unit-vector-space variance is R_eff/GRAVITY².
    const float R_dir = R_eff / (GRAVITY * GRAVITY);

    // ── Chi-squared innovation gate ────────────────────────────────────────
    // Mirrors ArduPilot's velTestRatio pattern:
    //   test_ratio = sum(innov²) / (sum(S_ii) * gate²)  < 1.0
    // S_ii = H[row] * P * H[row]^T + R_dir. H is sparse (nonzeros only at
    // {iQ0..iQ3} now — no bias columns) so only 4×4 inner products are
    // needed per row.
    {
        float innov_sq_sum = 0.0f;
        float S_sum        = 0.0f;
        const int nz[4] = { iQ0, iQ1, iQ2, iQ3 };

        for (int row = 0; row < 3; ++row) {
            float S_ii = R_dir;
            for (int a = 0; a < 4; ++a) {
                float ph = 0.0f;
                for (int b = 0; b < 4; ++b)
                    ph += _P[nz[a]][nz[b]] * H[row][nz[b]];
                S_ii += H[row][nz[a]] * ph;
            }

            innov_sq_sum += innov_dir[row] * innov_dir[row];
            S_sum        += S_ii;
        }

        // gate²: with GRAV_CHI2_GATE=5 this is a 5σ joint gate
        const float gate_sq = GRAV_CHI2_GATE * GRAV_CHI2_GATE;
        if (S_sum < 1e-10f || innov_sq_sum > S_sum * gate_sq) return;
    }

    // ── Rate-limit the resulting attitude correction ──────────────────────
    // Mirrors ArduPilot DCM's rate-limited _kp correction (see
    // GRAV_MAX_CORR_RAD comment in EKF.hpp) rather than letting a single
    // accepted-but-large window average snap the quaternion in one shot.
    const Quat q_before = _get_quat();

    const float R_diag[3] = { R_dir, R_dir, R_dir };
    _update(3, H, R_diag, innov_dir);
    _normalize_quat();

    Quat q_after = _get_quat();
    float dot = q_before.w*q_after.w + q_before.x*q_after.x
              + q_before.y*q_after.y + q_before.z*q_after.z;
    if (dot < 0.0f) {
        // Antipodal fix — stay on the shorter rotation path.
        q_after.w = -q_after.w; q_after.x = -q_after.x;
        q_after.y = -q_after.y; q_after.z = -q_after.z;
        dot = -dot;
    }
    dot = constrain_float(dot, -1.0f, 1.0f);
    const float angle = 2.0f * acosf(dot);

    if (angle > GRAV_MAX_CORR_RAD) {
        // nlerp back toward q_before so the applied rotation is capped at
        // GRAV_MAX_CORR_RAD. A true slerp would be exact; nlerp is a fine
        // approximation at these small angles (a few degrees at most) and
        // avoids extra trig on a per-window (not per-tick) call anyway.
        const float t = GRAV_MAX_CORR_RAD / angle;
        Quat q_clamped = {
            q_before.w*(1.0f-t) + q_after.w*t,
            q_before.x*(1.0f-t) + q_after.x*t,
            q_before.y*(1.0f-t) + q_after.y*t,
            q_before.z*(1.0f-t) + q_after.z*t
        };
        q_clamped = quat_norm(q_clamped);
        _x[iQ0] = q_clamped.w;
        _x[iQ1] = q_clamped.x;
        _x[iQ2] = q_clamped.y;
        _x[iQ3] = q_clamped.z;
    }

    // ── Separate scalar Z accel-bias trim ──────────────────────────────────
    // Split out from the tilt update above (which is now direction-only and
    // has no bias columns) so bias estimation and attitude correction can't
    // cross-contaminate each other. Z is still the only axis used for bias:
    // body-frame gravity Z-component is -g at any known attitude regardless
    // of horizontal motion, so a Z residual (after re-deriving the predicted
    // attitude post-tilt-update/clamp above) is attributable to bias, not
    // tilt — see the original rationale this preserves. H has no quaternion
    // columns, so this update cannot itself perturb attitude.
    {
        float R_rot2[3][3];
        quat_to_rot_body2ned(_get_quat(), R_rot2);
        const float g_pred_z = -R_rot2[2][2] * GRAVITY;
        const float innov_z  = accel[2] - g_pred_z - _x[iBaz];

        const float S = _P[iBaz][iBaz] + R_eff;
        const float gate_sq = GRAV_CHI2_GATE * GRAV_CHI2_GATE;
        if (!(S < 1e-10f || innov_z * innov_z > S * gate_sq)) {
            float H2[1][N] = {};
            H2[0][iBaz] = 1.0f;
            const float R_diag_z[1]    = { R_eff };
            const float innov_z_arr[1] = { innov_z };
            _update(1, H2, R_diag_z, innov_z_arr);
        }
    }
}

void EKF::update_position(const float xyz[3], float R_var)
{
    if (!_initialized) return;

    const float innov[3] = { xyz[0]-_x[iX], xyz[1]-_x[iY], xyz[2]-_x[iZ] };

    // ── Chi-squared innovation gate ────────────────────────────────────────
    // H is diagonal (row i observes state iX+i directly), so S_ii is just
    // that state's P diagonal + R — no matrix product needed. Rejects stale
    // or corrupted mocap position samples instead of snapping X/Y/Z to them.
    {
        float innov_sq_sum = 0.0f;
        float S_sum        = 0.0f;
        for (int i = 0; i < 3; ++i) {
            S_sum        += _P[iX+i][iX+i] + R_var;
            innov_sq_sum += innov[i] * innov[i];
        }
        const float gate_sq = MOCAP_CHI2_GATE * MOCAP_CHI2_GATE;
        if (S_sum < 1e-10f || innov_sq_sum > S_sum * gate_sq) return;
    }

    float H[3][N] = {};
    H[0][iX] = 1.0f;
    H[1][iY] = 1.0f;
    H[2][iZ] = 1.0f;

    const float R_diag[3] = { R_var, R_var, R_var };
    _update(3, H, R_diag, innov);
}

void EKF::update_altitude(float alt_up_m, float R_var)
{
    if (!_initialized) return;

    // ── NED sign convention ────────────────────────────────────────────────
    // EKF iZ is NED (down-positive). Barometric height above the boot-time
    // reference is up-positive (see MS5611::init warm-up capture). Explicit
    // flip here, matching update_gravity()'s own explicit NED sign handling
    // (its g_pred[] negation above) rather than pushing the sign convention
    // onto callers.
    const float z_meas = -alt_up_m;
    const float innov   = z_meas - _x[iZ];

    // ── Chi-squared innovation gate ────────────────────────────────────────
    // Single scalar observation of iZ directly (H is one-hot), so S is just
    // that state's P diagonal + R — same simplification as update_position's
    // per-axis gate. Guards against a baro glitch (prop wash / gust pressure
    // transient / SPI corruption) snapping Z and, via cross-covariance,
    // corrupting W / accel bias from a single bad sample.
    {
        const float S = _P[iZ][iZ] + R_var;
        const float gate_sq = BARO_CHI2_GATE * BARO_CHI2_GATE;
        if (S < 1e-10f || innov * innov > S * gate_sq) return;
    }

    float H[1][N] = {};
    H[0][iZ] = 1.0f;
    const float R_diag[1]    = { R_var };
    const float innov_arr[1] = { innov };
    _update(1, H, R_diag, innov_arr);
}

void EKF::update_ned_vel(const float vel_ned[3], float R_var)
{
    if (!_initialized) return;

    // Rotate NED velocity into body frame: v_body = R_b2n^T * v_ned
    float R[3][3];
    quat_to_rot_body2ned(_get_quat(), R);
    float v_body[3] = {
        R[0][0]*vel_ned[0] + R[1][0]*vel_ned[1] + R[2][0]*vel_ned[2],
        R[0][1]*vel_ned[0] + R[1][1]*vel_ned[1] + R[2][1]*vel_ned[2],
        R[0][2]*vel_ned[0] + R[1][2]*vel_ned[1] + R[2][2]*vel_ned[2]
    };

    const float innov[3] = { v_body[0]-_x[iU], v_body[1]-_x[iV], v_body[2]-_x[iW] };

    // ── Chi-squared innovation gate ────────────────────────────────────────
    // H is diagonal (row i observes state iU+i directly), so S_ii is just
    // that state's P diagonal + R. This is the primary guard against a
    // stale/corrupted mocap velocity sample snapping u/v (see the has_new_vel
    // split in MocapRaw — this gate is the second line of defense).
    {
        float innov_sq_sum = 0.0f;
        float S_sum        = 0.0f;
        for (int i = 0; i < 3; ++i) {
            S_sum        += _P[iU+i][iU+i] + R_var;
            innov_sq_sum += innov[i] * innov[i];
        }
        const float gate_sq = MOCAP_CHI2_GATE * MOCAP_CHI2_GATE;
        if (S_sum < 1e-10f || innov_sq_sum > S_sum * gate_sq) return;
    }

    float H[3][N] = {};
    H[0][iU] = 1.0f;
    H[1][iV] = 1.0f;
    H[2][iW] = 1.0f;

    const float R_diag[3] = { R_var, R_var, R_var };
    _update(3, H, R_diag, innov);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Generic measurement update: m ≤ 6 measurements.
 *
 * Algorithm (matching ArduPilot EKF3 FinishFusion structure):
 *   1. K  = P*H^T * (H*P*H^T + R)^-1
 *   2. x += K * innov
 *   3. Joseph form:  P = (I-KH)*P*(I-KH)^T + K*R*K^T
 *      Numerically positive-definite by construction — eliminates the
 *      covariance collapse seen under the previous (I-KH)*P form.
 *   4. Symmetrize:   P[i][j] = P[j][i] = 0.5*(P[i][j]+P[j][i])
 *      Mirrors ArduPilot FinishFusion triangle-averaging.
 *   5. Clamp diagonal (ConstrainVariances equivalent).
 *
 * Scratch memory: _wA and _wB (N×N EKF member arrays, not stack-allocated)
 * are reused here — they are only written by predict(), which is never
 * called re-entrantly with _update().
 *   _wA ← IKH = I - K*H
 *   _wB ← IKH * P   (= (I-KH)*P, before the right multiply)
 * ══════════════════════════════════════════════════════════════════════════ */

void EKF::_update(int m, const float H[][N], const float R_diag[], const float innov[])
{
    const int M_MAX = 6;

    // ── PHt = P * H^T  (N×m, on stack — 384 bytes) ───────────────────────
    float PHt[N][M_MAX] = {};
    for (int j = 0; j < m; ++j)
        for (int i = 0; i < N; ++i)
            for (int k = 0; k < N; ++k)
                PHt[i][j] += _P[i][k] * H[j][k];

    // ── S = H*PHt + diag(R)  (m×m, on stack) ─────────────────────────────
    float S[M_MAX][M_MAX] = {};
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < m; ++j) {
            for (int k = 0; k < N; ++k) S[i][j] += H[i][k] * PHt[k][j];
            if (i == j) S[i][j] += R_diag[i];
        }

    // ── S^-1 via Gauss-Jordan (pivoted) ───────────────────────────────────
    float Sinv[M_MAX][M_MAX] = {};
    float Sw  [M_MAX][M_MAX];
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) Sw[i][j] = S[i][j];
        Sinv[i][i] = 1.0f;
    }
    for (int col = 0; col < m; ++col) {
        int piv = col;
        for (int row = col + 1; row < m; ++row)
            if (fabsf(Sw[row][col]) > fabsf(Sw[piv][col])) piv = row;
        for (int j = 0; j < m; ++j) {
            float t = Sw[col][j];   Sw[col][j]   = Sw[piv][j];   Sw[piv][j]   = t;
                  t = Sinv[col][j]; Sinv[col][j] = Sinv[piv][j]; Sinv[piv][j] = t;
        }
        float diag = Sw[col][col];
        if (fabsf(diag) < 1e-12f) return;  // singular — skip update
        float inv_d = 1.0f / diag;
        for (int j = 0; j < m; ++j) { Sw[col][j] *= inv_d; Sinv[col][j] *= inv_d; }
        for (int row = 0; row < m; ++row) {
            if (row == col) continue;
            float fac = Sw[row][col];
            for (int j = 0; j < m; ++j) {
                Sw[row][j]   -= fac * Sw[col][j];
                Sinv[row][j] -= fac * Sinv[col][j];
            }
        }
    }

    // ── K = PHt * S^-1  (N×m) ─────────────────────────────────────────────
    float K[N][M_MAX] = {};
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < m; ++j)
            for (int k = 0; k < m; ++k)
                K[i][j] += PHt[i][k] * Sinv[k][j];

    // ── x += K * innov ────────────────────────────────────────────────────
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < m; ++j)
            _x[i] += K[i][j] * innov[j];

    // ── Joseph form: P = (I-KH) * P * (I-KH)^T  +  K * diag(R) * K^T ────
    // _wA = IKH = I - K*H
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
            _wA[i][j] = (i == j) ? 1.0f : 0.0f;
            for (int k = 0; k < m; ++k)
                _wA[i][j] -= K[i][k] * H[k][j];
        }

    // _wB = IKH * P  (so final product is _wB * IKH^T without a temp array)
    mat_mul<N>(_wA, _P, _wB);

    // _P = _wB * IKH^T  =  _wB[i][k] * _wA[j][k]
    memset(_P, 0, sizeof(_P));
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            for (int k = 0; k < N; ++k)
                _P[i][j] += _wB[i][k] * _wA[j][k];

    // _P += K * diag(R) * K^T  (positive-definite term that regularises P)
    for (int k = 0; k < m; ++k)
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                _P[i][j] += K[i][k] * R_diag[k] * K[j][k];

    // ── Symmetrize: P = 0.5*(P + P^T) ────────────────────────────────────
    // Mirrors ArduPilot FinishFusion: "P must end up symmetric, so average
    // the upper and lower differences, then store in both positions."
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < i; ++j) {
            float avg  = 0.5f * (_P[i][j] + _P[j][i]);
            _P[i][j]   = avg;
            _P[j][i]   = avg;
        }

    // ── ConstrainVariances: clamp diagonal ────────────────────────────────
    // Mirrors ArduPilot's ConstrainVariances() floor/ceiling per state group.
    for (int i = iQ0;  i <= iQ3;  ++i)
        if (_P[i][i] < P_MIN_QUAT) _P[i][i] = P_MIN_QUAT;
    for (int i = iBax; i <= iBaz; ++i) {
        if (_P[i][i] < P_MIN_BIAS_A) _P[i][i] = P_MIN_BIAS_A;
        if (_P[i][i] > P_MAX_BIAS_A) _P[i][i] = P_MAX_BIAS_A;
    }
    for (int i = iBgx; i <= iBgz; ++i) {
        if (_P[i][i] < P_MIN_BIAS_G) _P[i][i] = P_MIN_BIAS_G;
        if (_P[i][i] > P_MAX_BIAS_G) _P[i][i] = P_MAX_BIAS_G;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * Internal helpers
 * ══════════════════════════════════════════════════════════════════════════ */

void EKF::_normalize_quat()
{
    Quat q = quat_norm(_get_quat());
    _x[iQ0] = q.w;
    _x[iQ1] = q.x;
    _x[iQ2] = q.y;
    _x[iQ3] = q.z;
}

Quat EKF::_get_quat() const
{
    return { _x[iQ0], _x[iQ1], _x[iQ2], _x[iQ3] };
}

