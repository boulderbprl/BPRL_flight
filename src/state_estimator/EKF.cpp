#include "EKF.hpp"
#include "src/math/math.hpp"
#include <cmath>
#include <cstring>

/* ══════════════════════════════════════════════════════════════════════════
 * Construction / initialisation
 * ══════════════════════════════════════════════════════════════════════════ */

EKF::EKF()
    : _imu_idx(0), _initialized(false), _innov_norm(0.0f)
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
    // R_ned2body = R_b2n^T, so g_body[i] = R[j][i] * g_ned[j] = R[2][i] * g
    float g_body[3];
    g_body[0] = R[2][0] * GRAVITY;
    g_body[1] = R[2][1] * GRAVITY;
    g_body[2] = R[2][2] * GRAVITY;

    // ── Coriolis coupling: ω × v (body frame) ────────────────────────────
    const float vel[3] = { _x[iU], _x[iV], _x[iW] };
    float omega_x_v[3];
    cross3(gyro_corr, vel, omega_x_v);

    // ── Gravity-corrected, Coriolis-corrected body acceleration ───────────
    float a_true[3];
    for (int i = 0; i < 3; ++i)
        a_true[i] = accel_corr[i] - g_body[i] - omega_x_v[i];

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

    // Update innovation norm (exponential smoothing, τ ≈ 10 updates)
    float imag = innov[0]*innov[0] + innov[1]*innov[1]
               + innov[2]*innov[2] + innov[3]*innov[3];
    _innov_norm = INNOV_SMOOTH * _innov_norm + (1.0f - INNOV_SMOOTH) * imag;

    _update(4, H, R_diag, innov);
    _normalize_quat();
}

void EKF::update_gravity(const float accel[3], float R_var)
{
    if (!_initialized) return;

    // Gate: only correct attitude when vehicle is near-hover (|a| ~ g)
    float norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (fabsf(norm - GRAVITY) > GRAV_GATE_MS2) return;

    // Predicted gravity in body frame from current quaternion
    float R[3][3];
    quat_to_rot_body2ned(_get_quat(), R);
    const float g_pred[3] = { R[2][0]*GRAVITY, R[2][1]*GRAVITY, R[2][2]*GRAVITY };

    // Innovation: raw accel minus predicted (g_body + accel_bias)
    const float innov[3] = {
        accel[0] - g_pred[0] - _x[iBax],
        accel[1] - g_pred[1] - _x[iBay],
        accel[2] - g_pred[2] - _x[iBaz]
    };

    // H: 3×N — ∂g_body/∂q block + identity at accel bias columns
    float H[3][N] = {};
    const float g = GRAVITY;
    const float qw = _x[iQ0], qx = _x[iQ1], qy = _x[iQ2], qz = _x[iQ3];

    H[0][iQ0] = -2*qy*g;  H[0][iQ1] =  2*qz*g;  H[0][iQ2] = -2*qw*g;  H[0][iQ3] =  2*qx*g;
    H[1][iQ0] =  2*qx*g;  H[1][iQ1] =  2*qw*g;  H[1][iQ2] =  2*qz*g;  H[1][iQ3] =  2*qy*g;
    H[2][iQ0] =  0.0f;    H[2][iQ1] = -4*qx*g;  H[2][iQ2] = -4*qy*g;  H[2][iQ3] =  0.0f;

    H[0][iBax] = 1.0f;
    H[1][iBay] = 1.0f;
    H[2][iBaz] = 1.0f;

    const float R_diag[3] = { R_var, R_var, R_var };
    _update(3, H, R_diag, innov);
    _normalize_quat();
}

void EKF::update_position(const float xyz[3], float R_var)
{
    if (!_initialized) return;

    float H[3][N] = {};
    H[0][iX] = 1.0f;
    H[1][iY] = 1.0f;
    H[2][iZ] = 1.0f;

    const float R_diag[3] = { R_var, R_var, R_var };
    const float innov[3]  = { xyz[0]-_x[iX], xyz[1]-_x[iY], xyz[2]-_x[iZ] };
    _update(3, H, R_diag, innov);
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

    float H[3][N] = {};
    H[0][iU] = 1.0f;
    H[1][iV] = 1.0f;
    H[2][iW] = 1.0f;

    const float R_diag[3] = { R_var, R_var, R_var };
    const float innov[3]  = { v_body[0]-_x[iU], v_body[1]-_x[iV], v_body[2]-_x[iW] };
    _update(3, H, R_diag, innov);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Generic measurement update: m ≤ 6 measurements
 *
 * Computes: S = H*P*H^T + R  (m×m)
 *           K = P*H^T * S^-1 (N×m)
 *           x += K * innov
 *           P  = (I - K*H) * P
 * ══════════════════════════════════════════════════════════════════════════ */

void EKF::_update(int m, const float H[][N], const float R_diag[], const float innov[])
{
    const int M_MAX = 6;

    // ── S = H*P*H^T + R  (m×m on stack) ──────────────────────────────────
    float PHt[N][M_MAX] = {};   // P*H^T,  N×m
    float S   [M_MAX][M_MAX] = {};  // m×m

    for (int j = 0; j < m; ++j)          // column of H^T = row of H
        for (int i = 0; i < N; ++i)
            for (int k = 0; k < N; ++k)
                PHt[i][j] += _P[i][k] * H[j][k];  // P * H^T

    for (int i = 0; i < m; ++i)
        for (int j = 0; j < m; ++j) {
            for (int k = 0; k < N; ++k)
                S[i][j] += H[i][k] * PHt[k][j];
            if (i == j) S[i][j] += R_diag[i];
        }

    // ── S^-1 via Gauss-Jordan on augmented [S | I] ───────────────────────
    float Sinv[M_MAX][M_MAX] = {};
    // Copy S into working buffer and set Sinv = I
    float Sw[M_MAX][M_MAX];
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) Sw[i][j] = S[i][j];
        Sinv[i][i] = 1.0f;
    }
    // Forward elimination
    for (int col = 0; col < m; ++col) {
        // Pivot: find max row
        int piv = col;
        for (int row = col + 1; row < m; ++row)
            if (fabsf(Sw[row][col]) > fabsf(Sw[piv][col])) piv = row;
        // Swap rows
        for (int j = 0; j < m; ++j) {
            float t = Sw[col][j]; Sw[col][j] = Sw[piv][j]; Sw[piv][j] = t;
            t = Sinv[col][j]; Sinv[col][j] = Sinv[piv][j]; Sinv[piv][j] = t;
        }
        float diag = Sw[col][col];
        if (fabsf(diag) < 1e-12f) return;  // singular — skip update
        float inv_diag = 1.0f / diag;
        for (int j = 0; j < m; ++j) { Sw[col][j] *= inv_diag; Sinv[col][j] *= inv_diag; }
        for (int row = 0; row < m; ++row) {
            if (row == col) continue;
            float fac = Sw[row][col];
            for (int j = 0; j < m; ++j) {
                Sw[row][j]   -= fac * Sw[col][j];
                Sinv[row][j] -= fac * Sinv[col][j];
            }
        }
    }

    // ── K = P*H^T * S^-1  (N×m) ──────────────────────────────────────────
    float K[N][M_MAX] = {};
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < m; ++j)
            for (int k = 0; k < m; ++k)
                K[i][j] += PHt[i][k] * Sinv[k][j];

    // ── x += K * innov ────────────────────────────────────────────────────
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < m; ++j)
            _x[i] += K[i][j] * innov[j];

    // ── P = (I - K*H) * P ─────────────────────────────────────────────────
    // Compute KH (N×N), then P = P - KH*P
    float KH[N][N] = {};
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            for (int k = 0; k < m; ++k)
                KH[i][j] += K[i][k] * H[k][j];

    // Temporary copy of P for the subtraction
    float Ptmp[N][N];
    memcpy(Ptmp, _P, sizeof(_P));
    memset(_P, 0, sizeof(_P));
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N; ++k)
                s += KH[i][k] * Ptmp[k][j];
            _P[i][j] = Ptmp[i][j] - s;
        }
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

