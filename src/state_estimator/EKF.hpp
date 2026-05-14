#pragma once
#include "src/math/math.hpp"

/*
 * EKF — single-lane 16-state Extended Kalman Filter.
 *
 * One instance runs per onboard IMU. StateManager owns three instances,
 * selects the primary lane by innovation health, and dispatches shared
 * IMX5 quaternion updates to all lanes (IMX5 is optional — the filter
 * runs from onboard IMUs alone when the IMX5 is disconnected).
 *
 * State vector (0-indexed):
 *   [0-2]   X, Y, Z           — position, inertial NED (m)
 *   [3-5]   u, v, w           — velocity, body frame (m/s)
 *   [6-9]   q0,q1,q2,q3      — quaternion NED→Body [W,X,Y,Z] Hamilton
 *   [10-12] ba_x, ba_y, ba_z — accelerometer bias (m/s²), random walk
 *   [13-15] bg_x, bg_y, bg_z — gyroscope bias (rad/s), random walk
 *
 * p/q/r, u_dot/v_dot/w_dot, and p_dot/q_dot/r_dot are NOT Kalman states;
 * StateManager computes them from bias-corrected gyros and IMU accel and
 * inserts them when assembling the 19-element StateIdx output vector.
 *
 * Quaternion convention: scalar-first [W, X, Y, Z], Hamilton,
 * representing rotation from NED frame to Body frame.
 * Kinematic equation: dq/dt = 0.5 * q ⊗ {0, p-bg_x, q-bg_y, r-bg_z}
 */
class EKF {
public:
    // Internal Kalman state dimension — 16.
    static constexpr int N = 16;

    EKF();

    // Must be called once before predict()/update_*().
    // imu_index: which g_imu[i] slot this lane reads from (0–2).
    void init(int imu_index);

    // Prediction step — call at 500 Hz.
    // accel: body-frame specific force (m/s²), gyro: body-frame rates (rad/s).
    void predict(float dt, const float accel[3], const float gyro[3]);

    // Measurement updates — called asynchronously when each sensor delivers data.

    // IMX5 quaternion at 200 Hz.  R_var: measurement noise variance per component.
    void update_quaternion(const Quat& q_meas, float R_var);

    // Gravity-vector attitude + accel-bias update — gated on |accel| ≈ g.
    // accel: raw body-frame accelerometer reading (m/s²), NOT bias-corrected.
    void update_gravity(const float accel[3], float R_var);

    // Mocap NED position fusion — active when g_mocap.valid.
    void update_position(const float xyz[3], float R_var);
    void update_ned_vel (const float vel[3], float R_var);

    // Accessors
    const float* state()          const { return _x; }
    float        innovation_norm() const { return _innov_norm; }
    bool         is_valid()        const { return _initialized; }

private:
    // ── State index aliases ────────────────────────────────────────────────
    enum {
        iX=0,  iY=1,  iZ=2,
        iU=3,  iV=4,  iW=5,
        iQ0=6, iQ1=7, iQ2=8, iQ3=9,
        iBax=10, iBay=11, iBaz=12,   // accelerometer bias
        iBgx=13, iBgy=14, iBgz=15    // gyroscope bias
        // p/q/r, u_dot/v_dot/w_dot, p_dot/q_dot/r_dot — NOT Kalman states, see StateManager.
    };

    // ── Tuning parameters — all magic numbers live here ───────────────────

    // Physical
    static constexpr float GRAVITY       = 9.80665f;  // m/s²

    // Initial error covariance diagonals (P0)
    static constexpr float P0_POS  = 1.0f;   // position (m²)
    static constexpr float P0_VEL  = 0.5f;   // body velocity (m/s)²
    static constexpr float P0_QUAT = 0.1f;   // quaternion (unitless)

    // Initial error covariance for bias states
    static constexpr float P0_BIAS_A = 0.1f;   // accel bias initial variance (m/s²)²
    static constexpr float P0_BIAS_G = 0.01f;  // gyro bias initial variance (rad/s)²

    // Process noise diagonals (Q) — increase to trust IMU more, decrease to smooth more
    static constexpr float Q_POS    = 1e-4f;  // position (m²/s)
    static constexpr float Q_VEL    = 1e-3f;  // body velocity (m/s)²/s
    static constexpr float Q_QUAT   = 1e-5f;  // quaternion (unitless)/s
    static constexpr float Q_BIAS_A = 1e-6f;  // accel bias random walk — slow thermal drift
    static constexpr float Q_BIAS_G = 1e-7f;  // gyro bias random walk — even slower

    // Gravity-vector update gate — reject update if ||accel|| deviates from g by more than this
    static constexpr float GRAV_GATE_MS2 = 1.0f;  // m/s²

    // Innovation norm exponential smoothing — higher = slower response to changes
    // τ ≈ 1/(1 - INNOV_SMOOTH) updates  (0.9 → ~10-update time constant)
    static constexpr float INNOV_SMOOTH = 0.9f;

    // ── Filter state ──────────────────────────────────────────────────────
    int   _imu_idx;
    bool  _initialized;
    float _innov_norm;    // exponentially-smoothed innovation magnitude

    float _x[N];          // state vector
    float _P[N][N];       // error covariance
    float _Q[N][N];       // process noise covariance (diagonal, set in init)

    // ── Scratch matrices (members to keep them off the thread stack) ───────
    float _F[N][N];
    float _wA[N][N];
    float _wB[N][N];

    // ── Internal helpers ───────────────────────────────────────────────────
    void _normalize_quat();
    Quat _get_quat() const;
    void _build_F(float dt, const float gyro[3]);

    // Generic EKF measurement update for m ≤ 6 scalar measurements.
    // H[m][N]: observation matrix, R_diag[m]: per-channel noise variances,
    // innov[m]: pre-computed innovations (z - h(x)).
    void _update(int m, const float H[][N], const float R_diag[], const float innov[]);

};
