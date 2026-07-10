#pragma once
#include "EKF.hpp"
#include "src/FlightState.hpp"
#include "src/threads.hpp"   // IMURaw, CANIMURaw, MocapRaw

// Lowpass cutoff frequencies for derived derivative states (Hz)
#define STATEMGR_LP_UVWDOT_HZ     30.0f   // cutoff for u_dot/v_dot/w_dot
#define STATEMGR_LP_PQRDOT_HZ     20.0f   // cutoff for p_dot/q_dot/r_dot
#define STATEMGR_LP_PQ_HZ         20.0f   // cutoff for blended p/q (roll/pitch) fed to the rate PID
#define STATEMGR_LP_R_HZ           5.0f   // cutoff for blended r (yaw) fed to the rate PID 
// IMX5 angular rate blend weight: 0=pure onboard gyros, 1=pure IMX5
#define STATEMGR_IMX5_RATE_WEIGHT  0.3f

/*
 * StateManager — multi-lane EKF orchestrator.
 *
 * Runs three EKF lanes (N=13 states each), one per onboard IMU. Each lane
 * receives its own IMU's predict step; all lanes share the same IMX5
 * measurement updates.
 *
 * Quaternion output: hard-selected from the primary lane (lowest smoothed
 * innovation norm among valid lanes) — no blending to avoid antipodal issues.
 *
 * X/Y/Z and U/V/W output: soft-blended across all valid lanes weighted by
 * 1/innovation_norm, same as p/q/r below. Plain vectors have no antipodal
 * issue, so there is no reason to hard-select these from a single lane —
 * doing so let the raw output jump discontinuously every time _select_primary()
 * picked a different lane, since independent per-IMU integration drift means
 * lanes' u/v estimates diverge from each other even under identical motion.
 *
 * p/q/r output: soft-blended across all valid lanes weighted by
 * 1/innovation_norm, giving partial noise averaging with fault isolation.
 *
 * u_dot/v_dot/w_dot: gravity+Coriolis-corrected IMU accel, blended by the
 * same innovation-norm weights, then lowpass filtered at STATEMGR_LP_UVWDOT_HZ.
 *
 * p_dot/q_dot/r_dot: differentiated from blended p/q/r, lowpass filtered at
 * STATEMGR_LP_PQRDOT_HZ.
 *
 * Assembles the full 19-element StateIdx state vector for g_state[].
 */
class StateManager {
public:
    static constexpr int NUM_LANES = 3;

    // IMX5 / mocap / gravity measurement noise variances — tunable.
    static constexpr float R_QUAT      = 1e-3f;   // IMX5 quaternion component variance
    static constexpr float R_GRAVITY   = 0.5f;    // accel gravity-vector variance (m/s²)²
    static constexpr float R_MOCAP_POS = 1e-3f;   // mocap NED position variance (m²)
    static constexpr float R_MOCAP_VEL = 1e-2f;   // mocap NED velocity variance (m/s)²
    static constexpr float R_BARO_POS  = 0.5f;    // baro altitude variance (m²) — tune from bench log noise

    StateManager();

    void init();

    // Call once per StateEstThread tick (500 Hz).
    // dt: loop period in seconds.
    // imu: snapshot of g_imu[3] (taken under imu_mtx before this call).
    // can_imu: snapshot of g_can_imu (taken under can_imu_mtx before this call).
    // mocap: snapshot of g_mocap (taken under mocap_mtx before this call).
    // baro: snapshot of g_baro (taken under baro_mtx before this call).
    void update(float dt, const IMURaw imu[3], const CANIMURaw& can_imu, const MocapRaw& mocap, const BaroRaw& baro);

    // Full 19-element state output — maps 13-state EKF lanes onto StateIdx ordering
    // and fills in the 6 derived quantities (uvw_dot, pqr_dot).
    void get_state(float out[StateIdx::N]) const;

    // Derived Euler angles (from primary lane quaternion).
    float roll()    const;
    float pitch()   const;
    float yaw()     const;

    // Per-lane accessors — called by StateEstThread only (no mutex needed).
    void get_lane_euler(int lane, float& roll, float& pitch, float& yaw) const;
    void get_lane_pqr  (int lane, float& p,    float& q,    float& r)    const;
    int  primary_lane  () const { return _primary; }

private:
    EKF  _lanes[NUM_LANES];
    int  _primary;
    bool _initialized;

    // Soft-blended angular rates (weighted by 1/innovation_norm across valid lanes)
    float _blended_p, _blended_q, _blended_r;

    // Soft-blended position/velocity (same weights as _blended_p/q/r)
    float _blended_x, _blended_y, _blended_z;
    float _blended_u, _blended_v, _blended_w;

    // Body acceleration: blended gravity+Coriolis-corrected IMU accel
    float _blended_ud, _blended_vd, _blended_wd;

    // Angular acceleration via differentiation of blended rates + lowpass
    float _prev_p,    _prev_q,    _prev_r;
    float _pdot_filt, _qdot_filt, _rdot_filt;

    // Lowpass-filtered p/q/r fed to the rate PID (vibration rejection)
    float _p_filt, _q_filt, _r_filt;

    // Lowpass-filtered uvw_dot output
    float _ud_filt, _vd_filt, _wd_filt;

    // Per-lane bias-corrected angular rates (updated each update() call)
    float _lane_p[NUM_LANES], _lane_q[NUM_LANES], _lane_r[NUM_LANES];

    int  _select_primary() const;
};
