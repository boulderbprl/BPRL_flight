#pragma once
#include "EKF.hpp"
#include "src/FlightState.hpp"
#include "src/threads.hpp"   // IMURaw, CANIMURaw, MocapRaw

// Lowpass cutoff frequencies for derived derivative states (Hz)
#define STATEMGR_LP_UVWDOT_HZ     20.0f   // cutoff for u_dot/v_dot/w_dot (2nd-order; matches ArduPilot INS_ACCEL_FILTER default)
#define STATEMGR_LP_UVW_HZ        15.0f   // cutoff for blended u/v/w fed to the controllers (2nd-order)
#define STATEMGR_LP_PQRDOT_HZ     20.0f   // cutoff for p_dot/q_dot/r_dot
#define STATEMGR_LP_PQ_HZ         20.0f   // cutoff for blended p/q (roll/pitch) fed to the rate PID
#define STATEMGR_LP_R_HZ           5.0f   // cutoff for blended r (yaw) fed to the rate PID
// IMX5 angular rate blend weight: 0=pure onboard gyros, 1=pure IMX5
#define STATEMGR_IMX5_RATE_WEIGHT  0.3f

// IMX5 CAN measurement staleness gates (transport-delay compensation, not
// the ~1s link-down timeout in threads.cpp's CAN_TIMEOUT_TICKS). A reading
// older than this is treated as a missed/delayed CAN update and skipped for
// this tick rather than fused as if it were current; anything younger is
// still used, with the quaternion forward-propagated by its age first.
#define STATEMGR_CAN_QUAT_STALE_US   50000   // 50 ms (~10x the ~200Hz nominal quat period)
#define STATEMGR_CAN_RATES_STALE_US  50000   // 50 ms (~5x the ~100Hz nominal rates period)

// Motor vibration notch — tracks the fundamental rotation frequency (Hz)
// derived from average gated motor RPM, applied to p/q/r before the
// STATEMGR_LP_PQ_HZ/STATEMGR_LP_R_HZ lowpass (matches ArduPilot's harmonic
// notch: notch first, then LPF "to attenuate any notch induced noise").
#define STATEMGR_NOTCH_BW_HZ          10.0f   // notch bandwidth (sets Q = center/bandwidth)
#define STATEMGR_NOTCH_MAX_SLEW_FRAC   0.05f  // max fractional change in tracked center freq per update (matches ArduPilot's ±5%/update)

// Lane-blend weight smoothing: the raw 1/(1e-4+innovation_norm) weight is
// itself derived from a noisy instantaneous quantity, so low-pass it before
// renormalizing rather than using it directly every tick — slow enough that
// the blend ratio stops acting as its own noise carrier, fast enough to
// still respond to a genuine sensor fault within ~300ms.
#define STATEMGR_LP_BLENDW_HZ      3.0f

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
 * 1/innovation_norm, giving partial noise averaging with fault isolation,
 * then passed through a motor-vibration notch (center frequency tracked from
 * gated motor RPM) before the STATEMGR_LP_PQ_HZ/STATEMGR_LP_R_HZ lowpass —
 * notch first, LPF last, same order as ArduPilot's harmonic notch + INS LPF.
 *
 * u/v/w output: soft-blended as above, then 2nd-order lowpass filtered at
 * STATEMGR_LP_UVW_HZ. 
 *
 * u_dot/v_dot/w_dot: gravity+Coriolis-corrected IMU accel, blended by the
 * same innovation-norm weights, then 2nd-order lowpass filtered at
 * STATEMGR_LP_UVWDOT_HZ (matches ArduPilot's INS-level LowPassFilter2p on
 * raw accelerometer samples).
 *
 * p_dot/q_dot/r_dot: differentiated from the filtered p/q/r (post STATEMGR_LP_PQ_HZ/
 * STATEMGR_LP_R_HZ 2nd-order Butterworth), then lowpass filtered again at
 * STATEMGR_LP_PQRDOT_HZ.
 *
 * Assembles the full 19-element StateIdx state vector for g_state[].
 */
class StateManager {
public:
    static constexpr int NUM_LANES = 3;

    // IMX5 / mocap / gravity measurement noise variances — tunable.
    static constexpr float R_QUAT      = 1e-2f;   // IMX5 quaternion component variance
    static constexpr float R_GRAVITY   = 0.5f;    // accel gravity-vector variance (m/s²)²
    static constexpr float R_MOCAP_POS = 1e-3f;   // mocap NED position variance (m²)
    static constexpr float R_MOCAP_VEL = 1e-4f;   // mocap NED velocity variance (m/s)²
    static constexpr float R_BARO_POS  = 0.5f;    // baro altitude variance (m²) — tune from bench log noise

    StateManager();

    void init();

    // Call once per StateEstThread tick (800 Hz).
    // dt: loop period in seconds.
    // imu: snapshot of g_imu[3] (taken under imu_mtx before this call).
    // can_imu: snapshot of g_can_imu (taken under can_imu_mtx before this call).
    // mocap: snapshot of g_mocap (taken under mocap_mtx before this call).
    // baro: snapshot of g_baro (taken under baro_mtx before this call).
    // rpm: snapshot of g_rpm_gated[4] (taken under esc_mtx before this call) —
    // fault-gated mechanical RPM per motor, drives the vibration notch center frequency.
    // now_us: chVTGetSystemTimeX()-derived timestamp for this tick, used to age-gate
    // and forward-propagate the IMX5 CAN quaternion/rate measurements (see CANIMURaw's
    // *_timestamp_us fields) rather than fusing them as if they were current.
    void update(float dt, const IMURaw imu[3], const CANIMURaw& can_imu, const MocapRaw& mocap, const BaroRaw& baro, const uint32_t rpm[4], uint32_t now_us);

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

    // Boot-time heading zero: IMX5 reports an absolute (uncalibrated) heading
    // that doesn't start at 0 on power-up. Captured once from the first valid
    // IMX5 quaternion after init() and applied as a constant world-frame yaw
    // rotation to every subsequent quaternion measurement before fusion, so
    // the fused attitude (and therefore euler[2]/yaw() everywhere) reads ~0
    // heading at whatever orientation the vehicle powered on in.
    //
    // This boot-relative zero is NOT aligned to the mocap world frame's N/E
    // axes. PosControl::compute_lean_angles() and the body->NED velocity
    // rotation in FlightStateMachine::mode_pos_hold() both assume yaw is the
    // true angle to the mocap frame's North, so update() re-anchors
    // _yaw_offset_q (see _reoffset_yaw_from_mocap()) every time a fresh mocap
    // yaw arrives (MocapRaw::has_new_yaw), overriding the boot-relative zero
    // once mocap is connected.
    bool _yaw_zero_captured;
    Quat _yaw_offset_q;

    // Recompute _yaw_offset_q so that rotating `raw_q` (the latest raw IMX5
    // quaternion, pre-offset) by it yields a fused yaw equal to
    // `mocap_yaw_rad`. Called from update() whenever mocap.has_new_yaw.
    void _reoffset_yaw_from_mocap(float mocap_yaw_rad, const Quat& raw_q);

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

    // Motor vibration notch, applied to p/q/r before the lowpass below.
    // _notch_freq_hz is the slew-limited tracked center frequency (average
    // rotation frequency of currently-spinning gated motors); _p/q/r_notch_state
    // are the biquad delay memories (one independent notch per axis).
    float _notch_freq_hz = 0.0f;
    Biquad2pState _p_notch_state, _q_notch_state, _r_notch_state;

    // Lowpass-filtered p/q/r fed to the rate PID (vibration rejection,
    // 2nd-order Butterworth — matches ArduPilot's INS-level LowPassFilter2p
    // on gyro, steeper rolloff than a 1-pole filter at the same cutoff)
    float _p_filt, _q_filt, _r_filt;
    Biquad2pState _p_filt_state, _q_filt_state, _r_filt_state;

    // Lowpass-filtered uvw_dot output (2nd-order Butterworth)
    float _ud_filt, _vd_filt, _wd_filt;
    Biquad2pState _ud_filt_state, _vd_filt_state, _wd_filt_state;

    // Lowpass-filtered u/v/w output (2nd-order Butterworth, post-blend)
    float _u_filt, _v_filt, _w_filt;
    Biquad2pState _u_filt_state, _v_filt_state, _w_filt_state;

    // Per-lane bias-corrected angular rates (updated each update() call)
    float _lane_p[NUM_LANES], _lane_q[NUM_LANES], _lane_r[NUM_LANES];

    // Low-passed lane-blend weights (raw 1/innovation_norm smoothed at
    // STATEMGR_LP_BLENDW_HZ before renormalizing across lanes each tick)
    float _lane_weight_filt[NUM_LANES];

    int  _select_primary() const;

    // Derive the slew-limited notch center frequency from gated motor RPM
    // (average rotation frequency, Hz, across motors with rpm[i] > 0), and
    // update _notch_freq_hz in place. Returns 0.0f (notch disabled) when no
    // motor has a usable reading.
    void _update_notch_freq(const uint32_t rpm[4]);
};
