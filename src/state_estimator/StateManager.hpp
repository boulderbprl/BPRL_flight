#pragma once
#include "EKF.hpp"
#include "src/FlightState.hpp"
#include "src/threads.hpp"   // IMURaw, CANIMURaw, MocapRaw

// Lowpass cutoff frequencies for derived derivative states (Hz)
#define STATEMGR_LP_UVWDOT_HZ     20.0f   // cutoff for u_dot/v_dot/w_dot (2nd-order; matches ArduPilot INS_ACCEL_FILTER default)
#define STATEMGR_LP_UVW_HZ        15.0f   // cutoff for blended u/v/w fed to the controllers (2nd-order)
#define STATEMGR_LP_PQRDOT_HZ     20.0f   // cutoff for p_dot/q_dot/r_dot
// Optional 3rd (1st-order) stage cascaded after STATEMGR_LP_PQRDOT_HZ on
// p_dot/q_dot only — a static A/B knob to test whether a 2+1 order cascade
// helps INDI over the 2nd-order-only filter above. <=0 disables it
// (lowpass_alpha() passthrough), leaving p_dot/q_dot exactly as before.
#define STATEMGR_LP_PQRDOT_EXTRA_HZ 15.0f
#define STATEMGR_LP_PQ_HZ         20.0f   // cutoff for blended p/q (roll/pitch) fed to the rate PID
#define STATEMGR_LP_R_HZ           5.0f   // cutoff for blended r (yaw) fed to the rate PID
// Window length (s) for the accel fed into EKF::update_gravity()'s
// tilt/accel-bias fusion. Deliberately a windowed TIME AVERAGE — not an IIR
// lowpass — matching ArduPilot's no-GPS attitude reference (AP_AHRS_DCM::
// drift_correction / _ra_sum): accumulate accel*dt over this window, fuse
// the window's mean once, then reset, instead of fusing every tick.
//
// This matters specifically for a symmetric push-then-pull transient (e.g.
// sliding the vehicle forward then back on a bench): a true window average
// of the raw signal lets the opposite-signed halves cancel toward zero net
// specific force if both fall inside the window, whereas a recursive IIR
// lowpass only smooths the transient's envelope — it does not cancel it,
// so a 2nd-order Butterworth at a few Hz still passes most of a shove's
// low-frequency energy straight through. A boxcar average is the correct
// tool for rejecting exactly this class of disturbance.
//
// 0.2s exactly matches AP_AHRS_DCM::drift_correction()'s own no-GPS window
// (`if (_ra_deltat < 0.2f) return;`, checked every AHRS loop so it fires
// right at 0.2s, then _ra_sum/_ra_deltat reset) — confirmed by reading the
// reset logic at the end of that function, not just the initial gate check.
// With GPS, ArduPilot's window instead tracks the GPS fix rate (~0.1-0.2s
// at 5-10Hz), so 0.2s is representative either way. Window length is a
// smaller lever than it looks, though: DCM's real robustness against a
// push/pull transient comes mostly from EKF::GRAV_MAX_CORR_RAD (mirroring
// DCM's slow rate-limited _kp=0.2 correction) rather than from averaging
// alone — see that constant's comment in EKF.hpp.
#define STATEMGR_GRAVACC_AVG_S     0.2f
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
// Max fractional change in tracked center freq per update() call. This is
// per-*call*, not per-second, and update() now runs at 400 Hz (ControlThread)
// instead of the former 625 Hz (StateEstThread) — rescaled by 625/400 so the
// real-world slew rate (Hz/s) is unchanged: 0.05 * 625/400 = 0.078125.
// (ArduPilot's own "±5%/update" figure is against its own update rate, not
// directly comparable here.)
#define STATEMGR_NOTCH_MAX_SLEW_FRAC   0.078125f

// Lane-blend weight smoothing: the raw 1/(1e-4+innovation_norm) weight is
// itself derived from a noisy instantaneous quantity, so low-pass it before
// renormalizing rather than using it directly every tick — slow enough that
// the blend ratio stops acting as its own noise carrier, fast enough to
// still respond to a genuine sensor fault within ~300ms.
#define STATEMGR_LP_BLENDW_HZ      3.0f

/*
 * StateManager — multi-lane EKF orchestrator.
 *
 * Runs three EKF lanes (N=16 states each), one per onboard IMU. Each lane
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
 * p_dot/q_dot: differentiated from the notch-filtered p/q — ahead of the
 * STATEMGR_LP_PQ_HZ 2nd-order Butterworth used for the rate PID's P-term, so
 * p_dot/q_dot (feeding the INDI law) don't inherit that stage's extra lag —
 * then 2nd-order (Butterworth) lowpass filtered at STATEMGR_LP_PQRDOT_HZ,
 * then an optional 3rd (1st-order) stage at STATEMGR_LP_PQRDOT_EXTRA_HZ
 * (test knob for a 2+1 order cascade; <=0 disables it).
 * r_dot differentiates the fully-filtered r (post STATEMGR_LP_R_HZ) and keeps
 * the original 1st-order lowpass, since yaw has no INDI consumer for it.
 *
 * Assembles the full 19-element StateIdx state vector for g_state[].
 */
class StateManager {
public:
    static constexpr int NUM_LANES = 3;

    // IMX5 / mocap / gravity measurement noise variances — tunable.
    // IMX5 quaternion component variance — was 1e-2 (very tight/high-trust).
    // Raised 5x to weaken CAN quaternion fusion strength, paired with the new
    // QUAT_CHI2_GATE outlier gate added to EKF::update_quaternion() (that
    // path previously had no chi-squared gate at all, unlike every other
    // update_*() in EKF.cpp — a single corrupted CAN sample used to get
    // fused with no outlier rejection).
    static constexpr float R_QUAT      = 5e-2f;   // IMX5 quaternion component variance
    // accel gravity-vector variance (m/s²)² while disarmed/unaided (e.g. on
    // the bench) — was 0.5; doubled alongside the STATEMGR_GRAVACC_AVG_S
    // windowed average above, closer to ArduPilot DCM's deliberately slow
    // (AHRS_RP_P=0.2, rate-limited, not snap) tilt correction philosophy.
    // Revisit if unaided-but-disarmed tilt/accel-bias convergence becomes
    // noticeably sluggish on the bench log.
    static constexpr float R_GRAVITY   = 1.0f;
    // accel gravity-vector variance (m/s²)² while ARMED — unconditionally,
    // regardless of mocap/CAN connection. 100x R_GRAVITY, so
    // update_gravity()'s Kalman gain for tilt is pushed toward negligible
    // whenever flying. This mirrors ArduPilot's EKF3, which has NO direct
    // accel-as-gravity-vector fusion in ANY aiding mode:
    //   - Unaided (verified by reading NavEKF3_core::SelectVelPosFusion()'s
    //     PV_AidingMode==AID_NONE branch): fuses a synthetic "position hasn't
    //     moved" pseudo-measurement, gated to the baro rate, with
    //     EK3_NOAID_M_NSE defaulting to a 10 m (!) noise std — essentially
    //     inert once actually flying. Attitude is carried almost entirely by
    //     gyro integration plus the pre-arm gyro-bias calibration.
    //   - Aided (GPS/optical-flow): tilt is corrected purely through
    //     velocity/position fusion's cross-covariance leak into attitude —
    //     still no direct gravity-vector measurement anywhere.
    // Since neither EKF3 mode has an analog for update_gravity() at all, it
    // stays weak across the board while armed; CAN's update_quaternion()
    // (direct attitude fusion) and mocap's update_position()/update_ned_vel()
    // (indirect, cross-covariance-mediated — see step 5 in StateManager.cpp,
    // same mechanism as EKF3's aided case) are what carry aided-mode tilt
    // correction instead, matching EKF3's real division of labor. This
    // constant is the closest proportional analogy achievable within this
    // EKF's existing gravity-vector-fusion structure (a genuine unit-for-unit
    // match isn't possible — EKF3's synthetic measurement is position-space,
    // this is direction-space) rather than literally gating update_gravity()
    // off, so the existing chi-squared gate / rate-limit / windowing
    // machinery still applies uniformly. See StateManager::update()'s
    // `armed` parameter, used directly (unconditionally) in its step 1.5.
    static constexpr float R_GRAVITY_FLIGHT_NOAID = 100.0f;
    static constexpr float R_MOCAP_POS = 1e-3f;   // mocap NED position variance (m²)
    static constexpr float R_MOCAP_VEL = 1e-4f;   // mocap NED velocity variance (m/s)²
    static constexpr float R_BARO_POS  = 0.5f;    // baro altitude variance (m²) — tune from bench log noise

    StateManager();

    void init();

    // Call once per ControlThread tick (400 Hz).
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
    // armed: g_armed snapshot. Selects R_GRAVITY vs R_GRAVITY_FLIGHT_NOAID for
    // update_gravity() — unconditionally weakened whenever armed, regardless
    // of mocap/CAN connection, since ArduPilot's EKF3 has no direct
    // accel-as-gravity fusion in any aiding mode (see R_GRAVITY_FLIGHT_NOAID's
    // comment). Disarmed is the bench case, where the stronger correction
    // still applies.
    void update(float dt, const IMURaw imu[3], const CANIMURaw& can_imu, const MocapRaw& mocap, const BaroRaw& baro, const uint32_t rpm[4], uint32_t now_us, bool armed);

    // Full 19-element state output — maps 16-state EKF lanes onto StateIdx ordering
    // and fills in the 6 derived quantities (uvw_dot, pqr_dot).
    void get_state(float out[StateIdx::N]) const;

    // Derived Euler angles (from primary lane quaternion).
    float roll()    const;
    float pitch()   const;
    float yaw()     const;

    // Per-lane accessors — called by ControlThread only (no mutex needed).
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

    // Angular acceleration via differentiation of blended rates + lowpass.
    // _prev_p/_prev_q hold the previous *notch-filtered* p/q (pre STATEMGR_LP_PQ_HZ)
    // since that's what p_dot/q_dot now differentiate; _prev_r holds the previous
    // fully-filtered r, unchanged.
    float _prev_p,    _prev_q,    _prev_r;
    float _pdot_filt, _qdot_filt, _rdot_filt;
    Biquad2pState _pdot_filt_state, _qdot_filt_state;  // 2nd-order LPF state for p_dot/q_dot

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

    // Windowed accel accumulator feeding EKF::update_gravity() — see
    // STATEMGR_GRAVACC_AVG_S above. _grav_accel_sum is Σ(accel*dt) per lane
    // per axis since the last window boundary; _grav_accel_dt is that lane's
    // own accumulated valid dt (tracked per-lane, not just once globally, so
    // a lane that briefly drops out mid-window still gets a correct average
    // over the time it actually had data, not a value diluted by zeros).
    float _grav_accel_sum[NUM_LANES][3];
    float _grav_accel_dt[NUM_LANES];
    float _grav_window_dt;

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
