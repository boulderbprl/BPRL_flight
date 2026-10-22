#pragma once
#include <cstdint>

/*
 * DroneConfig — single source of truth per physical drone: controller gains,
 * RC channel mapping, motor mixing/geometry, sensors equipped, and logging.
 * Shared application code (controllers, EKF, mixer, drivers) is identical
 * across drones; only this data differs.
 *
 * One instance per drone: `configs/<Drone>/drone_config.cpp` defines
 * `kDroneConfig` fully spelled out (no defaults/override machinery — this
 * toolchain builds with no explicit -std=, defaulting to gnu++14, so C++20
 * designated initializers aren't reliably available). `make DRONE=<Drone>`
 * selects which one gets compiled in (see top-level Makefile).
 */

// Mirrors PID's constructor parameters exactly (see src/controllers/PID.hpp).
struct PidGains {
    float kp, ki, kd, imax;
    float filt_target_hz, filt_error_hz, filt_d_hz;
};

// Gains for AttitudePID (src/controllers/Attitude_PID.hpp).
struct AttitudePidGains {
    PidGains roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold;
    float    yaw_stick_gain;   // was AttitudePID::YAW_STICK_GAIN
};

// Gains for AttitudeINDI (src/controllers/Attitude_INDI.hpp).
struct AttitudeIndiGains {
    PidGains roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold;
    // G1_tau seed (N*m per rad/s^2) — offline-identified airframe effectiveness
    // (~Ixx, Iyy), the starting point for AttitudeINDI's live NLMS
    // adaptation. Was indi_gain_roll/indi_gain_pitch, which folded this seed
    // and an implicit gain of 1 together.
    float    g1_seed_roll;
    float    g1_seed_pitch;
    // INDI output gain (kappa) — decoupled control-authority multiplier on
    // top of the adaptive G1_tau estimate, tuned independently of the
    // physical-effectiveness identification. 1.0 reproduces pre-adaptation
    // behavior.
    float    indi_output_gain_roll;
    float    indi_output_gain_pitch;
    // NLMS adaptation rate (mu), mode-dependent — aggressive while PID/PID+PI
    // is the active controller (INDI in shadow, no closed-loop bias risk),
    // slow/trickle while INDI itself is active.
    float    nlms_mu_pid;
    float    nlms_mu_indi;
    float    yaw_gain;         // was AttitudeINDI::YAW_GAIN
};

// Gains for AttitudeJINDI (src/controllers/Attitude_JINDI.hpp) — roll_jerk
// (src/sensors/JerkFit.hpp) is a second, independent control path for roll,
// combined with the accel-level path via a convex combination — see
// Attitude_JINDI.hpp's class comment for the full derivation. Both of
// roll's paths share one live-adapted G1_hat (no separate "G2"), and roll's
// kappa output gain is replaced entirely by alpha. Pitch/yaw are plain
// accel-level INDI as in AttitudeIndiGains (no jerk sensor on pitch, so
// nothing to convex-combine there) and keep their own kappa
// (indi_output_gain_pitch).
struct AttitudeJindiGains {
    PidGains roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold;
    PidGains roll_accel;   // roll-only: accel error [rad/s^2] -> jerk target [rad/s^3]
    // Accel-level effectiveness seed (N*m per rad/s^2), shared by BOTH of
    // roll's paths (accel and jerk — see class comment on why there's one G,
    // not two). Pitch's is independent, same role as AttitudeIndiGains::g1_seed_pitch.
    float    g1_seed_roll;
    float    g1_seed_pitch;
    float    indi_output_gain_pitch;   // kappa, pitch only — roll has no kappa (see class comment)
    // H: roll's jerk-path actuator-dynamics constant — (G*H)^-1 = g1_roll / h_roll,
    // the jerk path's increment gain (see class comment's derivation).
    float    h_roll;
    // Convex-combination weight, roll: tau_roll = alpha*tau_accel + (1-alpha)*tau_jerk.
    // 1.0 = fully accel-path (jerk path computed but zero-weighted) — the
    // same "reproduces pre-adaptation behavior" convention as other
    // not-yet-characterized gains in this file.
    float    alpha;
    // NLMS adaptation rate (mu) for the shared G1 estimator — same
    // mode-dependent meaning as AttitudeIndiGains::nlms_mu_pid/indi. Set to
    // 0 for now (not yet characterized) — the machinery stays wired so it
    // can be turned on later without a code change.
    float    nlms_mu_pid;
    float    nlms_mu_indi;
    float    yaw_gain;
};

// Gains for AttitudePIDPI (src/controllers/Attitude_PID_PI.hpp) — ported from
// the BPRL ArduPilot fork's rate-PID + inner angular-acceleration-PI cascade
// (AC_AttitudeControl_Multi, "switched to PID" commit). roll_accel/pitch_accel
// are the inner PI loop closed on measured angular acceleration (kd expected
// to be 0 — a plain PI, not a PID).
struct AttitudePidPiGains {
    PidGains roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold;
    PidGains roll_accel, pitch_accel;
    float    yaw_stick_gain;
};

// Gains for AltControl (src/controllers/AltControl.hpp).
struct AltControlGains {
    PidGains pos_D, climb_rate;
    float    hover_thr;   // throttle [0,1] this airframe hovers at — the closed-loop altitude hold's starting point (read OUTP Thr in a steady hover)
};

// Gains for PosControl (src/controllers/PosControl.hpp).
struct PosControlGains {
    PidGains pos_N, pos_E, vel_N, vel_E;
};

// Gains for HeadingControl (src/controllers/HeadingControl.hpp) — POS_HOLD's
// heading loop, which feeds whichever attitude controller is active.
struct HeadingGains {
    float kp;          // 1/s     heading error → yaw-rate trim
    float stick_rate;  // rad/s   turn rate a full yaw stick commands in POS_HOLD
    float max_rate;    // rad/s   cap on the heading-error trim, and on the turn rate toward a commanded heading
    float max_accel;   // rad/s²  how fast the heading target's own rate may change
    float lag_s;       // s       yaw-rate loop's response lag — measure from a log (delay between TRAJ YawR and ATT R)
};

// RC channel index assignment (src/coms/Radio.cpp). Both SBUS and CRSF use
// the same 11-bit channel numbering, so one map covers either protocol.
//   ctrl_select     3-position switch: which attitude controller flies —
//                   ControllersConfig::switch_pos[low/mid/high]
//   poshold_switch  high = POS_HOLD (position loop outside whichever attitude
//                   controller ctrl_select picked); low/mid = STABILIZE
struct RcChannelMap {
    uint8_t thr, roll, pitch, yaw, arm, ctrl_select, poshold_switch, strain_cal_switch;
};

// The lab's RC transmitters. Each has its own channel layout, defined once
// in kRcMaps[] below; a drone picks one with DroneConfig::rc_tx. Moving a
// drone to the other transmitter is a one-word config change.
enum class RcTransmitter : uint8_t {
    TX1 = 0,   // RC controller 1
    TX2 = 1,   // RC controller 2
};

constexpr RcChannelMap kRcMaps[2] = {
    // TX1 — RC controller 1
    { 0,   // thr
      1,   // roll
      2,   // pitch
      3,   // yaw
      4,   // arm
      7,   // ctrl_select
      6,   // poshold_switch
      5 }, // strain_cal_switch (momentary)
    // TX2 — RC controller 2
    { 0,   // thr
      3,   // roll
      1,   // pitch
      2,   // yaw
      4,   // arm
      5,   // ctrl_select
      6,   // poshold_switch
      7 }, // strain_cal_switch (momentary)
};

// Motor mixing geometry (src/controllers/MotorMixer.hpp). Factor arrays are
// indexed [FR, RL, FL, RR] — always that logical frame-position order,
// regardless of which physical DShot lane/MOT pad each corner's ESC is
// actually wired to.
//
// motor_map[4] is that logical->physical translation: motor_map[FR/RL/FL/RR]
// = the physical output lane (0-3, i.e. DShot lane / MOT pad - 1) wired to
// that corner. {0,1,2,3} (identity) means lane N drives corner N directly —
// true when the ESC harness is wired FR->MOT1, RL->MOT2, FL->MOT3, RR->MOT4.
// If your physical wiring spins different corners than that, don't permute
// the factor arrays above (confusing to read/maintain, and every other
// motor-indexed consumer — MT test command, $TEL/SD-log rpm — silently goes
// out of sync with it) — set motor_map instead. It's applied at exactly two
// canonical boundaries, both in src/threads.cpp/MotorMixer.cpp: once
// forward (MotorMixer output, MT test command: logical -> physical lane)
// and once in reverse (telemetry rpm right after dshot_get_telemetry():
// physical lane -> logical), so every other consumer in the codebase
// (Unmixer/INDI, EKF, logging, debug tools) only ever sees logical
// FR/RL/FL/RR order and never needs to know the physical wiring.
struct MotorMixerConfig {
    float   roll_factor[4], pitch_factor[4], yaw_factor[4];
    uint8_t motor_map[4];   // [FR, RL, FL, RR] -> physical DShot lane (0-3)
    int32_t pwm_min, pwm_idle, pwm_max;
    float   att_scale, yaw_scale, max_angle_rad, yaw_headroom_min;
    // Thrust-curve expo for thrust linearization (ArduPilot MOT_THST_EXPO),
    // 0..1. 0 = linear PWM mixing (no linearization). See MotorMixer.hpp.
    float   thrust_expo;
};

// RPM-to-torque motor model + geometry (src/controllers/Unmixer.hpp).
// T_MAX_NM is NOT a field here — Unmixer derives it once at construction
// from arm_length_m/max_thrust_n so the two numbers can't drift apart.
struct UnmixerConfig {
    float arm_length_m;
    float motor_c0, motor_c1, motor_c2, motor_c3;   // bench thrust fit: F_N(rpm) = c3*rpm^3+c2*rpm^2+c1*rpm+c0
    float max_thrust_n;
};

// Angular-acceleration feedback filter for INDI/JINDI. One setting, applied
// identically to p_dot/q_dot (StateManager) and to each motor's RPM before
// the Unmixer's torque model, so measured torque and measured angular
// acceleration always carry the same filter delay — INDI's increment and its
// live G1 estimator both depend on that.
struct AngAccelFilterConfig {
    float lpf_hz;        // 2nd-order Butterworth cutoff
    float lpf_extra_hz;  // optional cascaded 1st-order stage; <= 0 disables it
};

// Which sensors are physically populated on this airframe. has_can_imx5_ins/
// has_mocap_link are metadata only for now (those paths already self-gate at
// runtime via g_can_imu.valid/g_mocap.valid staying false absent traffic);
// has_baro actually gates whether SPIThread bothers polling the barometer.
//
// has_encoder_rpm selects the CAN shaft-angle encoder nodes (src/sensors/
// EncoderRPM.hpp, one Feather M4 + AS5047P per node, CAN 0x70-0x73) as the
// RPM source for the EKF vibration-notch tracker and the INDI Unmixer, in
// place of DShot bidirectional telemetry. It only ever takes effect when
// DShot itself isn't the active motor protocol (MOTOR_PROTOCOL !=
// MOTOR_PROTO_DSHOT, src/coms/PWM.hpp — a per-board compile-time choice, see
// configs/<Drone>/config.mk) — DShot RPM always wins when DShot is selected.
// encoder_motor_map[node] is that node's logical motor index (FR=0, RL=1,
// FL=2, RR=3 — MotorMixerConfig's logical order, NOT the CAN/NODE_ID
// numbering) for whichever physical motor that encoder is actually clamped
// to; -1 means the node isn't wired to a motor on this airframe. Verify
// against the physical harness before enabling — a wrong mapping feeds one
// motor's RPM into another motor's notch/torque estimate.
struct SensorsConfig {
    bool    has_baro;
    bool    has_can_imx5_ins;
    bool    has_mocap_link;
    bool    has_encoder_rpm;
    int8_t  encoder_motor_map[4];   // [node0..node3] -> logical motor index, or -1
};

// Attitude controllers FlightStateMachine can fly. The numeric values go out
// in $TEL (tools/bprl_common.py ATTITUDE_CONTROLLER_NAMES mirrors this order)
// — append new ones at the end, don't reorder.
enum class AttCtrl : uint8_t {
    PID,        // AttitudePID — always built, the fallback
    INDI,       // AttitudeINDI
    PID_PI,     // AttitudePIDPI (rate PID + inner PI on measured angular accel) — see Attitude_PID_PI.hpp
    PID_JERK,   // AttitudePIDJerk (angle→rate→accel→jerk PID cascade on roll) — see Attitude_PID_Jerk.hpp
    JINDI,      // AttitudeJINDI (roll_jerk as a convex-combined second path) — see Attitude_JINDI.hpp
};

// Which controller flies at each position of the ctrl_select switch
// [low, mid, high]. The same controller may appear more than once. Every
// controller listed runs every tick (shadow mode, so its log stays live);
// ones not listed don't run at all. The poshold_switch then flies the
// selected controller either directly (STABILIZE) or under POS_HOLD.
struct ControllersConfig {
    AttCtrl switch_pos[3];
};

// One enable flag per message type in LogMessages.hpp's kLogDefs[] — gates
// LogThread's per-tick write() call sites. Does NOT affect
// Logger::write_schema_header(), which still emits every type's FMT record
// regardless (Logger itself stays config-agnostic).
struct LogEnableConfig {
    bool att, lin, rcin, outp, rpms, strt, imu1, imu2, imu3, indi, baro, ctun, mocp, traj, mavl;
    bool strn;                // strain gauge array — gates all four of STN1..STN4 together
    bool jindi, jpid;         // jerk-controller diagnostics (JIND / JPID)
};

struct LoggingConfig {
    float           log_rate_hz;
    LogEnableConfig enable;
};

struct DroneConfig {
    AttitudePidGains  pid;
    AttitudeIndiGains indi;
    AttitudeJindiGains jindi;
    AttitudePidPiGains pid_pi;
    AltControlGains   alt;
    PosControlGains   pos;
    HeadingGains      heading;
    RcTransmitter     rc_tx;        // which transmitter layout in kRcMaps[] this drone flies with
    MotorMixerConfig  mixer;
    UnmixerConfig     unmixer;
    AngAccelFilterConfig ang_accel_filt;  // shared by StateManager (p_dot/q_dot) and Unmixer (RPM)
    SensorsConfig     sensors;
    ControllersConfig controllers;
    LoggingConfig     logging;
};

extern const DroneConfig kDroneConfig;
