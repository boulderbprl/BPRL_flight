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
    // (~1/Ixx, 1/Iyy), the starting point for AttitudeINDI's live NLMS
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
    PidGains climb_rate;
};

// Gains for PosControl (src/controllers/PosControl.hpp).
struct PosControlGains {
    PidGains pos_N, pos_E, pos_D, vel_N, vel_E;
};

// RC channel index assignment (src/coms/Radio.cpp). Both SBUS and CRSF use
// the same 11-bit channel numbering, so one map covers either protocol.
struct RcChannelMap {
    uint8_t thr, roll, pitch, yaw, arm, flight_mode, indi_switch;
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
};

// RPM-to-torque motor model + geometry (src/controllers/Unmixer.hpp).
// T_MAX_NM is NOT a field here — Unmixer derives it once at construction
// from arm_length_m/max_thrust_n so the two numbers can't drift apart.
struct UnmixerConfig {
    float arm_length_m;
    float motor_c0, motor_c1, motor_c2, motor_c3;   // bench thrust fit
    float rpm_norm_center, rpm_norm_scale, max_thrust_n;
    float rpm_filt_hz, rpm_filt_extra_hz;
};

// Which sensors are physically populated on this airframe. has_can_imx5_ins/
// has_mocap_link are metadata only for now (those paths already self-gate at
// runtime via g_can_imu.valid/g_mocap.valid staying false absent traffic);
// has_baro actually gates whether SPIThread bothers polling the barometer.
struct SensorsConfig {
    bool has_baro;
    bool has_can_imx5_ins;
    bool has_mocap_link;
};

// Which attitude controllers this drone offers on the RC switch, beyond the
// always-present default (PID, always list index 0).
struct ControllersConfig {
    bool indi_enabled;
    bool pid_pi_enabled;
};

// One enable flag per message type in LogMessages.hpp's kLogDefs[] — gates
// LogThread's per-tick write() call sites. Does NOT affect
// Logger::write_schema_header(), which still emits every type's FMT record
// regardless (Logger itself stays config-agnostic).
struct LogEnableConfig {
    bool att, lin, rcin, outp, rpms, strn, imu1, imu2, imu3, indi, baro, ctun, mocp, enc0, enc1;
};

struct LoggingConfig {
    float           log_rate_hz;
    LogEnableConfig enable;
};

struct DroneConfig {
    AttitudePidGains  pid;
    AttitudeIndiGains indi;
    AttitudePidPiGains pid_pi;
    AltControlGains   alt;
    PosControlGains   pos;
    RcChannelMap      rc_map;
    MotorMixerConfig  mixer;
    UnmixerConfig     unmixer;
    SensorsConfig     sensors;
    ControllersConfig controllers;
    LoggingConfig     logging;
};

extern const DroneConfig kDroneConfig;
