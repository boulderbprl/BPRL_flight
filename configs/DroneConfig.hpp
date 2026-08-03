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
    float    indi_gain_roll;   // was AttitudeINDI::INDI_GAIN_ROLL
    float    indi_gain_pitch;  // was AttitudeINDI::INDI_GAIN_PITCH
    float    yaw_gain;         // was AttitudeINDI::YAW_GAIN
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
// indexed [FR, RL, FL, RR], matching out[]/hardware pinout.
struct MotorMixerConfig {
    float   roll_factor[4], pitch_factor[4], yaw_factor[4];
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
};

// One enable flag per message type in LogMessages.hpp's kLogDefs[] — gates
// LogThread's per-tick write() call sites. Does NOT affect
// Logger::write_schema_header(), which still emits every type's FMT record
// regardless (Logger itself stays config-agnostic).
struct LogEnableConfig {
    bool att, lin, rcin, outp, rpms, strn, imu1, imu2, imu3, indi, baro, ctun, mocp;
};

struct LoggingConfig {
    float           log_rate_hz;
    LogEnableConfig enable;
};

struct DroneConfig {
    AttitudePidGains  pid;
    AttitudeIndiGains indi;
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
