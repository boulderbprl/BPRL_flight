#include "configs/DroneConfig.hpp"

/*
 * Drone2 — CubeBlueH7.
 *
 * Values below are transcribed exactly from today's hardcoded constants
 * (Attitude_PID.cpp, Attitude_INDI.cpp, AltControl.cpp, PosControl.cpp,
 * MotorMixer.hpp, Unmixer.hpp, Radio.cpp, main.cpp's log rate) — identical to
 * Drone1's for now, since there was only ever one tuned set shared across
 * whichever board got compiled in. Free to diverge from here per airframe.
 *
 * Field order below matches each struct's declaration order in
 * configs/DroneConfig.hpp exactly (plain aggregate init, no designated
 * initializers — this toolchain defaults to gnu++14).
 */
const DroneConfig kDroneConfig = {
    // .pid — AttitudePidGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, yaw_stick_gain }
    {
        { 4.00f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // roll_att
        { 4.00f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // pitch_att
        { 0.08f, 0.05f, 0.002f, 0.5f, 20.0f, 0.0f, 30.0f },  // roll_rate
        { 0.09f, 0.06f, 0.002f, 0.5f, 20.0f, 0.0f, 30.0f },  // pitch_rate
        { 0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.60f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        3.0f,  // yaw_stick_gain
    },
    // .indi — AttitudeIndiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, indi_gain_roll, indi_gain_pitch, yaw_gain }
    {
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // roll_att
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // pitch_att
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate
        { 0.065f, 0.02f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f }, // yaw_rate
        { 0.60f, 0.05f, 0.000f, 0.3f,  0.0f,  0.0f, 30.0f }, // yaw_hold
        0.0035f, // indi_gain_roll
        0.0045f, // indi_gain_pitch
        1.5f,    // yaw_gain
    },
    // .alt — AltControlGains { climb_rate }
    {
        { 0.15f, 0.05f, 0.0f, 0.3f, 0.0f, 5.0f, 20.0f },  // climb_rate
    },
    // .pos — PosControlGains { pos_N, pos_E, pos_D, vel_N, vel_E }
    {
        { 1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_N
        { 1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_E
        { 1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_D
        { 2.0f, 1.00f, 0.000f, 0.8f, 0.0f, 20.0f, 20.0f }, // vel_N
        { 2.0f, 1.00f, 0.000f, 0.8f, 0.0f, 20.0f, 20.0f }, // vel_E
    },
    // .rc_map — RcChannelMap { thr, roll, pitch, yaw, arm, flight_mode, indi_switch }
    { 0, 1, 2, 3, 4, 6, 7 },
    // .mixer — MotorMixerConfig { roll_factor[4], pitch_factor[4], yaw_factor[4], pwm_min, pwm_idle, pwm_max, att_scale, yaw_scale, max_angle_rad, yaw_headroom_min }
    {
        { -1.0f, +1.0f, +1.0f, -1.0f },  // roll_factor  [FR, RL, FL, RR]
        { +1.0f, -1.0f, +1.0f, -1.0f },  // pitch_factor
        { +1.0f, +1.0f, -1.0f, -1.0f },  // yaw_factor
        50, 150, 900,                    // pwm_min, pwm_idle, pwm_max
        350.0f, 250.0f, 1.396f, 0.18f,   // att_scale, yaw_scale, max_angle_rad (~80deg), yaw_headroom_min
    },
    // .unmixer — UnmixerConfig { arm_length_m, motor_c0..c3, rpm_norm_center, rpm_norm_scale, max_thrust_n, rpm_filt_hz, rpm_filt_extra_hz }
    {
        0.1275f,                                  // arm_length_m
        2.4540f, 2.2831f, 0.5607f, 0.0134f,        // motor_c0, motor_c1, motor_c2, motor_c3
        2005.0f, 880.8f, 7.04f,                    // rpm_norm_center, rpm_norm_scale, max_thrust_n
        20.0f, 15.0f,                              // rpm_filt_hz, rpm_filt_extra_hz
    },
    // .sensors — SensorsConfig { has_baro, has_can_imx5_ins, has_mocap_link }
    { true, true, true },
    // .controllers — ControllersConfig { indi_enabled }
    { true },
    // .logging — LoggingConfig { log_rate_hz, enable{att,lin,rcin,outp,rpms,strn,imu1,imu2,imu3,indi,baro,ctun,mocp} }
    {
        50.0f,
        { true, true, true, true, true, true, true, true, true, true, true, true, true },
    },
};
