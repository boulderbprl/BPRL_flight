#include "configs/DroneConfig.hpp"

/*
 * Drone3 — Orqa QuadCore H7.
 *
 * New airframe/FC combination, not bench- or flight-tested. Gains and motor
 * geometry below are copied from Drone1 (configs/Drone1/drone_config.cpp) as
 * a syntactically-valid starting point ONLY — they were tuned for a
 * different, larger CAN-IMX5-equipped airframe and are almost certainly
 * wrong for whatever frame/motors/props this FC ends up on. In particular:
 *   - .unmixer motor_c0..c3 / max_thrust_n came from thrust-stand
 *     characterization of Drone1's specific motors — re-derive for this
 *     board's motors before trusting Unmixer's thrust/RPM estimates.
 *   - .mixer roll/pitch/yaw_factor assume motor_out[0..3] map physically to
 *     FR/RL/FL/RR. This board's four ESC pads are LINE_MOTOR0..3
 *     (boards/OrqaH7QuadCore/board.h: PD12/PA1/PA2/PB1) — wire ESCs to match
 *     that FR/RL/FL/RR order, or edit the factor tables to match your wiring.
 *   - .controllers below deliberately disables both INDI and PID+PI (plain
 *     PID only) for first bring-up, unlike Drone1's INDI-enabled default —
 *     safest starting point on unverified hardware.
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

    // .indi — AttitudeIndiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, g1_seed_roll, g1_seed_pitch, indi_output_gain_roll, indi_output_gain_pitch, nlms_mu_pid, nlms_mu_indi, yaw_gain }
    // Unused while .controllers.indi_enabled=false below; copied from Drone1 only to keep the struct valid.
    {
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // roll_att
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // pitch_att
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate
        { 0.065f, 0.02f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f }, // yaw_rate
        { 0.60f, 0.05f, 0.000f, 0.3f,  0.0f,  0.0f, 30.0f }, // yaw_hold
        0.0035f, // g1_seed_roll
        0.0045f, // g1_seed_pitch
        1.0f,    // indi_output_gain_roll (kappa)
        1.0f,    // indi_output_gain_pitch (kappa)
        0.05f,   // nlms_mu_pid
        0.005f,  // nlms_mu_indi
        1.5f,    // yaw_gain
    },

    // .pid_pi — AttitudePidPiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, roll_accel, pitch_accel, yaw_stick_gain }
    // Unused while .controllers.pid_pi_enabled=false below; copied from Drone1 only to keep the struct valid.
    {
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // roll_att
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // pitch_att
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate  ("SLC")
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate ("SLC")
        { 0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.60f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        { 0.7f,  0.8f,  0.0f, 0.5f, 50.0f, 0.0f, 0.0f },     // roll_accel  (inner PI, kd=0)
        { 0.7f,  0.8f,  0.0f, 0.5f, 50.0f, 0.0f, 0.0f },     // pitch_accel (inner PI, kd=0)
        3.0f,  // yaw_stick_gain
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
    // Index order [FR, RL, FL, RR] must match physical ESC wiring to
    // LINE_MOTOR0..3 (board.h) — verify motor spin direction/order on the
    // bench (props off) before first arm.
    {
        { -1.0f, +1.0f, +1.0f, -1.0f },  // roll_factor  [FR, RL, FL, RR]
        { +1.0f, -1.0f, +1.0f, -1.0f },  // pitch_factor
        { +1.0f, +1.0f, -1.0f, -1.0f },  // yaw_factor
        50, 150, 900,                    // pwm_min, pwm_idle, pwm_max
        350.0f, 250.0f, 1.396f, 0.18f,   // att_scale, yaw_scale, max_angle_rad (~80deg), yaw_headroom_min
    },

    // .unmixer — UnmixerConfig { arm_length_m, motor_c0..c3, rpm_norm_center, rpm_norm_scale, max_thrust_n, rpm_filt_hz, rpm_filt_extra_hz }
    // PLACEHOLDER — copied from Drone1's thrust-stand characterization, wrong
    // for this board's motors/props. Re-derive before trusting RPM/thrust telemetry.
    {
        0.1275f,                                  // arm_length_m
        2.4540f, 2.2831f, 0.5607f, 0.0134f,        // motor_c0, motor_c1, motor_c2, motor_c3
        2005.0f, 880.8f, 7.04f,                    // rpm_norm_center, rpm_norm_scale, max_thrust_n
        20.0f, 15.0f,                              // rpm_filt_hz, rpm_filt_extra_hz
    },

    // .sensors — SensorsConfig { has_baro, has_can_imx5_ins, has_mocap_link }
    // DPS310 (I2C) is the only barometer on this board; no CAN IMU, no mocap link.
    { true, false, false },

    // .controllers — ControllersConfig { indi_enabled, pid_pi_enabled }
    // Both disabled: plain PID only, for first bring-up on unverified hardware.
    { false, false },

    // .logging — LoggingConfig { log_rate_hz, enable{att,lin,rcin,outp,rpms,strn,imu1,imu2,imu3,indi,baro,ctun,mocp} }
    {
        50.0f,
        { true, true, true, true, true, true, true, true, true, true, true, true, true },
    },
};
