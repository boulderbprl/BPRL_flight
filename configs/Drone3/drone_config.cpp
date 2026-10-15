#include "configs/DroneConfig.hpp"

/*
 * Drone3 — Orqa QuadCore H7.
 *
 * New airframe/FC combination. IMU alignment, motor pinout/order, and motor
 * spin direction are bench-confirmed as of 2026-08-10 (see the .mixer
 * comment further down); gains below have been roughly bench-tuned as a
 * starting point but are not flight-refined, and this airframe has not yet
 * been flight-tested. Gains and motor geometry originated as a copy from
 * Drone1/Drone2 (configs/Drone1|Drone2/drone_config.cpp — mixed sources:
 * .pid/.pid_pi/.alt/.pos/.controllers match Drone1, .indi matches Drone2;
 * check individual gain blocks below before assuming a specific source),
 * tuned for different, larger CAN-IMX5-equipped airframes — treat as a
 * rough starting point, not a final match for this frame/motors/props. In
 * particular:
 *   - .unmixer is Drone1's — same motors and props, so the thrust fit is
 *     shared. Keep the two in sync.
 *   - .mixer roll/pitch/yaw_factor are the standard logical [FR,RL,FL,RR]
 *     order; .mixer.motor_map is what maps that onto this airframe's actual
 *     ESC wiring (motor_out[0..3] = MOT1..MOT4, boards/OrqaH7QuadCore/board.h
 *     LINE_MOTOR0..3: PD12/PD13/PA1/PA0 on the main ESC connector) — see the
 *     .mixer comment further down for the lane→corner mapping found on the
 *     bench. Re-derive motor_map if the ESC wiring changes.
 *   - .controllers below puts PID on all three switch positions —
 *     conservative starting point pending flight testing of the others on
 *     this airframe.
 *
 * Field order below matches each struct's declaration order in
 * configs/DroneConfig.hpp exactly (plain aggregate init, no designated
 * initializers — this toolchain defaults to gnu++14).
 */
const DroneConfig kDroneConfig = {
    // .pid — AttitudePidGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, yaw_stick_gain }
    {
        { 7.00f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // roll_att
        { 7.00f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // pitch_att
        { 0.07f, 0.04f, 0.0015f, 0.5f, 20.0f, 0.0f, 30.0f },  // roll_rate
        { 0.08f, 0.05f, 0.0015f, 0.5f, 20.0f, 0.0f, 30.0f },  // pitch_rate
        { 0.18f, 0.05f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.70f, 0.05f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        3.0f,  // yaw_stick_gain
    },

    // .indi — AttitudeIndiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, g1_seed_roll, g1_seed_pitch, indi_output_gain_roll, indi_output_gain_pitch, nlms_mu_pid, nlms_mu_indi, yaw_gain }
    // Copied verbatim from Drone1's .indi (same motors/props) — keep in sync until
    // this airframe has its own INDI tuning.
    {
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // roll_att
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // pitch_att
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate
        { 0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.60f, 0.050f, 0.000f, 0.3f,  0.0f,  0.0f, 30.0f }, // yaw_hold
        0.005f, // g1_seed_roll (was indi_gain_roll)
        0.0065f, // g1_seed_pitch (was indi_gain_pitch)
        1.0f,    // indi_output_gain_roll (kappa) — 1.0 reproduces pre-adaptation behavior
        1.0f,    // indi_output_gain_pitch (kappa)
        0.05f,   // nlms_mu_pid — aggressive adaptation rate while PID/PID+PI active; needs bench/flight tuning
        0.005f,  // nlms_mu_indi — slow/trickle adaptation rate while INDI active; needs bench/flight tuning
        3.0f,    // yaw_gain
    },

    // .jindi — AttitudeJindiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, roll_accel, g1_seed_roll, g1_seed_pitch, indi_output_gain_pitch, h_roll, alpha, nlms_mu_pid, nlms_mu_indi, yaw_gain }
    // Unused while no .controllers.switch_pos entry names AttCtrl::JINDI; kept only
    // to keep the struct valid — see Drone1's .jindi comment.
    {
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // roll_att
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // pitch_att
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate
        { 0.065f, 0.02f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f }, // yaw_rate
        { 0.60f, 0.05f, 0.000f, 0.3f,  0.0f,  0.0f, 30.0f }, // yaw_hold
        { 1.0f,  0.0f,  0.0f, 0.5f, 0.0f, 20.0f, 20.0f },    // roll_accel
        0.0035f, // g1_seed_roll
        0.0045f, // g1_seed_pitch
        1.0f,    // indi_output_gain_pitch (kappa, pitch only)
        40.0f,   // h_roll — 1000/25
        1.0f,    // alpha — fully accel-path for now, not yet characterized
        0.05f,   // nlms_mu_pid
        0.005f,  // nlms_mu_indi
        1.5f,    // yaw_gain
    },

    // .pid_pi — AttitudePidPiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, roll_accel, pitch_accel, yaw_stick_gain }
    // Unused while no .controllers.switch_pos entry names AttCtrl::PID_PI; copied from Drone1.
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

    // .alt — AltControlGains { pos_D, climb_rate, hover_thr }
    {
        { 1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_D
        { 0.15f, 0.05f, 0.0f, 0.3f, 0.0f, 5.0f, 20.0f },  // climb_rate
        0.27f,  // hover_thr (measured 0.26, trojTrack_test1 2026-10-08)
    },

    // .pos — PosControlGains { pos_N, pos_E, vel_N, vel_E }
    // Retuned from flight log trojTrack_test1 (2026-10-08): was vel P 2.0 /
    // I 0.7, which held to 3.6 cm RMS with a slow 5-13 s wander.
    //
    // How far these can go is set by the mocap latency, not the airframe:
    // that log shows the position/velocity the controller sees running
    // 60-100 ms behind the vehicle. With that delay in the loop
    // (tools/sim's degraded_link scenario), vel P 2.5 with pos P left at 1.0
    // is the best-damped of the sets tried; pos P 1.5 / vel P 3.0 rings for
    // several cycles and pos P 2.0 / vel P 4.0 is unstable. Don't raise
    // these further without first cutting that latency (or compensating for
    // it in the estimator) — re-run degraded_link before trying.
    {
        { 1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_N
        { 1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_E
        { 2.5f, 0.85f, 0.000f, 0.8f, 0.0f, 20.0f, 20.0f }, // vel_N
        { 2.5f, 0.85f, 0.000f, 0.8f, 0.0f, 20.0f, 20.0f }, // vel_E
    },

    // .heading — HeadingGains { kp, stick_rate, max_rate, max_accel, lag_s }
    // Starting point, not flight-tuned: kp matches the heading-lock trim
    // POS_HOLD flew with before HeadingControl (yaw_hold kp 0.7 x 2.5).
    { 1.75f,   // kp
      3.0f,    // stick_rate (same full-stick rate as yaw_stick_gain above)
      1.0f,    // max_rate
      4.0f,    // max_accel
      0.16f },  // lag_s (measured, trojTrack_test1 2026-10-08)

    // .rc_tx — RcTransmitter: which transmitter's channel layout (kRcMaps[] in DroneConfig.hpp)
    RcTransmitter::TX2,  // RC controller 2

    // .mixer — MotorMixerConfig { roll_factor[4], pitch_factor[4], yaw_factor[4], motor_map[4], pwm_min, pwm_idle, pwm_max, att_scale, yaw_scale, max_angle_rad, yaw_headroom_min }
    // Factor arrays are the standard [FR, RL, FL, RR] logical order — motor_map
    {
        { -1.0f, +1.0f, +1.0f, -1.0f },  // roll_factor  [FR, RL, FL, RR]
        { +1.0f, -1.0f, +1.0f, -1.0f },  // pitch_factor
        { +1.0f, +1.0f, -1.0f, -1.0f },  // yaw_factor
        { 1, 2, 3, 0 },                  // motor_map [FR,RL,FL,RR] -> physical lane
        50, 150, 900,                    // pwm_min, pwm_idle, pwm_max
        350.0f, 250.0f, 1.396f, 0.18f,   // att_scale, yaw_scale, max_angle_rad (~80deg), yaw_headroom_min
    },

    // .unmixer — UnmixerConfig { arm_length_m, motor_c0..c3, max_thrust_n }
    // Same motors, props and arm length as Drone1 — keep this identical to
    // Drone1's .unmixer (bench thrust fit, F_N(rpm) = c3*rpm^3+c2*rpm^2+c1*rpm+c0).
    {
        0.1275f,                                          // arm_length_m
        0.005144f, -4.5422e-6f, 7.5398e-9f, 5.8387e-14f,   // motor_c0, motor_c1, motor_c2, motor_c3
        7.04f,                                              // max_thrust_n
    },

    // .ang_accel_filt — AngAccelFilterConfig { lpf_hz, lpf_extra_hz }
    // Applied to both p_dot/q_dot and motor RPM (INDI/JINDI torque feedback).
    // Extra stage off on this airframe (2nd-order only).
    { 20.0f, 0.0f },

    // .sensors — SensorsConfig { has_baro, has_can_imx5_ins, has_mocap_link, has_encoder_rpm, encoder_motor_map }
    // DPS310 (I2C) is the only barometer on this board; no CAN IMU. Mocap over the MAVLink radio (UART7).
    // MOTOR_PROTOCOL is DShot on this board (no override in config.mk) — DShot RPM covers all 4 motors, so the CAN encoder rig stays off.
    { true, false, true, false, { -1, -1, -1, -1 } },

    // .controllers — ControllersConfig { switch_pos[3] }
    // Controller flown at each ctrl_select switch position [low, mid, high];
    // poshold_switch high runs POS_HOLD around it, low/mid flies it directly (STABILIZE).
    { { AttCtrl::PID, AttCtrl::PID, AttCtrl::INDI } },  // low/mid = PID, high = INDI (INDI not yet flight tested on this airframe)

    // .logging — LoggingConfig { log_rate_hz, log_enabled[19] }
    {
        50.0f, // log_rate_hz
        { true, // att
          true, // lin
          true, // rcin
          true, // outp
          true, // rpms
          false, // strt
          false, // imu1
          false, // imu2
          false, // imu3
          true, // indi
          false, // baro
          true, // ctun
          true, // mocp
          true, // traj
          true, // mavl
          false, // strn (STN1..STN4)
          false, // jindi
          false }, // jpid
    },
};
