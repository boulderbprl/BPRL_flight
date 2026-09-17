#include "configs/DroneConfig.hpp"

/*
 * Drone2 — CubeBlueH7.
 *
 * Values below started as an exact transcription of Drone1's hardcoded
 * constants (Attitude_PID.cpp, Attitude_INDI.cpp, AltControl.cpp,
 * PosControl.cpp, MotorMixer.hpp, Unmixer.hpp, Radio.cpp, main.cpp's log
 * rate), since there was only ever one tuned set shared across whichever
 * board got compiled in. Free to diverge from here per airframe.
 *
 * Drone1 is a 5in racing quad; Drone2 is the larger S500. The roll/pitch
 * angle-P and rate PID gains in .pid/.indi/.pid_pi (roll_att, pitch_att, and
 * .pid's roll_rate/pitch_rate) have since been updated to the S500-tuned
 * values from the ArduPilot fork (jd-hanley/ardupilot, Strain_rate branch,
 * AC_AttitudeControl_Multi.h) — angle P 4.5, rate P/I/D 0.135/0.135/0.0036,
 * rate D-term filter 20 Hz. .indi/.pid_pi's roll_rate/pitch_rate ("SLC",
 * 6.5 gain) are a different two-stage architecture with no ArduPilot rate-PID
 * equivalent, so they're untouched; their roll_accel/pitch_accel inner loop
 * already matched ArduPilot's tuned accel PI exactly. Yaw gains are also
 * untouched — still Drone1's.
 *
 * Field order below matches each struct's declaration order in
 * configs/DroneConfig.hpp exactly (plain aggregate init, no designated
 * initializers — this toolchain defaults to gnu++14).
 */
const DroneConfig kDroneConfig = {
    // .pid — AttitudePidGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, yaw_stick_gain }
    {
        { 4.50f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // roll_att  — S500 ArduPilot-tuned angle P (was Drone1's 4.00)
        { 4.50f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // pitch_att — S500 ArduPilot-tuned angle P (was Drone1's 4.00)
        { 0.135f, 0.135f, 0.0036f, 0.5f, 20.0f, 0.0f, 20.0f }, // roll_rate  — S500 ArduPilot-tuned (was Drone1's 0.08/0.05/0.002, filt_d 30)
        { 0.135f, 0.135f, 0.0036f, 0.5f, 20.0f, 0.0f, 20.0f }, // pitch_rate — S500 ArduPilot-tuned (was Drone1's 0.09/0.06/0.002, filt_d 30)
        { 0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.60f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        3.0f,  // yaw_stick_gain
    },
    
    // .indi — AttitudeIndiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, g1_seed_roll, g1_seed_pitch, indi_output_gain_roll, indi_output_gain_pitch, nlms_mu_pid, nlms_mu_indi, yaw_gain }
    {
        { 4.50f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // roll_att  — S500 ArduPilot-tuned angle P (was Drone1's 4.00)
        { 4.50f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // pitch_att — S500 ArduPilot-tuned angle P (was Drone1's 4.00)
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate
        { 0.065f, 0.02f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f }, // yaw_rate
        { 0.60f, 0.05f, 0.000f, 0.3f,  0.0f,  0.0f, 30.0f }, // yaw_hold
        0.0035f, // g1_seed_roll (was indi_gain_roll)
        0.0045f, // g1_seed_pitch (was indi_gain_pitch)
        1.0f,    // indi_output_gain_roll (kappa) — 1.0 reproduces pre-adaptation behavior
        1.0f,    // indi_output_gain_pitch (kappa)
        0.05f,   // nlms_mu_pid — aggressive adaptation rate while PID/PID+PI active; needs bench/flight tuning
        0.005f,  // nlms_mu_indi — slow/trickle adaptation rate while INDI active; needs bench/flight tuning
        1.5f,    // yaw_gain
    },
    
    // .pid_pi — AttitudePidPiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, roll_accel, pitch_accel, yaw_stick_gain }
    {
        { 4.50f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // roll_att  — S500 ArduPilot-tuned angle P (was Drone1's 4.00)
        { 4.50f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // pitch_att — S500 ArduPilot-tuned angle P (was Drone1's 4.00)
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate  ("SLC")
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate ("SLC")
        { 0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.60f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        { 0.7f,  0.8f,  0.0f, 0.5f, 50.0f, 0.0f, 0.0f },     // roll_accel  (inner PI, kd=0 — matches ArduPilot HEAD)
        { 0.7f,  0.8f,  0.0f, 0.5f, 50.0f, 0.0f, 0.0f },     // pitch_accel (inner PI, kd=0 — matches ArduPilot HEAD)
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
    {
        0,  // thr
        1,  // roll
        2,  // pitch
        3,  // yaw
        4,  // arm
        6,  // flight_mode
        7,  // indi_switch
    },
    
    // .mixer — MotorMixerConfig { roll_factor[4], pitch_factor[4], yaw_factor[4], motor_map[4], pwm_min, pwm_idle, pwm_max, att_scale, yaw_scale, max_angle_rad, yaw_headroom_min }
    {
        { -1.0f, +1.0f, +1.0f, -1.0f },  // roll_factor  [FR, RL, FL, RR]
        { +1.0f, -1.0f, +1.0f, -1.0f },  // pitch_factor
        { +1.0f, +1.0f, -1.0f, -1.0f },  // yaw_factor
        { 0, 1, 2, 3 },                  // motor_map — identity: ESC wired FR->MOT1..RR->MOT4
        150, 200, 950,                    // pwm_min, pwm_idle, pwm_max
        350.0f, 250.0f, 1.396f, 0.18f,   // att_scale, yaw_scale, max_angle_rad (~80deg), yaw_headroom_min
    },
    
    // .unmixer — UnmixerConfig { arm_length_m, motor_c0..c3, rpm_norm_center, rpm_norm_scale, max_thrust_n, rpm_filt_hz, rpm_filt_extra_hz }
    {
        0.3f,                                  // arm_length_m
        4.0396f, 4.4596f, 1.2797f, 0.0244f,        // motor_c0, motor_c1, motor_c2, motor_c3
        4841.0f, 2528.0f, 27.49f,                  // rpm_norm_center, rpm_norm_scale, max_thrust_n
        20.0f, 15.0f,                              // rpm_filt_hz, rpm_filt_extra_hz
    },
    
    // .sensors — SensorsConfig { has_baro, has_can_imx5_ins, has_mocap_link, has_encoder_rpm, encoder_motor_map }
    // MOTOR_PROTOCOL is MOTOR_PROTO_IOMCU on this board (config.mk override) — no DShot telemetry, so CAN
    // shaft-encoder RPM (src/sensors/EncoderRPM.hpp) substitutes in, all 4 motors now instrumented:
    //   NODE_ID 0 (CAN 0x70) — front-right (FR, logical 0)
    //   NODE_ID 1 (CAN 0x71) — back-right  (RR, logical 3)
    //   NODE_ID 2 (CAN 0x72) — back-left   (RL, logical 1)
    //   NODE_ID 3 (CAN 0x73) — front-left  (FL, logical 2)
    // see Strain_CAN/Feather_Code/Feather_Code.ino's NODE_ID comment for which physical board is which.
    {
        true,           // has_baro
        false,           // has_can_imx5_ins
        false,           // has_mocap_link
        true,           // has_encoder_rpm
        { 0, 3, 1, 2 }, // encoder_motor_map — [node0..node3] -> logical motor (FR=0/RL=1/FL=2/RR=3)
    },

    // .controllers — ControllersConfig { indi_enabled, pid_pi_enabled }
    {
        false,  // indi_enabled
        false,  // pid_pi_enabled — off by default, see .pid_pi comment above — not yet bench/flight tuned
    },
    
    // .logging — LoggingConfig { log_rate_hz, log_enabled[13] }
    {
        50.0f, // log_rate_hz
        { true, // att
          true, // lin
          true, // rcin
          true, // outp
          true, // rpms
          true, // strn
          false, // imu1
          false, // imu2
          false, // imu3
          true, // indi
          false, // baro
          false, // ctun
          false }, // mocp
    },
};
