#include "configs/DroneConfig.hpp"

/*
 * Drone1 — CubeOrangePlus.
 *
 * Values below are transcribed exactly from today's hardcoded constants
 * (Attitude_PID.cpp, Attitude_INDI.cpp, AltControl.cpp, PosControl.cpp,
 * MotorMixer.hpp, Unmixer.hpp, Radio.cpp, main.cpp's log rate) — this is the
 * one tuned set that used to apply to whichever board was compiled in.
 * Drone2 starts identical; they're free to diverge from here.
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
        { 0.09f, 0.06f, 0.002f, 0.5f, 20.0f, 0.0f, 30.0f },  // roll_rate
        { 0.09f, 0.06f, 0.002f, 0.5f, 20.0f, 0.0f, 30.0f },  // pitch_rate
        { 0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.60f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        3.0f,  // yaw_stick_gain
    },
    
    // .indi — AttitudeIndiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, g1_seed_roll, g1_seed_pitch, indi_output_gain_roll, indi_output_gain_pitch, nlms_mu_pid, nlms_mu_indi, yaw_gain }
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
    // roll_att/pitch_att/roll_rate/pitch_rate/yaw_rate/yaw_hold/roll_accel/
    // g1_seed_* originally copied from the (now removed) INDI-Jerk
    // controller's set, see Attitude_JINDI.hpp. h_roll = 1000/25 = 40.0,
    // given directly. alpha = 0 is full jerk-path reliance on roll (flown,
    // ee3b405); 1.0 would be plain accel-path-only. nlms_mu_* at 0 for now
    // (G1 adaptation machinery stays wired, just not live yet).
    {
        { 5.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // roll_att
        { 5.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // pitch_att
        { 10.5f,  0.50f, 0.25f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate P=7.0 was stable, and performed batter than base PID
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate
        { 0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.60f, 0.050f, 0.000f, 0.3f,  0.0f,  0.0f, 30.0f }, // yaw_hold
        { 25.00f,  40.0f,  0.0f, 1000.0f, 0.0f, 0.0f, 30.0f },    // roll_accel — accel error [rad/s^2] -> jerk target [rad/s^3]; ki 10 -> 20 (LOG0084: Pdot only reached 0.44-0.60 of AccR at 0.15-0.9 Hz, JINDI rang after gusts)
        0.0048f,  //0.006f,  // g1_seed_roll — shared by both of roll's paths, see class comment
        0.0065f, // g1_seed_pitch
        1.0f,    // indi_output_gain_pitch (kappa, pitch only)
        40.0f,   // h_roll — 1000/25
        0.0f,    // alpha — 0 = fully jerk-path on roll (flown); 1 = fully accel-path
        0.0f,    // nlms_mu_pid
        0.0f,    // nlms_mu_indi
        3.0f,    // yaw_gain
    },

    // .pid_pi — AttitudePidPiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, roll_accel, pitch_accel, yaw_stick_gain }
    {
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // roll_att
        { 4.00f, 0.00f, 0.000f, 0.5f,  0.0f,  0.0f, 30.0f }, // pitch_att
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate  ("SLC")
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate ("SLC")
        { 0.18f, 0.018f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.60f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold 
        { 0.7f,  0.8f,  0.0f, 0.5f, 50.0f, 0.0f, 0.0f },     // roll_accel  (inner PI, kd=0 — matches ArduPilot HEAD)
        { 0.7f,  0.8f,  0.0f, 0.5f, 50.0f, 0.0f, 0.0f },     // pitch_accel (inner PI, kd=0 — matches ArduPilot HEAD)
        3.0f,  // yaw_stick_gain
    },
    
    // .alt — AltControlGains { pos_D, climb_rate, hover_thr }
    {
        { 1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_D
        { 0.15f, 0.05f, 0.0f, 0.3f, 0.0f, 5.0f, 20.0f },  // climb_rate
        0.40f,  // hover_thr (the value every drone used before it was per-drone — measure and set)
    },
    
    // .pos — PosControlGains { pos_N, pos_E, vel_N, vel_E }
    // Copied from Drone3's flown set (trojTrack_test1, 2026-10-08) — same
    // motors/props and similar size. The limit is mocap latency, not the
    // airframe: see Drone3's .pos comment before raising these.
    {
        { 0.7f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_N
        { 0.7f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_E
        { 4.0f, 0.6f, 0.000f, 0.8f, 0.0f, 20.0f, 20.0f }, // vel_N
        { 4.0f, 0.6f, 0.000f, 0.8f, 0.0f, 20.0f, 20.0f }, // vel_E
    },
   
    // .heading — HeadingGains { kp, stick_rate, max_rate, max_accel, lag_s }
    // Starting point, not flight-tuned: kp matches the heading-lock trim
    // POS_HOLD flew with before HeadingControl (yaw_hold kp 0.7 x 2.5).
    { 1.75f,   // kp
      3.0f,    // stick_rate (same full-stick rate as yaw_stick_gain above)
      1.0f,    // max_rate
      4.0f,    // max_accel
      0.16f },  // lag_s (Drone3's measured value — re-measure on this airframe)

    // .rc_tx — RcTransmitter: which transmitter's channel layout (kRcMaps[] in DroneConfig.hpp)
    RcTransmitter::TX1,  // RC controller 1
   
    // .mixer — MotorMixerConfig { roll_factor[4], pitch_factor[4], yaw_factor[4], motor_map[4], pwm_min, pwm_idle, pwm_max, att_scale, yaw_scale, max_angle_rad, yaw_headroom_min, thrust_expo }
    {
        { -1.0f, +1.0f, +1.0f, -1.0f },  // roll_factor  [FR, RL, FL, RR]
        { +1.0f, -1.0f, +1.0f, -1.0f },  // pitch_factor
        { +1.0f, +1.0f, -1.0f, -1.0f },  // yaw_factor
        { 0, 1, 2, 3 },                  // motor_map — identity: ESC wired FR->MOT1..RR->MOT4
        50, 150, 900,                    // pwm_min, pwm_idle, pwm_max
        350.0f, 250.0f, 1.396f, 0.18f,   // att_scale, yaw_scale, max_angle_rad (~80deg), yaw_headroom_min
        0.0f,                            // thrust_expo — 0 = linear PWM mixing (no thrust linearization)
    },
    
    // .unmixer — UnmixerConfig { arm_length_m, motor_c0..c3, max_thrust_n }
    // motor_c0..c3: bench thrust fit, F_N(rpm) = c3*rpm^3+c2*rpm^2+c1*rpm+c0
    {
        0.1275f,                                          // arm_length_m
        0.005144f, -4.5422e-6f, 7.5398e-9f, 5.8387e-14f,   // motor_c0, motor_c1, motor_c2, motor_c3
        7.04f,                                              // max_thrust_n
    },

    // .ang_accel_filt — AngAccelFilterConfig { lpf_hz, lpf_extra_hz }
    // Applied to both p_dot/q_dot and motor RPM (INDI/JINDI torque feedback).
    { 20.0f, 15.0f },
   
    // .sensors — SensorsConfig { has_baro, has_can_imx5_ins, has_mocap_link, has_encoder_rpm, encoder_motor_map, can2_enable, imx5_can_bus, indi_requires_strain, encoder_rpm_sign }
    // MOTOR_PROTOCOL is DShot on this board (no override in config.mk) — DShot RPM covers all 4 motors, so the CAN encoder rig stays off.
    { true, true, true, false, { -1, -1, -1, -1 }, false, 1, false, { 0, 0, 0, 0 } },  // IMX5 on CAN1, CAN2 unused; INDI not in the switch list
    
    // .controllers — ControllersConfig { switch_pos[3], stick_max_angle_deg }
    // Controller flown at each ctrl_select switch position [low, mid, high];
    // poshold_switch high runs POS_HOLD around it, low/mid flies it directly (STABILIZE).
    { { AttCtrl::PID, AttCtrl::PID_JERK, AttCtrl::JINDI },  // low = PID, mid = PID_Jerk, high = JINDI (same as before the switch rework)
      57.29578f },  // stick_max_angle_deg — 1 rad, unchanged from the old fixed mapping
   
    // .logging — LoggingConfig { log_rate_hz, enable } — one LogEnableConfig flag per message type
    {
        70.0f, // log_rate_hz
        { true, // att
          true, // lin
          true, // rcin
          true, // outp
          false, // rpms
          true, // strt
          true, // imu1
          true, // imu2
          true, // imu3
          false, // indi (INDI not on any switch position — would log zeros)
          false, // baro
          true, // ptun (POS_HOLD tuning)
          true, // mocp
          true, // traj
          true, // mavl
          false, // strn (STN1..STN4 — no strain gauge array on this airframe)
          true, // jindi
          true, // jpid
          false, // ctun (AttitudePID tuning)
          true }, // sidd (system-ID sweep)
    },

    // .sysid — SysIdConfig { rc_channel, axes[4], f_start_hz, f_stop_hz, duration_s, magnitude[4], fade_in_s, fade_out_s }
    // Each flip of rc_channel sweeps the next axis in axes[] (wraps); flipping back
    // down mid-sweep cancels it and keeps the same axis. See src/controllers/extras/SysId.hpp.
    // Magnitudes are untested placeholders — start small and work up.
    { -1,                                    // rc_channel (0-based, like kRcMaps); -1 disables sysid
      { SysIdAxis::ROLL, SysIdAxis::PITCH, SysIdAxis::NONE, SysIdAxis::NONE },  // axes, in order
      0.5f, 20.0f,                          // f_start_hz, f_stop_hz — 0.5-20 Hz: this airframe's ~0-10 Hz bandwidth with margin
      60.0f,                                // duration_s (includes the 2-cycle start dwell and both fades)
      { 0.05f, 0.05f, 0.05f, 0.05f },       // magnitude [roll, pitch, yaw, heave] — torque cmd / throttle fraction
      5.0f, 2.0f },                         // fade_in_s, fade_out_s
};
