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
    //  — S500 ArduPilot-tuned 
    {
        { 6.0f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // roll_att 
        { 6.0f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // pitch_att 
        { 0.135f, 0.135f, 0.006f, 0.5f, 20.0f, 0.0f, 20.0f }, // roll_rate  
        { 0.135f, 0.135f, 0.006f, 0.5f, 20.0f, 0.0f, 20.0f }, // pitch_rate 
        { 0.2f, 0.05f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.80f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        4.0f,  // yaw_stick_gain
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

    // .jindi — AttitudeJindiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, roll_accel, g1_seed_roll, g1_seed_pitch, indi_output_gain_pitch, h_roll, alpha, nlms_mu_pid, nlms_mu_indi, yaw_gain }
    // Unused while no .controllers.switch_pos entry names AttCtrl::JINDI; copied from
    // this drone's .indi values only to keep the struct valid — see Drone1's .jindi comment for the full rationale.
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
    
    // .alt — AltControlGains { pos_D, climb_rate, hover_thr }
    {
        { 1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_D
        { 0.15f, 0.05f, 0.0f, 0.3f, 0.0f, 5.0f, 20.0f },  // climb_rate
        0.50f,  // hover_thr (the value every drone used before it was per-drone — measure and set)
    },
    
    // .pos — PosControlGains { pos_N, pos_E, vel_N, vel_E }
    {
        { 1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_N
        { 1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f,  20.0f }, // pos_E
        { 3.0f, 1.00f, 0.000f, 0.8f, 0.0f, 20.0f, 20.0f }, // vel_N
        { 3.0f, 1.00f, 0.000f, 0.8f, 0.0f, 20.0f, 20.0f }, // vel_E
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
    RcTransmitter::TX2,  // RC controller 1 — same channels this config had before the shared maps; TODO confirm on the bench which transmitter Drone2 flies with
    
    // .mixer — MotorMixerConfig { roll_factor[4], pitch_factor[4], yaw_factor[4], motor_map[4], pwm_min, pwm_idle, pwm_max, att_scale, yaw_scale, max_angle_rad, yaw_headroom_min, thrust_expo }
    {
        { -1.0f, +1.0f, +1.0f, -1.0f },  // roll_factor  [FR, RL, FL, RR]
        { +1.0f, -1.0f, +1.0f, -1.0f },  // pitch_factor
        { +1.0f, +1.0f, -1.0f, -1.0f },  // yaw_factor
        { 0, 1, 2, 3 },                  // motor_map — identity: ESC wired FR->MOT1..RR->MOT4
        150, 200, 950,                    // pwm_min, pwm_idle, pwm_max
        350.0f, 250.0f, 1.396f, 0.18f,   // att_scale, yaw_scale, max_angle_rad (~80deg), yaw_headroom_min
        0.65f,                           // thrust_expo — ArduPilot's MOT_THST_EXPO default (the S500 ArduPilot logs ran 0.65/0.68); not measured for these 12in props
    },
    
    // .unmixer — UnmixerConfig { arm_length_m, motor_c0..c3, max_thrust_n }
    // PLACEHOLDER — these 12in motors/props haven't been on the thrust stand.
    // motor_c0..c3 are the Strain_Drone_Integration branch's normalised-RPM
    // cubic (C0=4.0396 C1=4.4596 C2=1.2797 C3=0.0244, center=4841 scale=2528)
    // expanded to raw RPM, F_N(rpm) = c3*rpm^3+c2*rpm^2+c1*rpm+c0 — identical
    // curve, ~4.0 N/motor at 4841 RPM. Re-fit once measured.
    {
        0.3f,                                                   // arm_length_m
        0.021051f, -6.8474e-5f, 1.78308e-7f, 1.51028e-12f,      // motor_c0, motor_c1, motor_c2, motor_c3
        27.49f,                                                 // max_thrust_n
    },

    // .ang_accel_filt — AngAccelFilterConfig { lpf_hz, lpf_extra_hz }
    // Applied to both p_dot/q_dot and motor RPM (INDI/JINDI torque feedback).
    { 20.0f, 15.0f },
    
    // .sensors — SensorsConfig { has_baro, has_can_imx5_ins, has_mocap_link, has_encoder_rpm, encoder_motor_map }
    // MOTOR_PROTOCOL is MOTOR_PROTO_IOMCU on this board (config.mk override) — no DShot telemetry, so CAN
    // shaft-encoder RPM (src/sensors/EncoderRPM.hpp) substitutes in, all 4 motors now instrumented:
    //   NODE_ID 0 (CAN 0x70) — front-right (FR, logical 0)
    //   NODE_ID 1 (CAN 0x71) — back-right  (RR, logical 3)
    //   NODE_ID 2 (CAN 0x72) — back-left   (RL, logical 1)
    //   NODE_ID 3 (CAN 0x73) — front-left  (FL, logical 2)
    // see externalMCUs/Encoder_CAN/Feather_Code/Feather_Code.ino's NODE_ID comment for which physical board is which.
    {
        true,           // has_baro
        false,           // has_can_imx5_ins
        false,           // has_mocap_link
        true,           // has_encoder_rpm
        { 0, 3, 1, 2 }, // encoder_motor_map — [node0..node3] -> logical motor (FR=0/RL=1/FL=2/RR=3)
    },

    // .controllers — ControllersConfig { switch_pos[3], stick_max_angle_deg }
    // Controller flown at each ctrl_select switch position [low, mid, high];
    // poshold_switch high runs POS_HOLD around it, low/mid flies it directly (STABILIZE).
    { { AttCtrl::PID, AttCtrl::PID, AttCtrl::PID },  // PID only — the others are not yet bench/flight tuned on this airframe
      30.0f },  // stick_max_angle_deg — ArduPilot ANGLE_MAX default (was 1 rad = 57°)
    
    // .logging — LoggingConfig { log_rate_hz, enable } — one LogEnableConfig flag per message type
    {
        50.0f, // log_rate_hz
        { true, // att
          false, // lin
          true, // rcin
          true, // outp
          true, // rpms
          false, // strt
          true, // imu1
          true, // imu2
          true, // imu3
          false, // indi
          false, // baro
          false, // ptun
          false, // mocp
          false, // traj
          false, // mavl
          true, // strn (STN1..STN4)
          false, // jindi
          false, // jpid
          true, // ctun (AttitudePID tuning)
          false }, // sidd (system-ID sweep)
    },

    // .sysid — SysIdConfig { rc_channel, axes[4], f_start_hz, f_stop_hz, duration_s, magnitude[4], fade_in_s, fade_out_s }
    // Each flip of rc_channel sweeps the next axis in axes[] (wraps); flipping back
    // down mid-sweep cancels it and keeps the same axis. See src/controllers/extras/SysId.hpp.
    // Magnitudes are untested placeholders — start small and work up.
    { 6,                                    // rc_channel (0-based, like kRcMaps); -1 disables sysid
      { SysIdAxis::ROLL, SysIdAxis::PITCH, SysIdAxis::NONE, SysIdAxis::NONE },  // axes, in order
      0.1f, 10.0f,                          // f_start_hz, f_stop_hz — 0.3-10 Hz: this airframe's ~0-5 Hz bandwidth with margin
      40.0f,                                // duration_s (includes the 2-cycle start dwell and both fades)
      { 0.1f, 0.1f, 0.05f, 0.05f },       // magnitude [roll, pitch, yaw, heave] — torque cmd / throttle fraction
      5.0f, 2.0f },                         // fade_in_s, fade_out_s
};
