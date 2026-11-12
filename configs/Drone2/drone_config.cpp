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
 * rate D-term filter 20 Hz. .indi's roll_rate/pitch_rate ("SLC",
 * 6.5 gain) are a different two-stage architecture with no ArduPilot rate-PID
 * equivalent, so they're untouched. .pid_pi is the full ArduPilot cascade:
 * angle P 4.5, stock rate PID, then the accel PI (0.7/0.8) on p_dot/80. Yaw gains are also
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
        { 0.135f, 0.4f, 0.008f, 0.5f, 20.0f, 0.0f, 20.0f }, // roll_rate
        { 0.135f, 0.4f, 0.008f, 0.5f, 20.0f, 0.0f, 20.0f }, // pitch_rate
        { 0.4f, 0.1f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.80f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        4.0f,  // yaw_stick_gain
    },

    // .indi — AttitudeIndiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, g1_seed_roll, g1_seed_pitch, indi_output_gain_roll, indi_output_gain_pitch, nlms_mu_pid, nlms_mu_indi, yaw_gain }
    {
        { 6.0f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // roll_att 
        { 6.0f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // pitch_att 
        { 14.0f,  2.0f, 0.7f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate
        { 14.0f,  2.0f, 0.7f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate
        { 0.4f, 0.1f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.80f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        0.050f,  // g1_seed_roll  (was 0.043 = Ixx from cal flights 3+4)
        0.050f,  // g1_seed_pitch (was 0.062 for the poor pitch fit; fit B pitch tracks better, 2026-11-10; 0.046 = Iyy)
        1.0f,    // indi_output_gain_roll (kappa) — 1.0 reproduces pre-adaptation behavior
        1.0f,    // indi_output_gain_pitch (kappa)
        0.0f,   // nlms_mu_pid — aggressive adaptation rate while PID/PID+PI active; needs bench/flight tuning
        0.0f,  // nlms_mu_indi — slow/trickle adaptation rate while INDI active; needs bench/flight tuning
        1.5f,    // yaw_gain
    },

    // .jindi — AttitudeJindiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, roll_accel, g1_seed_roll, g1_seed_pitch, indi_output_gain_pitch, h_roll, alpha, nlms_mu_pid, nlms_mu_indi, yaw_gain }
    // Unused while no .controllers.switch_pos entry names AttCtrl::JINDI; copied from
    // this drone's .indi values only to keep the struct valid — see Drone1's .jindi comment for the full rationale.
    {
        { 6.0f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // roll_att
        { 6.0f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // pitch_att
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // roll_rate
        { 6.5f,  0.20f, 0.0f,   10.0f, 30.0f, 0.0f, 30.0f }, // pitch_rate
        { 0.2f, 0.05f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.80f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        { 1.0f,  0.0f,  0.0f, 0.5f, 0.0f, 20.0f, 20.0f },    // roll_accel
        0.043f,  // g1_seed_roll  — same as .indi
        0.046f,  // g1_seed_pitch — same as .indi
        1.0f,    // indi_output_gain_pitch (kappa, pitch only)
        40.0f,   // h_roll — 1000/25
        1.0f,    // alpha — fully accel-path for now, not yet characterized
        0.0f,   // nlms_mu_pid
        0.0f,  // nlms_mu_indi
        1.5f,    // yaw_gain
    },

    // .pid_pi — AttitudePidPiGains { roll_att, pitch_att, roll_rate, pitch_rate, yaw_rate, yaw_hold, roll_accel, pitch_accel, yaw_stick_gain, accel_norm_roll, accel_norm_pitch }
    {
        { 6.0f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // roll_att 
        { 6.0f, 0.00f, 0.000f, 0.5f, 0.0f,  0.0f, 30.0f },  // pitch_att 
        { 0.17f, 0.135f, 0.006f, 0.5f, 20.0f, 0.0f, 20.0f }, // roll_rate  
        { 0.17f, 0.135f, 0.006f, 0.5f, 20.0f, 0.0f, 20.0f }, // pitch_rate 
        { 0.4f, 0.1f, 0.000f, 0.5f, 20.0f, 2.5f, 5.0f },  // yaw_rate
        { 0.80f, 0.050f, 0.000f, 0.3f, 0.0f,  0.0f, 30.0f }, // yaw_hold
        { 0.7f, 6.0f, 0.0f, 0.5f, 50.0f, 0.0f, 0.0f },     // roll_accel  (inner PI, kd=0 — ArduPilot _pid_accel_roll)
        { 0.7f, 6.0f, 0.0f, 0.5f, 50.0f, 0.0f, 0.0f },     // pitch_accel (inner PI, kd=0 — ArduPilot _pid_accel_pitch)
        4.0f,  // yaw_stick_gain
        140.0f, // accel_norm_roll  — converts measured p_dot (rad/s²) to torque cmd fraction (0-1) for the inner PI
        140.0f, // accel_norm_pitch — same for q_dot. FUSED reads ~1.1x STRAIN; att_sim (Circle4/5 sensor models, 2026-11-12) keeps 140 fine on FUSED (GM 6.9/8.0)
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
      1.5f,    // max_rate (was 1.0; raised 2026-11-11 to stay above TrajectoryTracker::YAW_RATE_MAX 1.2)
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
    // Thrust stand (Calibration/thrust_data/StepsTestV2_2026-09-15, 2 runs,
    // 1200-1900 us, ~15.3 V): cubic in mechanical RPM, rms 0.05 N. Scaled x1.048
    // so the in-flight accelerometer at 2.7 kg balances (cal flights 3/4,
    // 2026-10-30): 6.62 N/motor at the 5990 RPM hover. Pure quadratic
    // equivalent ~1.85e-7*rpm^2 N (C_T 0.060). Reaction torque 0.0163*F_N N·m.
    // max_thrust_n is the curve at 1950 us (pwm_max, ~8730 RPM). That makes
    // T_MAX = 6.24 N·m, within ~4% of the measured 6.0-6.2 N·m per unit roll/pitch cmd at
    // hover, so normalize_torque() is consistent with the mixer (was 27.49 -> 2x off).
    {
        0.3f,                                                   // arm_length_m
        0.0628879f, -8.01856e-5f, 1.84985e-7f, 1.88320e-12f,    // motor_c0, motor_c1, motor_c2, motor_c3
        14.7f,                                                  // max_thrust_n
    },

    // .ang_accel_filt — AngAccelFilterConfig { lpf_hz, lpf_extra_hz, source_roll, source_pitch, rpm_lpf_hz, imu_lpf_hz, imu_lpf_extra_hz }
    // lpf_hz/lpf_extra_hz apply to the strain fit (and to the IMU p_dot/q_dot when both
    // imu_ fields are 0). imu_lpf_hz/imu_lpf_extra_hz: the IMU estimate's own chain —
    // 2-pole 20 Hz + 1-pole 15 Hz (≈ 11 + 10.6 ms at low frequency), since the IMU estimate
    // with a single 15 Hz pole is too noisy to fly. rpm_lpf_hz gives the
    // Unmixer's RPM torque its own single pole: 30 Hz ≈ 5.3 ms delay (was the shared
    // 15 Hz ≈ 10.6 ms). Raw-RPM torque is ~91% signal (flight 7), so it needs little filtering.
    // source_roll/source_pitch pick each axis's p_dot/q_dot independently: IMU, STRAIN (strain + RPM
    // fit B) or FUSED (fit B's inputs + 3 IMU taps, gust_fit_F_fused_py.mat) — SACC logs all three
    // either way. STRAIN/FUSED crossfade to IMU while the strain array (or RPM) is BAD.
    // FUSED reads ~0.9 of the true angular accel vs ~0.7 for STRAIN — effectively a higher G1.
    // The strain fits were made with a single 15 Hz pole only — replayed on cal flights 3/4,
    // the extra 20 Hz 2nd-order stage here adds 10-20 ms and drops R^2 from ~0.65 to ~0.55;
    // fly strain as { 0.0f, 15.0f, ... }. FUSED was trained on the 20 + 15 Hz IMU chain below —
    // refit it (strain_study export_F.py) if imu_lpf_hz/imu_lpf_extra_hz change.
    { 0.0f, 15.0f, // lpf_hz, lpf_extra_hz, 0 to disable ( the first filter is 2nd-order, the second is 1st-order)
      AngAccelSource::STRAIN, // source_roll  — IMU, STRAIN or FUSED
      AngAccelSource::STRAIN, // source_pitch — IMU, STRAIN or FUSED
      30.0f,   // rpm_lpf_hz — Unmixer's single-pole RPM torque filter
      20.0f, 15.0f }, // imu_lpf_hz, imu_lpf_extra_hz — IMU p_dot/q_dot only (2nd-order, then 1st-order)


    
    // .sensors — SensorsConfig { has_baro, has_can_imx5_ins, has_mocap_link, has_encoder_rpm, encoder_motor_map, can2_enable, imx5_can_bus, has_strain_gauges, encoder_rpm_sign, imx5_mount_rpy_deg, imx5_r_quat }
    // MOTOR_PROTOCOL is MOTOR_PROTO_IOMCU on this board (config.mk override) — no DShot telemetry, so CAN
    // shaft-encoder RPM (src/sensors/EncoderRPM.hpp) substitutes in, all 4 motors now instrumented:
    //   NODE_ID 0 (CAN 0x70) — front-right (FR, logical 0)
    //   NODE_ID 1 (CAN 0x71) — back-right  (RR, logical 3)
    //   NODE_ID 2 (CAN 0x72) — back-left   (RL, logical 1)
    //   NODE_ID 3 (CAN 0x73) — front-left  (FL, logical 2)
    // see externalMCUs/Encoder_CAN/Feather_Code/Feather_Code.ino's NODE_ID comment for which physical board is which.
    {
        true,           // has_baro
        true,           // has_can_imx5_ins — IMX6 on CAN2 (see imx5_can_bus)
        false,           // has_mocap_link
        true,           // has_encoder_rpm
        { 0, 3, 1, 2 }, // encoder_motor_map — [node0..node3] -> logical motor (FR=0/RL=1/FL=2/RR=3)
        true,           // can2_enable — IMX6 (IMX5 driver) is on the CAN2 port
        2,              // imx5_can_bus — encoders stay on CAN1
        true,           // has_strain_gauges — strain array fitted: run its poll + the strain/fused fits (INDI itself only needs RPM)
        { -1, -1, +1, +1 }, // encoder_rpm_sign [FR, RL, FL, RR] — FR/RL spin CCW, FL/RR CW; assumes the encoders read + for CW (TODO: confirm on the bench with ENC,read)
        { 180.0f, 0.0f, -45.0f }, // imx5_mount_rpy_deg — IMX6 upside down, 45° off in yaw (IMX's own insRotation now zeroed). LOG0023's unrotated rates fit exactly this rotation.
        1e-1f,          // imx5_r_quat — IMX6 sends quaternions at 400 Hz, 2x the IMX5's 200 Hz, so 2x Drone1's 5e-2 keeps the same fusion strength
    },

    // .controllers — ControllersConfig { switch_pos[3], stick_max_angle_deg }
    // Controller flown at each ctrl_select switch position [low, mid, high];
    // poshold_switch high runs POS_HOLD around it, low/mid flies it directly (STABILIZE).
    { { AttCtrl::PID, AttCtrl::PID_PI, AttCtrl::INDI },  // PID only — the others are not yet bench/flight tuned on this airframe
      30.0f },  // stick_max_angle_deg — ArduPilot ANGLE_MAX default (was 1 rad = 57°)
    
    // .logging — LoggingConfig { log_rate_hz, prealloc_mb, enable } — one LogEnableConfig flag per message type
    {
        100.0f, // log_rate_hz
        20,     // prealloc_mb — contiguous SD space reserved per log file
        { true, // att
          true, // lin
          true, // rcin
          true, // outp
          true, // rpms
          false, // strt
          true, // imu1
          false, // imu2
          false, // imu3
          true, // indi
          false, // baro
          true, // ptun
          false, // mocp
          false, // traj
          false, // mavl
          true, // strn (STN1..STN4)
          false, // jindi
          false, // jpid
          false, // ctun (AttitudePID tuning)
          false, // sidd (system-ID sweep)
          true }, // pidp (AttitudePIDPI cascade; also runs it in shadow)
    },

    // .sysid — SysIdConfig { rc_channel, axes[4], f_start_hz, f_stop_hz, duration_s, magnitude[4], fade_in_s, fade_out_s }
    // Each flip of rc_channel sweeps the next axis in axes[] (wraps); flipping back
    // down mid-sweep cancels it and keeps the same axis. See src/controllers/extras/SysId.hpp.
    // Magnitudes are untested placeholders — start small and work up.
    { 8,                                    // rc_channel (0-based, like kRcMaps); -1 disables sysid
      { SysIdAxis::ROLL, SysIdAxis::PITCH, SysIdAxis::NONE, SysIdAxis::NONE },  // axes, in order
      0.5f, 12.0f,                          // f_start_hz, f_stop_hz — 0.3-10 Hz: this airframe's ~0-5 Hz bandwidth with margin
      40.0f,                                // duration_s (includes the 2-cycle start dwell and both fades)
      { 0.15f, 0.15f, 0.05f, 0.05f },       // magnitude [roll, pitch, yaw, heave] — torque cmd / throttle fraction
      5.0f, 2.0f },                         // fade_in_s, fade_out_s
};
