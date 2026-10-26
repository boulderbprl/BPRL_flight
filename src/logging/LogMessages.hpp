#pragma once
#include <cstdint>
#include <cstddef>

/*
 * Binary log record format (ArduPilot DataFlash compatible):
 *   Header:   one FMT record per message type (89 bytes each, written at file open)
 *   Data:     [0xA3][0x95][msg_id][...packed struct body...]
 *
 * FMT record layout (89 bytes total):
 *   [0xA3][0x95][0x80][type_u8][length_u8][name_4][format_16][labels_64]
 *   length = total data record size including 3-byte header = 3 + sizeof(body)
 *   format = ArduPilot type codes: Q=uint64 H=uint16 f=float i=int32 h=int16 B=uint8
 *
 * Files produced by this logger can be opened in UAV Log Viewer
 * (plot.ardupilot.org). TimeUS must be the first field for the time axis.
 *
 * To add a new log set:
 *   1. Define a packed struct and LOG_MSG_* constant below.
 *   2. Add one entry to kLogDefs[] with the format code string and labels.
 *   3. Snapshot the data and call logger.write(LOG_MSG_*, msg) in LogThread.
 */

/* ── Position-hold tuning log (PTUN) ─────────────────────────────────────────
 * Temporary, gain-tuning-only message. Flip to 0 to strip it out of the
 * schema header and LogThread writes entirely once tuning is done. */
#define LOG_PTUN_ENABLED 1

/* ── Message IDs ─────────────────────────────────────────────────────────── */
constexpr uint8_t LOG_MSG_FMT  = 0x80U;  // schema header — written once at file open
constexpr uint8_t LOG_MSG_ATT  = 0x09U;  // angular states: attitude + rates + angular accel
constexpr uint8_t LOG_MSG_LIN  = 0x0AU;  // linear states: position + velocity + linear accel
constexpr uint8_t LOG_MSG_RCIN = 0x05U;  // RC stick inputs + flight mode + arm state
constexpr uint8_t LOG_MSG_OUTP = 0x06U;  // controller outputs entering the motor mixer
constexpr uint8_t LOG_MSG_RPMS = 0x07U;  // per-motor mechanical RPM from DShot GCR telemetry
constexpr uint8_t LOG_MSG_STRT = 0x08U;  // strain RATE sensor (CAN 0x69, 4 arms, in development) — not to be confused with STN1..STN4 (strain gauges)
constexpr uint8_t LOG_MSG_IMU1 = 0x0BU;  // raw accel + gyro, IMU1 (SPI1, CS=PG1)  (board-conditional chip — ICM-45686 on Drone1/CubeOrangePlus, ICM-42688 on Drone3/Orqa, ICM-20649 on Drone2/CubeBlue) (body-frame, post-rotation, pre-EKF)
constexpr uint8_t LOG_MSG_IMU2 = 0x0CU;  // raw accel + gyro, IMU2 (SPI4, CS=PC15) (board-conditional chip, see IMU1) (body-frame, post-rotation, pre-EKF)
constexpr uint8_t LOG_MSG_IMU3 = 0x0DU;  // raw accel + gyro, IMU3 (SPI4, CS=PC13) (board-conditional chip, see IMU1) (body-frame, post-rotation, pre-EKF)
constexpr uint8_t LOG_MSG_INDI = 0x0EU;  // INDI shadow-controller diagnostics (always logged, regardless of FlightStateMachine's active controller)
constexpr uint8_t LOG_MSG_BARO = 0x0FU;  // barometric pressure/temperature/altitude (MS5611, SPI1, CS=PD7)
constexpr uint8_t LOG_MSG_PTUN = 0x10U;  // TEMP: pos-hold NE tuning — outer pos + inner vel loop targets/errors, shadow lean-angle target
constexpr uint8_t LOG_MSG_MOCP = 0x11U;  // raw mocap position/velocity estimate, pre-EKF (MAVLink VISION_POSITION/SPEED_ESTIMATE)
constexpr uint8_t LOG_MSG_TRAJ = 0x12U;  // trajectory tracker state + reference, and POS_HOLD's heading target / yaw-rate command
constexpr uint8_t LOG_MSG_MAVL = 0x13U;  // MAVLink receive counters + last trajectory command and its result (5 Hz)
constexpr uint8_t LOG_MSG_STN1 = 0x14U;  // strain GAUGE array (I2C 0x09), ARM1 front right, channels 0-8, + chunk 0 receive rate
constexpr uint8_t LOG_MSG_STN2 = 0x15U;  // strain gauge array, ARM2 back right,  channels 9-17
constexpr uint8_t LOG_MSG_STN3 = 0x16U;  // strain gauge array, ARM3 back left,   channels 18-26, + chunk 1 receive rate
constexpr uint8_t LOG_MSG_STN4 = 0x17U;  // strain gauge array, ARM4 front left,  channels 27-35
constexpr uint8_t LOG_MSG_JKFT = 0x18U;  // Jerk estimates (z and roll) based on roll rate and four strain-rate channels (WIP)
constexpr uint8_t LOG_MSG_PIDJ = 0x19U;  // original single-stage AttitudePIDJerk diagnostics — retired, never written (rewritten controller logs LOG_MSG_JPID)
// 0x1A reserved — was INDJ (AttitudeINDIJerk, removed). Don't reuse: old logs still carry it.
constexpr uint8_t LOG_MSG_JINDI = 0x1BU; // AttitudeJINDI diagnostics — roll_jerk as an independent convex-combined path, see Attitude_JINDI.hpp
constexpr uint8_t LOG_MSG_JPID = 0x1CU;  // AttitudePIDJerk diagnostics — angle→rate→accel PI + jerk D on roll, see Attitude_PID_Jerk.hpp (new layout, not the retired LOG_MSG_PIDJ)
constexpr uint8_t LOG_MSG_CTUN = 0x1DU;  // AttitudePID roll/pitch tuning: angle + rate targets, errors, rate integrators (0x10 is PTUN — old logs' "CTUN" at 0x10 was today's PTUN)

/* ── Packed message bodies ───────────────────────────────────────────────── */

struct __attribute__((packed)) LogMsgATT {
    uint64_t time_us;    // microseconds since boot (TimeUS — required by UAV Log Viewer)
    float    roll;       // rad  (from g_euler[0])
    float    pitch;      // rad  (from g_euler[1])
    float    yaw;        // rad  (from g_euler[2])
    float    p;          // rad/s  body-frame roll rate
    float    q;          // rad/s  body-frame pitch rate
    float    r;          // rad/s  body-frame yaw rate
    float    p_dot;      // rad/s²  body-frame roll angular acceleration
    float    q_dot;      // rad/s²  body-frame pitch angular acceleration
    float    r_dot;      // rad/s²  body-frame yaw angular acceleration
};
// Format: "Qfffffffff"   Body size: 8+9×4 = 44 B   Record: 47 B

struct __attribute__((packed)) LogMsgLIN {
    uint64_t time_us;
    float    x;       // m  NED inertial position
    float    y;       // m
    float    z;       // m  (down positive)
    float    u;       // m/s  body-frame translational velocity
    float    v;       // m/s
    float    w;       // m/s
    float    u_dot;   // m/s²  body-frame translational acceleration (gravity-corrected)
    float    v_dot;   // m/s²
    float    w_dot;   // m/s²
};
// Format: "Qfffffffff"   Body: 8+9×4 = 44 B   Record: 47 B

struct __attribute__((packed)) LogMsgRCIN {
    uint64_t time_us;
    float    roll_stk;    // [-1, 1]  roll setpoint from RC
    float    pitch_stk;   // [-1, 1]  pitch setpoint from RC
    float    yaw_stk;     // [-1, 1]  yaw rate demand from RC
    float    thr_stk;     // [0, 1]   throttle from RC
    float    flight_mode; // [-1, 1]  raw flight-mode switch; <-0.33=STABILIZE, -0.33..0.33=ALT_HOLD, >0.33=POS_HOLD
    float    indi_stk;    // [-1, 1]  raw ctrl_select switch; position -> controller per ControllersConfig::switch_pos
    uint8_t  armed;       // 0=disarmed, 1=armed
};
// Format: "QffffffB"   Body: 8+6×4+1 = 33 B   Record: 36 B

struct __attribute__((packed)) LogMsgOUTP {
    uint64_t time_us;
    float    roll_tq;     // [-1, 1]  normalized roll torque command (PID output)
    float    pitch_tq;    // [-1, 1]  normalized pitch torque command
    float    yaw_tq;      // [-1, 1]  normalized yaw torque command
    float    throttle;    // [0, 1]   shaped throttle entering MotorMixer
};
// Format: "Qffff"   Body: 8+4×4 = 24 B   Record: 27 B

struct __attribute__((packed)) LogMsgRPMS {
    uint64_t time_us;
    int32_t  rpm0;   // FR  mechanical RPM (eRPM ÷ 7 pole pairs); 0 if no valid GCR frame
    int32_t  rpm1;   // RL
    int32_t  rpm2;   // FL
    int32_t  rpm3;   // RR
};
// Format: "Qiiii"   Body: 8+4×4 = 24 B   Record: 27 B

struct __attribute__((packed)) LogMsgSTRT {
    uint64_t time_us;
    int16_t  s0;    // arm FR strain rate (raw int16 from CAN 0x69)
    int16_t  s1;    // arm RL
    int16_t  s2;    // arm FL
    int16_t  s3;    // arm RR
    uint8_t  valid; // 1 once at least one CAN frame has arrived
};
// Format: "QhhhhB"   Body: 8+4×2+1 = 17 B   Record: 20 B

struct __attribute__((packed)) LogMsgJKFT {
    uint64_t time_us;
    float   JerkZ;    // m/s^3, raw fit output
    float   Pdd;      // rad/s^3, per-channel-bias-corrected (see JerkFit.hpp::estimate_jerk()'s strain_bias param)
    uint8_t valid;    // 1 once at least one strain rate CAN frame has arrived
    uint8_t cal_active; // 1 while the channel-6 strain-bias-cal switch is held — see radio_strain_cal() / ControlThread's calibration block
};
// Format: "QffBB"

struct __attribute__((packed)) LogMsgIMU {
    uint64_t time_us;
    float    ax;        // m/s²  body-frame accel (post axis-rotation, calibration-bias-corrected)
    float    ay;
    float    az;
    float    gx;         // rad/s  body-frame gyro (post axis-rotation, calibration-bias-corrected)
    float    gy;
    float    gz;
    uint8_t  valid;      // 1 if this IMU is currently reporting valid data
};
// Format: "QffffffB"   Body: 8+6×4+1 = 33 B   Record: 36 B
// Shared body layout for IMU1/IMU2/IMU3 — same struct, three distinct msg_ids/names
// so each IMU shows up as its own series in the log viewer.

struct __attribute__((packed)) LogMsgINDI {
    uint64_t time_us;
    float    unmix_roll;   // N·m  measured roll torque from Unmixer (RPM feedback)
    float    unmix_pitch;  // N·m  measured pitch torque from Unmixer
    float    delta_roll;   // N·m  INDI incremental roll torque correction
    float    delta_pitch;  // N·m  INDI incremental pitch torque correction
    float    cmd_roll;     // [-1, 1]  normalized roll torque commanded by INDI
    float    cmd_pitch;    // [-1, 1]  normalized pitch torque commanded by INDI
    float    accel_roll;   // rad/s²  INDI rate-PID commanded angular acceleration, roll
    float    accel_pitch;  // rad/s²  INDI rate-PID commanded angular acceleration, pitch
    float    g1_roll;      // N·m per rad/s²  live NLMS-adapted G1_hat, roll  (seed: DroneConfig::AttitudeIndiGains::g1_seed_roll)
    float    g1_pitch;     // N·m per rad/s²  live NLMS-adapted G1_hat, pitch (seed: DroneConfig::AttitudeIndiGains::g1_seed_pitch)
};
// Format: "Qffffffff"   Body: 8+8×4 = 40 B   Record: 43 B
// Always populated while a switch_pos entry names AttCtrl::INDI, whether or
// not INDI is the controller actually flying (OUTP) — shadow mode.

struct __attribute__((packed)) LogMsgPIDJ {
    uint64_t time_us;
    float    roll_jerk;      // rad/s^3  jerk estimate fed into the roll rate loop's damping term (JerkFit.hpp)
    float    cmd_roll;       // [-1, 1]  normalized roll torque commanded by AttitudePIDJerk (includes jerk term)
    float    roll_rate_tgt;  // rad/s    outer roll-loop angular rate target (AttitudePIDJerk::update() diagnostic)
    float    cmd_yaw;        // [-1, 1]  normalized yaw torque commanded by AttitudePIDJerk (no jerk term — identical to AttitudePID)
};
// Format: "Qffff"   Body: 8+4×4 = 24 B   Record: 27 B
// Retired — the original single-stage AttitudePIDJerk layout; the rewritten
// controller logs LOG_MSG_JPID instead and nothing calls
// logger.write(LOG_MSG_PIDJ, ...) anymore. Struct/msg_id kept only so old LOG*.BIN files taken before the
// switch still decode in UAV Log Viewer.

struct __attribute__((packed)) LogMsgJINDI {
    uint64_t time_us;
    float    unmix_roll;    // N·m  measured roll torque from Unmixer (RPM feedback)
    float    unmix_pitch;   // N·m  measured pitch torque from Unmixer
    float    tau_accel_roll; // N·m  accel path's own torque reference, pre-blend (instant-actuation assumption)
    float    tau_jerk_roll;  // N·m  jerk path's own torque reference, pre-blend (actuator-dynamics assumption, anchored to its own prior command)
    float    cmd_roll;      // [-1, 1]  normalized roll torque commanded by AttitudeJINDI (the alpha-blended result)
    float    cmd_pitch;     // [-1, 1]  normalized pitch torque commanded by AttitudeJINDI
    float    accel_roll;    // rad/s²  roll's rate-PID commanded angular acceleration (also the target fed into the accel→jerk stage)
    float    accel_pitch;   // rad/s²  pitch's rate-PID commanded angular acceleration
    float    g1_roll;       // N·m per rad/s²  live NLMS-adapted G1_hat, roll — shared by BOTH of roll's paths (no separate G2, see Attitude_JINDI.hpp)
    float    jerk_filt;     // rad/s³  measured roll jerk (JKFT.Pdd) after JINDI's JERK_MEAS_LPF_HZ lowpass + JERK_MEAS_HPF_HZ highpass — what the jerk error uses (AttitudeJINDI::roll_jerk_filt()). Was g1_pitch.
    float    jerk_cmd_roll; // rad/s³  reference/target jerk fed into the jerk path (AttitudeJINDI::jerk_cmd_roll()) — compare against JKFT.Pdd, the actual/measured roll_jerk
    float    alpha_roll;    // convex-combination weight, roll — static per-drone config value (DroneConfig::AttitudeJindiGains::alpha), repeats every tick, logged for reference
    float    tau_roll;      // N·m  alpha-blended combined torque reference, pre-normalise (AttitudeJINDI::combined_torque_roll()) — tau_accel_roll/tau_jerk_roll blended by alpha_roll, what normalize_torque() turns into cmd_roll
};
// Format: "Qfffffffffffff"   Body: 8+13×4 = 60 B   Record: 63 B
// Gated by DroneConfig::LoggingConfig::enable.jindi, same mechanism as
// LOG_MSG_INDI's log_en.indi — always populated while AttitudeJINDI is in
// the drone's config (a ControllersConfig::switch_pos entry names AttCtrl::JINDI), regardless of
// whether it's the controller actually driving OUTP (see $TEL's active
// controller index).

struct __attribute__((packed)) LogMsgJPID {
    uint64_t time_us;
    float    rate_tgt;   // rad/s    angle loop output — roll rate target
    float    accel_tgt;  // rad/s²   rate loop output — roll angular accel target
    float    jerk_tgt;   // rad/s³   d/dt(accel_tgt), 20 Hz lowpass — diagnostic only (the D term uses jerk_tgt = 0)
    float    torque_pi;  // N·m      accel stage P+I contribution
    float    torque_d;   // N·m      jerk D contribution (−Kd · 20 Hz-lowpassed jerk)
    float    torque_roll; // N·m     total roll torque (torque_pi + torque_d, clamped), pre-normalise
    float    cmd_roll;   // [-1, 1]  normalised roll torque commanded by AttitudePIDJerk
    float    cmd_pitch;  // [-1, 1]  normalised pitch torque commanded by AttitudePIDJerk (plain cascade, same as AttitudePID)
    float    cmd_yaw;    // [-1, 1]  normalised yaw torque commanded by AttitudePIDJerk (same as AttitudePID)
};
// Format: "Qfffffffff"   Body: 8+9×4 = 44 B   Record: 47 B
// Measured roll rate / P_DOT / strain jerk are not repeated here — see
// ATT.P, ATT.Pdot and JKFT.Pdd.
// Gated by DroneConfig::LoggingConfig::enable.jpid — always populated while
// AttitudePIDJerk is in the drone's config (a ControllersConfig::switch_pos
// entry names AttCtrl::PID_JERK), regardless of whether it's the controller actually
// driving OUTP (see $TEL's active controller index). Zeroed otherwise.

struct __attribute__((packed)) LogMsgBARO {
    uint64_t time_us;
    float    pressure_pa;  // Pa, compensated
    float    temp_c;       // °C, compensated
    float    alt_m;        // m, positive up, relative to MS5611::init() boot-time reference
    uint8_t  valid;        // 1 once MS5611 init + warm-up zero-reference capture completed
};
// Format: "QfffB"   Body: 8+3×4+1 = 21 B   Record: 24 B

#if LOG_PTUN_ENABLED
struct __attribute__((packed)) LogMsgPTUN {
    uint64_t time_us;
    float    pos_n_tgt;   // m    outer pos-hold loop N target (shadow — POS_HOLD flies as STABILIZE while tuning)
    float    pos_n_err;   // m    outer loop N error, target - measured
    float    pos_e_tgt;   // m    outer pos-hold loop E target (shadow)
    float    pos_e_err;   // m    outer loop E error
    float    vel_n_tgt;   // m/s  inner vel loop N target (shadow)
    float    vel_n_err;   // m/s  inner loop N error, target - measured
    float    vel_e_tgt;   // m/s  inner vel loop E target (shadow)
    float    vel_e_err;   // m/s  inner loop E error
    float    roll_tgt;    // rad  lean angle pos-hold would have sent to the attitude controller (shadow)
    float    pitch_tgt;   // rad  lean angle pos-hold would have sent to the attitude controller (shadow)
    float    climb_rate_tgt; // m/s  alt-hold climb rate target (positive = descend), AltControl::climb_rate_tgt()
    float    climb_rate_err; // m/s  climb rate error, target - measured (state[W])
};
// Format: "Qffffffffffff"   Body: 8+12×4 = 56 B   Record: 59 B
#endif

struct __attribute__((packed)) LogMsgCTUN {
    uint64_t time_us;
    float    roll_tgt;       // rad    roll angle target into AttitudePID (scaled stick, or POS_HOLD's lean target)
    float    roll_err;       // rad    roll_tgt - measured roll
    float    roll_rate_tgt;  // rad/s  angle loop output = rate loop target (before the rate PID's target filter)
    float    roll_rate_err;  // rad/s  roll_rate_tgt - measured p
    float    pitch_tgt;      // rad
    float    pitch_err;      // rad
    float    pitch_rate_tgt; // rad/s
    float    pitch_rate_err; // rad/s  pitch_rate_tgt - measured q
    float    roll_rate_i;    // roll rate PID integrator (normalised output units, ±imax)
    float    pitch_rate_i;   // pitch rate PID integrator
};
// Format: "Qffffffffff"   Body: 8+10×4 = 48 B   Record: 51 B

struct __attribute__((packed)) LogMsgMOCP {
    uint64_t time_us;
    float    x;      // m    NED position, raw from VISION_POSITION_ESTIMATE
    float    y;      // m
    float    z;      // m
    float    vx;     // m/s  NED velocity, raw from VISION_SPEED_ESTIMATE
    float    vy;     // m/s
    float    vz;     // m/s
    uint8_t  valid;  // 1 once mocap link is connected and receiving (g_mocap.valid)
};
// Format: "QffffffB"   Body: 8+6×4+1 = 33 B   Record: 36 B

struct __attribute__((packed)) LogMsgTRAJ {
    uint64_t time_us;
    uint8_t  state;        // TrajState: 0 idle (POS_HOLD on the sticks), 1 moving to a point / to the circle, 2 holding a point, 3 on the circle
    float    ref_n;        // m      reference position N the controllers are tracking (0 while idle — see PTUN for the stick targets)
    float    ref_e;        // m
    float    ref_d;        // m      includes the pilot's height offset
    float    yaw_tgt;      // rad    HeadingControl's heading target (live whenever POS_HOLD is flying, trajectory or not)
    float    yaw_rate_cmd; // rad/s  HeadingControl's output to the attitude controller's yaw-rate loop
    float    height_off;   // m      throttle-stick offset on the trajectory's D (negative = raised)
    float    path_speed;   // m/s    speed along the path
};
// Format: "QBfffffff"   Body: 8+1+7×4 = 37 B   Record: 40 B

// MAVLinkThread's receive counters (MavlinkDiag, src/coms/MAVLink.hpp) —
// the same numbers "MAV,diag" prints over USB, for when the vehicle is flying
// and USB isn't connected. All counts are totals since boot: the rate of a
// message is the difference between two records divided by the time between
// them. Logged at about 5 Hz, not every log tick.
//
// TODO (once the trajectory-command / radio-link debugging is finished): this
// is the verbose debugging layout. Cut it down to what stays useful:
//   - log at 1-2 Hz;
//   - add the mocap link's received rate: the average
//     VISION_POSITION_ESTIMATE rate [Hz] since the previous MAVL record
//     (from the change in vision_pos_rx). It belongs here rather than in
//     MOCP because MOCP is written at 50 Hz, too fast to measure a ~30 Hz
//     signal — at 1-2 Hz each record averages over 15-30 messages;
//   - keep mocap_timeouts, traj_cmd_rx and traj_cmd_rej (plus
//     last_cmd/last_result if still wanted);
//   - drop the raw byte/frame/heartbeat/unknown/vision counters.
struct __attribute__((packed)) LogMsgMAVL {
    uint64_t time_us;
    uint32_t bytes_rx;        // bytes read off the MAVLink UART
    uint32_t frames_ok;       // frames parsed with a good CRC
    uint32_t frames_bad_crc;
    uint32_t heartbeat_rx;
    uint32_t vision_pos_rx;   // VISION_POSITION_ESTIMATE
    uint32_t vision_speed_rx; // VISION_SPEED_ESTIMATE
    uint32_t unknown_rx;      // good frames this firmware does not handle
    uint32_t mocap_timeouts;  // times the mocap link went >300 ms without a position
    uint32_t traj_cmd_rx;     // trajectory COMMAND_LONGs received (retries of one command count once)
    uint32_t traj_cmd_rej;    // ... answered with anything other than ACCEPTED
    uint16_t last_cmd;        // MAVLink id of the latest trajectory command: 31010 origin, 31011 point, 31012 circle, 31013 stop; 0 = none yet
    uint8_t  last_result;     // its MAV_RESULT: 0 accepted, 1 rejected for now, 2 denied, 4 failed; 255 = not answered yet
};
// Format: "QIIIIIIIIIIHB"   Body: 8+10×4+2+1 = 51 B   Record: 54 B

/*
 * Strain gauge array (src/sensors/StrainGauge.hpp) — 36 raw int16 channels
 * (0.1 uV/count), one message per arm, 9 channels each. The sensor delivers
 * the 36 channels as two separate 18-channel I2C chunks (chunk 0 = ARM1+ARM2,
 * chunk 1 = ARM3+ARM4), so the first message of each chunk (STN1, STN3)
 * also carries that chunk's measured receive rate.
 */
struct __attribute__((packed)) LogMsgSTNA {  // STN2, STN4
    uint64_t time_us;
    int16_t  d[9];   // this arm's 9 channels, in channel order
};
// Format: "Qhhhhhhhhh"   Body: 8+9×2 = 26 B   Record: 29 B

struct __attribute__((packed)) LogMsgSTNH {  // STN1, STN3
    uint64_t time_us;
    int16_t  d[9];   // this arm's 9 channels, in channel order
    uint16_t hz;     // chunks actually received from the sensor per second, averaged
                     // over the last ~1 s — the I2C data rate, NOT the logging rate
    uint8_t  valid;  // 1 once at least one full 36-channel frame has been read
};
// Format: "QhhhhhhhhhHB"   Body: 8+9×2+2+1 = 29 B   Record: 32 B

/* ── Log descriptor table ────────────────────────────────────────────────── */

struct LogDef {
    uint8_t     msg_id;
    // Logger::write_schema_header() copies these into LogFmtHdr's fixed-size
    // FMT fields via strncpy(dst, src, sizeof(dst) - 1) — reserving 1 byte for
    // the null terminator — except `name`, which uses the full sizeof(dst)
    // since it's a raw positional (non-null-terminated) 4-byte field. A
    // string longer than its field's usable length is silently truncated by
    // strncpy, not rejected — UAV Log Viewer then shows the record with
    // fields missing past the cut, with no build or runtime error to flag it
    // (this happened once already, see the PTUN entry below). The
    // static_assert after kLogDefs[] below catches this at compile time for
    // every entry, so the exact limits only need to be correct here, not
    // re-verified by hand per entry.
    const char *name;    // ≤4 chars   (LogFmtHdr::name[4],    full 4 bytes used, no null reserved)
    const char *fmt;     // ≤15 chars  (LogFmtHdr::format[16], 1 byte reserved for null)
    const char *labels;  // ≤63 chars  (LogFmtHdr::labels[64], 1 byte reserved for null); TimeUS must be first
    size_t      body_size;
};

constexpr LogDef kLogDefs[] = {
    { LOG_MSG_ATT,
      "ATT",
      "Qfffffffff",
      "TimeUS,Roll,Pitch,Yaw,P,Q,R,Pdot,Qdot,Rdot",
      sizeof(LogMsgATT) },

    { LOG_MSG_LIN,
      "LIN",
      "Qfffffffff",
      "TimeUS,X,Y,Z,U,V,W,Udot,Vdot,Wdot",
      sizeof(LogMsgLIN) },

    { LOG_MSG_RCIN,
      "RCIN",
      "QffffffB",
      "TimeUS,RollStk,PitchStk,YawStk,ThrStk,FlightMode,IndiStk,Armed",
      sizeof(LogMsgRCIN) },

    { LOG_MSG_OUTP,
      "OUTP",
      "Qffff",
      "TimeUS,RollTq,PitchTq,YawTq,Thr",
      sizeof(LogMsgOUTP) },

    { LOG_MSG_RPMS,
      "RPMS",
      "Qiiii",
      "TimeUS,RPM0,RPM1,RPM2,RPM3",
      sizeof(LogMsgRPMS) },

    { LOG_MSG_STRT,
      "STRT",
      "QhhhhB",
      "TimeUS,S0,S1,S2,S3,Valid",
      sizeof(LogMsgSTRT) },

    { LOG_MSG_JKFT,
      "JKFT",
      "QffBB",
      "TimeUS,JerkZ,Pdd,Valid,CalAct",
      sizeof(LogMsgJKFT) },

    { LOG_MSG_IMU1,
      "IMU1",
      "QffffffB",
      "TimeUS,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,Valid",
      sizeof(LogMsgIMU) },

    { LOG_MSG_IMU2,
      "IMU2",
      "QffffffB",
      "TimeUS,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,Valid",
      sizeof(LogMsgIMU) },

    { LOG_MSG_IMU3,
      "IMU3",
      "QffffffB",
      "TimeUS,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,Valid",
      sizeof(LogMsgIMU) },

    { LOG_MSG_INDI,
      "INDI",
      "Qffffffffff",
      "TimeUS,UnmixR,UnmixP,DeltaR,DeltaP,CmdR,CmdP,AccR,AccP,G1R,G1P",
      sizeof(LogMsgINDI) },

    { LOG_MSG_BARO,
      "BARO",
      "QfffB",
      "TimeUS,Press,Temp,Alt,Valid",
      sizeof(LogMsgBARO) },

#if LOG_PTUN_ENABLED
    { LOG_MSG_PTUN,
      "PTUN",
      "Qffffffffffff",
      // labels[] is a fixed 64-byte FMT field (63 usable chars after the null
      // terminator, see Logger.cpp's LogFmtHdr) — the descriptive names used
      // in the struct comments above don't fit; strncpy silently truncates
      // past the limit, which is why UAV Log Viewer previously only showed
      // fields up to VelETgt. Keep this under 63 chars.
      "TimeUS,PNT,PNE,PET,PEE,VNT,VNE,VET,VEE,RolT,PitT,ClbT,ClbE",
      sizeof(LogMsgPTUN) },
#endif

    { LOG_MSG_CTUN,
      "CTUN",
      "Qffffffffff",
      "TimeUS,RolT,RolE,RRT,RRE,PitT,PitE,PRT,PRE,RI,PI",
      sizeof(LogMsgCTUN) },

    { LOG_MSG_MOCP,
      "MOCP",
      "QffffffB",
      "TimeUS,X,Y,Z,VX,VY,VZ,Valid",
      sizeof(LogMsgMOCP) },

    { LOG_MSG_TRAJ,
      "TRAJ",
      "QBfffffff",
      "TimeUS,State,RefN,RefE,RefD,YawT,YawR,HOff,Spd",
      sizeof(LogMsgTRAJ) },

    { LOG_MSG_MAVL,
      "MAVL",
      "QIIIIIIIIIIHB",
      "TimeUS,Bytes,Ok,Bad,HB,VisP,VisV,Unk,MocTO,TrjRx,TrjRej,Cmd,Res",
      sizeof(LogMsgMAVL) },

    { LOG_MSG_STN1,
      "STN1",
      "QhhhhhhhhhHB",
      "TimeUS,D0,D1,D2,D3,D4,D5,D6,D7,D8,Hz,Valid",
      sizeof(LogMsgSTNH) },

    { LOG_MSG_STN2,
      "STN2",
      "Qhhhhhhhhh",
      "TimeUS,D9,D10,D11,D12,D13,D14,D15,D16,D17",
      sizeof(LogMsgSTNA) },

    { LOG_MSG_STN3,
      "STN3",
      "QhhhhhhhhhHB",
      "TimeUS,D18,D19,D20,D21,D22,D23,D24,D25,D26,Hz,Valid",
      sizeof(LogMsgSTNH) },

    { LOG_MSG_STN4,
      "STN4",
      "Qhhhhhhhhh",
      "TimeUS,D27,D28,D29,D30,D31,D32,D33,D34,D35",
      sizeof(LogMsgSTNA) },

    { LOG_MSG_PIDJ,
      "PIDJ",
      "Qffff",
      "TimeUS,RollJerk,CmdR,RRateTgt,CmdY",
      sizeof(LogMsgPIDJ) },

    { LOG_MSG_JINDI,
      "JIND",
      "Qfffffffffffff",
      // TAR/TJR = tau_accel_roll/tau_jerk_roll (the two pre-blend path
      // references); AlR = alpha_roll; TC = tau_roll (the alpha-blended
      // combination, pre-normalise) — at exactly 63 chars, see LogDef's
      // comment on the cap; TC is as short as it can be and still fit.
      // Pdd here = filtered measured jerk (jerk_filt), not JKFT.Pdd raw.
      "TimeUS,UnR,UnP,TAR,TJR,CmdR,CmdP,AccR,AccP,G1R,Pdd,JCmdR,AlR,TC",
      sizeof(LogMsgJINDI) },

    { LOG_MSG_JPID,
      "JPID",
      "Qfffffffff",
      "TimeUS,RTgt,ATgt,JTgt,TqPI,TqD,TqR,CmdR,CmdP,CmdY",
      sizeof(LogMsgJPID) },
};

constexpr size_t kNumLogDefs = sizeof(kLogDefs) / sizeof(kLogDefs[0]);

/* ── Compile-time FMT field-size check ──────────────────────────────────────
 * Catches a name/fmt/labels string too long for its LogFmtHdr field (see
 * LogDef's comments above for the exact per-field limits) at build time,
 * instead of a silent strncpy truncation only noticed later as missing
 * columns in UAV Log Viewer. */
constexpr size_t const_strlen(const char *s)
{
    size_t n = 0;
    while (s[n] != '\0') { ++n; }
    return n;
}

constexpr bool log_defs_fit_fmt_fields()
{
    for (size_t i = 0; i < sizeof(kLogDefs) / sizeof(kLogDefs[0]); ++i) {
        if (const_strlen(kLogDefs[i].name)   > 4)  { return false; }
        if (const_strlen(kLogDefs[i].fmt)    > 15) { return false; }
        if (const_strlen(kLogDefs[i].labels) > 63) { return false; }
    }
    return true;
}

static_assert(log_defs_fit_fmt_fields(),
    "A LogDef name/fmt/labels string is too long for its LogFmtHdr FMT field "
    "(name<=4, fmt<=15, labels<=63 chars) and would be silently truncated by "
    "Logger::write_schema_header()'s strncpy, breaking UAV Log Viewer's column "
    "list for that message type — shorten the offending string in kLogDefs[].");
