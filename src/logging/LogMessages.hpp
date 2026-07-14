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

/* ── Controller-tuning log (CTUN) ────────────────────────────────────────────
 * Temporary, gain-tuning-only message. Flip to 0 to strip it out of the
 * schema header and LogThread writes entirely once tuning is done. */
#define LOG_CTUN_ENABLED 1

/* ── Message IDs ─────────────────────────────────────────────────────────── */
constexpr uint8_t LOG_MSG_FMT  = 0x80U;  // schema header — written once at file open
constexpr uint8_t LOG_MSG_ATT  = 0x09U;  // angular states: attitude + rates + angular accel
constexpr uint8_t LOG_MSG_LIN  = 0x0AU;  // linear states: position + velocity + linear accel
constexpr uint8_t LOG_MSG_RCIN = 0x05U;  // RC stick inputs + flight mode + arm state
constexpr uint8_t LOG_MSG_OUTP = 0x06U;  // controller outputs entering the motor mixer
constexpr uint8_t LOG_MSG_RPMS = 0x07U;  // per-motor mechanical RPM from DShot GCR telemetry
constexpr uint8_t LOG_MSG_STRN = 0x08U;  // strain rate sensor (CAN 0x69, 4 arms, in development)
constexpr uint8_t LOG_MSG_IMU1 = 0x0BU;  // raw accel + gyro, IMU1 (ICM-45686,  SPI1, CS=PG1)  (body-frame, post-rotation, pre-EKF)
constexpr uint8_t LOG_MSG_IMU2 = 0x0CU;  // raw accel + gyro, IMU2 (ICM-42688,  SPI4, CS=PC15) (body-frame, post-rotation, pre-EKF)
constexpr uint8_t LOG_MSG_IMU3 = 0x0DU;  // raw accel + gyro, IMU3 (ICM-42688,  SPI4, CS=PC13) (body-frame, post-rotation, pre-EKF)
constexpr uint8_t LOG_MSG_INDI = 0x0EU;  // INDI shadow-controller diagnostics (always logged, regardless of _use_indi)
constexpr uint8_t LOG_MSG_BARO = 0x0FU;  // barometric pressure/temperature/altitude (MS5611, SPI1, CS=PD7)
constexpr uint8_t LOG_MSG_CTUN = 0x10U;  // TEMP: pos-hold NE tuning — outer pos + inner vel loop targets/errors, shadow lean-angle target
constexpr uint8_t LOG_MSG_MOCP = 0x11U;  // raw mocap position/velocity estimate, pre-EKF (MAVLink VISION_POSITION/SPEED_ESTIMATE)

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
    uint8_t  armed;       // 0=disarmed, 1=armed
};
// Format: "QfffffB"   Body: 8+5×4+1 = 29 B   Record: 32 B

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

struct __attribute__((packed)) LogMsgSTRN {
    uint64_t time_us;
    int16_t  s0;    // arm FR strain rate (raw int16 from CAN 0x69)
    int16_t  s1;    // arm RL
    int16_t  s2;    // arm FL
    int16_t  s3;    // arm RR
    uint8_t  valid; // 1 once at least one CAN frame has arrived
};
// Format: "QhhhhB"   Body: 8+4×2+1 = 17 B   Record: 20 B

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
};
// Format: "Qffffffff"   Body: 8+8×4 = 40 B   Record: 43 B
// Always populated regardless of FlightStateMachine::_use_indi — this is INDI
// running in shadow mode alongside whichever controller actually flies (OUTP).

struct __attribute__((packed)) LogMsgBARO {
    uint64_t time_us;
    float    pressure_pa;  // Pa, compensated
    float    temp_c;       // °C, compensated
    float    alt_m;        // m, positive up, relative to MS5611::init() boot-time reference
    uint8_t  valid;        // 1 once MS5611 init + warm-up zero-reference capture completed
};
// Format: "QfffB"   Body: 8+3×4+1 = 21 B   Record: 24 B

#if LOG_CTUN_ENABLED
struct __attribute__((packed)) LogMsgCTUN {
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
    float    climb_rate_tgt; // m/s  alt-hold climb rate target (positive = descend), D-axis output of NED_update
    float    climb_rate_err; // m/s  climb rate error, target - measured (state[W])
};
// Format: "Qffffffffffff"   Body: 8+12×4 = 56 B   Record: 59 B
#endif

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

/* ── Log descriptor table ────────────────────────────────────────────────── */

struct LogDef {
    uint8_t     msg_id;
    const char *name;    // ≤4 chars, null-padded to 4 by strncpy (e.g. "ATT" → stored as "ATT\0")
    const char *fmt;     // ArduPilot format codes (≤16 chars): Q H f i h B …
    const char *labels;  // comma-separated field names (≤64 chars); TimeUS must be first
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
      "QfffffB",
      "TimeUS,RollStk,PitchStk,YawStk,ThrStk,FlightMode,Armed",
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

    { LOG_MSG_STRN,
      "STRN",
      "QhhhhB",
      "TimeUS,S0,S1,S2,S3,Valid",
      sizeof(LogMsgSTRN) },

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
      "Qffffffff",
      "TimeUS,UnmixR,UnmixP,DeltaR,DeltaP,CmdR,CmdP,AccR,AccP",
      sizeof(LogMsgINDI) },

    { LOG_MSG_BARO,
      "BARO",
      "QfffB",
      "TimeUS,Press,Temp,Alt,Valid",
      sizeof(LogMsgBARO) },

#if LOG_CTUN_ENABLED
    { LOG_MSG_CTUN,
      "CTUN",
      "Qffffffffffff",
      // labels[] is a fixed 64-byte FMT field (63 usable chars after the null
      // terminator, see Logger.cpp's LogFmtHdr) — the descriptive names used
      // in the struct comments above don't fit; strncpy silently truncates
      // past the limit, which is why UAV Log Viewer previously only showed
      // fields up to VelETgt. Keep this under 63 chars.
      "TimeUS,PNT,PNE,PET,PEE,VNT,VNE,VET,VEE,RolT,PitT,ClbT,ClbE",
      sizeof(LogMsgCTUN) },
#endif

    { LOG_MSG_MOCP,
      "MOCP",
      "QffffffB",
      "TimeUS,X,Y,Z,VX,VY,VZ,Valid",
      sizeof(LogMsgMOCP) },
};

constexpr size_t kNumLogDefs = sizeof(kLogDefs) / sizeof(kLogDefs[0]);
