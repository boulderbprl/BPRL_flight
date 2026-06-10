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

/* ── Message IDs ─────────────────────────────────────────────────────────── */
constexpr uint8_t LOG_MSG_FMT  = 0x80U;  // schema header — written once at file open
constexpr uint8_t LOG_MSG_ATT  = 0x09U;  // angular states: attitude + rates + angular accel
constexpr uint8_t LOG_MSG_LIN  = 0x0AU;  // linear states: position + velocity + linear accel
constexpr uint8_t LOG_MSG_RCIN = 0x05U;  // RC stick inputs + arm state
constexpr uint8_t LOG_MSG_OUTP = 0x06U;  // controller outputs entering the motor mixer
constexpr uint8_t LOG_MSG_RPMS = 0x07U;  // per-motor mechanical RPM from DShot GCR telemetry
constexpr uint8_t LOG_MSG_STRN = 0x08U;  // strain rate sensor (CAN 0x69, 4 arms, in development)

/* ── Packed message bodies ───────────────────────────────────────────────── */

struct __attribute__((packed)) LogMsgATT {
    uint64_t time_us;    // microseconds since boot (TimeUS — required by UAV Log Viewer)
    uint16_t rate_hz;    // logging rate (50)
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
// Format: "QHfffffffff"   Body size: 8+2+9×4 = 46 B   Record: 49 B

struct __attribute__((packed)) LogMsgLIN {
    uint64_t time_us;
    uint16_t rate_hz;
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
// Format: "QHfffffffff"   Body: 46 B   Record: 49 B

struct __attribute__((packed)) LogMsgRCIN {
    uint64_t time_us;
    uint16_t rate_hz;
    float    roll_stk;    // [-1, 1]  roll setpoint from RC
    float    pitch_stk;   // [-1, 1]  pitch setpoint from RC
    float    yaw_stk;     // [-1, 1]  yaw rate demand from RC
    float    thr_stk;     // [0, 1]   throttle from RC
    uint8_t  armed;       // 0=disarmed, 1=armed
};
// Format: "QHffffB"   Body: 8+2+4×4+1 = 27 B   Record: 30 B

struct __attribute__((packed)) LogMsgOUTP {
    uint64_t time_us;
    uint16_t rate_hz;
    float    roll_tq;     // [-1, 1]  normalized roll torque command (PID output)
    float    pitch_tq;    // [-1, 1]  normalized pitch torque command
    float    yaw_tq;      // [-1, 1]  normalized yaw torque command
    float    throttle;    // [0, 1]   shaped throttle entering MotorMixer
};
// Format: "QHffff"   Body: 8+2+4×4 = 26 B   Record: 29 B

struct __attribute__((packed)) LogMsgRPMS {
    uint64_t time_us;
    uint16_t rate_hz;
    int32_t  rpm0;   // FR  mechanical RPM (eRPM ÷ 7 pole pairs); 0 if no valid GCR frame
    int32_t  rpm1;   // RL
    int32_t  rpm2;   // FL
    int32_t  rpm3;   // RR
};
// Format: "QHiiii"   Body: 8+2+4×4 = 26 B   Record: 29 B

struct __attribute__((packed)) LogMsgSTRN {
    uint64_t time_us;
    uint16_t rate_hz;
    int16_t  s0;    // arm FR strain rate (raw int16 from CAN 0x69)
    int16_t  s1;    // arm RL
    int16_t  s2;    // arm FL
    int16_t  s3;    // arm RR
    uint8_t  valid; // 1 once at least one CAN frame has arrived
};
// Format: "QHhhhhB"   Body: 8+2+4×2+1 = 19 B   Record: 22 B

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
      "QHfffffffff",
      "TimeUS,Rate,Roll,Pitch,Yaw,P,Q,R,Pdot,Qdot,Rdot",
      sizeof(LogMsgATT) },

    { LOG_MSG_LIN,
      "LIN",
      "QHfffffffff",
      "TimeUS,Rate,X,Y,Z,U,V,W,Udot,Vdot,Wdot",
      sizeof(LogMsgLIN) },

    { LOG_MSG_RCIN,
      "RCIN",
      "QHffffB",
      "TimeUS,Rate,RollStk,PitchStk,YawStk,ThrStk,Armed",
      sizeof(LogMsgRCIN) },

    { LOG_MSG_OUTP,
      "OUTP",
      "QHffff",
      "TimeUS,Rate,RollTq,PitchTq,YawTq,Thr",
      sizeof(LogMsgOUTP) },

    { LOG_MSG_RPMS,
      "RPMS",
      "QHiiii",
      "TimeUS,Rate,RPM0,RPM1,RPM2,RPM3",
      sizeof(LogMsgRPMS) },

    { LOG_MSG_STRN,
      "STRN",
      "QHhhhhB",
      "TimeUS,Rate,S0,S1,S2,S3,Valid",
      sizeof(LogMsgSTRN) },
};

constexpr size_t kNumLogDefs = sizeof(kLogDefs) / sizeof(kLogDefs[0]);
