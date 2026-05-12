#pragma once
#include <cstdint>
#include <cstddef>

/*
 * Binary log record format:
 *   [0xA3][0x95][msg_id][...packed struct body...]
 *
 * To add a new log set:
 *   1. Define a packed struct and a LOG_MSG_* constant below.
 *   2. Add one entry to kLogDefs[].
 *   3. Snapshot the data and call logger.write(LOG_MSG_*, msg) in LogThread.
 *   4. Set the rate via kRates.log in main.cpp.
 */

/* ── Message IDs ─────────────────────────────────────────────────────────── */
constexpr uint8_t LOG_MSG_FMT   = 0x80U;  // schema header (written at open)
constexpr uint8_t LOG_MSG_IMU   = 0x01U;  // 3× on-board IMUs + CAN IMU
constexpr uint8_t LOG_MSG_STATE = 0x02U;  // fused flight state

/* ── Packed message bodies ───────────────────────────────────────────────── */

struct __attribute__((packed)) LogMsgIMU {
    uint32_t time_ms;
    /* IMU 0 — ICM-20948 primary (SPI1) */
    float ax0, ay0, az0, gx0, gy0, gz0; uint8_t valid0;
    /* IMU 1 — ICM-20948 ext (SPI4 PE4) */
    float ax1, ay1, az1, gx1, gy1, gz1; uint8_t valid1;
    /* IMU 2 — ICM-20602 (SPI4 PC13) */
    float ax2, ay2, az2, gx2, gy2, gz2; uint8_t valid2;
    /* External CAN IMU (IMX5) — quaternion NED→Body and body rates */
    float qw, qx, qy, qz;               // quaternion [W,X,Y,Z] Hamilton
    float can_p, can_q, can_r;          // body-frame rates (rad/s)
    uint8_t can_valid;
};

struct __attribute__((packed)) LogMsgState {
    uint32_t time_ms;
    float roll, pitch, yaw;     // rad
    float p, q, r;              // rad/s
    float z_pos, z_vel, z_accel;// m, m/s, m/s²
    float thr;                  // [0,1]
    uint8_t armed;
};

/* ── Log descriptor table ────────────────────────────────────────────────── */

struct LogDef {
    uint8_t     msg_id;
    const char *name;     // ≤4 chars, null-padded
    const char *labels;   // comma-separated field names for decoder tools
    size_t      body_size;
};

constexpr LogDef kLogDefs[] = {
    { LOG_MSG_IMU,
      "IMU ",
      "TimeMS,AX0,AY0,AZ0,GX0,GY0,GZ0,V0,"
      "AX1,AY1,AZ1,GX1,GY1,GZ1,V1,"
      "AX2,AY2,AZ2,GX2,GY2,GZ2,V2,"
      "QW,QX,QY,QZ,CanP,CanQ,CanR,CV",
      sizeof(LogMsgIMU) },

    { LOG_MSG_STATE,
      "STAT",
      "TimeMS,Roll,Pitch,Yaw,P,Q,R,ZPos,ZVel,ZAcc,Thr,Armed",
      sizeof(LogMsgState) },
};

constexpr size_t kNumLogDefs = sizeof(kLogDefs) / sizeof(kLogDefs[0]);
