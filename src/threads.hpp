#pragma once
#include "ch.h"
#include "hal.h"
#include "src/FlightState.hpp"
#include "src/coms/DShot.hpp"   // ESCTelemetry
#include "src/sensors/StrainRate.hpp"

/* ── Shared raw sensor data types ────────────────────────────────────────── */

struct IMURaw {
    float accel[3];  // m/s² (X fwd, Y right, Z down)
    float gyro[3];   // rad/s
    bool  valid;
};

struct CANIMURaw {
    // Quaternion NED→Body from IMX5 CID_INS_QUATN2B [W,X,Y,Z], Hamilton convention.
    // Replaces the previous Euler-angle fields (roll, pitch, yaw).
    float q0, q1, q2, q3;   // unit quaternion components (decoded from int16/10000)
    bool  has_new_quat;      // set by CAN callback; cleared after StateEstThread consumes it

    // Body-frame angular rates from IMX5 (100 Hz, CAN IDs 0x02–0x04)
    float p, q, r;           // rad/s
    // Body-frame specific force from IMX5 (100 Hz, same CAN frames as rates)
    float ax, ay, az;        // m/s²
    bool  has_new_rates;     // set by CAN callback; cleared after StateEstThread consumes it

    bool  valid;             // true while IMX5 frames are arriving
};

struct MocapRaw {
    float x, y, z;    // NED position (m)
    float vx, vy, vz; // NED velocity (m/s)
    bool  has_new;     // fresh data this tick — cleared by StateEstThread
    bool  valid;       // mocap link connected and receiving
};

/* ── Shared flight state — access only under respective mutex ─────────────
 * Defined in threads.cpp.                                                   */
extern mutex_t state_mtx;
extern float   g_state[StateIdx::N]; // full 19-element EKF state (StateIdx::*)
extern float   g_euler[3];           // [roll, pitch, yaw] (rad) derived from quaternion
extern float   g_input[5];           // InputIdx::*  (thrust, roll/pitch/yaw targets, flight_mode)
extern int32_t g_output[4];          // normalized motor commands 0–1000 [FR, RL, FL, RR] (0=disarm; protocol conversion in motor_output_write())
extern float   g_ctrl[4];            // [roll_tq, pitch_tq, yaw_tq, thrust] — PID outputs entering MotorMixer
extern bool    g_armed;
extern int     g_flight_mode;        // FlightMode enum value (0=STABILIZE, 1=ALT_HOLD, 2=POS_HOLD)

extern mutex_t imu_mtx;
extern IMURaw  g_imu[3];     // [0]=ICM-20948 primary, [1]=ext, [2]=ICM-20602

extern mutex_t   can_imu_mtx;
extern CANIMURaw g_can_imu;


extern mutex_t  mocap_mtx;
extern MocapRaw g_mocap;

extern mutex_t      esc_mtx;
extern ESCTelemetry g_esc_telem[4]; // [FR, RL, FL, RR] — written by DShot ISR

/* ── Motor test (always built) ───────────────────────────────────────────────
 * Set by USBCmdThread in response to "MT,<motor>,<pct>" USB commands.
 * ControlThread checks g_motor_test_active each tick; if set it bypasses the
 * PID+mixer and calls motor_output_write(g_motor_test_cmd) directly.
 * Safety: USBCmdThread refuses to arm test mode while g_armed is true.      */
extern mutex_t   motor_test_mtx;
extern bool      g_motor_test_active;
extern int32_t   g_motor_test_cmd[4]; // 0–1000 values [FR, RL, FL, RR]

/* ── Thread rates — passed as arg by main, stored locally per thread ──────
 * All rates live in main.cpp.  Change them there to retune loop timing.    */

struct LogRates {
    sysinterval_t period;  // 50 Hz → TIME_MS2I(20)
};

struct ThreadRates {
    sysinterval_t spi;      // SPIThread
    sysinterval_t est;      // StateEstThread
    sysinterval_t i2c;      // I2CThread
    sysinterval_t control;  // ControlThread
    sysinterval_t radio;    // RadioThread
    sysinterval_t heartbeat; // HeartbeatThread
    sysinterval_t debug;    // DebugThread (ignored if BPRL_DEBUG not set)
    LogRates       log;     // LogThread
};

/* ── Thread entry points (defined in threads.cpp) ────────────────────────
 * Working areas are also in threads.cpp; main calls threads_start().       */
void threads_start(const ThreadRates &rates);
