#pragma once
#include "ch.h"
#include "hal.h"

/* ── Shared raw sensor data types ────────────────────────────────────────── */

struct IMURaw {
    float accel[3];  // m/s² (X fwd, Y right, Z down)
    float gyro[3];   // rad/s
    bool  valid;
};

struct CANIMURaw {
    float roll, pitch, yaw;  // rad
    float p, q, r;            // rad/s
    bool  valid;
};

/* ── Shared flight state — access only under respective mutex ─────────────
 * Defined in threads.cpp.                                                   */
extern mutex_t state_mtx;
extern float   g_state[9];   // StateIdx::*  (roll…z_accel)
extern float   g_input[4];   // InputIdx::*  (thrust, roll/pitch/yaw targets)
extern int32_t g_output[4];  // motor PWM µs [FR, RL, FL, RR]
extern bool    g_armed;

extern mutex_t imu_mtx;
extern IMURaw  g_imu[3];     // [0]=ICM-20948 primary, [1]=ext, [2]=ICM-20602

extern mutex_t   can_imu_mtx;
extern CANIMURaw g_can_imu;

/* ── Thread rates — passed as arg by main, stored locally per thread ──────
 * All rates live in main.cpp.  Change them there to retune loop timing.    */
struct ThreadRates {
    sysinterval_t spi;      // SPIThread
    sysinterval_t can;      // CANThread
    sysinterval_t est;      // StateEstThread
    sysinterval_t i2c;      // I2CThread
    sysinterval_t control;  // ControlThread
    sysinterval_t radio;    // RadioThread
    sysinterval_t house;    // HouseThread
    sysinterval_t debug;    // DebugThread (ignored if BPRL_DEBUG not set)
};

/* ── Thread entry points (defined in threads.cpp) ────────────────────────
 * Working areas are also in threads.cpp; main calls threads_start().       */
void threads_start(const ThreadRates &rates);
