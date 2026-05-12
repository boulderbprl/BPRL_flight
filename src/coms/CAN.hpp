#pragma once
#include "hal.h"

/*
 * CAN bus driver — FDCAN1 at 500 kbps.
 *
 * Device registration:
 *   Call bprl_can_register() in main() before threads_start().
 *   can_drv_init() pre-registers the IMX5 IMU callbacks.
 *
 * IMX5 frame protocol (500 kbps, standard 11-bit IDs):
 *   ID 0x01  CID_INS_QUATN2B — quaternion NED→Body [W,X,Y,Z] (int16 ÷ 10000), 200 Hz
 *            bytes 0-1 = W (q0), 2-3 = X (q1), 4-5 = Y (q2), 6-7 = Z (q3)
 *   ID 0x02  bytes 0-1 = p rate  (int16 ÷ 1000 → rad/s), 2-3 = x accel (÷ 100 → m/s²)
 *   ID 0x03  bytes 0-1 = q rate,                          2-3 = y accel
 *   ID 0x04  bytes 0-1 = r rate,                          2-3 = z accel   (100 Hz each)
 *
 * Adding devices:
 *   1. Write a callback: void my_cb(const CANRxFrame &f, void *ctx)
 *   2. In main(): bprl_can_register(MY_ID, my_cb, nullptr);
 */

#define MAX_CAN_DEVICES 8
typedef void (*CANCallback)(const CANRxFrame &frame, void *ctx);

// Register a handler for a specific standard CAN ID.
void bprl_can_register(uint32_t id, CANCallback cb, void *ctx);

// Dispatch one received frame to its registered handler (called by CANThread).
void can_dispatch(const CANRxFrame &frame);

// Start FDCAN1 and register the built-in IMX5 callbacks.
void can_drv_init(void);
