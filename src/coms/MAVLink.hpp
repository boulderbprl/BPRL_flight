#pragma once

/*
 * Slimmed-down MAVLink handler on TELEM2 (USART3 / SD3, 115200 baud).
 *
 * Handles only what's needed:
 *   - Sends HEARTBEAT at 1 Hz (so MAVProxy can find the vehicle)
 *   - Responds to PARAM_REQUEST_LIST with an empty list (prevents MAVProxy retries)
 *   - Receives VISION_POSITION_ESTIMATE → writes g_mocap.{x,y,z}
 *   - Receives VISION_SPEED_ESTIMATE    → writes g_mocap.{vx,vy,vz}
 *
 * Coordinate convention: both messages are expected in NED (x=North, y=East,
 * z=Down), which maps directly to MocapRaw fields consumed by the EKF.
 */

void mavlink_comms_init();    // call once, before the thread loop starts
void mavlink_comms_update();  // call each thread tick (~100 Hz)
