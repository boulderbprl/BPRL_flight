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

// ── Diagnostics ──────────────────────────────────────────────────────────────
// Lets you tell, from the FC side alone (over USB, independent of the radio
// link), whether bytes are reaching SD3 at all and what's in them — without
// needing a working end-to-end GCS/mocap-bridge connection.
struct MavlinkDiag {
    uint32_t bytes_rx;        // total bytes read off SD3
    uint32_t frames_ok;       // MAVLINK_FRAMING_OK
    uint32_t frames_bad_crc;  // MAVLINK_FRAMING_BAD_CRC (dialect/version mismatch)
    uint32_t heartbeat_rx;
    uint32_t param_req_rx;
    uint32_t vision_pos_rx;
    uint32_t vision_speed_rx;
    uint32_t unknown_rx;      // frames_ok but msgid not handled here
};

// Copy out current diagnostic counters (safe to call from any thread).
void mavlink_get_diag(MavlinkDiag &out);
