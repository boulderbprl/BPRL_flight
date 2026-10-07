#pragma once

/*
 * Slimmed-down MAVLink handler, 115200 baud, on BPRL_MAVLINK_SD:
 *   Cube boards:      TELEM2  — USART3 / SD3
 *   BPRL_BOARD_ORQA:  TX7/RX7 — UART7 / SD7 (PE8 TX, PE7 RX; pin mux in
 *                     boards/OrqaH7QuadCore/board.c, driver enable in
 *                     cfg/mcuconf.h)
 *
 * Handles only what's needed:
 *   - Sends HEARTBEAT at 1 Hz (so MAVProxy can find the vehicle)
 *   - Responds to PARAM_REQUEST_LIST with an empty list (prevents MAVProxy retries)
 *   - Receives VISION_POSITION_ESTIMATE → writes g_mocap.{x,y,z}
 *   - Receives VISION_SPEED_ESTIMATE    → writes g_mocap.{vx,vy,vz}
 *   - Clears g_mocap.valid if no VISION_POSITION_ESTIMATE arrives for 300 ms
 *     (ArduPilot's AP_VISUALODOM_TIMEOUT_MS)
 *
 * Coordinate convention: both messages are expected in NED (x=North, y=East,
 * z=Down), which maps directly to MocapRaw fields consumed by the EKF.
 */

#if defined(BPRL_BOARD_ORQA)
#define BPRL_MAVLINK_SD SD7
#else
#define BPRL_MAVLINK_SD SD3
#endif

void mavlink_comms_init();    // call once, before the thread loop starts
void mavlink_comms_update();  // call each thread tick (~100 Hz)

// ── Diagnostics ──────────────────────────────────────────────────────────────
// Lets you tell, from the FC side alone (over USB, independent of the radio
// link), whether bytes are reaching the MAVLink UART at all and what's in them — without
// needing a working end-to-end GCS/mocap-bridge connection.
struct MavlinkDiag {
    uint32_t bytes_rx;        // total bytes read off BPRL_MAVLINK_SD
    uint32_t frames_ok;       // MAVLINK_FRAMING_OK
    uint32_t frames_bad_crc;  // MAVLINK_FRAMING_BAD_CRC (dialect/version mismatch)
    uint32_t heartbeat_rx;
    uint32_t param_req_rx;
    uint32_t vision_pos_rx;
    uint32_t vision_speed_rx;
    uint32_t unknown_rx;      // frames_ok but msgid not handled here
    uint32_t mocap_timeouts;  // times g_mocap.valid was dropped for >300 ms without a VISION_POSITION_ESTIMATE
};

// Copy out current diagnostic counters (safe to call from any thread).
void mavlink_get_diag(MavlinkDiag &out);
