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
 *   - Receives COMMAND_LONG trajectory commands (table below), passes them
 *     to ControlThread through g_traj_mailbox, and answers with COMMAND_ACK
 *
 * Trajectory commands (see src/controllers/TrajectoryTracker.hpp for what
 * each does). All positions are NED offsets in metres from the origin.
 * A parameter that is "not given" must be sent as NaN — zero is a real
 * value (a D of 0 means "the origin's height", not "where I am now").
 *
 *   command                 p1      p2  p3  p4  p5        p6         p7
 *   MAV_CMD_USER_1 (31010)  set origin to the current position (no parameters)
 *   MAV_CMD_USER_2 (31011)  point:  N   E   D
 *   MAV_CMD_USER_3 (31012)  circle: radius  N   E   D   focus dist  direction  speed
 *   MAV_CMD_USER_4 (31013)  stop — back to plain position hold (no parameters)
 *
 * COMMAND_ACK result:
 *   MAV_RESULT_ACCEPTED              done / started
 *   MAV_RESULT_TEMPORARILY_REJECTED  not now: origin not set, not flying in
 *                                    POS_HOLD, no valid position, set-origin
 *                                    during a trajectory, or a previous
 *                                    command is still being processed
 *   MAV_RESULT_DENIED                bad parameters (radius missing / out of range,
 *                                    altitude below the take-off floor, ...)
 *   MAV_RESULT_FAILED                ControlThread did not pick the command up
 *                                    (e.g. motor test running)
 * Other COMMAND_LONG commands are ignored, as before.
 *
 * The link loses frames, so senders should repeat a command until they see
 * its COMMAND_ACK, incrementing COMMAND_LONG.confirmation on each repeat
 * (0 on the first send). A repeat of the command already received is not
 * executed again; it only gets the same COMMAND_ACK re-sent.
 *
 * From MAVProxy (which fills omitted parameters with 0, so spell them out):
 *   long 31010 0 0 0 0 0 0 0                 set origin
 *   long 31011 1.0 0 nan 0 0 0 0             point 1 m North at the current height
 *   long 31012 1.0 nan nan nan nan nan nan   1 m circle about the origin, defaults
 *   long 31013 0 0 0 0 0 0 0                 stop
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
    uint32_t traj_cmd_rx;       // trajectory COMMAND_LONGs received (MAV_CMD_USER_1..4)
    uint32_t traj_cmd_rejected; // ... of which were answered with anything other than ACCEPTED
    uint16_t last_traj_cmd;     // MAVLink command id of the most recent trajectory command (0 = none yet)
    uint8_t  last_traj_result;  // its MAV_RESULT; 255 while ControlThread has not answered yet
};

// Copy out current diagnostic counters (safe to call from any thread).
void mavlink_get_diag(MavlinkDiag &out);
