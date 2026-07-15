#include "hal.h"
#include "src/coms/MAVLink.hpp"
#include "src/threads.hpp"

/*
 * MAVLink C library configuration — must come before the headers.
 * MAVLINK_USE_CONVENIENCE_FUNCTIONS enables the _send() helpers (heartbeat, etc.)
 * MAVLINK_SEND_UART_BYTES is the callback those helpers use to write bytes.
 * We use one channel (MAVLINK_COMM_0) mapped directly to SD3 (TELEM2).
 */
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_COMM_NUM_BUFFERS 1
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) \
    chnWrite(&SD3, (const uint8_t *)(buf), (size_t)(len))

/*
 * mavlink_system must be defined BEFORE common/mavlink.h is included because
 * mavlink_helpers.h (pulled in transitively) references it inside static inline
 * function bodies that are compiled at the point of inclusion.
 */
#include "mavlink_types.h"
mavlink_system_t mavlink_system = { 1, 1 }; // sysid=1, compid=1 (autopilot)

#include "common/mavlink.h"  // header-only; include path: third_party/mavlink

static const SerialConfig telem2_cfg = {
    115200,
    0,
    USART_CR2_STOP1_BITS,
    0,
    nullptr, nullptr
};

void mavlink_comms_init()
{
    sdStart(&SD3, &telem2_cfg);
}

/* ── Diagnostics ──────────────────────────────────────────────────────────── */

static volatile MavlinkDiag s_diag = {};

void mavlink_get_diag(MavlinkDiag &out)
{
    out.bytes_rx        = s_diag.bytes_rx;
    out.frames_ok        = s_diag.frames_ok;
    out.frames_bad_crc   = s_diag.frames_bad_crc;
    out.heartbeat_rx     = s_diag.heartbeat_rx;
    out.param_req_rx     = s_diag.param_req_rx;
    out.vision_pos_rx    = s_diag.vision_pos_rx;
    out.vision_speed_rx  = s_diag.vision_speed_rx;
    out.unknown_rx       = s_diag.unknown_rx;
}

/* ── Incoming message handlers ───────────────────────────────────────────── */

static void handle_vision_position(const mavlink_message_t *msg)
{
    mavlink_vision_position_estimate_t m;
    mavlink_msg_vision_position_estimate_decode(msg, &m);

    chMtxLock(&mocap_mtx);
    g_mocap.x           = m.x;
    g_mocap.y           = m.y;
    g_mocap.z           = m.z;
    g_mocap.yaw         = m.yaw;
    g_mocap.valid       = true;
    g_mocap.has_new_pos = true;
    g_mocap.has_new_yaw = true;
    chMtxUnlock(&mocap_mtx);
}

static void handle_vision_speed(const mavlink_message_t *msg)
{
    mavlink_vision_speed_estimate_t m;
    mavlink_msg_vision_speed_estimate_decode(msg, &m);

    chMtxLock(&mocap_mtx);
    g_mocap.vx          = m.x;
    g_mocap.vy          = m.y;
    g_mocap.vz          = m.z;
    g_mocap.has_new_vel = true;
    chMtxUnlock(&mocap_mtx);
}

static void handle_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        s_diag.heartbeat_rx++;
        break;

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        /*
         * MAVProxy immediately requests the full parameter list on connect.
         * We have no parameters, so respond with a single PARAM_VALUE with
         * param_count=0. This satisfies MAVProxy and stops retries.
         */
        s_diag.param_req_rx++;
        mavlink_msg_param_value_send(MAVLINK_COMM_0,
            "NONE", 0.0f, MAV_PARAM_TYPE_REAL32, 0, 0);
        break;

    case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
        s_diag.vision_pos_rx++;
        handle_vision_position(msg);
        break;

    case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
        s_diag.vision_speed_rx++;
        handle_vision_speed(msg);
        break;

    default:
        s_diag.unknown_rx++;
        break;
    }
}

/* ── Main update (called each thread tick) ───────────────────────────────── */

void mavlink_comms_update()
{
    /* Drain all available bytes from TELEM2 and parse them into MAVLink frames */
    uint8_t c;
    mavlink_message_t msg;
    mavlink_status_t  status;

    while (chnReadTimeout(&SD3, &c, 1, TIME_IMMEDIATE) == 1) {
        s_diag.bytes_rx++;
        const uint8_t framing = mavlink_frame_char(MAVLINK_COMM_0, c, &msg, &status);
        if (framing == MAVLINK_FRAMING_OK) {
            s_diag.frames_ok++;
            handle_message(&msg);
        } else if (framing == MAVLINK_FRAMING_BAD_CRC) {
            s_diag.frames_bad_crc++;
        }
    }

    const uint32_t now_ms = (uint32_t)TIME_I2MS(chVTGetSystemTime());

    /* Send heartbeat at 1 Hz so MAVProxy can find the vehicle */
    static uint32_t last_hb_ms = 0;
    if (now_ms - last_hb_ms >= 1000U) {
        last_hb_ms = now_ms;
        mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_GENERIC,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0,
            MAV_STATE_ACTIVE);

        /*
         * Also broadcast SYSTEM_TIME unsolicited at 1 Hz (no RTC on this
         * board, so time_unix_usec=0 — companion-computer bridges that sync
         * off time_boot_ms, e.g. the OptiTrack/ROS2 vision bridge, only read
         * that field). Without this, a bridge that gates its own message
         * sending on a successful time-sync handshake — waiting for a
         * SYSTEM_TIME reply to its own REQUEST_MESSAGE that this firmware
         * doesn't otherwise answer — would stall forever and never send
         * VISION_POSITION_ESTIMATE/VISION_SPEED_ESTIMATE at all.
         */
        mavlink_msg_system_time_send(MAVLINK_COMM_0, 0, now_ms);
    }

    /* Echo received vision data as LOCAL_POSITION_NED at 10 Hz.
     * Lets the GCS confirm data is arriving: `watch LOCAL_POSITION_NED` */
    static uint32_t last_pos_ms = 0;
    if (now_ms - last_pos_ms >= 100U) {
        last_pos_ms = now_ms;
        chMtxLock(&mocap_mtx);
        const MocapRaw snap = g_mocap;
        chMtxUnlock(&mocap_mtx);
        if (snap.valid) {
            mavlink_msg_local_position_ned_send(MAVLINK_COMM_0,
                now_ms,
                snap.x, snap.y, snap.z,
                snap.vx, snap.vy, snap.vz);
        }
    }
}
