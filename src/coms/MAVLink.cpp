#include "src/uptime.hpp"
#include "hal.h"
#include <cstring>
#include "src/coms/MAVLink.hpp"
#include "src/threads.hpp"

/*
 * MAVLink C library configuration — must come before the headers.
 * MAVLINK_USE_CONVENIENCE_FUNCTIONS enables the _send() helpers (heartbeat, etc.)
 * MAVLINK_SEND_UART_BYTES is the callback those helpers use to write bytes.
 * We use one channel (MAVLINK_COMM_0) mapped directly to BPRL_MAVLINK_SD
 * (per-board, see MAVLink.hpp).
 */
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_COMM_NUM_BUFFERS 1
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) \
    chnWrite(&BPRL_MAVLINK_SD, (const uint8_t *)(buf), (size_t)(len))

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
    sdStart(&BPRL_MAVLINK_SD, &telem2_cfg);
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
    out.mocap_timeouts   = s_diag.mocap_timeouts;
    out.traj_cmd_rx      = s_diag.traj_cmd_rx;
    out.traj_cmd_rejected = s_diag.traj_cmd_rejected;
    out.last_traj_cmd     = s_diag.last_traj_cmd;
    out.last_traj_result  = s_diag.last_traj_result;
}

/* ── Incoming message handlers ───────────────────────────────────────────── */

/*
 * Mocap link timeout — same rule and value as ArduPilot's
 * AP_VisualOdom_Backend::healthy() (AP_VISUALODOM_TIMEOUT_MS = 300, see
 * libraries/AP_VisualOdom/AP_VisualOdom.h): the link is healthy only while a
 * position message has arrived within the last 300 ms. Keyed off
 * VISION_POSITION_ESTIMATE alone, since that is the message that sets
 * g_mocap.valid in the first place.
 *
 * Held as a systime_t and compared with chVTTimeElapsedSinceX() rather than
 * as a TIME_I2MS() millisecond count: Drone3 builds with
 * CH_CFG_ST_RESOLUTION=16, where the tick counter wraps every ~6.5 s, and
 * only systime_t arithmetic is wrap-safe across that. Only ever touched from
 * MAVLinkThread (handler + mavlink_comms_update()), so it needs no lock.
 */
#define MOCAP_TIMEOUT_MS 300U
static systime_t s_last_vision_pos_time = 0;

static void handle_vision_position(const mavlink_message_t *msg)
{
    mavlink_vision_position_estimate_t m;
    mavlink_msg_vision_position_estimate_decode(msg, &m);

    s_last_vision_pos_time = chVTGetSystemTimeX();

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

/*
 * Trajectory commands — see the table in MAVLink.hpp. The command is only
 * decoded here; FlightStateMachine (on ControlThread) decides whether it is
 * acceptable, and its verdict comes back through g_traj_mailbox for
 * mavlink_comms_update() to send as the COMMAND_ACK.
 *
 * s_traj_* are only touched from MAVLinkThread, so they need no lock.
 */
#define TRAJ_CMD_TIMEOUT_MS 200U

// MAVLink's user-defined command ids (MAV_CMD_USER_1..4 in the spec). Spelled
// out here because the generated headers in third_party/mavlink predate
// their addition to the MAV_CMD enum.
static constexpr uint16_t MAV_CMD_TRAJ_SET_ORIGIN = 31010;   // MAV_CMD_USER_1
static constexpr uint16_t MAV_CMD_TRAJ_POINT      = 31011;   // MAV_CMD_USER_2
static constexpr uint16_t MAV_CMD_TRAJ_CIRCLE     = 31012;   // MAV_CMD_USER_3
static constexpr uint16_t MAV_CMD_TRAJ_STOP       = 31013;   // MAV_CMD_USER_4
static systime_t s_traj_cmd_time = 0;     // when the command in the mailbox was posted
static uint8_t   s_traj_ack_sysid  = 0;   // who to address its COMMAND_ACK to
static uint8_t   s_traj_ack_compid = 0;

/*
 * Retries. The radio link loses frames, so the sender repeats a command
 * until it sees the COMMAND_ACK, counting its attempts in
 * COMMAND_LONG.confirmation (the MAVLink convention). A repeat must not be
 * executed twice — the first copy may have arrived and only its ACK been
 * lost, and re-running e.g. a circle command would restart its approach.
 * So the last command posted is remembered: a repeat of it (confirmation > 0,
 * same command and parameters, within TRAJ_RETRY_WINDOW_MS) just gets the
 * same answer again, or nothing if the answer is still being worked out.
 */
#define TRAJ_RETRY_WINDOW_MS 5000U
static bool     s_last_valid    = false;
static uint16_t s_last_command  = 0;
static float    s_last_params[7] = {};
static bool     s_last_answered = false;
static uint8_t  s_last_result   = 0;
static uint32_t s_last_ms       = 0;      // bprl_millis() when it was posted

static void send_traj_ack(uint16_t command, uint8_t result, uint8_t sysid, uint8_t compid)
{
    if (result != MAV_RESULT_ACCEPTED) s_diag.traj_cmd_rejected++;
    mavlink_msg_command_ack_send(MAVLINK_COMM_0, command, result, 0, 0, sysid, compid);
}

// Returns false if the command isn't one of ours.
static bool handle_command_long(const mavlink_message_t *msg)
{
    mavlink_command_long_t m;
    mavlink_msg_command_long_decode(msg, &m);

    TrajCmdType type;
    switch (m.command) {
    case MAV_CMD_TRAJ_SET_ORIGIN: type = TrajCmdType::SET_ORIGIN; break;
    case MAV_CMD_TRAJ_POINT:      type = TrajCmdType::POINT;      break;
    case MAV_CMD_TRAJ_CIRCLE:     type = TrajCmdType::CIRCLE;     break;
    case MAV_CMD_TRAJ_STOP:       type = TrajCmdType::STOP;       break;
    default: return false;
    }
    if (m.target_system != 0 && m.target_system != mavlink_system.sysid) return false;

    s_diag.traj_cmd_rx++;

    const float params[7] = { m.param1, m.param2, m.param3, m.param4, m.param5, m.param6, m.param7 };

    // A repeat of the command we already have? (Bitwise compare: NaN
    // parameters must match too.)
    if (m.confirmation > 0 && s_last_valid && m.command == s_last_command &&
        memcmp(params, s_last_params, sizeof(params)) == 0 &&
        (bprl_millis() - s_last_ms) < TRAJ_RETRY_WINDOW_MS) {
        if (s_last_answered) {
            mavlink_msg_command_ack_send(MAVLINK_COMM_0, m.command, s_last_result, 0, 0,
                                         msg->sysid, msg->compid);
        }
        return true;
    }

    bool posted = false;
    chMtxLock(&traj_mtx);
    if (!g_traj_mailbox.cmd_pending && !g_traj_mailbox.result_pending) {
        g_traj_mailbox.cmd.type    = type;
        memcpy(g_traj_mailbox.cmd.p, params, sizeof(params));
        g_traj_mailbox.mav_command = m.command;
        g_traj_mailbox.cmd_pending = true;
        posted = true;
    }
    chMtxUnlock(&traj_mtx);

    if (posted) {
        s_traj_cmd_time   = chVTGetSystemTimeX();
        s_traj_ack_sysid  = msg->sysid;
        s_traj_ack_compid = msg->compid;

        s_last_valid    = true;
        s_last_command  = m.command;
        memcpy(s_last_params, params, sizeof(params));
        s_last_answered = false;
        s_last_ms       = bprl_millis();

        s_diag.last_traj_cmd    = m.command;
        s_diag.last_traj_result = 255;
    } else {
        // The previous command hasn't been answered yet.
        send_traj_ack(m.command, MAV_RESULT_TEMPORARILY_REJECTED, msg->sysid, msg->compid);
    }
    return true;
}

// Send the COMMAND_ACK for the command in the mailbox once ControlThread has
// ruled on it — or give up on it if ControlThread never does.
static void traj_ack_update()
{
    bool     send    = false;
    uint16_t command = 0;
    uint8_t  result  = MAV_RESULT_FAILED;

    chMtxLock(&traj_mtx);
    if (g_traj_mailbox.result_pending) {
        g_traj_mailbox.result_pending = false;
        command = g_traj_mailbox.mav_command;
        switch (g_traj_mailbox.result) {
        case TrajResult::ACCEPTED:  result = MAV_RESULT_ACCEPTED;             break;
        case TrajResult::BAD_PARAM: result = MAV_RESULT_DENIED;               break;
        case TrajResult::NOT_READY:
        case TrajResult::NO_ORIGIN: result = MAV_RESULT_TEMPORARILY_REJECTED; break;
        }
        send = true;
    } else if (g_traj_mailbox.cmd_pending &&
               chVTTimeElapsedSinceX(s_traj_cmd_time) > TIME_MS2I(TRAJ_CMD_TIMEOUT_MS)) {
        g_traj_mailbox.cmd_pending = false;
        command = g_traj_mailbox.mav_command;
        send    = true;
    }
    chMtxUnlock(&traj_mtx);

    if (send) {
        s_last_answered = true;
        s_last_result   = result;
        s_diag.last_traj_result = result;
        send_traj_ack(command, result, s_traj_ack_sysid, s_traj_ack_compid);
    }
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

    case MAVLINK_MSG_ID_COMMAND_LONG:
        if (!handle_command_long(msg)) s_diag.unknown_rx++;
        break;

    default:
        s_diag.unknown_rx++;
        break;
    }
}

/* ── Main update (called each thread tick) ───────────────────────────────── */

void mavlink_comms_update()
{
    /* Drain all available bytes from the MAVLink UART and parse them into MAVLink frames */
    uint8_t c;
    mavlink_message_t msg;
    mavlink_status_t  status;

    while (chnReadTimeout(&BPRL_MAVLINK_SD, &c, 1, TIME_IMMEDIATE) == 1) {
        s_diag.bytes_rx++;
        const uint8_t framing = mavlink_frame_char(MAVLINK_COMM_0, c, &msg, &status);
        if (framing == MAVLINK_FRAMING_OK) {
            s_diag.frames_ok++;
            handle_message(&msg);
        } else if (framing == MAVLINK_FRAMING_BAD_CRC) {
            s_diag.frames_bad_crc++;
        }
    }

    /* Mocap timeout: drop g_mocap.valid once position messages stop (see
     * MOCAP_TIMEOUT_MS). Clearing the has_new_* flags too keeps a sample
     * that landed just before the dropout from being fused as fresh on the
     * far side of it. The next VISION_POSITION_ESTIMATE sets valid again. */
    chMtxLock(&mocap_mtx);
    if (g_mocap.valid &&
        chVTTimeElapsedSinceX(s_last_vision_pos_time) > TIME_MS2I(MOCAP_TIMEOUT_MS)) {
        g_mocap.valid       = false;
        g_mocap.has_new_pos = false;
        g_mocap.has_new_vel = false;
        g_mocap.has_new_yaw = false;
        s_diag.mocap_timeouts++;
    }
    chMtxUnlock(&mocap_mtx);

    traj_ack_update();

    const uint32_t now_ms = bprl_millis();

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

    /* Echo received vision data as LOCAL_POSITION_NED at 2 Hz (was 10 Hz —
     * cut to leave the half-duplex radio more air time for the uplink).
     * Lets the GCS confirm data is arriving: `watch LOCAL_POSITION_NED` */
    static uint32_t last_pos_ms = 0;
    if (now_ms - last_pos_ms >= 500U) {
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
