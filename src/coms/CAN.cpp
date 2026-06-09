#include "src/coms/CAN.hpp"
#include "src/threads.hpp"

/*
 * FDCAN1 bit timing — 1000 kbps, HSE = 24 MHz.
 * BRP=1, TSEG1=18, TSEG2=5 → 24 Tq/bit, 79% sample point.
 * NBTP: NSJW=3, NBRP=0, NTSEG1=17, NTSEG2=4.
 */
static const CANConfig can_cfg = {
    0x06001104,   // NBTP
    0x00000000,   // DBTP: unused (classical CAN)
    0x00000000,   // CCCR: normal mode, FDOE=0
    0x00000000,   // TEST
    0x00000000,   // RXGFC (→ GFC): accept all in RxFIFO0
};

struct CANDevice {
    uint32_t    id;
    CANCallback callback;
    void       *ctx;
};
static CANDevice can_table[MAX_CAN_DEVICES];
static int       num_can_devices = 0;

static volatile CANDiag s_diag = {};

void bprl_can_register(uint32_t id, CANCallback cb, void *ctx)
{
    if (num_can_devices < MAX_CAN_DEVICES) {
        can_table[num_can_devices++] = {id, cb, ctx};
    }
}

void can_dispatch(const CANRxFrame &frame)
{
    const bool is_ext    = frame.common.XTD;
    const uint32_t match = is_ext ? frame.ext.EID : frame.std.SID;

    // Record every frame that reaches CANThread, regardless of ID.
    s_diag.total_rx++;
    s_diag.last_sid  = frame.std.SID;
    s_diag.last_eff  = is_ext ? 1U : 0U;
    s_diag.last_eid  = is_ext ? frame.ext.EID : 0U;
    s_diag.last_dlc  = frame.DLC;
    for (int i = 0; i < 8; i++) s_diag.last_data[i] = frame.data8[i];

    // Match on SID for standard frames, full EID for extended frames.
    // IMX5 may send either; this handles both without code changes.
    bool matched = false;
    for (int i = 0; i < num_can_devices; i++) {
        if (can_table[i].id == match) {
            can_table[i].callback(frame, can_table[i].ctx);
            matched = true;
        }
    }
    if (matched) s_diag.dispatched++;
}

void can_get_diag(CANDiag &out)
{
    out.total_rx   = s_diag.total_rx;
    out.dispatched = s_diag.dispatched;
    out.last_sid   = s_diag.last_sid;
    out.last_eid   = s_diag.last_eid;
    out.last_eff   = s_diag.last_eff;
    out.last_dlc   = s_diag.last_dlc;
    for (int i = 0; i < 8; i++) out.last_data[i] = s_diag.last_data[i];
}

// ── IMX5 callback ────────────────────────────────────────────────────────────

static inline int16_t be16s(const uint8_t *p)
{
    return static_cast<int16_t>((uint16_t(p[0]) << 8) | p[1]);
}

static void imx5_can_cb(const CANRxFrame &f, void *ctx)
{
    (void)ctx;
    chMtxLock(&can_imu_mtx);
    switch (f.std.SID) {
    case 0x01:
        // CID_INS_QUATN2B — quaternion NED→Body [W,X,Y,Z], each int16 scaled by 10000.
        g_can_imu.q0 = be16s(&f.data8[0]) * (1.0f / 10000.0f);  // W
        g_can_imu.q1 = be16s(&f.data8[2]) * (1.0f / 10000.0f);  // X
        g_can_imu.q2 = be16s(&f.data8[4]) * (1.0f / 10000.0f);  // Y
        g_can_imu.q3 = be16s(&f.data8[6]) * (1.0f / 10000.0f);  // Z
        g_can_imu.has_new_quat = true;
        g_can_imu.valid        = true;
        break;
    case 0x02:
        g_can_imu.p  = be16s(&f.data8[0]) * (1.0f / 1000.0f);
        g_can_imu.ax = be16s(&f.data8[2]) * (1.0f / 100.0f);
        g_can_imu.has_new_rates = true;
        break;
    case 0x03:
        g_can_imu.q  = be16s(&f.data8[0]) * (1.0f / 1000.0f);
        g_can_imu.ay = be16s(&f.data8[2]) * (1.0f / 100.0f);
        break;
    case 0x04:
        g_can_imu.r  = be16s(&f.data8[0]) * (1.0f / 1000.0f);
        g_can_imu.az = be16s(&f.data8[2]) * (1.0f / 100.0f);
        break;
    default: break;
    }
    chMtxUnlock(&can_imu_mtx);
}

void can_drv_init(void)
{
    canStart(&CAND1, &can_cfg);
    bprl_can_register(0x01, imx5_can_cb, nullptr);
    bprl_can_register(0x02, imx5_can_cb, nullptr);
    bprl_can_register(0x03, imx5_can_cb, nullptr);
    bprl_can_register(0x04, imx5_can_cb, nullptr);
}
