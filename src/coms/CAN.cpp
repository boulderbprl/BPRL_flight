#include "src/coms/CAN.hpp"
#include "src/threads.hpp"
#include <cstring>

/*
 * FDCAN1 bit timing — 1 Mbit/s, PLL2Q = 80 MHz.
 * BRP=5, TSEG1=13, TSEG2=2 → 16 Tq/bit, 87.5% sample point.
 * 80 MHz / (5 × 16) = 1 Mbit/s.
 * NBTP: NSJW=0, NBRP=4, NTSEG1=12, NTSEG2=1.
 */
static const CANConfig can_cfg = {
    0x00040C01,   // NBTP
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

// ── ID scanner ───────────────────────────────────────────────────────────────

static volatile bool s_scan_active = false;
static CANScanEntry  s_scan[CAN_SCAN_MAX];
static int           s_scan_n = 0;

void can_scan_start(void)
{
    s_scan_active = false;
    s_scan_n      = 0;
    memset(s_scan, 0, sizeof(s_scan));
    s_scan_active = true;
}

void can_scan_stop(void)  { s_scan_active = false; }

int can_scan_get(CANScanEntry *out, int max)
{
    int n = s_scan_n < max ? s_scan_n : max;
    memcpy(out, s_scan, (size_t)n * sizeof(CANScanEntry));
    return n;
}

// ─────────────────────────────────────────────────────────────────────────────

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

    // Accumulate per-ID counts for the scanner.
    if (s_scan_active) {
        bool found = false;
        for (int i = 0; i < s_scan_n && !found; i++) {
            if (s_scan[i].id == match && s_scan[i].is_ext == (uint8_t)is_ext) {
                s_scan[i].count++;
                found = true;
            }
        }
        if (!found && s_scan_n < CAN_SCAN_MAX) {
            s_scan[s_scan_n++] = {match, 1, (uint8_t)is_ext};
        }
    }

    // Match on SID for standard frames, full EID for extended frames.
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

static inline int16_t le16s(const uint8_t *p)
{
    int16_t v;
    __builtin_memcpy(&v, p, sizeof(v));
    return v;
}

static void imx5_can_cb(const CANRxFrame &f, void *ctx)
{
    (void)ctx;
    chMtxLock(&can_imu_mtx);
    switch (f.std.SID) {
    case 0x01:
        // CID_INS_QUATN2B — quaternion NED→Body [W,X,Y,Z], each int16 scaled by 10000.
        g_can_imu.q0 = le16s(&f.data8[0]) * (1.0f / 10000.0f);  // W
        g_can_imu.q1 = le16s(&f.data8[2]) * (1.0f / 10000.0f);  // X
        g_can_imu.q2 = le16s(&f.data8[4]) * (1.0f / 10000.0f);  // Y
        g_can_imu.q3 = le16s(&f.data8[6]) * (1.0f / 10000.0f);  // Z
        g_can_imu.has_new_quat = true;
        g_can_imu.valid        = true;
        break;
    case 0x02:
        g_can_imu.p  = le16s(&f.data8[0]) * (1.0f / 1000.0f);
        g_can_imu.ax = le16s(&f.data8[2]) * (1.0f / 100.0f);
        g_can_imu.has_new_rates = true;
        g_can_imu.valid         = true;
        break;
    case 0x03:
        g_can_imu.q  = le16s(&f.data8[0]) * (1.0f / 1000.0f);
        g_can_imu.ay = le16s(&f.data8[2]) * (1.0f / 100.0f);
        break;
    case 0x04:
        g_can_imu.r  = le16s(&f.data8[0]) * (1.0f / 1000.0f);
        g_can_imu.az = le16s(&f.data8[2]) * (1.0f / 100.0f);
        break;
    default: break;
    }
    chMtxUnlock(&can_imu_mtx);
}

int can_read_regs(CANRegEntry *out, int max)
{
    if (!out || max < 1) return 0;
    auto *f = CAND1.fdcan;
    static const struct { const char *n; uint32_t off; } regs[] = {
        { "CCCR",  0x018 },
        { "NBTP",  0x01C },
        { "RXGFC", 0x080 },
        { "RXF0C", 0x0A0 },
        { "RXF0S", 0x0A4 },
        { "RXESC", 0x1BC },
        { "PSR",   0x044 },
        { "ECR",   0x040 },
    };
    int n = 0;
    const uint8_t *base = reinterpret_cast<const uint8_t *>(f);
    for (auto &r : regs) {
        if (n >= max) break;
        uint32_t v;
        __builtin_memcpy(&v, base + r.off, 4);
        out[n++] = { r.n, v };
    }
    return n;
}

void can_drv_init(void)
{
    canStart(&CAND1, &can_cfg);
    bprl_can_register(0x01, imx5_can_cb, nullptr);
    bprl_can_register(0x02, imx5_can_cb, nullptr);
    bprl_can_register(0x03, imx5_can_cb, nullptr);
    bprl_can_register(0x04, imx5_can_cb, nullptr);
}
