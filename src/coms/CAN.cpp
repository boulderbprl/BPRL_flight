#include "src/coms/CAN.hpp"
#include "src/threads.hpp"
#include <cstring>

/*
 * FDCAN1 — direct register-level driver. Does NOT use ChibiOS's canStart()/
 * canReceiveTimeout()/CAND1.
 *
 * Why: ChibiOS's can_lld_start() (third_party/ChibiOS/os/hal/ports/STM32/LLD/
 * FDCANv1/hal_can_lld.c) writes NBTP/DBTP/CCCR/TEST/RXGFC but never writes
 * RXF0C — the register that gives RxFIFO0 an actual start address and
 * element count in message RAM. On STM32H7 that register resets to 0
 * (zero-depth FIFO), so canStart() "succeeds" but every frame is ACKed on
 * the wire (no bus errors, ACT=Idle, bus looks perfectly healthy) and then
 * discarded because there's nowhere to store it. Confirmed via CAN,regdump:
 * RXF0C read back 0x00000000 after canStart(). ArduPilot doesn't hit this
 * because it has its own FDCAN driver (AP_HAL_ChibiOS/CANFDIface.cpp) and
 * never goes through this ChibiOS HAL layer.
 *
 * Message RAM layout — FDCAN2 is unused (STM32_CAN_USE_FDCAN2=FALSE), so
 * FDCAN1 owns the whole message-RAM block starting at SRAMCAN_BASE:
 *   RX FIFO0: 16 elements x 4 words (2 header + 2 data, matches RXESC=0 →
 *   8-byte classical-CAN data field), offset 0, overwrite-on-full mode so a
 *   momentarily-stalled reader never blocks new data. No filters — RXGFC=0
 *   routes every standard/extended frame into FIFO0 directly.
 *
 * RX is interrupt-driven: FDCAN1_IT0 (STM32_FDCAN1_IT0_HANDLER, "Vector8C")
 * fires on RF0N (new message), RF0L (message lost / FIFO overflow), or BO
 * (bus-off) and just signals a binary semaphore — no register decoding or
 * FIFO draining happens in ISR context. CANThread blocks on that semaphore
 * and does the actual draining (bprl_can_poll()) at thread priority. This
 * is our own minimal handler, not ChibiOS's — hence HAL_USE_CAN=FALSE in
 * halconf.h, which compiles out ChibiOS's own conflicting definition of
 * this same vector (it would otherwise collide at link time).
 *
 * Bit timing: 1 Mbit/s, PLL2Q = 80 MHz. BRP=5, TSEG1=13, TSEG2=2 → 16 Tq/bit,
 * 87.5% sample point. 80 MHz / (5 × 16) = 1 Mbit/s.
 * NBTP: NSJW=0, NBRP=4, NTSEG1=12, NTSEG2=1.
 */
#define CAN_NBTP             0x00040C01U
#define CAN_RXF0_ELEMENTS    16U
#define CAN_RXF0_ELEM_WORDS  4U   // 2 header words + 2 data words (8-byte DLC)

static BSEMAPHORE_DECL(can_rx_bsem, TRUE);   // starts "taken" — first wait blocks
static volatile uint32_t s_msg_lost     = 0; // RXF0L events (hardware FIFO overflow)
static volatile uint32_t s_reinit_count = 0; // can_hw_reinit() calls (Bus_Off recoveries)

OSAL_IRQ_HANDLER(STM32_FDCAN1_IT0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();

    uint32_t ir = FDCAN1->IR;
    FDCAN1->IR  = ir;   // write-1-to-clear every flag observed

    if (ir & FDCAN_IR_RF0L) s_msg_lost++;

    chSysLockFromISR();
    chBSemSignalI(&can_rx_bsem);
    chSysUnlockFromISR();

    OSAL_IRQ_EPILOGUE();
}

msg_t bprl_can_wait_rx(sysinterval_t timeout)
{
    return chBSemWaitTimeout(&can_rx_bsem, timeout);
}

// Spin until CCCR.INIT reads the requested value, or give up after a
// generous bound. FDCAN synchronizes INIT set/clear across clock domains,
// so a few cycles of latency here is normal and not itself an error.
static bool fdcan1_wait_init(bool want_set)
{
    for (int i = 0; i < 100000; i++) {
        bool is_set = (FDCAN1->CCCR & FDCAN_CCCR_INIT) != 0;
        if (is_set == want_set) return true;
    }
    return false;
}

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
    out.msg_lost     = s_msg_lost;
    out.reinit_count = s_reinit_count;
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


// Configures FDCAN1 hardware (bit timing + message RAM). Idempotent —
// safe to call again for Bus_Off recovery without touching can_table[].
static void can_hw_init(void)
{
    rccEnableFDCAN(true);

    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    fdcan1_wait_init(true);
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;

    // Zero our slice of message RAM before (re)configuring it.
    volatile uint32_t *ram = reinterpret_cast<volatile uint32_t *>(SRAMCAN_BASE);
    for (uint32_t i = 0; i < CAN_RXF0_ELEMENTS * CAN_RXF0_ELEM_WORDS; i++) {
        ram[i] = 0;
    }

    FDCAN1->NBTP  = CAN_NBTP;
    FDCAN1->DBTP  = 0;               // unused (classical CAN, FDOE=0)
    FDCAN1->TEST  = 0;
    FDCAN1->RXGFC = 0;               // accept all std+ext frames into RxFIFO0, no filters
    FDCAN1->RXF0C = (CAN_RXF0_ELEMENTS << FDCAN_RXF0C_F0S_Pos) | FDCAN_RXF0C_F0OM;
    FDCAN1->RXESC = 0;               // 8-byte data field (classical CAN)

    FDCAN1->CCCR &= ~(FDCAN_CCCR_CCE | FDCAN_CCCR_INIT);
    fdcan1_wait_init(false);

    FDCAN1->IR  = (uint32_t)-1;   // clear any stale flags before enabling
    FDCAN1->IE  = FDCAN_IE_RF0NE | FDCAN_IE_RF0LE | FDCAN_IE_BOE;
    FDCAN1->ILE = FDCAN_ILE_EINT0;
    nvicEnableVector(STM32_FDCAN1_IT0_NUMBER, STM32_IRQ_FDCAN1_PRIORITY);
}

int can_read_regs(CANRegEntry *out, int max)
{
    if (!out || max < 1) return 0;
    static const struct { const char *n; uint32_t off; } regs[] = {
        { "CCCR",  0x018 },
        { "NBTP",  0x01C },
        { "RXGFC", 0x080 },
        { "RXF0C", 0x0A0 },
        { "RXF0S", 0x0A4 },
        { "RXESC", 0x0BC },   // was 0x1BC — wrong offset, read the wrong register
        { "PSR",   0x044 },
        { "ECR",   0x040 },
    };
    int n = 0;
    const uint8_t *base = reinterpret_cast<const uint8_t *>(FDCAN1);
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
    can_hw_init();

    bprl_can_register(0x01, imx5_can_cb, nullptr);
    bprl_can_register(0x02, imx5_can_cb, nullptr);
    bprl_can_register(0x03, imx5_can_cb, nullptr);
    bprl_can_register(0x04, imx5_can_cb, nullptr);
}

void can_hw_reinit(void)
{
    s_reinit_count++;
    can_hw_init();
}

bool bprl_can_poll(CANRxFrame &out)
{
    uint32_t rxf0s = FDCAN1->RXF0S;
    if ((rxf0s & FDCAN_RXF0S_F0FL_Msk) == 0) return false;   // FIFO empty

    uint32_t gi = (rxf0s & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
    volatile uint32_t *elem = reinterpret_cast<volatile uint32_t *>(SRAMCAN_BASE) +
                              gi * CAN_RXF0_ELEM_WORDS;

    out.header32[0] = elem[0];
    out.header32[1] = elem[1];
    out.data32[0]   = elem[2];
    out.data32[1]   = elem[3];

    FDCAN1->RXF0A = gi;   // free this slot
    return true;
}

bool can_is_bus_off(void)
{
    return (FDCAN1->PSR & FDCAN_PSR_BO_Msk) != 0;
}
