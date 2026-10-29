#include "src/uptime.hpp"
#include "src/coms/CAN.hpp"
#include "src/threads.hpp"
#include "configs/DroneConfig.hpp"
#include <cstring>

/*
 * FDCAN1/FDCAN2 — direct register-level driver. Does NOT use ChibiOS's canStart()/
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
 * Message RAM layout — FDCAN1 and FDCAN2 share one message-RAM block at
 * SRAMCAN_BASE; RXF0C.F0SA is a byte offset from that shared base, so each
 * instance just gets its own non-overlapping slice:
 *   FDCAN1 RX FIFO0: words   0..63
 *   FDCAN2 RX FIFO0: words  64..127
 *   Each FIFO is 16 elements x 4 words (2 header + 2 data, matches RXESC=0
 *   → 8-byte classical-CAN data field), overwrite-on-full mode so a
 *   momentarily-stalled reader never blocks new data. No filters — RXGFC=0
 *   routes every standard/extended frame into FIFO0 directly.
 * (BPRL_balance gets the same split by patching ChibiOS's FDCAN LLD and
 * using canStart() on CAND1/CAND2; this project bypasses that LLD.)
 *
 * RX is interrupt-driven: FDCAN1_IT0 ("Vector8C") / FDCAN2_IT0 ("Vector90")
 * fire on RF0N (new message), RF0L (message lost / FIFO overflow), or BO
 * (bus-off) and just signal one shared binary semaphore — no register
 * decoding or FIFO draining happens in ISR context. CANThread blocks on that
 * semaphore and drains both FIFOs (bprl_can_poll()) at thread priority.
 * These are our own minimal handlers, not ChibiOS's — hence HAL_USE_CAN=FALSE
 * in halconf.h, which compiles out ChibiOS's own conflicting definitions of
 * these vectors (they would otherwise collide at link time).
 *
 * FDCAN2 is only started when BPRL_BOARD_HAS_CAN2 (board.h) and
 * kDroneConfig.sensors.can2_enable are both set.
 *
 * Bit timing: 1 Mbit/s, PLL2Q = 80 MHz. BRP=5, TSEG1=13, TSEG2=2 → 16 Tq/bit,
 * 87.5% sample point. 80 MHz / (5 × 16) = 1 Mbit/s.
 * NBTP: NSJW=0, NBRP=4, NTSEG1=12, NTSEG2=1.
 */
#define CAN_NBTP             0x00040C01U
#define CAN_RXF0_ELEMENTS    16U
#define CAN_RXF0_ELEM_WORDS  4U   // 2 header words + 2 data words (8-byte DLC)

struct CANBusHW {
    FDCAN_GlobalTypeDef *regs;
    uint32_t             ram_word_off;   // start of this instance's RxFIFO0 in message RAM
    uint32_t             irq_num;
    uint32_t             irq_prio;
};

static const CANBusHW s_hw[CAN_NUM_BUSES] = {
    { FDCAN1, 0U,                                     STM32_FDCAN1_IT0_NUMBER, STM32_IRQ_FDCAN1_PRIORITY },
    { FDCAN2, CAN_RXF0_ELEMENTS * CAN_RXF0_ELEM_WORDS, STM32_FDCAN2_IT0_NUMBER, STM32_IRQ_FDCAN2_PRIORITY },
};

static bool s_bus_enabled[CAN_NUM_BUSES] = { false, false };

static BSEMAPHORE_DECL(can_rx_bsem, TRUE);   // starts "taken" — first wait blocks
static volatile uint32_t s_msg_lost[CAN_NUM_BUSES]     = {}; // RXF0L events (hardware FIFO overflow)
static volatile uint32_t s_reinit_count[CAN_NUM_BUSES] = {}; // can_hw_reinit() calls (Bus_Off recoveries)

static inline void can_serve_irq(CanBus bus)
{
    FDCAN_GlobalTypeDef *f = s_hw[bus].regs;
    uint32_t ir = f->IR;
    f->IR  = ir;   // write-1-to-clear every flag observed

    if (ir & FDCAN_IR_RF0L) s_msg_lost[bus]++;

    chSysLockFromISR();
    chBSemSignalI(&can_rx_bsem);
    chSysUnlockFromISR();
}

OSAL_IRQ_HANDLER(STM32_FDCAN1_IT0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    can_serve_irq(CAN_BUS_1);
    OSAL_IRQ_EPILOGUE();
}

OSAL_IRQ_HANDLER(STM32_FDCAN2_IT0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    can_serve_irq(CAN_BUS_2);
    OSAL_IRQ_EPILOGUE();
}

msg_t bprl_can_wait_rx(sysinterval_t timeout)
{
    return chBSemWaitTimeout(&can_rx_bsem, timeout);
}

// Spin until CCCR.INIT reads the requested value, or give up after a
// generous bound. FDCAN synchronizes INIT set/clear across clock domains,
// so a few cycles of latency here is normal and not itself an error.
static bool fdcan_wait_init(FDCAN_GlobalTypeDef *f, bool want_set)
{
    for (int i = 0; i < 100000; i++) {
        bool is_set = (f->CCCR & FDCAN_CCCR_INIT) != 0;
        if (is_set == want_set) return true;
    }
    return false;
}

struct CANDevice {
    uint32_t    id;
    CANCallback callback;
    void       *ctx;
};
static CANDevice can_table[CAN_NUM_BUSES][MAX_CAN_DEVICES];
static int       num_can_devices[CAN_NUM_BUSES] = { 0, 0 };

static volatile CANDiag s_diag[CAN_NUM_BUSES] = {};

void bprl_can_register(CanBus bus, uint32_t id, CANCallback cb, void *ctx)
{
    if (num_can_devices[bus] < MAX_CAN_DEVICES) {
        can_table[bus][num_can_devices[bus]++] = {id, cb, ctx};
    }
}

// ── ID scanner ───────────────────────────────────────────────────────────────

static volatile bool s_scan_active = false;
static CanBus        s_scan_bus    = CAN_BUS_1;
static CANScanEntry  s_scan[CAN_SCAN_MAX];
static int           s_scan_n = 0;

void can_scan_start(CanBus bus)
{
    s_scan_active = false;
    s_scan_bus    = bus;
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

void can_dispatch(CanBus bus, const CANRxFrame &frame)
{
    const bool is_ext    = frame.common.XTD;
    const uint32_t match = is_ext ? frame.ext.EID : frame.std.SID;
    volatile CANDiag &d  = s_diag[bus];

    // Record every frame that reaches CANThread, regardless of ID.
    d.total_rx++;
    d.last_sid  = frame.std.SID;
    d.last_eff  = is_ext ? 1U : 0U;
    d.last_eid  = is_ext ? frame.ext.EID : 0U;
    d.last_dlc  = frame.DLC;
    for (int i = 0; i < 8; i++) d.last_data[i] = frame.data8[i];

    // Accumulate per-ID counts for the scanner.
    if (s_scan_active && bus == s_scan_bus) {
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
    for (int i = 0; i < num_can_devices[bus]; i++) {
        const CANDevice &dev = can_table[bus][i];
        if (dev.id == match) {
            dev.callback(frame, dev.ctx);
            matched = true;
        }
    }
    if (matched) d.dispatched++;
}

void can_get_diag(CanBus bus, CANDiag &out)
{
    const volatile CANDiag &d = s_diag[bus];
    out.total_rx   = d.total_rx;
    out.dispatched = d.dispatched;
    out.last_sid   = d.last_sid;
    out.last_eid   = d.last_eid;
    out.last_eff   = d.last_eff;
    out.last_dlc   = d.last_dlc;
    for (int i = 0; i < 8; i++) out.last_data[i] = d.last_data[i];
    out.msg_lost     = s_msg_lost[bus];
    out.reinit_count = s_reinit_count[bus];
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
        g_can_imu.has_new_quat     = true;
        g_can_imu.quat_timestamp_us = bprl_micros();
        g_can_imu.valid        = true;
        break;
    case 0x02:
        g_can_imu.p  = le16s(&f.data8[0]) * (1.0f / 1000.0f);
        g_can_imu.ax = le16s(&f.data8[2]) * (1.0f / 100.0f);
        g_can_imu.has_new_rates      = true;
        g_can_imu.rates_timestamp_us = bprl_micros();
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


static inline volatile uint32_t *can_rxf0_base(CanBus bus)
{
    return reinterpret_cast<volatile uint32_t *>(SRAMCAN_BASE) + s_hw[bus].ram_word_off;
}

// Configures one FDCAN's hardware (bit timing + message RAM). Idempotent —
// safe to call again for Bus_Off recovery without touching can_table[].
// FDCAN1 and FDCAN2 share one kernel clock enable (rccEnableFDCAN), and
// re-enabling it doesn't reset either instance, so reinitializing one bus
// never disturbs the other.
static void can_hw_init(CanBus bus)
{
    FDCAN_GlobalTypeDef *f = s_hw[bus].regs;

    rccEnableFDCAN(true);

    f->CCCR |= FDCAN_CCCR_INIT;
    fdcan_wait_init(f, true);
    f->CCCR |= FDCAN_CCCR_CCE;

    // Zero our slice of message RAM before (re)configuring it.
    volatile uint32_t *ram = can_rxf0_base(bus);
    for (uint32_t i = 0; i < CAN_RXF0_ELEMENTS * CAN_RXF0_ELEM_WORDS; i++) {
        ram[i] = 0;
    }

    f->NBTP  = CAN_NBTP;
    f->DBTP  = 0;               // unused (classical CAN, FDOE=0)
    f->TEST  = 0;
    f->RXGFC = 0;               // accept all std+ext frames into RxFIFO0, no filters
    f->RXF0C = (CAN_RXF0_ELEMENTS << FDCAN_RXF0C_F0S_Pos) | FDCAN_RXF0C_F0OM |
               ((s_hw[bus].ram_word_off << 2) & FDCAN_RXF0C_F0SA_Msk);   // F0SA: byte offset
    f->RXESC = 0;               // 8-byte data field (classical CAN)

    f->CCCR &= ~(FDCAN_CCCR_CCE | FDCAN_CCCR_INIT);
    fdcan_wait_init(f, false);

    f->IR  = (uint32_t)-1;   // clear any stale flags before enabling
    f->IE  = FDCAN_IE_RF0NE | FDCAN_IE_RF0LE | FDCAN_IE_BOE;
    f->ILE = FDCAN_ILE_EINT0;
    nvicEnableVector(s_hw[bus].irq_num, s_hw[bus].irq_prio);
}

void can_read_status(CanBus bus, CANStatusRegs &out)
{
    FDCAN_GlobalTypeDef *f = s_hw[bus].regs;
    out.psr   = f->PSR;
    out.ecr   = f->ECR;
    out.rxf0s = f->RXF0S;
    out.cccr  = f->CCCR;
}

int can_read_regs(CanBus bus, CANRegEntry *out, int max)
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
    const uint8_t *base = reinterpret_cast<const uint8_t *>(s_hw[bus].regs);
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
    s_bus_enabled[CAN_BUS_1] = true;
    can_hw_init(CAN_BUS_1);
#if defined(BPRL_BOARD_HAS_CAN2)
    if (kDroneConfig.sensors.can2_enable) {
        s_bus_enabled[CAN_BUS_2] = true;
        can_hw_init(CAN_BUS_2);
    }
#endif

    // Registered on the configured bus even if that bus isn't running
    // (imx5_can_bus=2 without can2_enable) — HW,CANIMU then just stays 0.
    const CanBus imx5_bus = (kDroneConfig.sensors.imx5_can_bus == 2) ? CAN_BUS_2 : CAN_BUS_1;
    bprl_can_register(imx5_bus, 0x01, imx5_can_cb, nullptr);
    bprl_can_register(imx5_bus, 0x02, imx5_can_cb, nullptr);
    bprl_can_register(imx5_bus, 0x03, imx5_can_cb, nullptr);
    bprl_can_register(imx5_bus, 0x04, imx5_can_cb, nullptr);
}

bool can_bus_enabled(CanBus bus)
{
    return s_bus_enabled[bus];
}

void can_hw_reinit(CanBus bus)
{
    if (!s_bus_enabled[bus]) return;
    s_reinit_count[bus]++;
    can_hw_init(bus);
}

bool bprl_can_poll(CanBus bus, CANRxFrame &out)
{
    if (!s_bus_enabled[bus]) return false;

    FDCAN_GlobalTypeDef *f = s_hw[bus].regs;
    uint32_t rxf0s = f->RXF0S;
    if ((rxf0s & FDCAN_RXF0S_F0FL_Msk) == 0) return false;   // FIFO empty

    uint32_t gi = (rxf0s & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
    volatile uint32_t *elem = can_rxf0_base(bus) + gi * CAN_RXF0_ELEM_WORDS;

    out.header32[0] = elem[0];
    out.header32[1] = elem[1];
    out.data32[0]   = elem[2];
    out.data32[1]   = elem[3];

    f->RXF0A = gi;   // free this slot
    return true;
}

bool can_is_bus_off(CanBus bus)
{
    if (!s_bus_enabled[bus]) return false;
    return (s_hw[bus].regs->PSR & FDCAN_PSR_BO_Msk) != 0;
}
