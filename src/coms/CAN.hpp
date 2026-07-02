#pragma once
#include "hal.h"

/*
 * HAL_USE_CAN is FALSE (cfg/halconf.h) — this driver talks to FDCAN1
 * directly and owns the FDCAN1_IT0 vector itself (see CAN.cpp), so
 * ChibiOS's own CAN HAL/ISR must be compiled out to avoid a conflicting
 * definition of that interrupt vector. CANRxFrame is normally provided by
 * hal_can_lld.h; redefined here with an identical layout (it mirrors the
 * real FDCAN hardware Rx element R0/R1 header words) so the rest of this
 * file's callers (can_dispatch, imx5_can_cb, ...) are unchanged.
 */
#define CAN_MAX_DLC_BYTES 8   // classical CAN only, no FD frames in this project

struct CANRxFrame {
    union {
        struct {
            union {
                struct { uint32_t EID:29; } ext;
                struct { uint32_t _R1:18; uint32_t SID:11; } std;
                struct { uint32_t _R1:29; uint32_t RTR:1; uint32_t XTD:1; uint32_t ESI:1; } common;
            };
            uint32_t RXTS:16;
            uint32_t DLC:4;
            uint32_t BRS:1;
            uint32_t FDF:1;
            uint32_t _R2:2;
            uint32_t FIDX:7;
            uint32_t ANMF:1;
        };
        uint32_t header32[2];
    };
    union {
        uint8_t  data8[CAN_MAX_DLC_BYTES];
        uint16_t data16[CAN_MAX_DLC_BYTES / 2];
        uint32_t data32[CAN_MAX_DLC_BYTES / 4];
    };
};

/*
 * CAN bus driver — FDCAN1 at 1 Mbit/s.
 *
 * Device registration:
 *   Call bprl_can_register() in main() before threads_start().
 *   can_drv_init() pre-registers the IMX5 IMU callbacks.
 *
 * IMX5 frame protocol (1 Mbit/s, standard 11-bit IDs):
 *   ID 0x01  CID_INS_QUATN2B — quaternion NED→Body [W,X,Y,Z] (int16 ÷ 10000), 200 Hz
 *            bytes 0-1 = W (q0), 2-3 = X (q1), 4-5 = Y (q2), 6-7 = Z (q3)
 *   ID 0x02  bytes 0-1 = p rate  (int16 ÷ 1000 → rad/s), 2-3 = x accel (÷ 100 → m/s²)
 *   ID 0x03  bytes 0-1 = q rate,                          2-3 = y accel
 *   ID 0x04  bytes 0-1 = r rate,                          2-3 = z accel   (100 Hz each)
 *
 * Adding devices:
 *   1. Write a callback: void my_cb(const CANRxFrame &f, void *ctx)
 *   2. In main(): bprl_can_register(MY_ID, my_cb, nullptr);
 */

#define MAX_CAN_DEVICES 8
typedef void (*CANCallback)(const CANRxFrame &frame, void *ctx);

struct CANDiag {
    uint32_t total_rx;     // total frames received by CANThread (any ID)
    uint32_t dispatched;   // frames that matched a registered callback
    uint32_t last_sid;     // SID of most recent frame (11-bit standard ID field)
    uint32_t last_eid;     // EID of most recent frame (18-bit extension, 0 if std)
    uint8_t  last_eff;     // 1 = extended frame (29-bit ID), 0 = standard (11-bit)
    uint8_t  last_dlc;
    uint8_t  last_data[8];
    uint32_t msg_lost;      // RxFIFO0 overflow events (hardware dropped a frame, buffer was full)
    uint32_t reinit_count;  // times can_hw_reinit() has run (Bus_Off recoveries)
};

// Register a handler for a specific standard CAN ID.
void bprl_can_register(uint32_t id, CANCallback cb, void *ctx);

// Dispatch one received frame to its registered handler (called by CANThread).
void can_dispatch(const CANRxFrame &frame);

// Copy out current diagnostic counters (safe to call from any thread).
void can_get_diag(CANDiag &out);

// Start FDCAN1 and register the built-in IMX5 callbacks.
void can_drv_init(void);

// Re-run the FDCAN1 hardware init (bit timing + message RAM) without
// touching device registrations. Safe to call repeatedly, e.g. for
// Bus_Off recovery.
void can_hw_reinit(void);

// Poll RxFIFO0 for one frame (non-blocking). Returns false if empty.
// Called from CANThread instead of ChibiOS's canReceiveTimeout() — see the
// comment at the top of CAN.cpp for why.
bool bprl_can_poll(CANRxFrame &out);

// Blocks until the FDCAN1 ISR signals new-message/message-lost/bus-off, or
// timeout elapses. Returns MSG_OK or MSG_TIMEOUT (both handled the same way
// by the caller — the timeout just guarantees periodic bus-off checks).
msg_t bprl_can_wait_rx(sysinterval_t timeout);

// True if FDCAN1 has entered the Bus_Off protocol state.
bool can_is_bus_off(void);

// Read key FDCAN1 hardware registers into out[].  Returns number of entries.
// Format: out[i] = {name, value}.
struct CANRegEntry { const char *name; uint32_t value; };
int can_read_regs(CANRegEntry *out, int max);

// ── ID scanner (diagnostic) ───────────────────────────────────────────────
#define CAN_SCAN_MAX 48

struct CANScanEntry {
    uint32_t id;
    uint32_t count;
    uint8_t  is_ext;  // 1 = extended frame (29-bit EID), 0 = standard (11-bit SID)
};

// Start accumulating unique-ID counts.  Resets the buffer each call.
void can_scan_start(void);

// Stop accumulating.
void can_scan_stop(void);

// Copy the accumulated entries into `out` (up to `max`).  Returns entry count.
int  can_scan_get(CANScanEntry *out, int max);
