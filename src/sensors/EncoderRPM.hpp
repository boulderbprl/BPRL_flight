#pragma once
#include "ch.h"
#include "hal.h"
#include "src/sensors/SensorHealth.hpp"

/*
 * Shaft-angle encoder RPM sensor nodes — Feather M4 CAN Express + AS5047P,
 * one node per motor (4 motors, 4 nodes), for BDShot RPM cross-validation
 * on the strain-gauge drone. See externalMCUs/Encoder_CAN/Feather_Code/Feather_Code.ino
 * for the sensor-side firmware (NODE_ID selects which CAN address a given
 * board broadcasts on).
 *
 * CAN IDs 0x70 (node 0) .. 0x73 (node 3), one per node, 8-byte frame,
 * little-endian (matches STM32's native byte order):
 *   bytes 0-3  float    rpm         filtered mechanical RPM, signed
 *   bytes 4-5  uint16_t angle_raw   last good raw 14-bit shaft angle (0-16383/rev)
 *   byte  6    uint8_t  error_flag  AS5047P EF bit or SPI parity error on the last sample
 *   byte  7    uint8_t  status      ENCODER_STATUS_* bits below
 *
 * Each node samples at 2 kHz, rejects bad samples (bad SPI read, sample gap,
 * |RPM| > 20000 — the last good RPM is used instead), low-passes at 100 Hz
 * (2nd-order Butterworth, anti-aliasing only) and sends at 400 Hz. It also
 * decides its own health (ENCODER_STATUS_HEALTHY). Keep the frame layout
 * and status bits in sync with the sketch.
 *
 * Bad samples are rejected on the node: a bad SPI read, a sample gap, |RPM|
 * > 20000, or a jump of more than 1500 RPM from the last good sample
 * (accepted after 4 samples in a row agree — a real fast change). The node
 * goes unhealthy on a leaky bad-sample count — solid garbage trips it in
 * 50 ms, scattered bad samples never do — or at once on a magnet fault.
 */

// Status byte bits — mirrored in Feather_Code.ino (ENC_STATUS_*).
#define ENCODER_STATUS_HEALTHY  0x01  // the node's own verdict: RPM is usable
#define ENCODER_STATUS_HELD     0x02  // >=1 sample since its last frame was rejected and the last good RPM used
#define ENCODER_STATUS_MAGL     0x04  // magnet field too weak (node reports unhealthy)
#define ENCODER_STATUS_MAGH     0x08  // magnet field too strong (reported only)
#define ENCODER_STATUS_COF      0x10  // AS5047P CORDIC overflow (node reports unhealthy)

#define ENCODER_RPM_NUM_NODES   4
#define ENCODER_RPM_CAN_ID_BASE 0x70

struct EncoderRPMRaw {
    float    rpm;         // filtered mechanical RPM, signed by rotation direction
    uint16_t angle_raw;   // last raw 14-bit shaft angle (0-16383 counts / rev)
    uint8_t  error_flag;  // AS5047P EF bit or SPI parity error on the last sample
    uint8_t  status;      // ENCODER_STATUS_* bits from the node
    bool     valid;       // true once at least one frame has arrived from this node
    uint32_t last_rx_us;  // bprl_micros() when the last frame arrived — for EncoderRpmMonitor
    uint32_t rx_count;    // frames received from this node since boot
};

extern mutex_t       encoderRpm_mtx;
extern EncoderRPMRaw g_encoder_rpm[ENCODER_RPM_NUM_NODES];

// Per-node health and the RPM to use, evaluated once per ControlThread tick
// from a snapshot of g_encoder_rpm. The node judges its own samples
// (ENCODER_STATUS_HEALTHY, with its own debounce); this adds only what the
// node can't see. Per node:
//
//   GOOD     a frame within ENCODER_RPM_FRESH_US that carries HEALTHY.
//   HOLDING  no frame for ENCODER_RPM_FRESH_US..ENCODER_RPM_STALE_US (a
//            short CAN hiccup). The RPM is the last good one, moved on by
//            the change in the motor model below since then.
//   BAD      any of (ENCODER_FAULT_* bits say which):
//              NO_DATA  no frame ever received
//              STALE    no frame for more than ENCODER_RPM_STALE_US
//              NODE     the frame doesn't carry ENCODER_STATUS_HEALTHY (also
//                       true of a node still on older firmware, where byte 7
//                       was always 0)
//              RANGE    rpm is NaN or beyond ENCODER_RPM_MAX_ABS — can't come
//                       from a working node, so no debounce
//              STALL    while require_spin, |rpm| below ENCODER_RPM_MIN_SPIN
//                       for ENCODER_RPM_FAULT_TICKS; require_spin should be
//                       true only while every motor must be turning (flying,
//                       past spool-up) — a stalled or disconnected motor reads
//                       a perfectly healthy 0 RPM
//              SIGN     while require_spin, rpm has the wrong sign for that
//                       motor's spin direction (SensorsConfig::
//                       encoder_rpm_sign) for ENCODER_RPM_FAULT_TICKS —
//                       catches a NODE_ID flashed onto the wrong arm
//            RPM is 0 (the notch tracker then averages the motors still
//            reporting).
//
// There is no recovery dwell here: a node is GOOD again on its next good
// frame. The dwell before INDI flies again is FlightStateMachine's.
#define ENCODER_RPM_FRESH_US        7500U     // 3 missed frames at 400 Hz
#define ENCODER_RPM_STALE_US        50000U    // 50 ms — the hold INDI tolerates
#define ENCODER_RPM_MIN_SPIN        500.0f    // mechanical RPM; far below any in-flight motor speed
#define ENCODER_RPM_MAX_ABS         25000.0f  // the node clamps samples at 20000; filtered output can't exceed it
#define ENCODER_RPM_FAULT_TICKS     20U       // 50 ms at 400 Hz

#define ENCODER_FAULT_NO_DATA  0x01
#define ENCODER_FAULT_STALE    0x02
#define ENCODER_FAULT_NODE     0x04
#define ENCODER_FAULT_RANGE    0x08
#define ENCODER_FAULT_STALL    0x10
#define ENCODER_FAULT_SIGN     0x20

// Bits 5-7 of the status byte: frame counter, +1 per frame mod 8 (Feather
// ENC_STATUS_SEQ_SHIFT). Used only to count lost frames (seq_gaps).
#define ENCODER_STATUS_SEQ_SHIFT  5

/*
 * Motor model — commanded throttle -> RPM, used to carry a held RPM forward
 * while a node is HOLDING, and logged as a residual against the measured
 * RPM. Simple placeholder: steady-state RPM linear in the mixer's motor
 * command (0..1000 -> 0..1), reached through a first-order lag.
 *
 * TODO: replace with the measured PWM -> RPM curve and motor time constant
 * for Drone2 once they are bench-tested. Until then only the *change* in the
 * model is used (the hold) and the residual is log-only — don't gate health
 * on it.
 */
#define ENCODER_MODEL_RPM_AT_FULL   18000.0f  // TODO: measured RPM at motor command 1000
#define ENCODER_MODEL_TAU_S         0.030f    // TODO: measured motor time constant

struct EncoderNodeHealth {
    SensorHealth state;
    float        rpm_abs;    // |RPM| to use: live (GOOD), carried forward (HOLDING), 0 (BAD)
    uint8_t      fault;      // ENCODER_FAULT_* bits behind a BAD state, else 0
    float        model_rpm;  // motor model's RPM for this node's motor
};

class EncoderRpmMonitor {
public:
    // motor_cmd[node]: last tick's mixer command for that node's motor,
    // 0..1 (0 when disarmed). expected_sign[node]: +1/-1 the sign the node's
    // rpm must have in flight, 0 = don't check. dt_s: ControlThread period.
    // Call from one thread only.
    void update(const EncoderRPMRaw snap[ENCODER_RPM_NUM_NODES], uint32_t now_us,
                bool require_spin, const float motor_cmd[ENCODER_RPM_NUM_NODES],
                const int8_t expected_sign[ENCODER_RPM_NUM_NODES], float dt_s,
                EncoderNodeHealth out[ENCODER_RPM_NUM_NODES]);

    // Frames lost on the bus since boot, from the frame counter.
    uint32_t seq_gaps(int node) const { return _node[node].seq_gaps; }

private:
    struct Node {
        float    last_good_abs   = 0.0f;  // |rpm| of the last GOOD tick
        float    model_at_good   = 0.0f;  // model_rpm at that tick
        float    model_rpm       = 0.0f;
        uint32_t slow_ticks      = 0;
        uint32_t sign_ticks      = 0;
        uint32_t last_rx_count   = 0;
        uint8_t  last_seq        = 0;
        uint32_t seq_gaps        = 0;
    };
    Node _node[ENCODER_RPM_NUM_NODES];
};

// Call after can_drv_init() in main.cpp. Registers one CAN callback per node
// (0x70, 0x71, ... up to ENCODER_RPM_NUM_NODES).
void encoder_rpm_init(void);
