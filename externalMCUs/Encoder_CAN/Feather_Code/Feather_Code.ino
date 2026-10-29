/*
 * Feather M4 CAN Express — AS5047P shaft-angle encoder -> mechanical RPM
 * -> USB debug print -> CAN frame to the Cube Orange FC (BPRL_flight).
 *
 * Wiring:
 *   AS5047P MISO -> Feather MI
 *   AS5047P MOSI -> Feather MO
 *   AS5047P SCK  -> Feather SCK (PA17)
 *   AS5047P CS   -> Feather A5  (PA05, bit-banged — AS5047P's read protocol
 *                    needs two back-to-back CS-framed transfers per
 *                    register read, so it isn't driven by hardware SS)
 *   Magnet centered ~2 mm above the AS5047P die, on the motor shaft axis.
 *
 * Libraries (Arduino Library Manager):
 *   "Adafruit CANSAME5x"   — on-chip MCAN peripheral, Sandeep Mistry CAN.h
 *                            style API (CAN.beginPacket/write/endPacket).
 *   SPI.h ships with the Adafruit SAMD core.
 *
 * CAN bus:
 *   1 Mbit/s, classical (non-FD) 11-bit standard frames — matches
 *   src/coms/CAN.hpp on the FC. IDs 0x01-0x04 (IMX5) and 0x69 (StrainRate)
 *   are already taken there; each node sends on CAN_ID_ENCODER_RPM_TABLE
 *   [NODE_ID] (0x70 + NODE_ID). The FC side is src/sensors/EncoderRPM.cpp —
 *   keep EncoderRPMFrame and the ENC_STATUS_* bits below in sync with
 *   src/sensors/EncoderRPM.hpp.
 *
 * Pipeline (all on this board):
 *   2 kHz angle read -> angle-difference RPM -> sample rejection (bad SPI
 *   read, sample gap, |RPM| > RPM_MAX, or a jump of more than RPM_MAX_STEP
 *   from the last good RPM: the last good RPM is used instead) -> 2nd-order
 *   Butterworth anti-aliasing low-pass at RPM_FILTER_CUTOFF_HZ
 *   -> CAN frame every SAMPLES_PER_CAN_FRAME samples (400 Hz), sent right
 *   after the sample it carries. Any lower-frequency filtering is the FC's
 *   job.
 *
 * Health: this node decides its own health and sends it as
 * ENC_STATUS_HEALTHY in the frame's status byte — the FC only adds what
 * this board can't see (frames arriving at all, motors expected to spin).
 *
 * Multi-node bring-up: set NODE_ID below to a unique index (0, 1, ...)
 * before flashing each board — this is the only per-board edit needed.
 *
 * Debug output goes to the native USB serial (Serial) and is compiled out
 * entirely when DEBUG is 0. When enabled, open the Arduino Serial Monitor
 * at 115200. If you actually want the hardware UART pins instead, swap
 * every `Serial.` below for `Serial1.` and add Serial1.begin(115200) in
 * setup().
 */

#include <SPI.h>
#include <CANSAME5x.h>
#include <math.h>

// ── Per-board identity — set before flashing each node ─────────────────
#define NODE_ID  0   // unique per board: 0, 1, ... — selects this node's CAN ID

// ── Debug logging — set to 0 to compile out all Serial UART traffic ────
#define DEBUG 0

#if DEBUG
  #define DEBUG_BEGIN(...)   Serial.begin(__VA_ARGS__)
  #define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_BEGIN(...)
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// ── Pin assignments ─────────────────────────────────────────────────────
#define AS5047P_CS_PIN   A5   // PA05, bit-banged chip select (confirmed wired to A5 on the bench, not A6)

// ── AS5047P register map (14-bit addresses) ────────────────────────────
#define AS5047P_REG_ERRFL     0x0001  // error flags, read-to-clear
#define AS5047P_REG_DIAAGC    0x3FFC  // diagnostics + automatic gain control
#define AS5047P_REG_ANGLECOM  0x3FFF  // dynamic-angle-compensated angle

#define AS5047P_CMD_READ  0x4000  // RW bit (14) = 1 -> read
#define AS5047P_ADDR_MASK 0x3FFF

// DIAAGC bits (AS5047P datasheet, DIAAGC register)
#define AS5047P_DIAAGC_LF    (1U << 8)   // offset compensation loop finished — 1 in normal operation
#define AS5047P_DIAAGC_COF   (1U << 9)   // CORDIC overflow — angle invalid
#define AS5047P_DIAAGC_MAGH  (1U << 10)  // field too strong (AGC at minimum) — magnet too close
#define AS5047P_DIAAGC_MAGL  (1U << 11)  // field too weak (AGC at maximum) — magnet too far / gone

static const SPISettings AS5047P_SPI_SETTINGS(1000000, MSBFIRST, SPI_MODE1);

// ── CAN configuration ───────────────────────────────────────────────────
#define CAN_BAUD_BPS 1000000

// One CAN ID per node, indexed by NODE_ID — extend as more boards are added.
static const uint32_t CAN_ID_ENCODER_RPM_TABLE[] = { 0x70, 0x71, 0x72, 0x73 };
#define CAN_ID_ENCODER_RPM CAN_ID_ENCODER_RPM_TABLE[NODE_ID]

CANSAME5x CAN;

// ── Sample/print/tx pacing ──────────────────────────────────────────────
#define SAMPLE_INTERVAL_US     500     // 2 kHz encoder sampling
#define SAMPLE_RATE_HZ         (1000000.0f / SAMPLE_INTERVAL_US)
#define SAMPLES_PER_CAN_FRAME  5       // 2 kHz / 5 = 400 Hz CAN tx, matches the FC's 400 Hz ControlThread
#define PRINT_INTERVAL_US      50000   // 20 Hz USB debug print
#define DIAG_INTERVAL_SAMPLES  40      // DIAAGC magnet check every 40 samples = 50 Hz

// ── RPM filtering / rejection ───────────────────────────────────────────
// Anti-aliasing only, ahead of the 2 kHz -> 400 Hz decimation onto CAN:
// 2nd-order Butterworth, ~2.25 ms group delay at low frequency.
#define RPM_FILTER_CUTOFF_HZ  100.0f

// Any instantaneous |RPM| above this is a bad sample — the last good RPM is
// used in its place.
#define RPM_MAX  20000.0f

// Angle differencing can only unwrap less than half a revolution between
// samples. At RPM_MAX that's 0.5 rev / (RPM_MAX / 60) — a longer gap than
// this (e.g. endPacket() blocking up to ~8 ms when no node ACKs) can't be
// unwrapped, so that sample is rejected and differencing restarts.
#define MAX_SAMPLE_DT_US  ((uint32_t)(30000000.0f / RPM_MAX))   // 1500 us
static_assert(MAX_SAMPLE_DT_US > SAMPLE_INTERVAL_US, "RPM_MAX too high for SAMPLE_INTERVAL_US");

// A sample more than this far from the last good RPM is a spike, not the
// motor: a full-range throttle step with a ~30 ms motor time constant moves
// ~330 RPM per 0.5 ms sample, so this leaves ~4x margin. Rejecting by jump
// adds no delay (unlike a median filter). A real change faster than this —
// or the motor having moved on while this node was rejecting samples — is
// accepted once JUMP_REACCEPT_SAMPLES rejected samples in a row agree with
// each other to within RPM_MAX_STEP, so the gate can't lock itself out.
#define RPM_MAX_STEP           1500.0f
#define JUMP_REACCEPT_SAMPLES  4       // 2 ms at 2 kHz

// Health: a leaky bad-sample counter. Each bad sample adds BAD_SAMPLE_WEIGHT,
// each good one takes 1 off; the node is unhealthy from when the counter
// reaches BAD_LEVEL_FAULT until it has drained back to 0. Scattered bad
// samples never trip it (it trips only while more than 1 in
// BAD_SAMPLE_WEIGHT + 1 samples is bad); solid garbage trips it in
// BAD_LEVEL_FAULT / BAD_SAMPLE_WEIGHT samples (50 ms) — the FC tolerates
// that long a hold anyway. The counter saturates at BAD_LEVEL_FAULT, so
// recovery is at most BAD_LEVEL_FAULT good samples (100 ms).
#define BAD_SAMPLE_WEIGHT  2
#define BAD_LEVEL_FAULT    200

// Frame status byte — mirrored in src/sensors/EncoderRPM.hpp.
#define ENC_STATUS_HEALTHY  0x01  // this node's own verdict: RPM is usable
#define ENC_STATUS_HELD     0x02  // >=1 sample since the last frame was rejected and the last good RPM used
#define ENC_STATUS_MAGL     0x04  // last DIAAGC read: field too weak (unhealthy)
#define ENC_STATUS_MAGH     0x08  // last DIAAGC read: field too strong (reported only)
#define ENC_STATUS_COF      0x10  // last DIAAGC read: CORDIC overflow (unhealthy)
#define ENC_STATUS_SEQ_SHIFT 5    // bits 5-7: frame counter, +1 per frame (mod 8) — lets the FC count lost frames

// Frame sent to the FC — 8 bytes to match the CAN.hpp convention
// (IMX5/StrainRate frames are all fixed 8-byte classical frames).
struct __attribute__((packed)) EncoderRPMFrame {
    float    rpm;         // filtered mechanical RPM, signed by rotation direction
    uint16_t angle_raw;   // last good raw 14-bit shaft angle (0-16383 counts / rev)
    uint8_t  error_flag;  // AS5047P EF bit or parity error on the last sample
    uint8_t  status;      // ENC_STATUS_* bits
};

// 2nd-order Butterworth low-pass, direct form II transposed.
struct Biquad {
    float b0, b1, b2, a1, a2;
    float z1, z2;
};

static Biquad     s_rpm_lpf;
static uint32_t   s_next_sample_us  = 0;
static uint32_t   s_last_print_us   = 0;
static uint32_t   s_sample_count    = 0;
static uint16_t   s_prev_angle_raw  = 0;     // last good angle, and when it was read
static uint32_t   s_prev_angle_us   = 0;
static bool       s_have_prev       = false;
static float      s_rpm_last_good   = 0.0f;  // last accepted instantaneous RPM
static float      s_rpm_filt        = 0.0f;
static uint16_t   s_angle_raw       = 0;
static uint8_t    s_error_flag      = 0;
static uint16_t   s_diaagc          = AS5047P_DIAAGC_LF;
static uint32_t   s_jump_streak     = 0;     // consecutive samples rejected by the jump gate
static float      s_jump_candidate  = 0.0f;  // the last of those samples' RPM
static uint32_t   s_bad_level       = BAD_LEVEL_FAULT;  // leaky bad-sample counter, see BAD_LEVEL_FAULT — starts full: 100 ms clean after boot
static bool       s_faulted         = true;
static uint8_t    s_status_held     = 0;     // ENC_STATUS_HELD accumulated since the last frame
static uint8_t    s_frame_seq       = 0;

// ── Bus_Off watchdog ─────────────────────────────────────────────────────
// CANSAME5x::endPacket() (Adafruit_CAN/src/CANSAME5x.cpp, ~line 344-384)
// returns a truthy value on BOTH the real-success path and its 8ms-timeout
// fallthrough, so the `if (!CAN.endPacket())` check below can never fire —
// this sketch has no visibility into a stuck Bus_Off node through the
// library's own API. The library's bus_autorecover() (clears CCCR.INIT
// when PSR.BO is set) only runs from inside endPacket()/parsePacket(), and
// this node never calls parsePacket() (TX-only), so endPacket() is the
// only place it's ever triggered — the very call whose result we can't
// trust. Poll the SAME51 CAN0 peripheral's PSR.BO bit directly instead,
// and force a full CAN.begin() if it's been stuck Bus_Off longer than the
// library's per-call recovery should ever need.
#define BUS_OFF_REINIT_TICKS  80   // ~200 ms of continuous Bus_Off at the 400 Hz CAN tx cadence

static uint32_t   s_bus_off_ticks   = 0;
static uint32_t   s_bus_off_reinits = 0;

static void check_can_bus_health(void)
{
    if (CAN0->PSR.bit.BO) {
        s_bus_off_ticks++;
        if (s_bus_off_ticks >= BUS_OFF_REINIT_TICKS) {
            s_bus_off_reinits++;
            DEBUG_PRINT("CAN stuck Bus_Off, forcing reinit, count=");
            DEBUG_PRINTLN(s_bus_off_reinits);
            CAN.begin(CAN_BAUD_BPS);
            s_bus_off_ticks = 0;
        }
    } else {
        s_bus_off_ticks = 0;
    }
}

static void biquad_init_lowpass(Biquad &f, float cutoff_hz, float sample_hz)
{
    // Bilinear transform with pre-warping, Q = 1/sqrt(2).
    const float k    = tanf((float)M_PI * cutoff_hz / sample_hz);
    const float norm = 1.0f / (1.0f + (float)M_SQRT2 * k + k * k);
    f.b0 = k * k * norm;
    f.b1 = 2.0f * f.b0;
    f.b2 = f.b0;
    f.a1 = 2.0f * (k * k - 1.0f) * norm;
    f.a2 = (1.0f - (float)M_SQRT2 * k + k * k) * norm;
    f.z1 = 0.0f;
    f.z2 = 0.0f;
}

static float biquad_apply(Biquad &f, float x)
{
    const float y = f.b0 * x + f.z1;
    f.z1 = f.b1 * x - f.a1 * y + f.z2;
    f.z2 = f.b2 * x - f.a2 * y;
    return y;
}

static uint16_t calc_even_parity_bit(uint16_t value)
{
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 0x1;
}

static uint16_t as5047p_transfer16(uint16_t value)
{
    uint16_t result;
    SPI.beginTransaction(AS5047P_SPI_SETTINGS);
    digitalWrite(AS5047P_CS_PIN, LOW);
    result = SPI.transfer16(value);
    digitalWrite(AS5047P_CS_PIN, HIGH);
    SPI.endTransaction();
    return result;
}

// AS5047P read protocol: the response to a register-read command is
// shifted out on the *next* SPI transaction, so each logical read here is
// two back-to-back command frames (the second just repeats the first as a
// filler so we get a clean, self-contained result every call).
// Returns false if the read can't be trusted: the response's EF bit is set
// (the previous frame had an error) or its even-parity bit doesn't match.
static bool as5047p_read_register(uint16_t address, uint16_t *value_out, uint8_t *error_flag_out)
{
    uint16_t cmd = AS5047P_CMD_READ | (address & AS5047P_ADDR_MASK);
    cmd |= calc_even_parity_bit(cmd) << 15;

    as5047p_transfer16(cmd);
    uint16_t response = as5047p_transfer16(cmd);

    const uint8_t ef        = (response >> 14) & 0x1;
    const bool    parity_ok = calc_even_parity_bit(response & 0x7FFF) == (response >> 15);
    if (error_flag_out != nullptr) {
        *error_flag_out = ef;
    }
    *value_out = response & 0x3FFF;
    return !ef && parity_ok;
}

static void as5047p_clear_error(void)
{
    uint16_t unused;
    as5047p_read_register(AS5047P_REG_ERRFL, &unused, nullptr);
}

// Called for a sample the jump gate would reject: true once
// JUMP_REACCEPT_SAMPLES of them in a row agree with each other, i.e. the
// motor really is at the new speed.
static bool jump_reaccepted(float rpm)
{
    if (s_jump_streak > 0 && fabsf(rpm - s_jump_candidate) <= RPM_MAX_STEP) {
        s_jump_streak++;
    } else {
        s_jump_streak = 1;
    }
    s_jump_candidate = rpm;
    return s_jump_streak >= JUMP_REACCEPT_SAMPLES;
}

// One 2 kHz sample: read the angle, form RPM, reject bad samples, filter,
// and update this node's health. Returns true if the sample was good.
static bool process_sample(uint32_t now_us)
{
    uint16_t angle;
    uint8_t  ef;
    bool good = as5047p_read_register(AS5047P_REG_ANGLECOM, &angle, &ef);
    s_error_flag = ef;
    if (!good) {
        as5047p_clear_error();
    }

    float rpm_inst = s_rpm_last_good;
    if (good) {
        s_angle_raw = angle;
        const uint32_t dt_us = now_us - s_prev_angle_us;
        if (!s_have_prev) {
            // First good read — nothing to difference against yet.
        } else if (dt_us == 0 || dt_us > MAX_SAMPLE_DT_US) {
            // Too long since the last good read to unwrap: reject, and
            // restart differencing from here.
            good = false;
        } else {
            int32_t delta = (int32_t)angle - (int32_t)s_prev_angle_raw;
            if (delta > 8192)  delta -= 16384;   // wrapped forward through 0
            if (delta < -8192) delta += 16384;   // wrapped backward through 0

            const float revs = (float)delta / 16384.0f;
            const float rpm  = revs * (60.0f * 1000000.0f / (float)dt_us);
            if (fabsf(rpm) > RPM_MAX) {
                good = false;
            } else if (fabsf(rpm - s_rpm_last_good) > RPM_MAX_STEP &&
                       !jump_reaccepted(rpm)) {
                good = false;
            } else {
                s_jump_streak   = 0;
                rpm_inst        = rpm;
                s_rpm_last_good = rpm;
            }
        }
        // A good read is always the new differencing reference, even if
        // its RPM was rejected.
        s_prev_angle_raw = angle;
        s_prev_angle_us  = now_us;
        s_have_prev      = true;
    }

    // The filter steps every sample at a fixed rate; a rejected sample
    // feeds it the last good RPM.
    s_rpm_filt = biquad_apply(s_rpm_lpf, rpm_inst);

    if (!good) {
        s_status_held |= ENC_STATUS_HELD;
    }
    return good;
}

static void check_magnet(void)
{
    uint16_t diaagc;
    if (as5047p_read_register(AS5047P_REG_DIAAGC, &diaagc, nullptr)) {
        s_diaagc = diaagc;
    } else {
        as5047p_clear_error();   // keep the last good DIAAGC
    }
}

static uint8_t magnet_status(void)
{
    uint8_t st = 0;
    if (s_diaagc & AS5047P_DIAAGC_MAGL) st |= ENC_STATUS_MAGL;
    if (s_diaagc & AS5047P_DIAAGC_MAGH) st |= ENC_STATUS_MAGH;
    if (s_diaagc & AS5047P_DIAAGC_COF)  st |= ENC_STATUS_COF;
    return st;
}

static void update_health(bool sample_good)
{
    if (!sample_good) {
        s_bad_level += BAD_SAMPLE_WEIGHT;
        if (s_bad_level > BAD_LEVEL_FAULT) s_bad_level = BAD_LEVEL_FAULT;
    } else if (s_bad_level > 0) {
        s_bad_level--;
    }

    // MAGH alone is reported but not a fault: the angle is still valid with
    // AGC at its limit, and an over-strong magnet is a bench fix, not a
    // mid-flight failure. MAGL is what a magnet coming off the shaft looks
    // like, so it faults straight away (and holds the counter at the fault
    // level, so recovery still takes BAD_LEVEL_FAULT clean samples).
    if (magnet_status() & (ENC_STATUS_MAGL | ENC_STATUS_COF)) {
        s_bad_level = BAD_LEVEL_FAULT;
    }

    if (s_bad_level >= BAD_LEVEL_FAULT) s_faulted = true;
    else if (s_bad_level == 0)          s_faulted = false;
}

static bool node_healthy(void)
{
    return !s_faulted;
}

static void send_can_frame(void)
{
    EncoderRPMFrame frame;
    frame.rpm        = s_rpm_filt;
    frame.angle_raw  = s_angle_raw;
    frame.error_flag = s_error_flag;
    frame.status     = (node_healthy() ? ENC_STATUS_HEALTHY : 0) | s_status_held | magnet_status()
                     | (uint8_t)((s_frame_seq & 0x07) << ENC_STATUS_SEQ_SHIFT);
    s_status_held    = 0;
    s_frame_seq++;

    CAN.beginPacket(CAN_ID_ENCODER_RPM);
    CAN.write((uint8_t *)&frame, sizeof(frame));
    if (!CAN.endPacket()) {
        DEBUG_PRINTLN("CAN tx failed");   // see check_can_bus_health() — this rarely fires in practice
    }
}

void setup()
{
    DEBUG_BEGIN(115200);

    pinMode(AS5047P_CS_PIN, OUTPUT);
    digitalWrite(AS5047P_CS_PIN, HIGH);
    SPI.begin();

    if (!CAN.begin(CAN_BAUD_BPS)) {
        DEBUG_PRINTLN("CAN init failed");
        while (1) { delay(1000); }
    }

    as5047p_clear_error();
    biquad_init_lowpass(s_rpm_lpf, RPM_FILTER_CUTOFF_HZ, SAMPLE_RATE_HZ);
    s_next_sample_us = micros();

    DEBUG_PRINT("AS5047P encoder + CAN RPM node started, NODE_ID=");
    DEBUG_PRINT(NODE_ID);
    DEBUG_PRINT(" CAN ID=0x");
    DEBUG_PRINTLN(CAN_ID_ENCODER_RPM, HEX);
}

void loop()
{
    uint32_t now_us = micros();

    // Fixed 2 kHz cadence (not "500 us since the last one"), so the filter's
    // assumed sample rate holds on average. If the loop falls more than a
    // whole period behind (endPacket() blocking), resync rather than
    // bursting catch-up samples.
    if ((int32_t)(now_us - s_next_sample_us) >= 0) {
        s_next_sample_us += SAMPLE_INTERVAL_US;
        if ((int32_t)(now_us - s_next_sample_us) >= 0) {
            s_next_sample_us = now_us + SAMPLE_INTERVAL_US;
        }

        const bool good = process_sample(now_us);
        if (++s_sample_count % DIAG_INTERVAL_SAMPLES == 0) {
            check_magnet();
        }
        update_health(good);

        // Send right after the sample it carries, so the frame's data age is
        // the same every frame.
        if (s_sample_count % SAMPLES_PER_CAN_FRAME == 0) {
            send_can_frame();
            check_can_bus_health();
        }
    }

    if ((uint32_t)(now_us - s_last_print_us) >= PRINT_INTERVAL_US) {
        s_last_print_us = now_us;
        float angle_deg = (float)s_angle_raw * (360.0f / 16384.0f);
        DEBUG_PRINT("angle_raw=");
        DEBUG_PRINT(s_angle_raw);
        DEBUG_PRINT("  angle_deg=");
        DEBUG_PRINT(angle_deg, 2);
        DEBUG_PRINT("  rpm=");
        DEBUG_PRINT(s_rpm_filt, 1);
        DEBUG_PRINT("  agc=");
        DEBUG_PRINT(s_diaagc & 0xFF);
        DEBUG_PRINT(node_healthy() ? "  OK" : "  UNHEALTHY");
        if (s_error_flag)                   { DEBUG_PRINT("  [EF]"); }
        if (s_diaagc & AS5047P_DIAAGC_MAGL) { DEBUG_PRINT("  [MAGL]"); }
        if (s_diaagc & AS5047P_DIAAGC_MAGH) { DEBUG_PRINT("  [MAGH]"); }
        if (s_diaagc & AS5047P_DIAAGC_COF)  { DEBUG_PRINT("  [COF]"); }
        DEBUG_PRINTLN();
    }
}
