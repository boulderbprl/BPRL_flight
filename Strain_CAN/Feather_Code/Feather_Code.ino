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
 *   [NODE_ID] (0x70 + NODE_ID). Wire the FC side with
 *   bprl_can_register(0x70 + n, my_cb, nullptr) per node — not done here,
 *   this sketch only covers the sensor node.
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

static const SPISettings AS5047P_SPI_SETTINGS(1000000, MSBFIRST, SPI_MODE1);

// ── CAN configuration ───────────────────────────────────────────────────
#define CAN_BAUD_BPS 1000000

// One CAN ID per node, indexed by NODE_ID — extend as more boards are added.
static const uint32_t CAN_ID_ENCODER_RPM_TABLE[] = { 0x70, 0x71 };
#define CAN_ID_ENCODER_RPM CAN_ID_ENCODER_RPM_TABLE[NODE_ID]

CANSAME5x CAN;

// ── Sample/print/tx pacing ──────────────────────────────────────────────
#define SAMPLE_INTERVAL_US  2000    // 500 Hz encoder sampling
#define PRINT_INTERVAL_US   50000   // 20 Hz USB debug print
#define CAN_INTERVAL_US     10000   // 100 Hz CAN tx

// RPM low-pass filter — one-pole EMA. Raw angle-derivative RPM is noisy at
// 500 Hz sampling; alpha trades lag for smoothness. Retune on the bench.
#define RPM_FILTER_ALPHA 0.3f

// Frame sent to the FC — 8 bytes to match the CAN.hpp convention
// (IMX5/StrainRate frames are all fixed 8-byte classical frames).
struct __attribute__((packed)) EncoderRPMFrame {
    float    rpm;         // filtered mechanical RPM, signed by rotation direction
    uint16_t angle_raw;   // last raw 14-bit shaft angle (0-16383 counts / rev)
    uint8_t  error_flag;  // AS5047P EF bit latched on the last sample
    uint8_t  reserved;
};

static uint32_t   s_last_sample_us = 0;
static uint32_t   s_last_print_us  = 0;
static uint32_t   s_last_can_us    = 0;
static uint16_t   s_prev_angle_raw = 0;
static bool       s_have_prev      = false;
static float      s_rpm_filt       = 0.0f;
static uint16_t   s_angle_raw      = 0;
static uint8_t     s_error_flag    = 0;

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
static uint16_t as5047p_read_register(uint16_t address, uint8_t *error_flag_out)
{
    uint16_t cmd = AS5047P_CMD_READ | (address & AS5047P_ADDR_MASK);
    cmd |= calc_even_parity_bit(cmd) << 15;

    as5047p_transfer16(cmd);
    uint16_t response = as5047p_transfer16(cmd);

    if (error_flag_out != nullptr) {
        *error_flag_out = (response >> 14) & 0x1;
    }
    return response & 0x3FFF;
}

static void as5047p_clear_error(void)
{
    uint8_t unused;
    as5047p_read_register(AS5047P_REG_ERRFL, &unused);
}


static void update_rpm(uint16_t angle_raw, uint32_t dt_us)
{
    if (!s_have_prev || dt_us == 0) {
        s_prev_angle_raw = angle_raw;
        s_have_prev = true;
        return;
    }

    int32_t delta = (int32_t)angle_raw - (int32_t)s_prev_angle_raw;
    if (delta > 8192)  delta -= 16384;   // wrapped forward through 0
    if (delta < -8192) delta += 16384;   // wrapped backward through 0

    float revs = (float)delta / 16384.0f;
    float rpm_inst = revs * (60.0f * 1000000.0f / (float)dt_us);

    s_rpm_filt += RPM_FILTER_ALPHA * (rpm_inst - s_rpm_filt);
    s_prev_angle_raw = angle_raw;
}

static void send_can_frame(void)
{
    EncoderRPMFrame frame;
    frame.rpm        = s_rpm_filt;
    frame.angle_raw  = s_angle_raw;
    frame.error_flag = s_error_flag;
    frame.reserved   = 0;

    CAN.beginPacket(CAN_ID_ENCODER_RPM);
    CAN.write((uint8_t *)&frame, sizeof(frame));
    if (!CAN.endPacket()) {
        DEBUG_PRINTLN("CAN tx failed");
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

    DEBUG_PRINT("AS5047P encoder + CAN RPM node started, NODE_ID=");
    DEBUG_PRINT(NODE_ID);
    DEBUG_PRINT(" CAN ID=0x");
    DEBUG_PRINTLN(CAN_ID_ENCODER_RPM, HEX);
}

void loop()
{
    uint32_t now_us = micros();

    if ((uint32_t)(now_us - s_last_sample_us) >= SAMPLE_INTERVAL_US) {
        uint32_t dt_us = now_us - s_last_sample_us;
        s_last_sample_us = now_us;

        s_angle_raw = as5047p_read_register(AS5047P_REG_ANGLECOM, &s_error_flag);
        if (s_error_flag) {
            as5047p_clear_error();
        }
        update_rpm(s_angle_raw, dt_us);
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
        if (s_error_flag) {
            DEBUG_PRINT("  [EF]");
        }
        DEBUG_PRINTLN();
    }

    if ((uint32_t)(now_us - s_last_can_us) >= CAN_INTERVAL_US) {
        s_last_can_us = now_us;
        send_can_frame();
    }
}
