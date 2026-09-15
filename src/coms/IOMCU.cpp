#include "src/coms/IOMCU.hpp"
#include "src/coms/PWM.hpp"
#include <cstring>

// This entire file compiles to nothing unless MOTOR_PROTOCOL ==
// MOTOR_PROTO_IOMCU — mirrors DShot.cpp's own guard and for the same
// reason: SD6 (USART6) only exists as a real ChibiOS driver object when
// STM32_SERIAL_USE_USART6 is TRUE, which cfg/mcuconf.h only sets for
// BPRL_BOARD_CUBEBLUE (and BPRL_BOARD_ORQA, for an unrelated CRSF use of
// the same peripheral). Referencing SD6 unconditionally here fails to
// compile at all on boards where it's FALSE (a build error, not a silent
// runtime issue) — every caller of this file's public API (PWM.cpp,
// threads.cpp's IOMCUThread and "IOMCU,status" command) is already guarded
// by the same #if, so this is safe.
#if MOTOR_PROTOCOL == MOTOR_PROTO_IOMCU

// ── Wire format constants (see IOMCU.hpp for the full protocol writeup) ────
static constexpr uint8_t  CODE_READ        = 0;
static constexpr uint8_t  CODE_WRITE       = 1;
static constexpr uint8_t  CODE_SUCCESS     = 0;
static constexpr uint8_t  PKT_MAX_REGS     = 22;   // count is a 6-bit field

static constexpr uint8_t  PAGE_SETUP       = 50;
static constexpr uint8_t  PAGE_DIRECT_PWM  = 54;

static constexpr uint8_t  SETUP_ARMING            = 1;
static constexpr uint8_t  SETUP_DEFAULTRATE       = 3;
static constexpr uint8_t  SETUP_FORCE_SAFETY_OFF  = 12;
static constexpr uint8_t  SETUP_IGNORE_SAFETY     = 20;
static constexpr uint8_t  SETUP_CHANNEL_MASK      = 27;

static constexpr uint16_t FORCE_SAFETY_MAGIC          = 22027U;
static constexpr uint16_t ARMING_IO_ARM_OK            = 1U << 0;
static constexpr uint16_t ARMING_RC_HANDLING_DISABLED = 1U << 6;
static constexpr uint16_t CHANNEL_MASK_4CH            = 0x000FU;  // MAIN1-4
static constexpr uint16_t DEFAULT_RATE_HZ             = 400U;     // matches PWM.cpp's AUX rate

static constexpr sysinterval_t IOMCU_REPLY_TIMEOUT = TIME_MS2I(10);

static const SerialConfig iomcu_cfg = {
    1500000,                  // 1.5 Mbaud, fixed by the IO firmware's own protocol
    0,                        // 8 data bits, no parity
    USART_CR2_STOP1_BITS,
    0,
    nullptr, nullptr
};

MUTEX_DECL(iomcu_cmd_mtx);
IOMCUCmd g_iomcu_cmd = {};

// ── CRC-8, poly 0x07, init 0, no reflection, no final XOR ──────────────────
// crc = table[crc ^ byte] — standard table-driven form of the bit-shift
// algorithm (crc = (crc<<1) ^ (0x07 if crc&0x80 else 0), 8 times, per byte).
static const uint8_t kCrc8Table[256] = {
    0x00U, 0x07U, 0x0EU, 0x09U, 0x1CU, 0x1BU, 0x12U, 0x15U, 0x38U, 0x3FU, 0x36U, 0x31U,
    0x24U, 0x23U, 0x2AU, 0x2DU, 0x70U, 0x77U, 0x7EU, 0x79U, 0x6CU, 0x6BU, 0x62U, 0x65U,
    0x48U, 0x4FU, 0x46U, 0x41U, 0x54U, 0x53U, 0x5AU, 0x5DU, 0xE0U, 0xE7U, 0xEEU, 0xE9U,
    0xFCU, 0xFBU, 0xF2U, 0xF5U, 0xD8U, 0xDFU, 0xD6U, 0xD1U, 0xC4U, 0xC3U, 0xCAU, 0xCDU,
    0x90U, 0x97U, 0x9EU, 0x99U, 0x8CU, 0x8BU, 0x82U, 0x85U, 0xA8U, 0xAFU, 0xA6U, 0xA1U,
    0xB4U, 0xB3U, 0xBAU, 0xBDU, 0xC7U, 0xC0U, 0xC9U, 0xCEU, 0xDBU, 0xDCU, 0xD5U, 0xD2U,
    0xFFU, 0xF8U, 0xF1U, 0xF6U, 0xE3U, 0xE4U, 0xEDU, 0xEAU, 0xB7U, 0xB0U, 0xB9U, 0xBEU,
    0xABU, 0xACU, 0xA5U, 0xA2U, 0x8FU, 0x88U, 0x81U, 0x86U, 0x93U, 0x94U, 0x9DU, 0x9AU,
    0x27U, 0x20U, 0x29U, 0x2EU, 0x3BU, 0x3CU, 0x35U, 0x32U, 0x1FU, 0x18U, 0x11U, 0x16U,
    0x03U, 0x04U, 0x0DU, 0x0AU, 0x57U, 0x50U, 0x59U, 0x5EU, 0x4BU, 0x4CU, 0x45U, 0x42U,
    0x6FU, 0x68U, 0x61U, 0x66U, 0x73U, 0x74U, 0x7DU, 0x7AU, 0x89U, 0x8EU, 0x87U, 0x80U,
    0x95U, 0x92U, 0x9BU, 0x9CU, 0xB1U, 0xB6U, 0xBFU, 0xB8U, 0xADU, 0xAAU, 0xA3U, 0xA4U,
    0xF9U, 0xFEU, 0xF7U, 0xF0U, 0xE5U, 0xE2U, 0xEBU, 0xECU, 0xC1U, 0xC6U, 0xCFU, 0xC8U,
    0xDDU, 0xDAU, 0xD3U, 0xD4U, 0x69U, 0x6EU, 0x67U, 0x60U, 0x75U, 0x72U, 0x7BU, 0x7CU,
    0x51U, 0x56U, 0x5FU, 0x58U, 0x4DU, 0x4AU, 0x43U, 0x44U, 0x19U, 0x1EU, 0x17U, 0x10U,
    0x05U, 0x02U, 0x0BU, 0x0CU, 0x21U, 0x26U, 0x2FU, 0x28U, 0x3DU, 0x3AU, 0x33U, 0x34U,
    0x4EU, 0x49U, 0x40U, 0x47U, 0x52U, 0x55U, 0x5CU, 0x5BU, 0x76U, 0x71U, 0x78U, 0x7FU,
    0x6AU, 0x6DU, 0x64U, 0x63U, 0x3EU, 0x39U, 0x30U, 0x37U, 0x22U, 0x25U, 0x2CU, 0x2BU,
    0x06U, 0x01U, 0x08U, 0x0FU, 0x1AU, 0x1DU, 0x14U, 0x13U, 0xAEU, 0xA9U, 0xA0U, 0xA7U,
    0xB2U, 0xB5U, 0xBCU, 0xBBU, 0x96U, 0x91U, 0x98U, 0x9FU, 0x8AU, 0x8DU, 0x84U, 0x83U,
    0xDEU, 0xD9U, 0xD0U, 0xD7U, 0xC2U, 0xC5U, 0xCCU, 0xCBU, 0xE6U, 0xE1U, 0xE8U, 0xEFU,
    0xFAU, 0xFDU, 0xF4U, 0xF3U,
};

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) crc = kCrc8Table[crc ^ data[i]];
    return crc;
}

// Builds byte0..3 directly rather than a C bitfield struct — avoids relying
// on compiler-specific bitfield packing for byte0's count(6)|code(2) split.
static uint8_t s_txbuf[4 + PKT_MAX_REGS * 2];
static uint8_t s_rxbuf[4 + PKT_MAX_REGS * 2];

bool iomcu_write_registers(uint8_t page, uint8_t offset, uint8_t count, const uint16_t *regs)
{
    if (count == 0 || count > PKT_MAX_REGS) return false;

    const size_t len = (size_t)count * 2 + 4;
    s_txbuf[0] = (uint8_t)(count | (CODE_WRITE << 6));
    s_txbuf[1] = 0;  // CRC placeholder, filled below
    s_txbuf[2] = page;
    s_txbuf[3] = offset;
    for (uint8_t i = 0; i < count; i++) {
        s_txbuf[4 + i * 2]     = (uint8_t)(regs[i] & 0xFFU);
        s_txbuf[4 + i * 2 + 1] = (uint8_t)(regs[i] >> 8);
    }
    s_txbuf[1] = crc8(s_txbuf, len);

    while (chnReadTimeout(&SD6, s_rxbuf, 1, TIME_IMMEDIATE) == 1) { /* discard stale */ }

    chnWrite(&SD6, s_txbuf, len);

    // Reply to a write is always a fixed 4-byte ACK.
    size_t got = 0;
    while (got < 4) {
        size_t n = chnReadTimeout(&SD6, s_rxbuf + got, 4 - got, IOMCU_REPLY_TIMEOUT);
        if (n == 0) return false;  // timeout
        got += n;
    }
    if ((s_rxbuf[0] & 0x3FU) != 0) return false;             // ACK count must be 0
    if (((s_rxbuf[0] >> 6) & 0x3U) != CODE_SUCCESS) return false;
    const uint8_t rx_crc = s_rxbuf[1];
    s_rxbuf[1] = 0;
    return crc8(s_rxbuf, 4) == rx_crc;
}

bool iomcu_read_registers(uint8_t page, uint8_t offset, uint8_t count, uint16_t *regs)
{
    if (count == 0 || count > PKT_MAX_REGS) return false;

    // Compact 4-byte read request — count here means "registers wanted in
    // the reply", not payload present in this request. CRC covers just
    // these 4 bytes.
    s_txbuf[0] = (uint8_t)(count | (CODE_READ << 6));
    s_txbuf[1] = 0;
    s_txbuf[2] = page;
    s_txbuf[3] = offset;
    s_txbuf[1] = crc8(s_txbuf, 4);

    while (chnReadTimeout(&SD6, s_rxbuf, 1, TIME_IMMEDIATE) == 1) { /* discard stale */ }

    chnWrite(&SD6, s_txbuf, 4);

    const size_t want = (size_t)count * 2 + 4;
    size_t got = 0;
    while (got < want) {
        size_t n = chnReadTimeout(&SD6, s_rxbuf + got, want - got, IOMCU_REPLY_TIMEOUT);
        if (n == 0) return false;  // timeout
        got += n;
    }
    if ((s_rxbuf[0] & 0x3FU) < count) return false;
    if (((s_rxbuf[0] >> 6) & 0x3U) != CODE_SUCCESS) return false;
    const uint8_t rx_crc = s_rxbuf[1];
    s_rxbuf[1] = 0;
    if (crc8(s_rxbuf, want) != rx_crc) return false;

    for (uint8_t i = 0; i < count; i++) {
        regs[i] = (uint16_t)s_rxbuf[4 + i * 2] | ((uint16_t)s_rxbuf[4 + i * 2 + 1] << 8);
    }
    return true;
}

void iomcu_init(void)
{
    sdStart(&SD6, &iomcu_cfg);

    // One-time setup. Each of these can silently no-op if the IO isn't
    // responding yet right at boot — IOMCUThread's own loop re-sends PWM
    // continuously afterward, but these setup registers are NOT re-sent
    // periodically, only here. If the IO is slow to boot relative to the
    // FMU, a retry loop here would be more robust than this straight-line
    // sequence; not added yet since the IO is a fixed, always-present chip
    // that boots well before the FMU's own bring-up completes in practice.
    uint16_t reg;

    reg = CHANNEL_MASK_4CH;
    iomcu_write_registers(PAGE_SETUP, SETUP_CHANNEL_MASK, 1, &reg);

    reg = CHANNEL_MASK_4CH;
    iomcu_write_registers(PAGE_SETUP, SETUP_IGNORE_SAFETY, 1, &reg);

    reg = FORCE_SAFETY_MAGIC;
    iomcu_write_registers(PAGE_SETUP, SETUP_FORCE_SAFETY_OFF, 1, &reg);

    reg = DEFAULT_RATE_HZ;
    iomcu_write_registers(PAGE_SETUP, SETUP_DEFAULTRATE, 1, &reg);

    reg = ARMING_IO_ARM_OK | ARMING_RC_HANDLING_DISABLED;
    iomcu_write_registers(PAGE_SETUP, SETUP_ARMING, 1, &reg);
}

void iomcu_set_pwm(const uint16_t pwm_us[IOMCU_NUM_CHANNELS])
{
    chMtxLock(&iomcu_cmd_mtx);
    memcpy(g_iomcu_cmd.pwm_us, pwm_us, sizeof(g_iomcu_cmd.pwm_us));
    chMtxUnlock(&iomcu_cmd_mtx);
}

bool iomcu_send_now(void)
{
    uint16_t pwm_us[IOMCU_NUM_CHANNELS];
    chMtxLock(&iomcu_cmd_mtx);
    memcpy(pwm_us, g_iomcu_cmd.pwm_us, sizeof(pwm_us));
    chMtxUnlock(&iomcu_cmd_mtx);
    return iomcu_write_registers(PAGE_DIRECT_PWM, 0, IOMCU_NUM_CHANNELS, pwm_us);
}

#endif // MOTOR_PROTOCOL == MOTOR_PROTO_IOMCU
