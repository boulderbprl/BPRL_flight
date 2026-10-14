#include "src/coms/CalFlash.hpp"
#include "hal.h"
#include <cstring>

/*
 * STM32H743 Bank 2, sector 7 flash driver.
 *
 * Bank 2 registers (FLASH_TypeDef, stm32h743xx.h):
 *   FLASH->KEYR2  – unlock key register
 *   FLASH->CR2    – control (PG, SER, SNB, PSIZE, FW, START, LOCK)
 *   FLASH->SR2    – status  (BSY, QW, WBNE, errors)
 *   FLASH->CCR2   – clear status
 *
 * Programming unit: 256 bits = 32 bytes = 8 × 32-bit words.
 * CalibData is 128 bytes → 4 programming cycles.
 */

static constexpr uint32_t CAL_ADDR    = 0x081E0000UL;
static constexpr uint32_t CAL_MAGIC   = 0x42525943UL;   // 'BPRC'
static constexpr uint32_t CAL_VERSION = 1U;

/* ── CRC-32/ISO-HDLC (polynomial 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF) ── */
static uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc >> 1) ^ (0xEDB88320UL & -(crc & 1U));
    }
    return crc ^ 0xFFFFFFFFUL;
}

bool cal_load(CalibData &out)
{
    memset(&out, 0, sizeof(out));

    // The Cortex-M7 D-Cache is enabled at startup (crt1.c) and caches flash
    // reads like any other memory. Without invalidating here, a read that
    // happens to hit a line cached from an earlier cal_load() (e.g. the one
    // at boot) returns stale data even after cal_save()/cal_clear() just
    // erased and reprogrammed the underlying flash — reads would silently
    // see pre-write content instead of what was just written.
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t *>(CAL_ADDR),
                                  static_cast<int32_t>(sizeof(CalibData)));

    const CalibData *flash = reinterpret_cast<const CalibData *>(CAL_ADDR);
    if (flash->magic   != CAL_MAGIC)   return false;
    if (flash->version != CAL_VERSION) return false;

    // CRC covers bytes [0 .. sizeof(CalibData) - 4 - 44) = [0 .. 80)
    constexpr size_t crc_len = sizeof(CalibData) - sizeof(uint32_t) - 44;
    uint32_t computed = crc32(reinterpret_cast<const uint8_t *>(flash), crc_len);
    if (computed != flash->crc32) return false;

    memcpy(&out, flash, sizeof(out));
    return true;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Write path — flash erase/program, only reachable via CAL,set/commit/clear
 * over USB, which threads.cpp only compiles in under BPRL_DEBUG. Gated here
 * too (not just at the call site) so the interrupt-disabling critical
 * sections below don't exist in a flight build at all, rather than merely
 * being unreachable in one.
 * ════════════════════════════════════════════════════════════════════════════ */
#ifdef BPRL_DEBUG

// Key sequence to unlock Bank 2 control register
static constexpr uint32_t KEY1 = 0x45670123UL;
static constexpr uint32_t KEY2 = 0xCDEF89ABUL;

// Error bits in SR2 (STRBERR | PGSERR | INCERR | OPERR | RDPERR | RDSERR | SNECCERR | DBECCERR)
static constexpr uint32_t SR_ERR_MASK = 0x7EEA0000UL;

/* ── Wait for Bank 2 busy flags to clear ── */
static void b2_wait()
{
    while (FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE)) {}
}

/* ── Unlock Bank 2 ── */
static void b2_unlock()
{
    if (FLASH->CR2 & FLASH_CR_LOCK) {
        FLASH->KEYR2 = KEY1;
        FLASH->KEYR2 = KEY2;
    }
}

/* ── Lock Bank 2 ── */
static void b2_lock()
{
    FLASH->CR2 |= FLASH_CR_LOCK;
}

/* ── Clear Bank 2 error flags ── */
static void b2_clear_errors()
{
    FLASH->CCR2 = SR_ERR_MASK;
}

/* ── Erase Bank 2 sector 7 ──
 * Same interrupt-disabled-during-the-operation treatment as
 * b2_program_word() and for the same reason (ArduPilot's own H7 erase path
 * does the same, unconditionally by default). This does mean every other
 * thread — SPIThread, ControlThread, DShot's ISRs, everything — is fully
 * stalled for however long the 128KB erase actually takes (worst case on
 * the order of ~1s per the H7 reference manual). Acceptable here because
 * CAL,clear/CAL,commit are only ever issued from this ground-based
 * calibration tool with the vehicle disarmed and stationary — this must
 * not be reused anywhere that could run while armed/flying. */
static bool b2_erase_sector7()
{
    b2_wait();
    b2_clear_errors();

    chSysLock();
    // SER=1, SNB=7, PSIZE=word (0b10)
    FLASH->CR2 = FLASH_CR_SER | (7U << FLASH_CR_SNB_Pos) | FLASH_CR_PSIZE_1;
    FLASH->CR2 |= FLASH_CR_START;
    b2_wait();
    chSysUnlock();

    bool ok = !(FLASH->SR2 & SR_ERR_MASK);
    FLASH->CR2 = 0;
    return ok;
}

/* ── Program one 32-byte flash word at |addr| from |src| (8 × uint32_t) ──
 * Sequenced to match ArduPilot's own STM32H7 driver
 * (AP_HAL_ChibiOS/hwdef/common/flash.c: stm32h7_flash_write32), which is
 * flight-tested on real H7 boards — our original sequence (wait once, write
 * all 8 words back-to-back, no barrier, interrupts left enabled throughout)
 * reproducibly dropped individual 32-byte words on real hardware despite
 * SR2 reporting no error, different words each time depending on what else
 * was running. Three differences from that original, each taken from the
 * ArduPilot driver:
 *   1. Wait for BSY|QW before *every* word, not once before the block —
 *      the controller's write queue can't necessarily absorb 8 back-to-back
 *      stores without stalling.
 *   2. A __DSB() after the write loop — plain stores to a peripheral/flash
 *      address aren't guaranteed to have left the CPU's write buffer yet
 *      when the next instruction (checking BSY) executes; without a
 *      barrier, that check can pass before the writes have actually
 *      reached the flash controller.
 *   3. The whole write held inside a critical section (interrupts off) —
 *      DShot's timer ISRs fire far more often here than on the Cube boards
 *      (4 timers vs. 2) and at a higher NVIC priority than anything else in
 *      this driver; one landing mid-sequence is the leading suspect for the
 *      dropped words. ArduPilot's own driver disables interrupts around
 *      this exact call by default (STM32_FLASH_DISABLE_ISR defaults to 1).
 */
static bool b2_program_word(uint32_t addr, const uint32_t *src)
{
    b2_wait();
    b2_clear_errors();

    FLASH->CR2 = FLASH_CR_PG | FLASH_CR_PSIZE_1;

    volatile uint32_t *dst = reinterpret_cast<volatile uint32_t *>(addr);
    chSysLock();
    for (int i = 0; i < 8; ++i) {
        while (FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW)) {}
        dst[i] = src[i];
    }
    __DSB();
    chSysUnlock();

    b2_wait();

    bool ok = !(FLASH->SR2 & SR_ERR_MASK);
    FLASH->CR2 = 0;
    return ok;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Public API (write path)
 * ════════════════════════════════════════════════════════════════════════════ */

bool cal_save(const CalibData &d)
{
    // Build a mutable copy with magic, version, and CRC filled in
    CalibData buf;
    memcpy(&buf, &d, sizeof(buf));
    buf.magic   = CAL_MAGIC;
    buf.version = CAL_VERSION;
    memset(buf._pad, 0xFF, sizeof(buf._pad));

    constexpr size_t crc_len = sizeof(CalibData) - sizeof(uint32_t) - 44;
    buf.crc32 = crc32(reinterpret_cast<const uint8_t *>(&buf), crc_len);

    // SR2's error bits have proven unreliable as a completion signal on this
    // board — a word can pass the "no error" check yet never actually land
    // in flash (reproduced repeatedly, inconsistently, word-for-word
    // different each time). Rather than trust that status alone, read back
    // and compare after every attempt, and retry the whole erase+program
    // cycle from scratch if it doesn't match — only report success once the
    // stored bytes are actually verified.
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
        b2_unlock();
        ok = b2_erase_sector7();

        if (ok) {
            const uint32_t *src = reinterpret_cast<const uint32_t *>(&buf);
            for (int word = 0; word < (int)(sizeof(buf) / 32); ++word) {
                if (!b2_program_word(CAL_ADDR + static_cast<uint32_t>(word * 32), src + word * 8)) {
                    ok = false;
                    break;
                }
            }
        }
        b2_lock();

        if (ok) {
            SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t *>(CAL_ADDR),
                                          static_cast<int32_t>(sizeof(CalibData)));
            const void *flash = reinterpret_cast<const void *>(CAL_ADDR);
            ok = (memcmp(flash, &buf, sizeof(buf)) == 0);
        }
    }
    return ok;
}

void cal_clear()
{
    b2_unlock();
    b2_erase_sector7();
    b2_lock();
}

#endif // BPRL_DEBUG
