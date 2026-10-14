#pragma once
#include "ch.h"
#include <cstdint>

/*
 * Monotonic time since boot.
 *
 * Use these instead of TIME_I2US()/TIME_I2MS() on chVTGetSystemTime[X]().
 * systime_t is only CH_CFG_ST_RESOLUTION bits wide, and the Orqa build
 * (configs/Drone3/config.mk) has to run it at 16 bits — at
 * CH_CFG_ST_FREQUENCY = 10 kHz that counter wraps every 6.5536 s, so any
 * absolute time derived straight from it jumps back to zero every 6.5 s.
 * chVTGetTimeStampI() extends the tick count to 64 bits inside the kernel
 * (it needs calling at least once per wrap; ControlThread alone does that
 * 400 times a second), so these never go backwards on any board.
 *
 * Safe from thread, locked or ISR context.
 */
static_assert(1000000U % CH_CFG_ST_FREQUENCY == 0U, "tick period must be a whole number of microseconds");

static inline uint64_t bprl_micros64()
{
    const syssts_t sts = chSysGetStatusAndLockX();
    const systimestamp_t ticks = chVTGetTimeStampI();
    chSysRestoreStatusX(sts);
    return (uint64_t)ticks * (1000000U / CH_CFG_ST_FREQUENCY);
}

// 32-bit views. Both wrap (µs after ~71 min, ms after ~49 days), but only at
// 2^32, so unsigned differences between two readings stay correct.
static inline uint32_t bprl_micros() { return (uint32_t)bprl_micros64(); }
static inline uint32_t bprl_millis() { return (uint32_t)(bprl_micros64() / 1000U); }
