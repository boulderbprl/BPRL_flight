/*
 * ThreadTiming.hpp — opt-in per-thread execution-time / CPU-utilization
 * instrumentation for schedulability testing.
 *
 * Disabled by default. Build with UDEFS_EXTRA=-DBPRL_TIMING to enable; every
 * macro below compiles to nothing (and every byte of ThreadTiming.cpp is
 * excluded) when the flag is absent, so a production build pays zero cost.
 *
 * What it measures, per registered thread:
 *   - execution time per tick (min/avg/max, microseconds) — wall-clock time
 *     from TIMING_TICK_BEGIN to TIMING_TICK_END, i.e. the thread's own work,
 *     excluding the sleep-until-next-period call.
 *   - deadline misses — ticks where exec time exceeded the thread's own
 *     nominal period (only meaningful for threads registered with a nonzero
 *     period; event-driven threads pass period_ticks=0 and are excluded from
 *     the miss/utilization accounting but still get exec-time stats).
 *   - CPU utilization — running average exec time / period, summed across
 *     all rate-tracked threads. This is the Liu & Layland U = sum(C_i/T_i)
 *     figure: if it's above ~0.69 for 9 rate-monotonic tasks (or clearly
 *     above 1.0 regardless of priority assignment), the task set is not
 *     guaranteed schedulable and per-thread deadline misses are the more
 *     reliable signal to look at.
 *
 * Usage inside a periodic thread's loop:
 *
 *   static int tid = TIMING_REGISTER("ctrl", period);   // once, before the loop
 *   while (true) {
 *       TIMING_TICK_BEGIN(tid);
 *       ... thread work ...
 *       TIMING_TICK_END(tid);
 *       next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
 *   }
 *
 * For event-driven threads (CAN RX, USB commands), register with
 * period_ticks=0 and wrap just the per-event work (not the blocking wait) —
 * exec-time stats are still collected but excluded from the utilization sum.
 *
 * Query at runtime over USB with "TIM,status" (see threads.cpp's
 * usb_cmd_dispatch), or read g_timing_stats[] directly.
 */
#pragma once
#include "ch.h"
#include "hal.h"
#include <cstddef>
#include <cstdint>

#ifdef BPRL_TIMING

constexpr int TIMING_MAX_THREADS = 12;

struct ThreadTimingStats {
    const char    *name;          // nullptr = unused slot
    sysinterval_t  period_ticks;  // 0 = event-driven, excluded from utilization sum
    bool           hard_rt;       // false = soft-deadline/I/O-bound (e.g. log) — excluded from timing_hrt_utilization_pct()
    uint32_t       exec_min_us;
    uint32_t       exec_max_us;
    uint64_t       exec_sum_us;   // divide by sample_count for the running average
    uint32_t       sample_count;
    uint32_t       miss_count;    // ticks where exec_us > period_us
    uint32_t       tick_start_us; // scratch, set by TICK_BEGIN
};

extern ThreadTimingStats g_timing_stats[TIMING_MAX_THREADS];
extern int               g_timing_count;

// Registers a thread once (call at thread startup, before its loop). Returns
// a small integer id used by TIMING_TICK_BEGIN/END. Not thread-safe against
// concurrent registration, but all registrations happen during
// threads_start()-adjacent startup code before the scheduler is under load.
// hard_rt=false marks a soft-deadline, I/O-bound thread (e.g. LogThread) whose
// worst-case execution time is inherently unbounded (SD-card wear-leveling
// stalls, etc.) rather than a CPU-scheduling property — excluded from
// timing_hrt_utilization_pct() so that figure reflects only threads a classic
// RM/Liu-Layland analysis actually applies to. Still included in
// timing_total_utilization_pct() and the per-thread report either way.
int timing_register(const char *name, sysinterval_t period_ticks, bool hard_rt = true);

void timing_tick_begin(int id);
void timing_tick_end(int id);

// Sums avg_exec_us/period_us over every rate-tracked (period_ticks != 0)
// registered thread. >100.0f means the periodic task set cannot possibly be
// schedulable; the Liu & Layland bound for n rate-monotonic tasks is
// n*(2^(1/n)-1)*100, e.g. ~69.3% for a large n — treat that as a soft
// warning line, not a hard cutoff, since priorities here aren't strictly
// rate-monotonic (see the priority-ordering note in threads_start()).
float timing_total_utilization_pct();

// Same as timing_total_utilization_pct() but skips every thread registered
// with hard_rt=false (see ThreadTimingStats::hard_rt above) — a truer figure
// for "is the hard-real-time part of this task set schedulable" when the
// task set also includes soft-deadline I/O-bound threads like LogThread.
float timing_hrt_utilization_pct();

// Resets all collected stats (min/max/avg/misses) without unregistering.
void timing_reset();

// Formats the full multi-line report ("$THD,..." per thread + "$CPU,..."
// totals) into buf. Returns the number of bytes written (excludes the null
// terminator). Safe to call from any thread; does not block.
size_t timing_format_report(char *buf, size_t buflen);

#define TIMING_REGISTER(name, period) timing_register((name), (period))
#define TIMING_REGISTER_SOFT(name, period) timing_register((name), (period), false)
#define TIMING_TICK_BEGIN(id) timing_tick_begin(id)
#define TIMING_TICK_END(id)   timing_tick_end(id)

#else // !BPRL_TIMING

#define TIMING_REGISTER(name, period) (0)
#define TIMING_REGISTER_SOFT(name, period) (0)
#define TIMING_TICK_BEGIN(id) ((void)(id))
#define TIMING_TICK_END(id)   ((void)(id))

#endif // BPRL_TIMING
