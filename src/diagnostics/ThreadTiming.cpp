#include "src/diagnostics/ThreadTiming.hpp"

#ifdef BPRL_TIMING

#include "chprintf.h"
#include "memstreams.h"

ThreadTimingStats g_timing_stats[TIMING_MAX_THREADS] = {};
int               g_timing_count = 0;

int timing_register(const char *name, sysinterval_t period_ticks)
{
    if (g_timing_count >= TIMING_MAX_THREADS) {
        return -1;  // out of slots — increase TIMING_MAX_THREADS
    }
    const int id = g_timing_count++;
    ThreadTimingStats &s = g_timing_stats[id];
    s.name          = name;
    s.period_ticks  = period_ticks;
    s.exec_min_us   = UINT32_MAX;
    s.exec_max_us   = 0;
    s.exec_sum_us   = 0;
    s.sample_count  = 0;
    s.miss_count    = 0;
    return id;
}

void timing_tick_begin(int id)
{
    if (id < 0) return;
    g_timing_stats[id].tick_start_us = (uint32_t)TIME_I2US(chVTGetSystemTimeX());
}

void timing_tick_end(int id)
{
    if (id < 0) return;
    ThreadTimingStats &s = g_timing_stats[id];
    const uint32_t now_us = (uint32_t)TIME_I2US(chVTGetSystemTimeX());
    const uint32_t exec_us = now_us - s.tick_start_us;  // wraps correctly (uint32 subtraction)

    if (exec_us < s.exec_min_us) s.exec_min_us = exec_us;
    if (exec_us > s.exec_max_us) s.exec_max_us = exec_us;
    s.exec_sum_us += exec_us;
    s.sample_count++;

    if (s.period_ticks != 0) {
        const uint32_t period_us = (uint32_t)TIME_I2US(s.period_ticks);
        if (exec_us > period_us) s.miss_count++;
    }
}

float timing_total_utilization_pct()
{
    float total = 0.0f;
    for (int i = 0; i < g_timing_count; i++) {
        const ThreadTimingStats &s = g_timing_stats[i];
        if (s.period_ticks == 0 || s.sample_count == 0) continue;
        const float avg_us   = (float)s.exec_sum_us / (float)s.sample_count;
        const float period_us = (float)TIME_I2US(s.period_ticks);
        if (period_us > 0.0f) total += 100.0f * avg_us / period_us;
    }
    return total;
}

void timing_reset()
{
    for (int i = 0; i < g_timing_count; i++) {
        ThreadTimingStats &s = g_timing_stats[i];
        s.exec_min_us  = UINT32_MAX;
        s.exec_max_us  = 0;
        s.exec_sum_us  = 0;
        s.sample_count = 0;
        s.miss_count   = 0;
    }
}

size_t timing_format_report(char *buf, size_t buflen)
{
    MemoryStream ms;
    msObjectInit(&ms, (uint8_t *)buf, buflen > 0 ? buflen - 1 : 0, 0);
    BaseSequentialStream *out = (BaseSequentialStream *)&ms;

    for (int i = 0; i < g_timing_count; i++) {
        const ThreadTimingStats &s = g_timing_stats[i];
        const float avg_us = s.sample_count
            ? (float)s.exec_sum_us / (float)s.sample_count : 0.0f;
        const uint32_t period_us = s.period_ticks ? (uint32_t)TIME_I2US(s.period_ticks) : 0;
        const float util_pct = (period_us > 0) ? (100.0f * avg_us / (float)period_us) : -1.0f;

        if (util_pct >= 0.0f) {
            chprintf(out,
                "$THD,%s,period_us=%lu,exec_avg_us=%lu,exec_max_us=%lu,util_pct=%.1f,misses=%lu,n=%lu\r\n",
                s.name, (unsigned long)period_us, (unsigned long)avg_us,
                (unsigned long)s.exec_max_us, (double)util_pct,
                (unsigned long)s.miss_count, (unsigned long)s.sample_count);
        } else {
            chprintf(out,
                "$THD,%s,period_us=event,exec_avg_us=%lu,exec_max_us=%lu,n=%lu\r\n",
                s.name, (unsigned long)avg_us, (unsigned long)s.exec_max_us,
                (unsigned long)s.sample_count);
        }
    }

    chprintf(out, "$CPU,util_pct=%.1f,n_threads=%d\r\n",
             (double)timing_total_utilization_pct(), g_timing_count);

    return ms.eos;
}

#endif // BPRL_TIMING
