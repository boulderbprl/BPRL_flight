// Host-simulation stand-in for src/uptime.hpp: time is whatever the
// simulation loop says it is, so PID's dt is exactly one control period.
#pragma once
#include <cstdint>

extern uint64_t g_sim_time_us;

static inline uint64_t bprl_micros64() { return g_sim_time_us; }
static inline uint32_t bprl_micros()   { return (uint32_t)g_sim_time_us; }
static inline uint32_t bprl_millis()   { return (uint32_t)(g_sim_time_us / 1000U); }
