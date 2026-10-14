// Host-simulation stand-in for src/threads.hpp: only what the controller
// sources use from it. Keep CONTROL_PERIOD_US in step with the real header.
#pragma once
#include <cstdint>
#include "src/FlightState.hpp"

struct IMURaw    { float accel[3]; float gyro[3]; bool valid; };
struct CANIMURaw {
    float q0, q1, q2, q3; bool has_new_quat; uint32_t quat_timestamp_us;
    float p, q, r; float ax, ay, az; bool has_new_rates; uint32_t rates_timestamp_us;
    bool valid;
};
struct MocapRaw {
    float x, y, z; float vx, vy, vz; float yaw;
    bool has_new_pos, has_new_vel, has_new_yaw, valid;
};
struct BaroRaw { float pressure_pa, temperature_c, alt_m; bool has_new, valid; };

static constexpr uint32_t CONTROL_PERIOD_US = 2500;
static constexpr float    CONTROL_DT_S      = CONTROL_PERIOD_US * 1.0e-6f;
