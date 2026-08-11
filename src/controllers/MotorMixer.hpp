#pragma once
#include <cstdint>
#include "configs/DroneConfig.hpp"

/*
 * X-frame quadcopter motor mixer — ported from FreeRTOS/Tiva controllers.c.
 *
 * Layout (top view, ArduPilot X-frame numbering):
 *     FL [2]       FR [0]
 *          \       /
 *          [  body  ]
 *          /       \
 *     RL [1]       RR [3]
 *
 * All internal math (roll/pitch/yaw factors, the cmd[]/rp[] locals below) is
 * in that logical [FR, RL, FL, RR] order. out[] is physical DShot lane order
 * instead — cfg.motor_map[FR/RL/FL/RR] gives the physical lane each logical
 * corner is wired to, applied once at the very end of update(). See
 * MotorMixerConfig in configs/DroneConfig.hpp for the full rationale; if
 * your airframe's ESC wiring doesn't spin FR/RL/FL/RR in lane 0..3 order,
 * fix motor_map there rather than permuting the factor arrays here.
 *
 * Output range: 0 (disarmed/hardcoded) … 1000 (100% throttle), protocol-agnostic.
 * pwm_idle: armed idle spin at zero stick (base throttle). pwm_min: correction floor
 * (can be below idle). Safety: all motors → 0 when disarmed or |angle| > max_angle_rad.
 *
 * Geometry/scaling all come from MotorMixerConfig (per-drone) — see
 * configs/DroneConfig.hpp.
 */
class MotorMixer {
public:
    explicit MotorMixer(const MotorMixerConfig &cfg) : _cfg(cfg) {}

    void update(const float attitude_cmds[3], float thrust,
                bool armed, const float state[],
                int32_t out[4]) const;

    bool should_disarm(const float state[]) const;

private:
    MotorMixerConfig _cfg;
};
