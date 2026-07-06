#pragma once
#include <cstdint>

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
 * ESC channels: ch0=FR, ch1=RL, ch2=FL, ch3=RR
 * Output range: 0 (disarmed/hardcoded) … 1000 (100% throttle), protocol-agnostic.
 * PWM_IDLE: armed idle spin at zero stick (base throttle). PWM_MIN: correction floor
 * (can be below IDLE). Safety: all motors → 0 when disarmed or |angle| > ~80°
 */
class MotorMixer {
public:
    MotorMixer() = default;

    void update(const float attitude_cmds[3], float thrust,
                bool armed, const float state[],
                int32_t out[4]) const;

    static bool should_disarm(const float state[]);

    static constexpr int32_t PWM_MIN  = 50;   // minimum during attitude corrections
    static constexpr int32_t PWM_IDLE = 150;  // armed idle spin (zero stick, level)
    static constexpr int32_t PWM_MAX  = 900;  // maximum throttle ceiling

private:
    static constexpr float ATT_SCALE = 350.0f;
    static constexpr float YAW_SCALE = 250.0f;
    static constexpr float MAX_ANGLE = 1.396f;  // ~80°
    static constexpr float YAW_HEADROOM_MIN = 0.18f;  // min fraction of PWM span reserved for yaw
};
