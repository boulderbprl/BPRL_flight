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
 * PWM range: 1000 µs (idle) … 1950 µs (full throttle)
 * Safety: all motors → PWM_IDLE when disarmed or |angle| > ~80°
 */
class MotorMixer {
public:
    MotorMixer() = default;

    void update(const float attitude_cmds[3], float thrust,
                bool armed, const float state[],
                int32_t out_pwm[4]) const;

    static bool should_disarm(const float state[]);

    static constexpr int32_t PWM_IDLE = 1000;
    static constexpr int32_t PWM_MIN  = 1100;
    static constexpr int32_t PWM_MAX  = 1950;

private:
    static constexpr float ATT_SCALE = 300.0f;
    static constexpr float YAW_SCALE = 150.0f;
    static constexpr float MAX_ANGLE = 1.396f;  // ~80°
};
