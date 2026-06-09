#include "MotorMixer.hpp"
#include <cmath>

static inline float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void MotorMixer::update(const float cmds[3], float thrust,
                        bool armed, const float state[],
                        int32_t out[4]) const
{
    if (!armed || should_disarm(state)) {
        for (int i = 0; i < 4; i++) out[i] = 0;
        return;
    }

    const float thr = (float)PWM_IDLE + thrust * (float)(PWM_MAX - PWM_IDLE);
    const float r   = cmds[0] * ATT_SCALE;
    const float p   = cmds[1] * ATT_SCALE;
    const float y   = cmds[2] * YAW_SCALE;

    out[0] = (int32_t)clamp(thr - r + p + y, (float)PWM_MIN, (float)PWM_MAX); // FR
    out[1] = (int32_t)clamp(thr + r - p + y, (float)PWM_MIN, (float)PWM_MAX); // RL
    out[2] = (int32_t)clamp(thr + r + p - y, (float)PWM_MIN, (float)PWM_MAX); // FL
    out[3] = (int32_t)clamp(thr - r - p - y, (float)PWM_MIN, (float)PWM_MAX); // RR
}

bool MotorMixer::should_disarm(const float state[])
{
    return fabsf(state[0]) > MAX_ANGLE || fabsf(state[1]) > MAX_ANGLE;
}
