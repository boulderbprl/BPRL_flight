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

    // Motor order: FR, RL, FL, RR (matches out[] / hardware pinout).
    static constexpr float roll_factor[4]  = { -1.0f, +1.0f, +1.0f, -1.0f };
    static constexpr float pitch_factor[4] = { +1.0f, -1.0f, +1.0f, -1.0f };
    static constexpr float yaw_factor[4]   = { +1.0f, +1.0f, -1.0f, -1.0f };

    const float span = (float)(PWM_MAX - PWM_MIN);
    float thr     = (float)PWM_IDLE + thrust * (float)(PWM_MAX - PWM_IDLE);
    const float r = cmds[0] * ATT_SCALE;
    const float p = cmds[1] * ATT_SCALE;
    float y       = cmds[2] * YAW_SCALE;

    // Step 1: roll+pitch only, per motor.
    float rp[4];
    for (int i = 0; i < 4; i++)
        rp[i] = roll_factor[i] * r + pitch_factor[i] * p;

    // Step 2: how much yaw headroom each motor has left after roll+pitch,
    // then guarantee yaw a minimum share of the output span regardless of
    // how much roll+pitch demand there is (mirrors ArduPilot's MOT_YAW_HEADROOM).
    float yaw_allowed = YAW_SCALE;
    for (int i = 0; i < 4; i++) {
        const float base = thr + rp[i];
        const float room = (y * yaw_factor[i] >= 0.0f)
            ? ((float)PWM_MAX - base)
            : (base - (float)PWM_MIN);
        yaw_allowed = fminf(yaw_allowed, fmaxf(room, 0.0f) / fabsf(yaw_factor[i]));
    }
    yaw_allowed = fmaxf(yaw_allowed, YAW_HEADROOM_MIN * span);
    y = clamp(y, -yaw_allowed, yaw_allowed);

    // Step 3: combine roll+pitch+yaw; if the combined spread still can't fit
    // in the output span, scale rp+yaw down together (never yaw alone) and
    // let throttle absorb whatever headroom is left.
    float cmd[4], lo = 0.0f, hi = 0.0f;
    for (int i = 0; i < 4; i++) {
        cmd[i] = rp[i] + yaw_factor[i] * y;
        lo = fminf(lo, cmd[i]);
        hi = fmaxf(hi, cmd[i]);
    }
    if (hi - lo > span) {
        const float scale = span / (hi - lo);
        for (int i = 0; i < 4; i++) cmd[i] *= scale;
        lo *= scale; hi *= scale;
    }
    thr = clamp(thr, (float)PWM_MIN - lo, (float)PWM_MAX - hi);

    for (int i = 0; i < 4; i++)
        out[i] = (int32_t)clamp(thr + cmd[i], (float)PWM_MIN, (float)PWM_MAX);
}

bool MotorMixer::should_disarm(const float state[])
{
    return fabsf(state[0]) > MAX_ANGLE || fabsf(state[1]) > MAX_ANGLE;
}
