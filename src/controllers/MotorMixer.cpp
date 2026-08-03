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
    const float *roll_factor  = _cfg.roll_factor;
    const float *pitch_factor = _cfg.pitch_factor;
    const float *yaw_factor   = _cfg.yaw_factor;
    const float pwm_min  = (float)_cfg.pwm_min;
    const float pwm_idle = (float)_cfg.pwm_idle;
    const float pwm_max  = (float)_cfg.pwm_max;

    const float span = pwm_max - pwm_min;
    float thr     = pwm_idle + thrust * (pwm_max - pwm_idle);
    const float r = cmds[0] * _cfg.att_scale;
    const float p = cmds[1] * _cfg.att_scale;
    float y       = cmds[2] * _cfg.yaw_scale;

    // Step 1: roll+pitch only, per motor.
    float rp[4];
    for (int i = 0; i < 4; i++)
        rp[i] = roll_factor[i] * r + pitch_factor[i] * p;

    // Step 2: how much yaw headroom each motor has left after roll+pitch,
    // then guarantee yaw a minimum share of the output span regardless of
    // how much roll+pitch demand there is (mirrors ArduPilot).
    float yaw_allowed = _cfg.yaw_scale;
    for (int i = 0; i < 4; i++) {
        const float base = thr + rp[i];
        const float room = (y * yaw_factor[i] >= 0.0f)
            ? (pwm_max - base)
            : (base - pwm_min);
        yaw_allowed = fminf(yaw_allowed, fmaxf(room, 0.0f) / fabsf(yaw_factor[i]));
    }
    yaw_allowed = fmaxf(yaw_allowed, _cfg.yaw_headroom_min * span);
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
    thr = clamp(thr, pwm_min - lo, pwm_max - hi);

    for (int i = 0; i < 4; i++)
        out[i] = (int32_t)clamp(thr + cmd[i], pwm_min, pwm_max);
}

bool MotorMixer::should_disarm(const float state[]) const
{
    return fabsf(state[0]) > _cfg.max_angle_rad || fabsf(state[1]) > _cfg.max_angle_rad;
}
