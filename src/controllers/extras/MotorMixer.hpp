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
 *
 * Thrust linearization (cfg.thrust_expo > 0): mixing happens in thrust
 * space, not PWM space, so att_scale/yaw_scale give the same torque at any
 * throttle. Same curve as ArduPilot's MOT_THST_EXPO — with the actuator a
 * and thrust T both normalised to [0, 1] over [pwm_min, pwm_max]:
 *   T = (1 - e) * a + e * a^2
 * The throttle maps to thrust and back to the identical PWM, so throttle
 * feel and hover_thr are unchanged. Only the attitude terms are linearised.
 * att_scale/yaw_scale are now "PWM at unit thrust slope" (a = 0.5 for e > 0).
 * thrust_expo = 0 is the old linear-PWM mixer, bit-for-bit.
 *
 * limits() reports which axes this tick's commands could not be fully
 * delivered on (ArduPilot's motors.limit.roll/pitch/yaw). AttitudePID
 * uses it next tick to stop its rate integrators winding up into
 * saturation. All axes read as limited while disarmed, as in ArduPilot
 * when the motors aren't spooled up.
 */
struct MixerLimits {
    bool roll  = true;
    bool pitch = true;
    bool yaw   = true;
};

class MotorMixer {
public:
    explicit MotorMixer(const MotorMixerConfig &cfg) : _cfg(cfg) {}

    void update(const float attitude_cmds[3], float thrust,
                bool armed, const float state[],
                int32_t out[4]);

    bool should_disarm(const float state[]) const;

    const MixerLimits &limits() const { return _limits; }

private:
    // PWM <-> thrust-equivalent PWM on [pwm_min, pwm_max]; identity when thrust_expo <= 0.
    float pwm_to_thrust(float pwm) const;
    float thrust_to_pwm(float thrust_pwm) const;

    MotorMixerConfig _cfg;
    MixerLimits      _limits;
};
