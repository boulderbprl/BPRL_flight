#include "src/coms/PWM.hpp"

#if MOTOR_PROTOCOL == MOTOR_PROTO_DSHOT
#include "src/coms/DShot.hpp"

void motor_output_init(void) { dshot_init(); }

// val=0 → DShot 0 (disarm); val 1–1000 → DShot 48–2047 (linear)
void motor_output_write(const int32_t val[4])
{
    uint16_t throttle[4];
    for (int i = 0; i < 4; i++) {
        if (val[i] <= 0) {
            throttle[i] = 0;
        } else {
            const int32_t clamped = val[i] > 1000 ? 1000 : val[i];
            throttle[i] = (uint16_t)(48U + (uint32_t)(clamped - 1) * 1999U / 999U);
        }
    }
    dshot_write(throttle);
}

#elif MOTOR_PROTOCOL == MOTOR_PROTO_PWM

void motor_output_init(void)
{
    // TODO: configure TIM1 PWM channels
}

// val=0 → 1000 µs (ESC idle); val 1–1000 → 1001–2000 µs (linear)
void motor_output_write(const int32_t val[4])
{
    for (int i = 0; i < 4; i++) {
        const int32_t clamped = val[i] < 0 ? 0 : (val[i] > 1000 ? 1000 : val[i]);
        const uint32_t pwm_us = 1000U + (uint32_t)clamped;
        (void)pwm_us; // TODO: write pwm_us to TIM1 CCR for channel i
    }
}

#else
#error "Unknown MOTOR_PROTOCOL. Use MOTOR_PROTO_DSHOT or MOTOR_PROTO_PWM."
#endif
