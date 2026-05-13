#include "src/coms/PWM.hpp"
#include "src/coms/DShot.hpp"

/*
 * Motor output via DShot600 bidirectional.
 * The interface (motor_output_init / motor_output_write) is unchanged;
 * only the implementation body is replaced as described in main.cpp.
 *
 * Throttle mapping: pwm_us 1000 µs → DShot 0 (disarm)
 *                   pwm_us 1001–1950 µs → DShot 48–2047 (linear)
 */

void motor_output_init(void)
{
    dshot_init();
}

void motor_output_write(const int32_t pwm_us[4])
{
    uint16_t throttle[4];
    for (int i = 0; i < 4; i++) {
        if (pwm_us[i] <= 1000) {
            throttle[i] = 0; // disarm
        } else {
            int32_t v    = pwm_us[i] > 1950 ? 1950 : pwm_us[i];
            throttle[i]  = (uint16_t)(48 + ((v - 1000) * (2047 - 48)) / 950);
        }
    }
    dshot_write(throttle);
}
