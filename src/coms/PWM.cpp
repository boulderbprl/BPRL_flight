#include "src/coms/PWM.hpp"

void motor_output_init(void)
{
    // TODO: pwmStart(&PWMD1, &pwm_cfg);
    // TODO: pwmEnableChannel(&PWMD1, 0..3, PWM_PERCENTAGE_TO_WIDTH(..., 0));
}

void motor_output_write(const int32_t pwm_us[4])
{
    // TODO: for (int i = 0; i < 4; i++)
    //           pwmEnableChannel(&PWMD1, i,
    //               PWM_FRACTION_TO_WIDTH(&PWMD1, 20000, (uint32_t)pwm_us[i]));
    (void)pwm_us;
}
