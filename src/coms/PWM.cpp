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

/*
 * Standard servo PWM — free-running 400 Hz refresh, 1000-2000 µs pulse width.
 *
 * Uses the same TIM1/TIM4 pins as DShot.cpp's Cube-board branch (see that
 * file's header comment for the full pin table and rotation rationale,
 * neither of which applies here — this is plain free-running output-compare
 * PWM, no DMA, no IC, no telemetry):
 *
 *   Motor 0 (FR) -> PE11 = TIM1_CH2, AF1
 *   Motor 1 (RL) -> PE9  = TIM1_CH1, AF1
 *   Motor 2 (FL) -> PD13 = TIM4_CH2, AF2
 *   Motor 3 (RR) -> PE13 = TIM1_CH3, AF1
 *
 * This driver owns TIM1/TIM4 outright in this build configuration —
 * dshot_init() is never called when MOTOR_PROTOCOL == MOTOR_PROTO_PWM, so
 * there's no register conflict with DShot.cpp despite the shared pins.
 *
 * Timer clock is 200 MHz (STM32H7 APB2/APB1 timer clock at this project's
 * 400 MHz core clock, PSC=0 -> 200 MHz, matching DShot.cpp's DS_ARR comment).
 * PSC=199 divides that to 1 MHz (1 tick = 1 µs), so CCR is directly the
 * pulse width in microseconds — no separate µs<->tick conversion needed.
 */

static constexpr uint32_t PWM_TIMER_HZ   = 1000000U;                       // 1 MHz -> 1 tick = 1 µs
static constexpr uint32_t PWM_PSC        = (200000000U / PWM_TIMER_HZ) - 1U; // 199
static constexpr uint32_t PWM_REFRESH_HZ = 400U;                           // matches ControlThread's loop rate
static constexpr uint32_t PWM_ARR        = (PWM_TIMER_HZ / PWM_REFRESH_HZ) - 1U; // 2499 @ 400 Hz
static constexpr uint32_t PWM_IDLE_US    = 1000U;                          // ESC idle pulse width

void motor_output_init(void)
{
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
    RCC->APB1LENR |= RCC_APB1LENR_TIM4EN;
    RCC->AHB4ENR  |= RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIODEN;

    /* PE9=TIM1_CH1, PE11=TIM1_CH2, PE13=TIM1_CH3 -> AF1, medium speed. */
    palSetPadMode(GPIOE, 9U,  PAL_MODE_ALTERNATE(1) | PAL_STM32_OSPEED_MID2);
    palSetPadMode(GPIOE, 11U, PAL_MODE_ALTERNATE(1) | PAL_STM32_OSPEED_MID2);
    palSetPadMode(GPIOE, 13U, PAL_MODE_ALTERNATE(1) | PAL_STM32_OSPEED_MID2);
    /* PD13=TIM4_CH2 -> AF2, medium speed. */
    palSetPadMode(GPIOD, 13U, PAL_MODE_ALTERNATE(2) | PAL_STM32_OSPEED_MID2);

    rccResetTIM1();
    TIM1->PSC   = PWM_PSC;
    TIM1->ARR   = PWM_ARR;
    TIM1->CR2   = 0U;
    /* PWM mode 1 (110), active-high (no CCxP), preload enabled. */
    TIM1->CCMR1 = STM32_TIM_CCMR1_OC1M(6) | TIM_CCMR1_OC1PE
                | STM32_TIM_CCMR1_OC2M(6) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = STM32_TIM_CCMR2_OC3M(6) | TIM_CCMR2_OC3PE;
    TIM1->CCR1  = PWM_IDLE_US;
    TIM1->CCR2  = PWM_IDLE_US;
    TIM1->CCR3  = PWM_IDLE_US;
    TIM1->CCER  = STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC2E | STM32_TIM_CCER_CC3E;
    TIM1->EGR   = TIM_EGR_UG;
    TIM1->SR    = 0U;
    TIM1->BDTR  = TIM_BDTR_MOE;   // TIM1 is an advanced-control timer — outputs stay disabled without MOE
    TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;

    rccResetTIM4();
    TIM4->PSC   = PWM_PSC;
    TIM4->ARR   = PWM_ARR;
    TIM4->CR2   = 0U;
    TIM4->CCMR1 = STM32_TIM_CCMR1_OC2M(6) | TIM_CCMR1_OC2PE;
    TIM4->CCR2  = PWM_IDLE_US;
    TIM4->CCER  = STM32_TIM_CCER_CC2E;
    TIM4->EGR   = TIM_EGR_UG;
    TIM4->SR    = 0U;
    TIM4->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;
}

// val=0 → 1000 µs (ESC idle); val 1–1000 → 1001–2000 µs (linear)
void motor_output_write(const int32_t val[4])
{
    uint32_t pwm_us[4];
    for (int i = 0; i < 4; i++) {
        const int32_t clamped = val[i] < 0 ? 0 : (val[i] > 1000 ? 1000 : val[i]);
        pwm_us[i] = PWM_IDLE_US + (uint32_t)clamped;
    }
    // [FR, RL, FL, RR] -> TIM1_CH2, TIM1_CH1, TIM4_CH2, TIM1_CH3 — same
    // motor/pin mapping as DShot.cpp's Cube-board branch (see header comment above).
    TIM1->CCR2 = pwm_us[0];  // M0 FR -> PE11
    TIM1->CCR1 = pwm_us[1];  // M1 RL -> PE9
    TIM4->CCR2 = pwm_us[2];  // M2 FL -> PD13
    TIM1->CCR3 = pwm_us[3];  // M3 RR -> PE13
}

#else
#error "Unknown MOTOR_PROTOCOL. Use MOTOR_PROTO_DSHOT or MOTOR_PROTO_PWM."
#endif
