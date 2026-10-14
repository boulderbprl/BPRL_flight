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
 * REWRITTEN to use ChibiOS's own PWMDriver (PWMD1/PWMD4) instead of hand-
 * rolled TIM1/TIM4 register writes. The hand-rolled version (register
 * values checked character-by-character against DShot.cpp's proven-working
 * TIM1/TIM4 setup, matching everywhere except PSC/ARR/CCxP — none of which
 * should explain total signal loss) left all 3 TIM1 channels dead on real
 * hardware while the 1 TIM4 channel worked, with no register-level bug
 * ever found. Rather than keep guessing at raw registers, this now matches
 * ArduPilot's own approach: AP_HAL_ChibiOS::RCOutput never hand-rolls TIM1/
 * TIM4 for plain PWM either — every mode (PWM, OneShot, DShot) goes through
 * pwmStart()/PWMConfig, the actual tested ChibiOS driver, which the earlier
 * version of this file reimplemented from scratch instead of using.
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
 * there's no conflict with DShot.cpp's raw register access despite the
 * shared pins (DShot.cpp never calls pwmStart()/pwmStop() on PWMD1/PWMD4).
 *
 * Polarity: PWM_OUTPUT_ACTIVE_HIGH on every channel. An earlier version of
 * this file used active-low (CCxP), reasoning from DShot.cpp's use of it on
 * these same channels that this board's AUX stage inverts the signal in
 * hardware — wrong, and it didn't fix anything when tried: cross-checked
 * against ArduPilot's own RCOutput.cpp, DShot there sets
 * `active_high = is_bidir_dshot_enabled(group) ? false : true`, and plain
 * PWM's set_freq_group() unconditionally forces any ACTIVE_LOW channel back
 * to ACTIVE_HIGH before starting it. Active-low there is a bidirectional-
 * DShot protocol choice, not evidence of hardware inversion.
 */

static constexpr uint32_t PWM_TIMER_HZ   = 1000000U;   // 1 MHz -> 1 tick = 1 µs
static constexpr uint32_t PWM_REFRESH_HZ = 400U;       // matches ControlThread's loop rate
static constexpr pwmcnt_t PWM_PERIOD_TICKS = PWM_TIMER_HZ / PWM_REFRESH_HZ; // 2500 @ 400 Hz
static constexpr uint32_t PWM_IDLE_US    = 1000U;      // ESC idle pulse width

// PWMChannelConfig arrays must be sized PWM_CHANNELS (6 on H7, even though
// TIM1/TIM4 only have 4 real OC channels) — pwm_lld_start() only ever reads
// indices 0-3, so 4/5 are unused padding, never touched.
static PWMConfig pwm1_cfg = {
    PWM_TIMER_HZ,
    PWM_PERIOD_TICKS,
    nullptr,
    {
        {PWM_OUTPUT_ACTIVE_HIGH, nullptr},  // CH1 = PE9  = M1 RL
        {PWM_OUTPUT_ACTIVE_HIGH, nullptr},  // CH2 = PE11 = M0 FR
        {PWM_OUTPUT_ACTIVE_HIGH, nullptr},  // CH3 = PE13 = M3 RR
        {PWM_OUTPUT_DISABLED,    nullptr},  // CH4 unused
        {PWM_OUTPUT_DISABLED,    nullptr},
        {PWM_OUTPUT_DISABLED,    nullptr},
    },
    0, 0, 0
};

static PWMConfig pwm4_cfg = {
    PWM_TIMER_HZ,
    PWM_PERIOD_TICKS,
    nullptr,
    {
        {PWM_OUTPUT_DISABLED,    nullptr},  // CH1 unused
        {PWM_OUTPUT_ACTIVE_HIGH, nullptr},  // CH2 = PD13 = M2 FL
        {PWM_OUTPUT_DISABLED,    nullptr},
        {PWM_OUTPUT_DISABLED,    nullptr},
        {PWM_OUTPUT_DISABLED,    nullptr},
        {PWM_OUTPUT_DISABLED,    nullptr},
    },
    0, 0, 0
};

void motor_output_init(void)
{
    /* PE9=TIM1_CH1, PE11=TIM1_CH2, PE13=TIM1_CH3 -> AF1, medium speed. */
    palSetPadMode(GPIOE, 9U,  PAL_MODE_ALTERNATE(1) | PAL_STM32_OSPEED_MID2);
    palSetPadMode(GPIOE, 11U, PAL_MODE_ALTERNATE(1) | PAL_STM32_OSPEED_MID2);
    palSetPadMode(GPIOE, 13U, PAL_MODE_ALTERNATE(1) | PAL_STM32_OSPEED_MID2);
    /* PD13=TIM4_CH2 -> AF2, medium speed. */
    palSetPadMode(GPIOD, 13U, PAL_MODE_ALTERNATE(2) | PAL_STM32_OSPEED_MID2);

    // pwmStart() handles clock enable, timer reset, NVIC vectors, and (per
    // pwm_lld_start()) unconditionally ORs in BDTR_MOE for advanced timers
    // — no manual RCC/BDTR/EGR/CR1 bring-up needed.
    pwmStart(&PWMD1, &pwm1_cfg);
    pwmStart(&PWMD4, &pwm4_cfg);

    pwmEnableChannel(&PWMD1, 0, PWM_IDLE_US);  // CH1 PE9  (M1 RL)
    pwmEnableChannel(&PWMD1, 1, PWM_IDLE_US);  // CH2 PE11 (M0 FR)
    pwmEnableChannel(&PWMD1, 2, PWM_IDLE_US);  // CH3 PE13 (M3 RR)
    pwmEnableChannel(&PWMD4, 1, PWM_IDLE_US);  // CH2 PD13 (M2 FL)
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
    pwmEnableChannel(&PWMD1, 1, pwm_us[0]);  // M0 FR -> PE11 (CH2)
    pwmEnableChannel(&PWMD1, 0, pwm_us[1]);  // M1 RL -> PE9  (CH1)
    pwmEnableChannel(&PWMD4, 1, pwm_us[2]);  // M2 FL -> PD13 (CH2)
    pwmEnableChannel(&PWMD1, 2, pwm_us[3]);  // M3 RR -> PE13 (CH3)
}

#elif MOTOR_PROTOCOL == MOTOR_PROTO_IOMCU
#include "src/coms/IOMCU.hpp"

// Standard servo PWM via the carrier board's IO co-processor (MAIN 1-4),
// for boards where the FMU's own AUX outputs are unreliable — see
// IOMCU.hpp for the full protocol writeup and why this only publishes to
// a shared struct here rather than touching the UART directly (this
// function is called from ControlThread's 400 Hz tick and must never
// block on a UART round-trip; IOMCUThread in threads.cpp owns the actual
// transaction).

void motor_output_init(void) { iomcu_init(); }

// val=0 → 1000 µs (ESC idle); val 1–1000 → 1001–2000 µs (linear)
void motor_output_write(const int32_t val[4])
{
    uint16_t pwm_us[4];
    for (int i = 0; i < 4; i++) {
        const int32_t clamped = val[i] < 0 ? 0 : (val[i] > 1000 ? 1000 : val[i]);
        pwm_us[i] = (uint16_t)(1000U + (uint32_t)clamped);
    }
    // val[i] -> MAIN(i+1) — a fresh assignment (no pre-existing physical-
    // lane convention for MAIN the way AUX has); see IOMCU.hpp.
    iomcu_set_pwm(pwm_us);
}

#else
#error "Unknown MOTOR_PROTOCOL. Use MOTOR_PROTO_DSHOT, MOTOR_PROTO_PWM, or MOTOR_PROTO_IOMCU."
#endif
