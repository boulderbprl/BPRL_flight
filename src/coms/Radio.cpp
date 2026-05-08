#include "src/coms/Radio.hpp"

static float _thr   = 0.0f;
static float _roll  = 0.0f;
static float _pitch = 0.0f;
static float _yaw   = 0.0f;
static bool  _armed = false;

void radio_input_init(void)
{
    // TODO: configure ICU (HAL_USE_ICU) for PWM capture on LINE_RC_INPUT,
    //       or configure USART6 for SBUS (inverted UART, 100000 baud, 8E2).
}

void radio_input_update(void)
{
    // TODO: read ICU-captured pulse widths and normalise:
    //   uint32_t thr_us = icu_lld_get_width(&ICUD8);
    //   _thr   = (thr_us  - 1000.0f) / 1000.0f;  // [0, 1]
    //   _roll  = (ch2_us  - 1500.0f) /  500.0f;  // [-1, 1]
    //   _pitch = (ch3_us  - 1500.0f) /  500.0f;
    //   _yaw   = (ch4_us  - 1500.0f) /  500.0f;
    //   _armed = (arm_us  > 1700);
}

float radio_thr(void)   { return _thr;   }
float radio_roll(void)  { return _roll;  }
float radio_pitch(void) { return _pitch; }
float radio_yaw(void)   { return _yaw;   }
bool  radio_armed(void) { return _armed; }
