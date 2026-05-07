#pragma once
#include <cstdint>

/*
 * Cascade PID controller - ported from FreeRTOS/Tiva PID.c
 *
 * Features:
 *   - Low-pass filtered derivative (1st-order, default cutoff 30 Hz)
 *   - Anti-windup integrator clamping (±imax)
 *   - Stale-input detection: resets integrator after a 200 ms gap
 *   - First-sample derivative suppression (no spike on first call)
 *
 * Timing: ChibiOS system tick (10 kHz → 100 µs resolution).
 * Compatible with ArduPilot's AP_Math conventions (radians, rad/s).
 */
class PID {
public:
    // fcut_hz: derivative low-pass filter cutoff (Hz)
    PID(float kp, float ki, float kd, float imax, float fcut_hz = 30.0f);

    // Compute PID output for the given error sample.
    float update(float error);

    // Reset integrator and derivative state.
    void reset();

    void set_gains(float kp, float ki, float kd);

private:
    static constexpr float STALE_TIMEOUT_US = 200000.0f; // 200 ms

    float    _kp, _ki, _kd;
    float    _imax;
    float    _fcut_hz;
    float    _integrator;
    float    _last_error;
    float    _last_derivative;
    bool     _deriv_valid;
    uint32_t _last_t_us;

    static uint32_t now_us();
};
