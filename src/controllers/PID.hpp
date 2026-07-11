#pragma once
#include <cstdint>

/*
 * Cascade PID controller — ported from FreeRTOS/Tiva PID.c.
 *
 * Filtering matches ArduPilot's AC_PID structure:
 *   target  --[1st-order LPF @ filt_target_hz]--> target_filt
 *   error   = target_filt - measurement
 *   error   --[1st-order LPF @ filt_error_hz]--> error_filt   (feeds P and I)
 *   derivative of error_filt --[1st-order LPF @ filt_d_hz]--> feeds D
 *
 * filt_target_hz / filt_error_hz default to 0 (disabled -> passthrough),
 * matching ArduPilot's FLTT/FLTE default-off behaviour.
 *
 * Other features:
 *   - Anti-windup integrator clamping (±imax)
 *   - Stale-input detection: resets integrator after 200 ms gap
 *   - First-sample derivative suppression
 */
class PID {
public:
    PID(float kp, float ki, float kd, float imax,
        float filt_target_hz = 0.0f, float filt_error_hz = 0.0f, float filt_d_hz = 20.0f);

    float update(float target, float measurement);
    void  reset();
    void  set_gains(float kp, float ki, float kd);

private:
    static constexpr float STALE_TIMEOUT_US = 200000.0f;

    float    _kp, _ki, _kd;
    float    _imax;
    float    _filt_target_hz, _filt_error_hz, _filt_d_hz;
    float    _integrator;
    float    _target_filt;
    float    _error_filt;
    float    _last_derivative;
    bool     _deriv_valid;
    uint32_t _last_t_us;

    static uint32_t now_us();
};
