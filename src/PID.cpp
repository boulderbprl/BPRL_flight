#include "PID.hpp"
#include "ch.h"
#include <cmath>
#include <algorithm>

// Inline helpers that mirror ArduPilot's AP_Math conventions.
// Replace with AP_Math versions if/when that library is ported in.
static inline float constrain_float(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

PID::PID(float kp, float ki, float kd, float imax, float fcut_hz)
    : _kp(kp), _ki(ki), _kd(kd)
    , _imax(imax), _fcut_hz(fcut_hz)
    , _integrator(0.0f), _last_error(0.0f), _last_derivative(0.0f)
    , _deriv_valid(false), _last_t_us(0)
{}

float PID::update(float error)
{
    const uint32_t now = now_us();

    if (!_deriv_valid) {
        _last_t_us   = now;
        _last_error  = error;
        _deriv_valid = true;
        return _kp * error;
    }

    const float dt = (float)(now - _last_t_us) * 1.0e-6f;

    // Stale input: reset and return P-only output
    if (dt > STALE_TIMEOUT_US * 1.0e-6f) {
        reset();
        _last_t_us   = now;
        _last_error  = error;
        _deriv_valid = true;
        return _kp * error;
    }

    _last_t_us = now;

    // Integrator with anti-windup
    _integrator += _ki * error * dt;
    _integrator  = constrain_float(_integrator, -_imax, _imax);

    // Derivative: 1st-order low-pass filter on raw (error difference / dt)
    float output = _kp * error + _integrator;
    if (dt > 0.0f) {
        const float raw_d = (error - _last_error) / dt;
        const float rc    = 1.0f / (2.0f * M_PI * _fcut_hz);
        const float alpha = dt / (dt + rc);
        _last_derivative += alpha * (raw_d - _last_derivative);
        output += _kd * _last_derivative;
    }

    _last_error = error;
    return output;
}

void PID::reset()
{
    _integrator      = 0.0f;
    _last_error      = 0.0f;
    _last_derivative = 0.0f;
    _deriv_valid     = false;
}

void PID::set_gains(float kp, float ki, float kd)
{
    _kp = kp; _ki = ki; _kd = kd;
}

// ChibiOS system time: 10 kHz tick → 100 µs resolution.
// TIME_I2US converts ticks to microseconds.
uint32_t PID::now_us()
{
    return (uint32_t)TIME_I2US(chVTGetSystemTimeX());
}
