#include "PID.hpp"
#include "src/math/math.hpp"
#include "ch.h"

PID::PID(float kp, float ki, float kd, float imax,
         float filt_target_hz, float filt_error_hz, float filt_d_hz)
    : _kp(kp), _ki(ki), _kd(kd)
    , _imax(imax)
    , _filt_target_hz(filt_target_hz), _filt_error_hz(filt_error_hz), _filt_d_hz(filt_d_hz)
    , _integrator(0.0f), _target_filt(0.0f), _error_filt(0.0f)
    , _last_derivative(0.0f)
    , _deriv_valid(false), _last_t_us(0)
{}

float PID::update(float target, float measurement)
{
    const uint32_t now = now_us();

    if (!_deriv_valid) {
        _last_t_us   = now;
        _target_filt = target;
        _error_filt  = target - measurement;
        _deriv_valid = true;
        return _kp * _error_filt;
    }

    const float dt = (float)(now - _last_t_us) * 1.0e-6f;

    if (dt > STALE_TIMEOUT_US * 1.0e-6f) {
        reset();
        _last_t_us   = now;
        _target_filt = target;
        _error_filt  = target - measurement;
        _deriv_valid = true;
        return _kp * _error_filt;
    }

    _last_t_us = now;

    // Filter target, then compute error from the filtered target and filter
    // that (matches ArduPilot AC_PID: P and I both act on the filtered error).
    const float alpha_t = lowpass_alpha(_filt_target_hz, dt);
    _target_filt = lowpass(target, _target_filt, alpha_t);

    const float error_last = _error_filt;
    const float raw_error  = _target_filt - measurement;
    const float alpha_e    = lowpass_alpha(_filt_error_hz, dt);
    _error_filt = lowpass(raw_error, _error_filt, alpha_e);

    _integrator += _ki * _error_filt * dt;
    _integrator  = constrain_float(_integrator, -_imax, _imax);

    float output = _kp * _error_filt + _integrator;
    if (dt > 0.0f) {
        const float raw_d   = derivative(_error_filt, error_last, dt);
        const float alpha_d = lowpass_alpha(_filt_d_hz, dt);
        _last_derivative = lowpass(raw_d, _last_derivative, alpha_d);
        output += _kd * _last_derivative;
    }

    return output;
}

void PID::reset()
{
    _integrator      = 0.0f;
    _target_filt     = 0.0f;
    _error_filt      = 0.0f;
    _last_derivative = 0.0f;
    _deriv_valid     = false;
}

void PID::set_gains(float kp, float ki, float kd)
{
    _kp = kp; _ki = ki; _kd = kd;
}

uint32_t PID::now_us()
{
    return (uint32_t)TIME_I2US(chVTGetSystemTimeX());
}
