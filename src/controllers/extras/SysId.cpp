#include "SysId.hpp"
#include "src/math/math.hpp"
#include "src/threads.hpp"   // CONTROL_DT_S
#include <cmath>

// ── Chirp ────────────────────────────────────────────────────────────────────

void Chirp::init(float record_s, float f_start_hz, float f_stop_hz,
                 float fade_in_s, float fade_out_s, float dwell_s)
{
    _record   = record_s;
    _w_min    = 2.0f * (float)M_PI * f_start_hz;
    _w_max    = 2.0f * (float)M_PI * f_stop_hz;
    _fade_in  = fade_in_s;
    _fade_out = fade_out_s;
    _dwell    = dwell_s;
    _B        = logf(_w_max / _w_min);
    _w        = _w_min;
    _complete = false;
}

float Chirp::update(float t, float magnitude)
{
    float window;
    if (t <= 0.0f) {
        window = 0.0f;
    } else if (t <= _fade_in) {
        window = 0.5f - 0.5f * cosf((float)M_PI * t / _fade_in);
    } else if (t <= _record - _fade_out) {
        window = 1.0f;
    } else if (t <= _record) {
        window = 0.5f - 0.5f * cosf((float)M_PI * (t - (_record - _fade_out)) / _fade_out + (float)M_PI);
    } else {
        window = 0.0f;
    }

    float out;
    if (t <= 0.0f) {
        _w  = _w_min;
        out = 0.0f;
    } else if (t <= _dwell) {
        // Phase chosen so it reaches 0 exactly where the sweep below starts.
        _w  = _w_min;
        out = window * magnitude * sinf(_w_min * (t - _dwell));
    } else if (t <= _record) {
        if (fabsf(_w_max - _w_min) < 1e-6f) {
            _w  = _w_min;
            out = window * magnitude * sinf(_w_min * t);
        } else {
            const float s = (t - _dwell) / (_record - _dwell);
            _w  = _w_min * expf(_B * s);
            out = window * magnitude * sinf((_w_min * (_record - _dwell) / _B) * (expf(_B * s) - 1.0f));
        }
    } else {
        _w  = _w_max;
        out = 0.0f;
    }

    _complete = t > _record;
    return out;
}

float Chirp::frequency_hz() const
{
    return _w / (2.0f * (float)M_PI);
}

// ── SysId ────────────────────────────────────────────────────────────────────

SysId::SysId(const SysIdConfig &cfg)
    : _cfg(cfg)
{
    while (_n_axes < 4 && _cfg.axes[_n_axes] != SysIdAxis::NONE) ++_n_axes;
    _enabled = _cfg.rc_channel >= 0 && validate();
    if (_enabled) {
        _chirp.init(_cfg.duration_s, _cfg.f_start_hz, _cfg.f_stop_hz,
                    _cfg.fade_in_s, _cfg.fade_out_s, dwell_s());
    }
}

// Same checks ArduPilot makes before a sweep (SystemID "Parameter Error"),
// plus the fades fitting inside the sweep.
bool SysId::validate() const
{
    return _n_axes > 0
        && _cfg.rc_channel < 16
        && _cfg.f_start_hz > 0.0f && _cfg.f_stop_hz > 0.0f
        && _cfg.fade_in_s >= 0.0f && _cfg.fade_out_s >= 0.0f
        && _cfg.duration_s > dwell_s()
        && _cfg.fade_in_s + _cfg.fade_out_s <= _cfg.duration_s;
}

void SysId::set_switch(bool on)
{
    if (!_enabled) return;

    if (on && !_sw) {
        // Rising edge: apply() starts it on the next flying tick.
        _start_pending = true;
    } else if (!on && _sw) {
        // Falling edge: cancel. _idx is untouched, so the next flip retries this axis.
        _start_pending = false;
        _state  = SysIdState::IDLE;
        _output = 0.0f;
    }
    _sw = on;
}

void SysId::abort()
{
    _start_pending = false;
    _output = 0.0f;
    if (_state == SysIdState::RUNNING) _state = SysIdState::ABORTED;
}

void SysId::apply(bool flying, float cmds[3], float &thrust)
{
    _output = 0.0f;
    if (!flying) {
        abort();
        return;
    }

    if (_start_pending) {
        _start_pending = false;
        _state = SysIdState::RUNNING;
        _t     = 0.0f;
        _chirp.init(_cfg.duration_s, _cfg.f_start_hz, _cfg.f_stop_hz,
                    _cfg.fade_in_s, _cfg.fade_out_s, dwell_s());
    }
    if (_state != SysIdState::RUNNING) return;

    const SysIdAxis axis = _cfg.axes[_idx];
    _t += CONTROL_DT_S;
    const float sample = _chirp.update(_t, _cfg.magnitude[(int)axis - 1]);

    if (_chirp.completed()) {
        _state = SysIdState::DONE;
        _idx   = (_idx + 1) % _n_axes;
        return;
    }

    _output = sample;
    switch (axis) {
        case SysIdAxis::ROLL:  cmds[0] = constrain_float(cmds[0] + sample, -1.0f, 1.0f); break;
        case SysIdAxis::PITCH: cmds[1] = constrain_float(cmds[1] + sample, -1.0f, 1.0f); break;
        case SysIdAxis::YAW:   cmds[2] = constrain_float(cmds[2] + sample, -1.0f, 1.0f); break;
        case SysIdAxis::HEAVE: thrust  = constrain_float(thrust  + sample,  0.0f, 1.0f); break;
        case SysIdAxis::NONE:  break;
    }
}

SysIdDiag SysId::diag() const
{
    SysIdDiag d;
    d.state   = _state;
    d.axis    = _n_axes > 0 ? _cfg.axes[_idx] : SysIdAxis::NONE;
    d.time_s  = (_state == SysIdState::RUNNING) ? _t : 0.0f;
    d.freq_hz = (_state == SysIdState::RUNNING) ? _chirp.frequency_hz() : 0.0f;
    d.output  = _output;
    return d;
}
