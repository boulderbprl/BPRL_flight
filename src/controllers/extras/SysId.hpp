#pragma once
#include <cstdint>
#include "configs/DroneConfig.hpp"

/*
 * Chirp — exponential sine sweep, ported from ArduPilot's AP_Math/chirp.cpp
 * (Leonard Hall, 2020). The frequency rises exponentially from f_start to
 * f_stop, so each decade gets the same time:
 *   w(t) = w_start · exp(B · (t - t_dwell) / (T - t_dwell)),  B = ln(w_stop / w_start)
 * It first dwells at f_start for t_dwell, and the amplitude is cosine-faded
 * in over fade_in and out over fade_out. t runs 0 … T (= record); the
 * output is 0 outside that and completed() turns true once t > T.
 */
class Chirp {
public:
    void init(float record_s, float f_start_hz, float f_stop_hz,
              float fade_in_s, float fade_out_s, float dwell_s);

    float update(float t, float magnitude);

    float frequency_hz() const;
    bool  completed() const { return _complete; }

private:
    float _record   = 0.0f;
    float _w_min    = 0.0f;   // rad/s
    float _w_max    = 0.0f;   // rad/s
    float _fade_in  = 0.0f;
    float _fade_out = 0.0f;
    float _dwell    = 0.0f;
    float _B        = 0.0f;   // ln(w_max / w_min)
    float _w        = 0.0f;   // current frequency, rad/s
    bool  _complete = false;
};

enum class SysIdState : uint8_t {
    IDLE,      // switch low, or waiting for the next rising edge
    RUNNING,   // chirp being injected
    DONE,      // sweep completed (axis advanced) — switch low to re-arm
    ABORTED,   // stopped because the vehicle stopped flying (axis NOT advanced) — switch low to re-arm
};

// One tick's sysid state for SIDD logging (see LogMsgSIDD).
struct SysIdDiag {
    SysIdState state;
    SysIdAxis  axis;      // axis being swept, or the one the next flip will sweep
    float      time_s;    // time into the current sweep
    float      freq_hz;   // instantaneous chirp frequency
    float      output;    // chirp value added this tick (0 unless RUNNING)
};

/*
 * SysId — RC-triggered system-identification sweeps, injected on top of the
 * flight controller's output right before MotorMixer (ArduPilot's
 * SID_AXIS = MIX_ROLL/PITCH/YAW/THROTTLE, but without taking over the
 * flight mode: the sticks and whichever controller is flying keep running
 * normally, and the chirp is just added to what they command).
 *
 * Owned by FlightStateMachine, which calls apply() at the very end of each
 * ACTIVE tick — after the spool-up ramp and landed detection, so a heave
 * sweep can't trip the landed check — and abort() on every disarmed /
 * ground-idle tick.
 *
 * Switch logic (SysIdConfig::rc_channel, via set_switch() every tick):
 *   rising edge   start a sweep on axes[_idx], if flying
 *   falling edge  cancel a running sweep; _idx unchanged, so the next flip
 *                 retries the same axis
 *   sweep done    stop injecting, _idx → next axis (wraps). Nothing more
 *                 happens until the switch goes low and back high.
 *   not flying    abort (same as cancel: _idx unchanged)
 *
 * Disabled entirely (set_switch() ignored, nothing injected) if rc_channel
 * is -1 or the config fails validate() — see the constructor.
 */
class SysId {
public:
    explicit SysId(const SysIdConfig &cfg);

    bool enabled() const { return _enabled; }

    // Raw switch level (radio_sysid()), every control tick.
    void set_switch(bool on);

    // Adds this tick's chirp to cmds[] (normalised torque) or thrust.
    // flying = ACTIVE and past the spool-up ramp; false aborts.
    void apply(bool flying, float cmds[3], float &thrust);

    // Vehicle not flying: stop any sweep and drop a pending start.
    void abort();

    SysIdDiag diag() const;

private:
    // Start-frequency dwell before the sweep, as ArduPilot: two full cycles.
    float dwell_s() const { return 2.0f / _cfg.f_start_hz; }
    bool  validate() const;

    SysIdConfig _cfg;
    Chirp       _chirp;
    bool        _enabled;
    int         _n_axes = 0;
    int         _idx    = 0;   // axes[] entry the current / next sweep uses
    bool        _sw            = false;
    bool        _start_pending = false;
    SysIdState  _state  = SysIdState::IDLE;
    float       _t      = 0.0f;
    float       _output = 0.0f;
};
