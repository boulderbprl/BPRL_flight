#pragma once
#include "PID.hpp"

/*
 * Altitude controller — throttle passthrough and closed-loop altitude hold.
 *
 * compute_throttle(): manual throttle with expo shaping and tilt boost.
 *
 * alt_hold cascade:
 *   stick (pilot_thr [0,1]) → stick_to_climb_rate() → rate_tgt [m/s]
 *       rate error → _climb_rate_pid → accel_tgt [m/s²]
 *       accel error → _accel_pid    → delta_thr
 *       thr_cmd = constrain(THR_MID - delta_thr, 0, 1)
 *
 * Vertical axis uses body-frame W velocity (positive = descend), consistent
 * with the NED D convention used by PosControl (vel_tgt[2] > 0 = descend).
 *
 * Conventions:
 *   vD       current body-frame W velocity [m/s]   (positive = descending)
 *   vD_dot   body-frame W_DOT acceleration [m/s²]  (positive = accelerating down)
 *   rate_tgt climb rate target [m/s]                (positive = descending)
 */
class AltControl {
public:
    AltControl();

    // Expo throttle with tilt boost; identical to the function removed from
    // AttitudePID/AttitudeINDI.
    float compute_throttle(float roll, float pitch, float thr_in) const;

    // Full cascade: stick → climb rate → accel → throttle (ALT_HOLD mode).
    float alt_hold_from_stick(float pilot_thr, float vD, float vD_dot);

    // Inner two loops only: rate → accel → throttle (POS_HOLD mode).
    float alt_hold_from_rate(float rate_tgt, float vD, float vD_dot);

    void reset_all();

private:
    float stick_to_climb_rate(float pilot_thr) const;
    float run_cascade(float rate_tgt, float vD, float vD_dot);

    PID _climb_rate_pid;  // rate error   → accel target [m/s²]
    PID _accel_pid;       // accel error  → throttle delta

    static constexpr float THR_MID        = 0.4f;
    static constexpr float MAX_CLIMB_RATE = 3.0f;   // m/s
    static constexpr float DEADBAND       = 0.05f;  // fraction of stick half-range
};
