#pragma once
#include "PID.hpp"

/*
 * Altitude controller — throttle passthrough and closed-loop altitude hold.
 *
 * compute_throttle(): manual throttle with expo shaping and tilt boost.
 *
 * alt_hold loop:
 *   stick (pilot_thr [0,1]) → stick_to_climb_rate() → rate_tgt [m/s]
 *       rate error → _climb_rate_pid (target+error filtered @ 5 Hz) → delta_thr
 *       thr_cmd = constrain(THR_MID - delta_thr, 0, 1)
 *
 * The former inner acceleration loop (fed by the differentiated, noisy
 * body-frame accel estimate) has been removed: it drove throttle changes
 * fast enough to overheat the motors. The climb-rate PID now commands
 * throttle directly.
 *
 * Target/error filter cutoff matches ArduPilot's AC_PosControl default
 * (POSCONTROL_VEL_Z_FILT_HZ = 5 Hz).
 *
 * Vertical axis uses body-frame W velocity (positive = descend), consistent
 * with the NED D convention used by PosControl (vel_tgt[2] > 0 = descend).
 *
 * Conventions:
 *   vD       current body-frame W velocity [m/s]   (positive = descending)
 *   rate_tgt climb rate target [m/s]                (positive = descending)
 */
class AltControl {
public:
    AltControl();

    // Expo throttle with tilt boost; identical to the function removed from
    // AttitudePID/AttitudeINDI.
    float compute_throttle(float roll, float pitch, float thr_in) const;

    // Full loop: stick → climb rate → throttle (ALT_HOLD mode).
    float alt_hold_from_stick(float pilot_thr, float vD);

    // Inner loop only: rate → throttle (POS_HOLD mode).
    float alt_hold_from_rate(float rate_tgt, float vD);

    void reset_all();

private:
    float stick_to_climb_rate(float pilot_thr) const;
    float run_loop(float rate_tgt, float vD);

    PID _climb_rate_pid;  // rate error → throttle delta

    static constexpr float THR_MID        = 0.4f;
    static constexpr float MAX_CLIMB_RATE = 3.0f;   // m/s
    static constexpr float DEADBAND       = 0.05f;  // fraction of stick half-range
};
