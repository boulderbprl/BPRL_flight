#pragma once
#include "PID.hpp"
#include "configs/DroneConfig.hpp"

/*
 * Altitude controller — throttle passthrough and closed-loop altitude hold.
 *
 * compute_throttle(): manual throttle with expo shaping and tilt boost.
 *
 * track(): the single z-axis feedback loop — an altitude target plus a
 * climb-rate feed-forward → throttle. Two things feed it:
 *
 *   alt_hold()          the pilot's throttle stick (ALT_HOLD, POS_HOLD)
 *   TrajectoryTracker   a commanded altitude (POS_HOLD, see TrajectoryTracker.hpp)
 *
 *   alt_hold():
 *     stick (pilot_thr [0,1]) → stick_to_climb_rate() → climb_stick [m/s]
 *     alt_tgt += climb_stick * dt        (target frozen while stick is in the deadband)
 *     track(alt_tgt, climb_stick)
 *
 *   track():
 *     rate_tgt = climb_rate_ff + _pos_pid(alt_tgt − cur_D)
 *       rate error → _climb_rate_pid (target+error filtered @ 5 Hz) → delta_thr
 *       thr_cmd = constrain(hover_thr - delta_thr, 0, 1)
 *
 * hover_thr is per-drone (AltControlGains): the closer it is to the real
 * hover throttle, the less the climb-rate integrator has to make up after
 * take-off or a mode change. THR_MID below is only the manual throttle
 * curve's mid-stick value (compute_throttle(), STABILIZE).
 *
 * Same scheme as ArduPilot's set_pos_target_z_from_climb_rate_cm(): the
 * pilot's climb rate moves the altitude target and is fed forward onto the
 * position loop's output, so centring the stick holds the current altitude.
 *
 * The former inner acceleration loop (fed by the differentiated, noisy
 * body-frame accel estimate) has been removed: it drove throttle changes
 * fast enough to overheat the motors. The climb-rate PID now commands
 * throttle directly.
 *
 * Target/error filter cutoff matches ArduPilot's AC_PosControl default
 * (POSCONTROL_VEL_Z_FILT_HZ = 5 Hz).
 *
 * Conventions (NED, D positive down):
 *   cur_D    current D position [m]
 *   vD       current D velocity [m/s]               (positive = descending)
 *   rate_tgt climb rate target [m/s]                (positive = descending)
 */
class AltControl {
public:
    explicit AltControl(const AltControlGains &g);

    // Expo throttle with tilt boost; identical to the function removed from
    // AttitudePID/AttitudeINDI.
    float compute_throttle(float roll, float pitch, float thr_in) const;

    // Stick → altitude target + climb rate → throttle (ALT_HOLD and POS_HOLD).
    float alt_hold(float pilot_thr, float cur_D, float vD);

    // Altitude target [m, D] + climb-rate feed-forward [m/s, positive =
    // descending] → throttle. The target becomes alt_hold()'s own, so going
    // back to the stick carries on from the last altitude commanded here.
    float track(float alt_tgt_D, float climb_rate_ff, float cur_D, float vD);

    // Throttle stick [0,1] → climb rate [m/s], positive = descending; zero
    // inside the centre deadband.
    static float stick_to_climb_rate(float pilot_thr);

    // Altitude target [m, D] currently being held.
    float alt_tgt() const { return _alt_tgt_D; }

    // Climb-rate target [m/s, positive = descending] the last track() call
    // fed to the inner loop — for logging.
    float climb_rate_tgt() const { return _rate_tgt; }

    void reset_all();

private:
    PID _pos_pid;         // altitude error → climb-rate correction
    PID _climb_rate_pid;  // rate error → throttle delta

    float _hover_thr;     // from DroneConfig — see AltControlGains::hover_thr

    float _alt_tgt_D     = 0.0f;
    bool  _alt_tgt_valid = false;   // false → seed _alt_tgt_D from cur_D on the next call
    float _rate_tgt      = 0.0f;

    static constexpr float THR_MID        = 0.4f;
    static constexpr float MAX_CLIMB_RATE = 3.0f;   // m/s, limit on stick + altitude correction
    static constexpr float DEADBAND       = 0.05f;  // fraction of stick half-range
    static constexpr float ALT_LEASH_M    = 1.0f;   // m, max distance the altitude target may lead/lag the vehicle
};
