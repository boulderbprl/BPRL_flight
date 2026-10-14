#pragma once
#include "configs/DroneConfig.hpp"

/*
 * Heading controller — heading reference → yaw-rate command for the
 * attitude controller's yaw-rate loop. Used in POS_HOLD only; STABILIZE and
 * ALT_HOLD keep each attitude controller's own rate + heading-lock trim.
 *
 * It owns a shaped heading target that is always kinematically followable:
 * the commanded rate (_rate_tgt) is slew-limited to max_accel. The target
 * the feedback acts on is then integrated not from that rate but from a
 * prediction of what the vehicle will actually do with it (_rate_pred:
 * _rate_tgt through a first-order lag, lag_s, matching the yaw-rate loop's
 * response). Without that, the target runs ahead of a vehicle that always
 * responds lag_s late; the feedback closes the gap during the turn, and when
 * the turn stops the vehicle's late response carries it past the target —
 * the 6-12° swing-back seen in flight. Same idea as PosControl's _accel_pred.
 *
 * Two ways to drive it:
 *
 *   update_rate()     pilot yaw stick: the target turns at the demanded rate.
 *   update_heading()  trajectory: the target turns toward yaw_des (itself
 *                     moving at rate_ff), approaching at up to max_rate.
 *
 * Output (both):
 *   yaw_rate_cmd = _rate_tgt + constrain(kp · wrap(target − yaw), ±max_rate)
 *
 * so the rate loop gets the commanded rate (un-lagged) as feed-forward and
 * the heading error only has to trim. Same structure as PosControl's
 * vel_des + pos_P(pos_des − pos).
 *
 * All angles rad, rates rad/s, NED (positive = nose right / clockwise from
 * above).
 */
class HeadingControl {
public:
    explicit HeadingControl(const HeadingGains &g) : _g(g) {}

    float update_rate(float yaw_now, float rate_req);
    float update_heading(float yaw_now, float yaw_des, float rate_ff);

    void reset() { _valid = false; }

    // The estimator's yaw just stepped by delta_rad with no physical
    // rotation — see AttitudeController::yaw_frame_reset().
    void yaw_frame_reset(float delta_rad);

    // For logging.
    float target()   const { return _target; }
    float rate_cmd() const { return _rate_cmd; }

private:
    float step(float yaw_now, float rate_des);

    HeadingGains _g;

    bool  _valid      = false;  // false → seed the targets from the measured yaw on the next update
    float _target_cmd = 0.0f;   // where the commanded rate alone would have the heading [rad] — what update_heading() steers toward yaw_des
    float _target     = 0.0f;   // where the vehicle is expected to be [rad] — what the feedback acts on, and what target() reports
    float _rate_tgt   = 0.0f;   // commanded rate [rad/s], fed forward
    float _rate_pred  = 0.0f;   // _rate_tgt lagged by lag_s — what _target integrates
    float _rate_cmd   = 0.0f;   // last output [rad/s]

    static constexpr float SHAPE_KP = 4.0f;   // 1/s, approach-rate gain close to yaw_des (linear end of the sqrt profile)
    static constexpr float LEASH    = 0.6f;   // rad, max distance the target may get from the vehicle
};
