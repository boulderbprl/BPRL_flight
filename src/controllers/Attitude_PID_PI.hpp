#pragma once
#include "PID.hpp"
#include "AttitudeController.hpp"
#include "configs/DroneConfig.hpp"

/*
 * Three-stage cascade attitude controller — outer attitude loop + rate loop
 * ("SLC") + inner PI loop closed on measured angular acceleration. Ported
 * from the BPRL ArduPilot fork.
 *
 * Outer loop:  attitude error [rad]   -> angular rate target [rad/s]     (P)
 * SLC loop:    rate error [rad/s]     -> angular-accel target [rad/s^2]  (PID)
 * Inner loop:  accel error [rad/s^2]  -> normalised torque out [-1, 1]   (PI, kd=0)
 *
 * Yaw has no inner accel loop (mirrors the ArduPilot source) — same rate PID +
 * heading-lock trim as AttitudePID/AttitudeINDI;
 *
 * Conventions (own 6-element internal state, built from euler[]/state_full[]
 * inside update() to match the shared AttitudeController interface):
 *   state6[0..2]  roll, pitch, yaw in radians
 *   state6[3..5]  p, q, r body-frame rates in rad/s
 *   input[1..3]   roll_tgt, pitch_tgt, yaw_rate_tgt [-1, 1]
 *   out_cmds[3]   normalised torque [roll, pitch, yaw] in [-1, 1]
 */
class AttitudePIDPI : public AttitudeController {
public:
    explicit AttitudePIDPI(const AttitudePidPiGains &g);

    void update(const float euler[3], const float state_full[], const float input[],
                const float current_torque[2], const Unmixer &unmixer,
                float out_cmds[3]) override;
    void reset_all() override;

private:
    PID _roll_att;
    PID _pitch_att;
    PID _roll_rate;    // "SLC": rate error -> angular-accel target
    PID _pitch_rate;
    PID _roll_accel;   // inner loop: accel error -> normalised torque (PI, kd=0)
    PID _pitch_accel;
    PID _yaw_rate;
    PID _yaw_hold;     // heading-lock trim: heading error [rad] -> corrective rate [rad/s]

    float _yaw_stick_gain;     // from DroneConfig; mirrors AttitudePidGains::yaw_stick_gain
    float _yaw_target;        // held heading target [rad]
    bool  _yaw_target_valid;  // false until first update() captures a target

    static constexpr float YAW_STICK_DEADBAND = 0.10f;  // normalised stick [-1,1], matches FlightStateMachine::STICK_DEADBAND
    static constexpr float YAW_HOLD_MAX_RATE  = 0.3f;   // rad/s cap on the heading-hold trim — needs flight tuning
};
