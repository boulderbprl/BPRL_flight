#pragma once
#include "PID.hpp"
#include "src/math/math.hpp"
#include "configs/DroneConfig.hpp"
#include <cstdint>

/*
 * Horizontal position controller (PosControl) — pilot sticks → N/E lean angles.
 * The z axis is handled entirely by AltControl::alt_hold().
 *
 * Same scheme as ArduPilot's AC_Loiter::calc_desired_velocity(): the sticks
 * command an acceleration, which is integrated into a velocity target and
 * then into a position target. The position loop runs on that moving target
 * all the time, so there is no "hold point" to latch — when the velocity
 * target reaches zero the position target simply stops moving.
 *
 *   Target generator (open loop):
 *     sticks → lean angle → accel_pilot [m/s²]   (jerk-limited, rotated by yaw)
 *     accel_cmd  = accel_pilot − drag − brake    drag  = STICK_ACCEL_MAX·|vel_des|/MAX_SPEED
 *                                                brake = sticks centred for BRAKE_DELAY_S
 *     accel_pred = accel_cmd lagged by ATT_LAG_S (what the vehicle will actually do)
 *     vel_des += accel_pred·dt
 *     pos_des += vel_des·dt                      (leashed to the vehicle)
 *
 *   Feedback (unchanged cascade) + feed-forward:
 *     vel_tgt   = vel_des + pos_P(pos_des − pos)
 *     accel_tgt = accel_cmd + LPF(vel_PID(vel_tgt − vel))
 *     accel_tgt → yaw rotation + atan2 → roll/pitch targets
 *
 * Conventions:
 *   state[0..2]  N, E, D in inertial frame [m]
 *   state[3..5]  roll, pitch, yaw body-frame [rad]
 *   state[6..8]  vN, vE, vD inertial frame [m/s]
 *   stick_fwd / stick_right   normalised [-1, 1], positive = forward / right
 *   att_cmds[2]  [roll_tgt, pitch_tgt] [rad]
 */
class PosControl {
public:
    explicit PosControl(const PosControlGains &g);

    // One control tick: sticks + state → lean-angle targets.
    void update(const float state[], float stick_fwd, float stick_right, float att_cmds[2]);

    void reset_all();

    // Targets the last update() fed to the feedback loops, [N, E] — for logging.
    float pos_tgt(int axis) const { return _pos_des[axis]; }
    float vel_tgt(int axis) const { return _vel_tgt[axis]; }

    // Stick frame. true: roll/pitch sticks command right/forward acceleration
    // relative to the nose, rotated by yaw into N/E (ArduPilot Loiter
    // behaviour). false: sticks command N/E directly, so "forward" only
    // matches the nose when the vehicle faces north.
    static constexpr bool  STICKS_BODY_FRAME = true;
    static constexpr float STICK_DEADBAND    = 0.10f;   // normalised [-1,1]

private:
    static float stick_to_accel(float stick);
    void compute_lean_angles(float yaw_rad, float accel_N_tgt, float accel_E_tgt,
                             float &roll_tgt, float &pitch_tgt);

    PID _pos_N;
    PID _pos_E;
    PID _vel_N;
    PID _vel_E;

    // Target generator state, [N, E].
    bool  _tgt_valid      = false;   // false → seed targets from the measured state on the next update()
    float _pos_des[2]     = {};      // position target [m]
    float _vel_des[2]     = {};      // velocity target from the sticks alone [m/s]
    float _vel_tgt[2]     = {};      // _vel_des + position correction [m/s]
    float _accel_pilot[2] = {};      // jerk-limited stick acceleration [m/s²]
    float _accel_pred[2]  = {};      // commanded accel (stick − drag − brake) lagged by ATT_LAG_S — what the targets integrate [m/s²]
    float _brake_accel    = 0.0f;    // braking deceleration along −vel_des [m/s²]
    float _brake_delay_s  = 0.0f;    // time the sticks have been centred [s]

    // 2nd-order LPF on the velocity PID's output (the feedback part of the
    // accel target only — the stick feed-forward bypasses it), in the
    // inertial frame before the yaw rotation + atan2 in compute_lean_angles.
    Biquad2pState _accel_N_filt;
    Biquad2pState _accel_E_filt;

    static constexpr float GRAVITY_MSS   = 9.80665f;
    static constexpr float MAX_LEAN_deg  = 30.0f;   // limit on the total lean target (feed-forward + feedback)
    static constexpr float ACCEL_FILT_HZ = 10.0f;   // feedback accel-target LPF cutoff

    // ── Stick / target-generator tuning ──────────────────────────────────────
    static constexpr float MAX_STICK_LEAN_deg = 20.0f;   // lean angle commanded by a full stick
    static constexpr float STICK_ACCEL_MAX    = 3.5693f; // g·tan(MAX_STICK_LEAN_deg) [m/s²]
    static constexpr float STICK_JERK_MAX     = 20.0f;   // m/s³, how fast the stick accel (and so the lean command) may change
    static constexpr float ATT_LAG_S          = 0.15f;   // s, attitude loop's lean-angle response lag (flight log: roll 0.16 s, pitch 0.12 s) — re-measure if the attitude gains change
    static constexpr float MAX_SPEED          = 5.0f;    // m/s, speed a full stick settles at (ArduPilot LOIT_SPEED)
    static constexpr float BRAKE_DELAY_S      = 0.3f;    // sticks centred this long before braking starts (LOIT_BRK_DELAY, 1 s there)
    static constexpr float BRAKE_GAIN         = 2.0f;    // 1/s, brake decel per m/s of target speed
    static constexpr float BRAKE_ACCEL_MAX    = 2.5f;    // m/s², cap on brake decel (LOIT_BRK_ACCEL)
    static constexpr float BRAKE_JERK_MAX     = 10.0f;   // m/s³, how fast brake decel ramps (LOIT_BRK_JERK, 5 there)
    static constexpr float VEL_CORR_MAX       = 2.0f;    // m/s, cap on the position loop's velocity correction per axis
    static constexpr float POS_LEASH_M        = 2.0f;    // m, max distance the position target may get from the vehicle
};
