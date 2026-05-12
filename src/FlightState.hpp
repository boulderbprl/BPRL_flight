#pragma once
#include <cstdint>

/*
 * Shared flight state indices — 19-state EKF output written by StateEstThread,
 * read by ControlThread, LogThread, and DebugThread.
 *
 * Units are SI throughout:
 *   Position:    metres         (NED inertial frame)
 *   Velocity:    m/s            (body frame)
 *   Accel:       m/s²           (body frame, gravity-corrected)
 *   Quaternion:  dimensionless  (NED→Body, Hamilton [W,X,Y,Z], scalar-first)
 *   Angles:      radians        (derived from quaternion by StateManager)
 *   Rates:       rad/s          (body frame)
 *   Ang. accel:  rad/s²         (body frame, 30 Hz lowpass filtered)
 *   PWM:         microseconds
 *   Normalised inputs: [-1, 1] or [0, 1]
 *
 * g_state[] is sized to hold all 19 EKF outputs.  The ControlThread reads
 * only the attitude and rate slots (indices 0–5); the full vector is
 * available for logging and future INDI use.
 */
namespace StateIdx {
    // ── Position — inertial NED frame (m) ────────────────────────────────
    constexpr int X     = 0;
    constexpr int Y     = 1;
    constexpr int Z_POS = 2;

    // ── Translational velocity — body frame (m/s) ─────────────────────────
    constexpr int U     = 3;
    constexpr int V     = 4;
    constexpr int W     = 5;

    // ── Translational acceleration — body frame, gravity-corrected (m/s²) ─
    constexpr int U_DOT = 6;
    constexpr int V_DOT = 7;
    constexpr int W_DOT = 8;

    // ── Quaternion NED→Body [W,X,Y,Z] Hamilton, scalar-first ─────────────
    constexpr int Q0    = 9;
    constexpr int Q1    = 10;
    constexpr int Q2    = 11;
    constexpr int Q3    = 12;

    // ── Angular velocity — body frame (rad/s) ─────────────────────────────
    constexpr int P     = 13;
    constexpr int Q     = 14;
    constexpr int R     = 15;

    // ── Angular acceleration — body frame, 50 Hz lowpass filtered (rad/s²) ─
    // Used by the incremental nonlinear dynamic inversion controller (future).
    constexpr int P_DOT = 16;
    constexpr int Q_DOT = 17;
    constexpr int R_DOT = 18;

    // Total state dimension
    constexpr int N = 19;
}

namespace InputIdx {
    constexpr int THRUST    = 0; // throttle [0, 1]
    constexpr int ROLL_TGT  = 1; // roll  setpoint [-1, 1]
    constexpr int PITCH_TGT = 2; // pitch setpoint [-1, 1]
    constexpr int YAW_RATE  = 3; // yaw rate demand [-1, 1]
}
