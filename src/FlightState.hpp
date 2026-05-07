#pragma once
#include <cstdint>

/*
 * Shared flight state indices - used in both the standalone ChibiOS
 * build (main.cpp) and the ArduPilot build (BPRL.cpp).
 *
 * Units are SI throughout:
 *   Angles:  radians    (matches AP_AHRS output)
 *   Rates:   rad/s
 *   Accel:   m/s²
 *   PWM:     microseconds
 *   Normalised inputs: [-1, 1] or [0, 1]
 */
namespace StateIdx {
    constexpr int ROLL    = 0;   // roll  angle (rad)
    constexpr int PITCH   = 1;   // pitch angle (rad)
    constexpr int YAW     = 2;   // yaw   angle (rad)
    constexpr int P       = 3;   // roll  rate  (rad/s)
    constexpr int Q       = 4;   // pitch rate  (rad/s)
    constexpr int R       = 5;   // yaw   rate  (rad/s)
    constexpr int Z_POS   = 6;   // altitude    (m)
    constexpr int Z_VEL   = 7;   // climb rate  (m/s)
    constexpr int Z_ACCEL = 8;   // vertical accel (m/s²)
}

namespace InputIdx {
    constexpr int THRUST    = 0; // throttle [0, 1]
    constexpr int ROLL_TGT  = 1; // roll  setpoint [-1, 1]
    constexpr int PITCH_TGT = 2; // pitch setpoint [-1, 1]
    constexpr int YAW_RATE  = 3; // yaw rate demand [-1, 1]
}
