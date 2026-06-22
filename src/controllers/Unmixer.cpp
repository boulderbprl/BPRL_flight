#include "Unmixer.hpp"
#include <cmath>
#include <algorithm>

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float Unmixer::motor_force_N(uint32_t rpm)
{
    const float r   = static_cast<float>(rpm);
    const float r2  = r * r;
    const float r3  = r2 * r;
    const float f_kg = MOTOR_P1 * r3 + MOTOR_P2 * r2 + MOTOR_P3 * r + MOTOR_P4;
    return clampf(f_kg, 0.0f, 1e6f) * 9.81f;   // kg → N, clamp to non-negative
}

void Unmixer::compute(const uint32_t rpm[4], float torque_Nm[2]) const
{
    const float F0 = motor_force_N(rpm[0]);  // FR
    const float F1 = motor_force_N(rpm[1]);  // RL
    const float F2 = motor_force_N(rpm[2]);  // FL
    const float F3 = motor_force_N(rpm[3]);  // RR

    static constexpr float INV_SQRT2 = 0.70710678f;
    const float lever = ARM_LENGTH_M * INV_SQRT2;

    torque_Nm[0] = lever * (-F0 + F1 + F2 - F3);  // roll
    torque_Nm[1] = lever * ( F0 - F1 + F2 - F3);  // pitch
}

float Unmixer::normalize_torque(float torque_Nm) const
{
    return clampf(torque_Nm / T_MAX_NM, -1.0f, 1.0f);
}
