#pragma once
#include <cstdint>

/*
 * Unmixer — converts per-motor RPM to physical roll/pitch torques (N·m).
 *
 * Motor model (bench thrust fit, normalised angular velocity cubic → thrust in N).
 * The bench fit was done against motor angular velocity in rad/s, not RPM, so
 * incoming mechanical RPM is converted first:
 *   omega    = rpm * (pi / 30)                       // RPM → rad/s
 *   rpm_norm = (omega - RPM_NORM_CENTER) / RPM_NORM_SCALE
 *   F_N      = C3*rpm_norm³ + C2*rpm_norm² + C1*rpm_norm + C0
 *   F_N      = max(F_N, 0)   // guard small negative thrust near zero RPM
 *
 * X-frame geometry (matches MotorMixer numbering, NED body frame X-fwd Y-right):
 *   Motor 0 (FR): position (+L/√2, +L/√2)
 *   Motor 1 (RL): position (-L/√2, -L/√2)
 *   Motor 2 (FL): position (+L/√2, -L/√2)
 *   Motor 3 (RR): position (-L/√2, +L/√2)
 *
 * Torques (from r × F, thrust upward):
 *   roll_Nm  = (ARM_LENGTH/√2) * (-F0 + F1 + F2 - F3)
 *   pitch_Nm = (ARM_LENGTH/√2) * ( F0 - F1 + F2 - F3)
 *
 * Signs are consistent with MotorMixer: positive roll cmd increases RL/FL motors,
 * producing positive roll_Nm here.
 *
 * Note: the bench fit also gives motor reaction (drag) torque as a function of
 * thrust — torque_Nm = 0.03*((F_N - 2.991)/2.183) + 0.0423 — but that isn't
 * wired in here since yaw currently uses a rate PID rather than INDI torque
 * feedback (see AttitudeINDI). Add it if yaw moves to torque-based control.
 */
class Unmixer {
public:
    Unmixer() = default;

    // rpm[4]: per-motor mechanical RPM [FR, RL, FL, RR]
    // torque_Nm[2]: output [roll, pitch] in N·m
    void compute(const uint32_t rpm[4], float torque_Nm[2]) const;

    // Clamp and normalise a physical torque (N·m) to [-1, 1] for MotorMixer.
    float normalize_torque(float torque_Nm) const;

    // ── Motor polynomial: normalised angular velocity (rad/s) → thrust in N ───
    // From static motor thrust bench test/fit (fit against omega, not raw RPM).
    static constexpr float RPM_TO_RADS     = 3.14159265f / 30.0f;  // RPM → rad/s
    static constexpr float RPM_NORM_CENTER = 2005.0f;  // rad/s
    static constexpr float RPM_NORM_SCALE  = 880.8f;   // rad/s
    static constexpr float MOTOR_C3 = 0.0134f;  // N / rpm_norm³
    static constexpr float MOTOR_C2 = 0.5607f;  // N / rpm_norm²
    static constexpr float MOTOR_C1 = 2.2831f;  // N / rpm_norm
    static constexpr float MOTOR_C0 = 2.4540f;  // N (rpm_norm = 0 offset)

    // ── Drone geometry ────────────────────────────────────────────────────────
    static constexpr float ARM_LENGTH_M = 0.1275f;  // motor-to-centre arm length (m)

    // ── Normalisation ─────────────────────────────────────────────────────────
    // Maximum single-motor thrust from the bench fit (N).
    static constexpr float MAX_THRUST_N = 7.04f;
    // Maximum achievable roll/pitch torque in N·m used to normalise to [-1, 1]:
    //   T_MAX = 2*sin(45°) * ARM_LENGTH_M * MAX_THRUST_N
    static constexpr float T_MAX_NM = 1.2693981f;

private:
    static float motor_force_N(uint32_t rpm);
};
