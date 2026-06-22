#pragma once
#include <cstdint>

/*
 * Unmixer — converts per-motor RPM to physical roll/pitch torques (N·m).
 *
 * Motor model: F_kg = p1*rpm³ + p2*rpm² + p3*rpm + p4  (RPM → force in kg)
 *              F_N  = F_kg * 9.81
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
 * All motor polynomial coefficients and geometry constants are placeholders —
 * update from bench motor characterisation and airframe measurement.
 */
class Unmixer {
public:
    Unmixer() = default;

    // rpm[4]: per-motor mechanical RPM [FR, RL, FL, RR]
    // torque_Nm[2]: output [roll, pitch] in N·m
    void compute(const uint32_t rpm[4], float torque_Nm[2]) const;

    // Clamp and normalise a physical torque (N·m) to [-1, 1] for MotorMixer.
    float normalize_torque(float torque_Nm) const;

    // ── Motor polynomial: RPM → force in kg ───────────────────────────────────
    // Fill these from a static motor thrust bench test.
    static constexpr float MOTOR_P1 = 0.0f;   // kg / RPM³
    static constexpr float MOTOR_P2 = 0.0f;   // kg / RPM²
    static constexpr float MOTOR_P3 = 0.0f;   // kg / RPM
    static constexpr float MOTOR_P4 = 0.0f;   // kg (zero-RPM offset, usually 0)

    // ── Drone geometry ────────────────────────────────────────────────────────
    static constexpr float ARM_LENGTH_M = 0.225f;  // motor-to-centre arm length (m)

    // ── Normalisation ─────────────────────────────────────────────────────────
    // Maximum achievable roll/pitch torque in N·m used to normalise to [-1, 1].
    // Update once motor characterisation is complete:
    //   T_MAX ≈ ARM_LENGTH/√2 * F_max_per_motor_N * 2
    static constexpr float T_MAX_NM = 1.0f;

private:
    static float motor_force_N(uint32_t rpm);
};
