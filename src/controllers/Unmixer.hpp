#pragma once
#include <cstdint>
#include "src/math/math.hpp"
#include "configs/DroneConfig.hpp"

/*
 * Unmixer — converts per-motor RPM to physical roll/pitch torques (N·m).
 *
 * Gated RPM is 2nd-order (Butterworth) lowpass filtered per motor at
 * RPM_FILT_HZ before the motor model, to knock down eRPM telemetry
 * noise/quantization feeding current_torque (the baseline AttitudeINDI adds
 * delta_torque onto), then an optional 3rd (1st-order) stage at
 * RPM_FILT_EXTRA_HZ — a static A/B knob to test whether a 2+1 order cascade
 * helps INDI; <=0 disables it (lowpass_alpha() passthrough), leaving the
 * 2nd-order-only behaviour. dt is a fixed constant rather than measured,
 * since compute() is only ever called from ControlThread's fixed 400 Hz
 * period (see main.cpp's kRates.control) — no self-timing needed.
 *
 * Motor model (bench thrust fit, normalised angular velocity cubic → thrust in N).
 * The bench fit was done against motor angular velocity in rad/s, not RPM, so
 * incoming (filtered) mechanical RPM is converted first:
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
 *
 * Motor model/geometry all come from UnmixerConfig (per-drone) — see
 * configs/DroneConfig.hpp. T_MAX_NM is derived once at construction from
 * arm_length_m/max_thrust_n rather than being its own config field, so the
 * two numbers can't drift out of sync in a config file.
 */
class Unmixer {
public:
    explicit Unmixer(const UnmixerConfig &cfg)
        : _cfg(cfg)
        // T_MAX = 2*sin(45°) * arm_length_m * max_thrust_n
        , _t_max_nm(2.0f * 0.70710678f * cfg.arm_length_m * cfg.max_thrust_n)
    {}

    // rpm[4]: per-motor mechanical RPM [FR, RL, FL, RR]
    // torque_Nm[2]: output [roll, pitch] in N·m
    void compute(const uint32_t rpm[4], float torque_Nm[2]);

    // Clamp and normalise a physical torque (N·m) to [-1, 1] for MotorMixer.
    float normalize_torque(float torque_Nm) const;

    // RPM → rad/s (fixed unit conversion, not per-drone).
    static constexpr float RPM_TO_RADS = 3.14159265f / 30.0f;
    // compute() is only ever called from ControlThread's fixed 400 Hz period
    // (see main.cpp's kRates.control) — a system/thread-rate constant, not
    // per-drone tuning, so this stays fixed rather than moving into
    // UnmixerConfig.
    static constexpr float RPM_FILT_DT_S = 0.0025f;

private:
    static float motor_force_N(const UnmixerConfig &cfg, float rpm);

    UnmixerConfig _cfg;
    float         _t_max_nm;

    Biquad2pState _rpm_filt_state[4];
    float _rpm_extra_filt[4] = {};  // 1st-order LPF memory for the rpm_filt_extra_hz stage
};
