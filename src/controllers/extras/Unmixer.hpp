#pragma once
#include <cstdint>
#include "src/math/math.hpp"
#include "configs/DroneConfig.hpp"
#include "src/threads.hpp"   // CONTROL_DT_S

/*
 * Unmixer — converts per-motor RPM to physical roll/pitch torques (N·m).
 *
 * Gated RPM is lowpass filtered per motor before the motor model, to knock
 * down eRPM telemetry noise/quantization feeding current_torque (the
 * baseline AttitudeINDI adds delta_torque onto). The filter is the drone's
 * AngAccelFilterConfig — 2nd-order Butterworth at lpf_hz, then an optional
 * 1st-order stage at lpf_extra_hz (<=0 disables it) — the exact same
 * setting StateManager uses for p_dot/q_dot, so the two INDI feedback
 * signals stay delay-matched by construction. dt is a fixed constant rather than measured,
 * since compute() is only ever called from ControlThread's fixed 400 Hz
 * period (CONTROL_DT_S in src/threads.hpp) — no self-timing needed.
 *
 * Motor model (bench thrust fit, mechanical RPM cubic → thrust in N):
 *   F_N      = C3*rpm³ + C2*rpm² + C1*rpm + C0
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
    Unmixer(const UnmixerConfig &cfg, const AngAccelFilterConfig &filt)
        : _cfg(cfg)
        , _filt(filt)
        // T_MAX = 2*sin(45°) * arm_length_m * max_thrust_n
        , _t_max_nm(2.0f * 0.70710678f * cfg.arm_length_m * cfg.max_thrust_n)
    {}

    // rpm[4]: per-motor mechanical RPM [FR, RL, FL, RR]
    // torque_Nm[2]: output [roll, pitch] in N·m
    void compute(const uint32_t rpm[4], float torque_Nm[2]);

    // Clamp and normalise a physical torque (N·m) to [-1, 1] for MotorMixer.
    float normalize_torque(float torque_Nm) const;

    // The same physical torque bound normalize_torque() divides by — for
    // controllers that need to clamp an internal torque-valued register
    // against reality directly (e.g. AttitudeJINDI's jerk-path commanded-
    // torque accumulator, which has no other anchor to physical torque).
    float max_torque_nm() const { return _t_max_nm; }

private:
    static float motor_force_N(const UnmixerConfig &cfg, float rpm);

    UnmixerConfig _cfg;
    AngAccelFilterConfig _filt;
    float         _t_max_nm;

    Biquad2pState _rpm_filt_state[4];
    float _rpm_extra_filt[4] = {};  // 1st-order LPF memory for the lpf_extra_hz stage
};
