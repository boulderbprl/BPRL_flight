#include "Unmixer.hpp"
#include <cmath>
#include <algorithm>

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float Unmixer::motor_force_N(const UnmixerConfig &cfg, float rpm)
{
    const float omega = rpm * RPM_TO_RADS;   // mechanical RPM → rad/s
    const float rn = (omega - cfg.rpm_norm_center) / cfg.rpm_norm_scale;
    const float f_n = ((cfg.motor_c3 * rn + cfg.motor_c2) * rn + cfg.motor_c1) * rn + cfg.motor_c0;
    return clampf(f_n, 0.0f, cfg.max_thrust_n);   // guard small negative thrust, clamp to bench max
}

void Unmixer::compute(const uint32_t rpm[4], float torque_Nm[2])
{
    float rpm_filt[4];
    const float alpha_extra = lowpass_alpha(_cfg.rpm_filt_extra_hz, RPM_FILT_DT_S);
    for (int i = 0; i < 4; ++i) {
        const float stage2p = lowpass2p(static_cast<float>(rpm[i]), _rpm_filt_state[i],
                                         _cfg.rpm_filt_hz, RPM_FILT_DT_S);
        _rpm_extra_filt[i] = lowpass(stage2p, _rpm_extra_filt[i], alpha_extra);
        rpm_filt[i] = _rpm_extra_filt[i];
    }

    const float F0 = motor_force_N(_cfg, rpm_filt[0]);  // FR
    const float F1 = motor_force_N(_cfg, rpm_filt[1]);  // RL
    const float F2 = motor_force_N(_cfg, rpm_filt[2]);  // FL
    const float F3 = motor_force_N(_cfg, rpm_filt[3]);  // RR

    static constexpr float INV_SQRT2 = 0.70710678f;
    const float lever = _cfg.arm_length_m * INV_SQRT2;

    torque_Nm[0] = lever * (-F0 + F1 + F2 - F3);  // roll
    torque_Nm[1] = lever * ( F0 - F1 + F2 - F3);  // pitch
}

float Unmixer::normalize_torque(float torque_Nm) const
{
    return clampf(torque_Nm / _t_max_nm, -1.0f, 1.0f);
}
