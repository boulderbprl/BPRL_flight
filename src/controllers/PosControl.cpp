#include "PosControl.hpp"
#include "src/math/math.hpp"
#include "ch.h"
#include <cmath>

PosControl::PosControl()
    : _pos_N(1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f, 20.0f)
    , _pos_E(1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f, 20.0f)
    , _pos_D(1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f, 20.0f)
    , _vel_N(2.0f, 1.0f, 0.000f, 0.8f, 0.0f, 20.0f, 20.0f) // 20 Hz filter on error, 20 Hz filter on derivative
    , _vel_E(2.0f, 1.0f, 0.000f, 0.8f, 0.0f, 20.0f, 20.0f)
{}

void PosControl::NED_update(const float state[], const float pos_tgt[3],
                            float vel_tgt[3])
{
    float vel_N_tgt = _pos_N.update(pos_tgt[0], state[0]);
    float vel_E_tgt = _pos_E.update(pos_tgt[1], state[1]);
    float vel_D_tgt = _pos_D.update(pos_tgt[2], state[2]);  // fixed: was _pos_N

    vel_tgt[0] = constrain_float(vel_N_tgt, -MAX_VEL_NE, MAX_VEL_NE);
    vel_tgt[1] = constrain_float(vel_E_tgt, -MAX_VEL_NE, MAX_VEL_NE);
    vel_tgt[2] = constrain_float(vel_D_tgt, -MAX_VEL_D,  MAX_VEL_D);
}

void PosControl::NE_rate_update(const float state[], const float vel_NE_tgt[2],
                                float att_cmds[2])
{
    const float accel_N_tgt_raw = _vel_N.update(vel_NE_tgt[0], state[6]);
    const float accel_E_tgt_raw = _vel_E.update(vel_NE_tgt[1], state[7]);

    const uint32_t now = now_us();
    const float dt_s = _accel_filt_valid ? (float)(now - _last_accel_t_us) * 1.0e-6f : 0.0f;
    _last_accel_t_us   = now;
    _accel_filt_valid  = true;

    // Filter the accel target in the inertial N/E frame (before the yaw
    // rotation + atan2 in compute_lean_angles) so the smoothing is
    // yaw-invariant instead of acting on an already-rotated lean angle.
    const float accel_N_tgt = lowpass2p(accel_N_tgt_raw, _accel_N_filt, ACCEL_FILT_HZ, dt_s);
    const float accel_E_tgt = lowpass2p(accel_E_tgt_raw, _accel_E_filt, ACCEL_FILT_HZ, dt_s);

    const float yaw_rad = state[5];
    float roll_tgt  = 0.0f;
    float pitch_tgt = 0.0f;
    compute_lean_angles(yaw_rad, accel_N_tgt, accel_E_tgt, roll_tgt, pitch_tgt);

    att_cmds[0] = constrain_float(roll_tgt,  -degreesToRadians(MAX_LEAN_deg),
                                               degreesToRadians(MAX_LEAN_deg));
    att_cmds[1] = constrain_float(pitch_tgt, -degreesToRadians(MAX_LEAN_deg),
                                               degreesToRadians(MAX_LEAN_deg));
}

void PosControl::compute_lean_angles(float yaw_rad, float accel_N_tgt,
                                     float accel_E_tgt,
                                     float &roll_tgt, float &pitch_tgt)
{
    const float accel_N_body =  cosf(yaw_rad) * accel_N_tgt + sinf(yaw_rad) * accel_E_tgt;
    const float accel_E_body = -sinf(yaw_rad) * accel_N_tgt + cosf(yaw_rad) * accel_E_tgt;

    pitch_tgt = atan2f(-accel_N_body, GRAVITY_MSS);
    roll_tgt  = atan2f( accel_E_body * cosf(pitch_tgt), GRAVITY_MSS);
}

void PosControl::reset_all()
{
    _pos_N.reset(); _pos_E.reset(); _pos_D.reset();
    _vel_N.reset(); _vel_E.reset();

    _accel_N_filt = Biquad2pState();
    _accel_E_filt = Biquad2pState();
    _accel_filt_valid = false;
}

uint32_t PosControl::now_us()
{
    return (uint32_t)TIME_I2US(chVTGetSystemTimeX());
}
