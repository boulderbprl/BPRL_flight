#include "PosControl.hpp"
#include "src/math/math.hpp"
#include <cmath>

PosControl::PosControl()
    : _pos_N(1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f, 20.0f)
    , _pos_E(1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f, 20.0f)
    , _pos_D(1.0f, 0.00f, 0.000f, 0.0f, 0.0f, 0.0f, 20.0f)
    , _vel_N(2.0f, 1.0f, 0.500f, 0.0f, 0.0f, 0.0f, 20.0f)
    , _vel_E(2.0f, 1.0f, 0.500f, 0.0f, 0.0f, 0.0f, 20.0f)
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
    const float accel_N_tgt = _vel_N.update(vel_NE_tgt[0], state[6]);
    const float accel_E_tgt = _vel_E.update(vel_NE_tgt[1], state[7]);

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

    roll_tgt  = atan2f(-accel_N_body, GRAVITY_MSS);
    pitch_tgt = atan2f( accel_E_body * cosf(roll_tgt), GRAVITY_MSS);  // fixed: was accel_E_tgt
}

void PosControl::reset_all()
{
    _pos_N.reset(); _pos_E.reset(); _pos_D.reset();
    _vel_N.reset(); _vel_E.reset();
}
