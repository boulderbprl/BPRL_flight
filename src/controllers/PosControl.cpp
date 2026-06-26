#include "PosControl.hpp"
#include "src/math/math.hpp"
#include <cmath>
#include <algorithm>

PosControl::PosControl()
    : _pos_N  (1.0f, 0.00f, 0.000f, 0.0f, 20.0f)
    , _pos_E  (1.0f, 0.00f, 0.000f, 0.0f, 20.0f)
    , _pos_D  (1.0f, 0.00f, 0.000f, 0.0f, 20.0f)
    , _vel_N  (2.0f, 1.0f, 0.500f, 0.0f, 20.0f)
    , _vel_E  (2.0f, 1.0f, 0.500f, 0.0f, 20.0f)
    , _vel_D  (5.0f, 2.0f, 0.000f, 0.0f, 20.0f)
{}

// This function needs to be updated to consider stick inputs 

void PosControl::NED_update(const float state[], const float pos_tgt[3],
                    float out_cmds_vel[3])
{   

    float vel_N_tgt  = _pos_N.update(pos_tgt[0] - state[0]);
    float vel_E_tgt  = _pos_E.update(pos_tgt[1] - state[1]);

    float vel_D_tgt  = _pos_N.update(pos_tgt[2] - state[2]);


    out_cmds_vel[0]  = constrain_float(vel_N_tgt, -MAX_VEL_NE, MAX_VEL_NE);
    out_cmds_vel[1]  = constrain_float(vel_E_tgt, -MAX_VEL_NE, MAX_VEL_NE);
    out_cmds_vel[2]  = constrain_float(vel_D_tgt, -MAX_VEL_D, MAX_VEL_D);


}

void PosControl::D_rate_update(const float state[], const float vel_D_tgt,
                    float thr_cmd)
{

    float delta_thr = _vel_D.update(vel_D_tgt - state[8]);

    thr_cmd = constrain_float(THR_MID - delta_thr, 0.0,1.0);

}

void PosControl::NE_rate_update(const float state[], const float vel_NE_tgt[2],
                    float att_cmds[2])
{
    
    // update the NE controller to get out_cmds[2] [ax_body_cmd, ay_body_cmd]

    const float accel_N_tgt  = _vel_N.update(vel_NE_tgt[0] - state[6]);
    const float accel_E_tgt = _vel_E.update(vel_NE_tgt[1]- state[7]);

    // Rotate world-frame accel cmds into yaw-aligned body frame
    const float yaw_rad = state[5];
    float roll_tgt; 
    float pitch_tgt;

    compute_lean_angles(yaw_rad, accel_N_tgt, accel_E_tgt, roll_tgt, 
                    pitch_tgt);
    
    roll_tgt = constrain_float(roll_tgt,-degreesToRadians(MAX_LEAN_deg),
                            degreesToRadians(MAX_LEAN_deg));

    pitch_tgt = constrain_float(pitch_tgt,-degreesToRadians(MAX_LEAN_deg),
        degreesToRadians(MAX_LEAN_deg));

    att_cmds[0] = roll_tgt;
    att_cmds[1] = pitch_tgt;
}


void PosControl::compute_lean_angles(const float yaw_rad, const float accel_N_tgt, 
                    const float accel_E_tgt, float roll_tgt, 
                    float pitch_tgt)
{
    
    const float accel_N_body = cosf(yaw_rad)*accel_N_tgt + sinf(yaw_rad)*accel_E_tgt; 
    const float accel_E_body = -sinf(yaw_rad)*accel_N_tgt + cosf(yaw_rad)*accel_E_tgt; 

    // convert to lean angles

    roll_tgt = atan2(-accel_N_body,GRAVITY_MSS); 
    pitch_tgt = atan2(accel_E_tgt*cosf(roll_tgt),GRAVITY_MSS);

}

float PosControl::compute_throttle(const float state[],
                               const float input[]) const
{
    const float thr_in  = input[0];
    const float expo    = -(THR_MID - 0.5f) / 0.375f;
    const float thr_exp = thr_in * (1.0f - expo)
                          + expo * thr_in * thr_in * thr_in;
    const float boost   = 1.0f / std::min(cosf(state[0]), cosf(state[1]));
    return constrain_float(thr_exp * boost, 0.0f, 1.0f);
}

void PosControl::reset_all()
{
    _pos_N.reset();  _vel_N.reset();
    _pos_E.reset(); _vel_E.reset();
    _pos_D.reset(); _vel_D.reset();
}
