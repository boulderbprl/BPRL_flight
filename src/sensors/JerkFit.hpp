#pragma once
#include "StrainRate.hpp"

/*
 * JKFT — linear regression fit from 4-channel strain rate + roll rate to
 * vertical and roll jerk (WIP). Strain picks up arm flex fast enough to
 * carry jerk-band information that's already lost by the time gyro rate
 * has been twice differentiated and lowpass-filtered down to p_dot
 * (StateIdx::P_DOT), so this fits jerk directly from the raw channels
 * instead of differentiating p_dot again.
 *
 * Shared by LogThread (LOG_MSG_JKFT, at log rate) and ControlThread
 * (AttitudePIDJerk's roll-jerk damping term, at 400 Hz) — see threads.cpp.
 */
struct JerkEstimate {
    float z_jerk;     // m/s^3   vertical jerk estimate
    float roll_jerk;  // rad/s^3 roll jerk estimate ("Pdd")
};

// JKFT_MATRIX has no intercept term, so its raw output isn't mean-zero unless
// each strain channel is itself zero-mean first. strain_bias[4] is that
// per-channel zero-offset — live-calibrated on the bench via the channel-6
// momentary switch (see ControlThread's strain calibration block in
// threads.cpp and radio_strain_cal()), not a fixed measured constant: it
// resets to {0,0,0,0} every boot and is only ever as good as the most recent
// calibration this session. Pass {0,0,0,0} (uncalibrated) to get the raw,
// unbiased-only-by-luck fit.
inline JerkEstimate estimate_jerk(const StrainRateRaw &strain, const float strain_bias[4], float p)
{
    // constexpr float JKFT_MATRIX[5][2] = {
    //     {  -0.0613f,   -0.0578f}, // s0
    //     {   0.0842f,    0.0401f}, // s1
    //     {  -0.0122f,    0.0789f}, // s2
    //     {   0.0253f,   -0.0575f}, // s3
    //     {  -0.4319f,  -33.6420f}, // p
    // };
    constexpr float JKFT_MATRIX[5][2] = {
        {  -0.017783f,   -0.23087f}, // s0
        {   0.026056f,    0.21854f}, // s1
        {  -0.013622f,    0.057366f}, // s2
        {   0.022922f,   -0.0023487f}, // s3
        {  -0.70644f ,  -70.504f}, // p
    };

    const float inputs[5] = { (float)strain.val[0] - strain_bias[0],
                               (float)strain.val[1] - strain_bias[1],
                               (float)strain.val[2] - strain_bias[2],
                               (float)strain.val[3] - strain_bias[3], p };

    JerkEstimate out{};
    for (uint8_t i = 0; i < 5; i++) {
        out.z_jerk    += inputs[i] * JKFT_MATRIX[i][0];
        out.roll_jerk += inputs[i] * JKFT_MATRIX[i][1];
    }
    return out;
}
