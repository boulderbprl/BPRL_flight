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
    float roll_jerk;  // rad/s^3 roll jerk estimate ("Pdd"), bias-corrected (see ROLL_JERK_BIAS)
};

// JKFT_MATRIX has no intercept term, so its raw output isn't mean-zero.
// Measured steady-state mean of raw Pdd; subtracted below so roll_jerk is
// zero-centred for the AttitudePIDJerk damping term. Re-measure and update
// if the strain sensors are recalibrated or the fit is re-derived.
constexpr float ROLL_JERK_BIAS = 37.3646f; // rad/s^3

inline JerkEstimate estimate_jerk(const StrainRateRaw &strain, float p)
{
    constexpr float JKFT_MATRIX[5][2] = {
        {  -0.0613f,   -0.0578f}, // s0
        {   0.0842f,    0.0401f}, // s1
        {  -0.0122f,    0.0789f}, // s2
        {   0.0253f,   -0.0575f}, // s3
        {  -0.4319f,  -33.6420f}, // p
    };

    const float inputs[5] = { (float)strain.val[0], (float)strain.val[1],
                               (float)strain.val[2], (float)strain.val[3], p };

    JerkEstimate out{};
    for (uint8_t i = 0; i < 5; i++) {
        out.z_jerk    += inputs[i] * JKFT_MATRIX[i][0];
        out.roll_jerk += inputs[i] * JKFT_MATRIX[i][1];
    }
    out.roll_jerk -= ROLL_JERK_BIAS;
    return out;
}
