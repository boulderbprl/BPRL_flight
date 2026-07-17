#include "math.hpp"
#include <cmath>



/* ── Scalar helpers ──────────────────────────────────────────────────────── */

float constrain_float(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float degreesToRadians(float deg) {
    return deg * (M_PI / 180.0);
}

float wrap_pi(float rad)
{
    while (rad > (float)M_PI)  rad -= 2.0f * (float)M_PI;
    while (rad < -(float)M_PI) rad += 2.0f * (float)M_PI;
    return rad;
}
/* ── Signal processing ───────────────────────────────────────────────────── */

float lowpass_alpha(float fc_hz, float dt_s)
{
    if (fc_hz <= 0.0f) return 1.0f;  // disabled -> passthrough
    // alpha = dt / (dt + 1/(2*pi*fc))
    const float tau = 1.0f / (6.2831853f * fc_hz);
    return dt_s / (dt_s + tau);
}

float lowpass(float input, float prev_out, float alpha)
{
    return alpha * input + (1.0f - alpha) * prev_out;
}

float lowpass2p(float input, Biquad2pState& state, float fc_hz, float dt_s)
{
    if (fc_hz <= 0.0f || dt_s <= 0.0f) return input;  // disabled -> passthrough

    const float sample_freq = 1.0f / dt_s;
    const float cutoff      = fminf(fc_hz, sample_freq * 0.4f);  // stay under Nyquist

    // Bilinear-transform Butterworth biquad (Q = cos(pi/4)), same formula as
    // ArduPilot's DigitalBiquadFilter::compute_params.
    static constexpr float COS_PI_4 = 0.70710678f;
    const float fr  = sample_freq / cutoff;
    const float ohm = tanf(3.14159265f / fr);
    const float c   = 1.0f + 2.0f * COS_PI_4 * ohm + ohm * ohm;

    const float b0 = ohm * ohm / c;
    const float b1 = 2.0f * b0;
    const float b2 = b0;
    const float a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    const float a2 = (1.0f - 2.0f * COS_PI_4 * ohm + ohm * ohm) / c;

    if (!state.initialised) {
        state.delay1 = state.delay2 = input * (1.0f / (1.0f + a1 + a2));
        state.initialised = true;
    }

    const float delay0 = input - state.delay1 * a1 - state.delay2 * a2;
    const float output = delay0 * b0 + state.delay1 * b1 + state.delay2 * b2;

    state.delay2 = state.delay1;
    state.delay1 = delay0;

    return output;
}

NotchCoeffs notch_coeffs(float center_hz, float bandwidth_hz, float dt_s)
{
    if (center_hz <= 0.0f || bandwidth_hz <= 0.0f || dt_s <= 0.0f) {
        return NotchCoeffs{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
    }

    const float sample_freq = 1.0f / dt_s;
    const float f0 = fminf(center_hz, sample_freq * 0.48f);  // stay under Nyquist
    const float Q  = f0 / bandwidth_hz;

    const float w0    = 6.2831853f * f0 / sample_freq;
    const float alpha = sinf(w0) / (2.0f * Q);
    const float cosw0 = cosf(w0);

    const float a0_inv = 1.0f / (1.0f + alpha);
    NotchCoeffs c;
    c.b0 =  a0_inv;
    c.b1 = -2.0f * cosw0 * a0_inv;
    c.b2 =  a0_inv;
    c.a1 = -2.0f * cosw0 * a0_inv;
    c.a2 = (1.0f - alpha) * a0_inv;
    c.enabled = true;
    return c;
}

float notch_apply(float input, Biquad2pState& state, const NotchCoeffs& coeffs)
{
    if (!coeffs.enabled) {
        state.initialised = false;  // re-arm a clean start if the notch re-enables later
        return input;
    }

    if (!state.initialised) {
        state.delay1 = state.delay2 = 0.0f;
        state.initialised = true;
    }

    const float delay0 = input - state.delay1 * coeffs.a1 - state.delay2 * coeffs.a2;
    const float output = delay0 * coeffs.b0 + state.delay1 * coeffs.b1 + state.delay2 * coeffs.b2;

    state.delay2 = state.delay1;
    state.delay1 = delay0;

    return output;
}

float derivative(float current, float prev, float dt_s)
{
    return (current - prev) / dt_s;
}

float integrate(float value, float dt_s)
{
    return value * dt_s;
}

uint32_t rpm_gate(RpmGateState& state, uint32_t raw_rpm)
{
    static constexpr uint32_t RPM_VALID_THRESHOLD = 100;

    if (raw_rpm > RPM_VALID_THRESHOLD) {
        state.seen_valid = true;
        state.last_good  = raw_rpm;
        return raw_rpm;
    }
    if (state.seen_valid) {
        return state.last_good;
    }
    return raw_rpm;
}

/* ── 3-vector helpers ────────────────────────────────────────────────────── */

void cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

/* ── Quaternion ──────────────────────────────────────────────────────────── */

Quat quat_mul(const Quat& a, const Quat& b)
{
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

Quat quat_conj(const Quat& q)
{
    return { q.w, -q.x, -q.y, -q.z };
}

Quat quat_norm(const Quat& q)
{
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-10f) return { 1.0f, 0.0f, 0.0f, 0.0f };  // guard near-zero
    float inv = 1.0f / n;
    return { q.w*inv, q.x*inv, q.y*inv, q.z*inv };
}

float quat_dot(const Quat& a, const Quat& b)
{
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

void quat_to_euler(const Quat& q, float& roll, float& pitch, float& yaw)
{
    // ZYX Euler from q_NED→Body [W, X, Y, Z] (Hamilton, scalar-first).
    // roll  = atan2(2(WX + YZ), 1 - 2(X² + Y²))
    // pitch = asin( 2(WY - ZX) )
    // yaw   = atan2(2(WZ + XY), 1 - 2(Y² + Z²))
    const float w = q.w, x = q.x, y = q.y, z = q.z;

    roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));

    float sp = 2.0f*(w*y - z*x);
    sp = constrain_float(sp, -1.0f, 1.0f);
    pitch = asinf(sp);

    yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
}

void quat_to_rot_body2ned(const Quat& q, float R[3][3])
{
    // For q_NED→Body, the rotation matrix that maps body vectors to NED is
    // R_body2ned = R(q)^T = R(q*).
    // Using q* = {w, -x, -y, -z}, the DCM entries are:
    const float w = q.w, x = q.x, y = q.y, z = q.z;

    R[0][0] = 1.0f - 2.0f*(y*y + z*z);
    R[0][1] = 2.0f*(x*y - w*z);
    R[0][2] = 2.0f*(x*z + w*y);

    R[1][0] = 2.0f*(x*y + w*z);
    R[1][1] = 1.0f - 2.0f*(x*x + z*z);
    R[1][2] = 2.0f*(y*z - w*x);

    R[2][0] = 2.0f*(x*z - w*y);
    R[2][1] = 2.0f*(y*z + w*x);
    R[2][2] = 1.0f - 2.0f*(x*x + y*y);
}
