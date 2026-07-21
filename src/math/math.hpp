#pragma once
#include <cstdint>

/*
 * BPRL math utilities — scalar helpers, IIR filter, quaternion ops.
 *
 * Quaternion convention throughout: Hamilton scalar-first [W, X, Y, Z],
 * representing the rotation from NED frame to Body frame (q_NED→Body).
 * Kinematic propagation equation:  dq/dt = 0.5 * q ⊗ ω_pure
 * where ω_pure = {0, p, q, r} (body-frame angular velocity).
 */

/* ── Scalar helpers ──────────────────────────────────────────────────────── */

float degreesToRadians(float deg);

float constrain_float(float v, float lo, float hi);

// Wrap an angle (radians) into [-pi, pi].
float wrap_pi(float rad);

/* ── Signal processing ───────────────────────────────────────────────────── */

// Compute IIR lowpass coefficient: alpha = dt / (dt + 1 / (2*pi*fc))
// fc_hz <= 0 disables filtering (returns alpha = 1, i.e. passthrough).
float lowpass_alpha(float fc_hz, float dt_s);

// First-order IIR lowpass: y_k = alpha * x_k + (1-alpha) * y_{k-1}
float lowpass(float input, float prev_out, float alpha);

// State for the 2nd-order Butterworth biquad direct-form-II bilinear-transform delay elements, 
// matching ArduPilot's DigitalBiquadFilter.
struct Biquad2pState {
    float delay1      = 0.0f;
    float delay2      = 0.0f;
    bool  initialised = false;
};

// Second-order (2-pole) Butterworth IIR lowpass — ~-40 dB/decade roll-off
// above fc_hz, vs. ~-20 dB/decade for lowpass() above. Coefficients are
// recomputed from fc_hz/dt_s every call (same variable-dt tolerance as
// lowpass_alpha()). fc_hz <= 0 disables filtering (returns input unchanged).
float lowpass2p(float input, Biquad2pState& state, float fc_hz, float dt_s);

// Second-order notch (RBJ biquad form), tracking a moving center frequency —
// e.g. a motor's blade-pass frequency. Split into a coefficient step and an
// apply step so multiple axes sharing the same center_hz/bandwidth_hz/dt_s
// (e.g. p/q/r all tracking one motor-vibration frequency) pay the sinf()/
// cosf() cost once per tick rather than once per axis.

struct NotchCoeffs {
    float b0, b1, b2, a1, a2;
    bool  enabled;  // false when disabled: center_hz <= 0, bandwidth_hz <= 0, dt_s <= 0,
                    // or center_hz >= NOTCH_NYQUIST_CUTOFF * (1/dt_s) (see notch_coeffs())
};

// bandwidth_hz sets notch width (Q = center_hz/bandwidth_hz); coefficients
// are recomputed fresh every call from center_hz/dt_s, so the caller drives
// frequency tracking (and should slew-limit center_hz itself between calls
// to avoid a filter transient on a sudden frequency jump).
//
// Matches ArduPilot's HarmonicNotchFilter::set_center_frequency(): once
// center_hz reaches NOTCH_NYQUIST_CUTOFF (0.48) of the sample rate, the
// notch is disabled outright rather than clamped to the cutoff — clamping
// would keep notching at the wrong (stale) frequency once the real one has
// walked past Nyquist, which is worse than passing the signal through
// unfiltered. Re-enables automatically once center_hz drops back below the
// cutoff on a later call.
static constexpr float NOTCH_NYQUIST_CUTOFF = 0.48f;
NotchCoeffs notch_coeffs(float center_hz, float bandwidth_hz, float dt_s);

// Applies pre-computed notch coefficients to one axis's delay memory (same
// direct-form-II structure as lowpass2p()). Disabled coefficients return
// input unchanged and re-arm the state for next time the notch is enabled.
float notch_apply(float input, Biquad2pState& state, const NotchCoeffs& coeffs);

// Backward-difference numerical derivative
float derivative(float current, float prev, float dt_s);

// State for rpm_gate() below.
struct RpmGateState {
    uint32_t last_good  = 0;
    bool     seen_valid = false;
};

// RPM plausibility gate: motors never legitimately report spinning below
// ~100 RPM while armed, so a reading below that threshold — after having
// once seen a real (>100 RPM) reading — is treated as a missed telemetry
// frame rather than a real drop to near-zero, and the last good value is
// held instead. Before the first >100 RPM reading (pre-spool-up / disarmed),
// raw_rpm is returned unchanged since there's nothing valid to hold yet.
uint32_t rpm_gate(RpmGateState& state, uint32_t raw_rpm);

// Trapezoidal integration (stateless helper — caller owns the accumulator)
float integrate(float value, float dt_s);

/* ── 3-vector helpers ────────────────────────────────────────────────────── */

// out = a × b
void cross3(const float a[3], const float b[3], float out[3]);

/* ── Quaternion ──────────────────────────────────────────────────────────── */

struct Quat {
    float w, x, y, z;   // [W, X, Y, Z] scalar-first
};

// Hamilton product: a ⊗ b
Quat  quat_mul(const Quat& a, const Quat& b);

// Conjugate: q* = {w, -x, -y, -z}
Quat  quat_conj(const Quat& q);

// Enforce unit norm
Quat  quat_norm(const Quat& q);

// Scalar dot product (for antipodal check)
float quat_dot(const Quat& a, const Quat& b);

// Extract Euler angles (roll/pitch/yaw) from q_NED→Body.
// Convention: ZYX (yaw-pitch-roll), angles in radians.
void quat_to_euler(const Quat& q, float& roll, float& pitch, float& yaw);

// Build 3×3 rotation matrix R_body2ned (v_NED = R * v_body).
// For q = q_NED→Body: R_body2ned = R(q)^T  (conjugate rotation).
// R is stored row-major: R[row][col].
void quat_to_rot_body2ned(const Quat& q, float R[3][3]);

/* ── Fixed-size N×N matrix operations ───────────────────────────────────────
 * Template parameter Dim is the matrix dimension (compile-time constant).
 * C must not alias A or B in mat_mul.
 * ─────────────────────────────────────────────────────────────────────────── */

#include <cstring>

template<int Dim> inline void mat_mul(const float A[Dim][Dim], const float B[Dim][Dim], float C[Dim][Dim])
{
    memset(C, 0, Dim * Dim * sizeof(float));
    for (int i = 0; i < Dim; ++i)
        for (int k = 0; k < Dim; ++k) {
            if (A[i][k] == 0.0f) continue;
            for (int j = 0; j < Dim; ++j)
                C[i][j] += A[i][k] * B[k][j];
        }
}

template<int Dim> inline void mat_add(const float A[Dim][Dim], const float B[Dim][Dim], float C[Dim][Dim])
{
    for (int i = 0; i < Dim; ++i)
        for (int j = 0; j < Dim; ++j)
            C[i][j] = A[i][j] + B[i][j];
}

template<int Dim> inline void mat_trans(const float A[Dim][Dim], float AT[Dim][Dim])
{
    for (int i = 0; i < Dim; ++i)
        for (int j = 0; j < Dim; ++j)
            AT[j][i] = A[i][j];
}
