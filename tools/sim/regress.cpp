// Refactor regression: drives PosControl::update() and AltControl::alt_hold()
// (the stick paths ALT_HOLD and POS_HOLD fly) closed-loop against a simple
// plant and prints every output at full precision. Built once against a
// snapshot of the sources and once against the working tree; the two outputs
// must be identical. See tools/sim/run.sh.
#include "src/controllers/PosControl.hpp"
#include "src/controllers/AltControl.hpp"
#include "configs/DroneConfig.hpp"
#include "src/uptime.hpp"
#include "src/threads.hpp"
#include <cmath>
#include <cstdio>

uint64_t g_sim_time_us = 1000000;

int main()
{
    PosControl pos(kDroneConfig.pos);
    AltControl alt(kDroneConfig.alt);

    const float dt = CONTROL_DT_S, g = 9.80665f, lag = 0.15f;
    float p[3] = { 0.3f, -0.2f, -1.0f }, v[3] = { 0.4f, 0.0f, 0.0f };
    float roll = 0.0f, pitch = 0.0f, yaw = 0.6f;

    for (int k = 0; k < 400 * 40; ++k) {
        const float t = k * dt;
        g_sim_time_us += CONTROL_PERIOD_US;

        // Stick script: pushes, releases, a diagonal, and throttle moves.
        float fwd = 0.0f, right = 0.0f, thr = 0.5f;
        if (t > 2.0f  && t < 5.0f)  fwd = 0.8f;
        if (t > 8.0f  && t < 10.0f) right = -0.6f;
        if (t > 13.0f && t < 16.0f) { fwd = -1.0f; right = 0.5f; }
        if (t > 20.0f && t < 22.0f) fwd = 0.07f;            // inside the deadband
        if (t > 4.0f  && t < 7.0f)  thr = 0.8f;
        if (t > 15.0f && t < 18.0f) thr = 0.25f;
        if (t > 25.0f && t < 30.0f) yaw += 0.5f * dt;       // heading change mid-hold
        if (k == 400 * 32) { pos.reset_all(); alt.reset_all(); }

        const float state9[9] = { p[0], p[1], p[2], roll, pitch, yaw, v[0], v[1], v[2] };
        float att[2];
        pos.update(state9, fwd, right, att);
        const float throttle = alt.alt_hold(thr, p[2], v[2]);

        printf("%.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", att[0], att[1], throttle,
               pos.pos_tgt(0), pos.pos_tgt(1), pos.vel_tgt(0), pos.vel_tgt(1), alt.climb_rate_tgt());

        // Plant: first-order lean response, point mass.
        roll  += (att[0] - roll)  * dt / lag;
        pitch += (att[1] - pitch) * dt / lag;
        const float a_fwd = -g * tanf(pitch), a_right = g * tanf(roll);
        const float aN = cosf(yaw) * a_fwd - sinf(yaw) * a_right;
        const float aE = sinf(yaw) * a_fwd + cosf(yaw) * a_right;
        const float aD = g * (1.0f - throttle / 0.4f);
        v[0] += aN * dt; v[1] += aE * dt; v[2] += aD * dt;
        p[0] += v[0] * dt; p[1] += v[1] * dt; p[2] += v[2] * dt;
    }
    return 0;
}
