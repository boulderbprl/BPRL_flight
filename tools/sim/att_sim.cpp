// Attitude-loop simulation: the real FlightStateMachine (STABILIZE) and every
// attitude controller the drone config builds, flying a rotational plant with
// models of the sensors those controllers use — gyro rate, strain p_dot/q_dot,
// and motor RPM through the real Unmixer. Defaults are Drone2 (S500) as
// identified from the 2026-11-05 controller-comparison flight
// (Controller_testing/Flight1_nov5). See tools/sim/README.md, "Attitude sim".
//
// Modes:
//   att_sim margins [axis]                loop-break frequency response at the actuator,
//                                         per controller slot: crossover, PM, GM, delay
//                                         margin, peak sensitivity
//   att_sim robust                        time-domain: extra dead time / plant-gain scale
//                                         each controller survives
//   att_sim step                          20 deg roll and pitch steps: overshoot, settling
//   att_sim dist                          hover under a random external torque, sensor
//                                         noise on: attitude error and command rms
//   att_sim replay <sticks.csv> <slot> <out.csv>
//                                         fly logged sticks (t,roll,pitch,yaw columns,
//                                         normalised [-1,1]) and write the response
//
// Every plant / sensor number can be overridden from the environment
// (K_R=140 att_sim margins). Names are listed in struct Model.
#include "src/controllers/FlightStateMachine.hpp"
#include "src/controllers/extras/MotorMixer.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include "src/threads.hpp"
#include "src/uptime.hpp"
#include "configs/DroneConfig.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <complex>
#include <string>
#include <vector>
#include <functional>

uint64_t g_sim_time_us = 1000000;

static const float DT = CONTROL_DT_S;
static const float G = 9.80665f;
static const float R2D = 57.29578f;

static float envf(const char *k, float d) { const char *v = getenv(k); return v ? (float)atof(v) : d; }

// ── Model parameters (env-overridable) ───────────────────────────────────────
// Identified against the logged command (OUTP) with the attitude target as an
// instrument, 0.7-8 Hz, 2026-11-05 flight:
//   command -> gyro:     K / (s (tau s + 1)) e^{-sT}, through the 20 Hz 2-pole gyro LPF
//   command -> unmixer:  ~5.5-6.2 N·m per unit, same lag, ~14 ms (incl. the 30 Hz RPM pole)
//   command -> strain:   K/(tau s+1) (a + c/s) e^{-sTs}   (a + c/s: c·rate leaks in)
struct Model {
    float K[2]      = { envf("K_R", 131.0f), envf("K_P", 117.0f) };    // rad/s² per unit command
    float K_YAW     = envf("K_Y", 10.0f);
    float TAU       = envf("TAU", 0.045f);                             // motor lag [s]
    int   DEAD      = (int)envf("DEAD", 4);                            // command dead time [ticks of 2.5 ms]
    float GU        = envf("GU", 5.8f);                                // real torque the RPMs show, N·m per unit command
    int   ENC_DELAY = (int)envf("ENC_DELAY", 1);                       // RPM sample age [ticks]
    float RPM_NOISE = envf("RPM_NOISE", 15.0f);                        // rpm, uniform ±/2 per motor per tick
    float SA[2]     = { envf("SA_R", 0.91f), envf("SA_P", 0.74f) };    // strain gain on true p_dot/q_dot
    float SC[2]     = { envf("SC_R", -0.2f), envf("SC_P", -2.9f) };    // strain leak of true rate [rad/s² per rad/s]
    int   SDEL[2]   = { (int)envf("SD_R", 2), (int)envf("SD_P", 4) };  // strain delay after the true accel [ticks]
    float SNOISE    = envf("SNOISE", 0.7f);                            // strain white noise rms [rad/s²]
    float GNOISE    = envf("GNOISE", 0.03f);                           // gyro white noise rms [rad/s] before the LPF (flight: 0.3-0.4 dps at 12-45 Hz)
    float GLPF      = envf("GLPF", 20.0f), RLPF = envf("RLPF", 5.0f);  // StateManager rate LPFs (2-pole)
    float TDIST     = envf("TDIST", 0.0f);                             // external torque rms [N·m], ~1.5 Hz band
    float INERTIA   = envf("INERTIA", 0.045f);                         // only scales TDIST into rad/s²
};

// ── Rotational plant + sensors ───────────────────────────────────────────────
struct Plant {
    Model m;
    float roll = 0, pitch = 0, yaw = 0, p = 0, q = 0, r = 0;
    float acc[3] = { 0, 0, 0 };            // true angular acceleration
    float mot[3] = { 0, 0, 0 };            // command after dead time and motor lag
    float k_scale = 1.0f;                  // robustness tests: multiplies the true plant gain
    int   extra_dead = 0;                  // robustness tests: added command dead time [ticks]
    float inject[3] = { 0, 0, 0 };         // loop-break injection, added to the command

    static constexpr int H = 256;
    float cq[H][3] = {};
    float aq[H][2] = {}, rq[H][2] = {};    // true accel / rate history for the strain model
    uint32_t rq_rpm[H][4] = {};
    long  n = 0;
    Biquad2pState gs[3] = {};
    float rate_meas[3] = { 0, 0, 0 }, strain[2] = { 0, 0 };
    uint32_t rpm[4] = { 0, 0, 0, 0 };
    float dist[2] = { 0, 0 }, dist_f[2] = { 0, 0 };
    unsigned seed = 777u;
    float urand() { seed = seed * 1103515245u + 12345u; return ((seed >> 8) & 0xffff) / 65535.0f - 0.5f; }   // uniform, rms 0.289

    static float rpm_of(float F)
    {
        const auto &c = kDroneConfig.unmixer;
        float w = 6000.0f;
        for (int i = 0; i < 20; ++i) {
            const float f  = ((c.motor_c3 * w + c.motor_c2) * w + c.motor_c1) * w + c.motor_c0 - F;
            const float df = (3 * c.motor_c3 * w + 2 * c.motor_c2) * w + c.motor_c1;
            w -= f / df;
        }
        return w < 0 ? 0 : w;
    }

    // u_ctrl: controller output this tick. Returns the command the plant got (u_ctrl + injection).
    void step(const float u_ctrl[3], float u_plant[3])
    {
        for (int i = 0; i < 3; ++i) u_plant[i] = u_ctrl[i] + inject[i];
        float *slot = cq[n % H];
        for (int i = 0; i < 3; ++i) slot[i] = u_plant[i];
        const float *d = cq[(n - m.DEAD - extra_dead + 4 * H) % H];
        const float a = DT / (m.TAU + DT);
        for (int i = 0; i < 3; ++i) mot[i] += a * (d[i] - mot[i]);

        for (int i = 0; i < 2; ++i) {
            const float al = DT / (1.0f / (6.2832f * 1.5f) + DT);
            dist_f[i] += al * (urand() * m.TDIST * 45.6f - dist_f[i]);   // 45.6: makes TDIST the rms of dist[]
            dist[i]   += al * (dist_f[i] - dist[i]);
        }
        acc[0] = k_scale * m.K[0] * mot[0] + dist[0] / m.INERTIA;
        acc[1] = k_scale * m.K[1] * mot[1] + dist[1] / m.INERTIA;
        acc[2] = k_scale * m.K_YAW * mot[2];
        p += acc[0] * DT; q += acc[1] * DT; r += acc[2] * DT;
        const float sr = sinf(roll), cr = cosf(roll), cp = cosf(pitch), tp = tanf(pitch);
        roll  += (p + (q * sr + r * cr) * tp) * DT;
        pitch += (q * cr - r * sr) * DT;
        yaw    = wrap_pi(yaw + ((q * sr + r * cr) / cp) * DT);

        // Gyro: StateManager's 2-pole Butterworth (the firmware's own lowpass2p)
        const float tr[3] = { p, q, r };
        for (int i = 0; i < 3; ++i)
            rate_meas[i] = lowpass2p(tr[i] + urand() * 3.464f * m.GNOISE, gs[i], i < 2 ? m.GLPF : m.RLPF, DT);

        // Strain p_dot/q_dot, end to end (what ATT.Pdot/Qdot logged): a·accel + c·rate, delayed, + noise
        float *ah = aq[n % H], *rh = rq[n % H];
        ah[0] = acc[0]; ah[1] = acc[1]; rh[0] = p; rh[1] = q;
        for (int i = 0; i < 2; ++i) {
            const long k = (n - m.SDEL[i] + 4 * H) % H;
            strain[i] = m.SA[i] * aq[k][i] + m.SC[i] * rq[k][i] + urand() * 3.464f * m.SNOISE;
        }

        // Per-motor RPM from the real torque (GU N·m per unit of lagged command), sampled ENC_DELAY late
        {
            const float L  = kDroneConfig.unmixer.arm_length_m * 0.70710678f;
            const float Fb = 2.7f * G / 4.0f;
            const float t_r = m.GU * mot[0] * k_scale, t_p = m.GU * mot[1] * k_scale;
            static const float sgr[4] = { -1, +1, +1, -1 }, sgp[4] = { +1, -1, +1, -1 };
            uint32_t *now = rq_rpm[n % H];
            for (int i = 0; i < 4; ++i)
                now[i] = (uint32_t)lroundf(rpm_of(Fb + (sgr[i] * t_r + sgp[i] * t_p) / (4.0f * L)) + urand() * m.RPM_NOISE);
            const uint32_t *old = rq_rpm[(n - m.ENC_DELAY + 4 * H) % H];
            memcpy(rpm, old, sizeof(rpm));
        }
        ++n;
    }

    void fill(float state_full[StateIdx::N], float euler[3]) const
    {
        memset(state_full, 0, sizeof(float) * StateIdx::N);
        state_full[StateIdx::Z_POS] = -1.0f;
        state_full[StateIdx::P] = rate_meas[0]; state_full[StateIdx::Q] = rate_meas[1]; state_full[StateIdx::R] = rate_meas[2];
        state_full[StateIdx::P_DOT] = strain[0]; state_full[StateIdx::Q_DOT] = strain[1]; state_full[StateIdx::R_DOT] = acc[2];
        euler[0] = roll; euler[1] = pitch; euler[2] = yaw;
    }
};

// ── Plant + FlightStateMachine in STABILIZE ──────────────────────────────────
struct Sim {
    Plant plant;
    FlightStateMachine fsm{ kDroneConfig };
    MotorMixer mixer{ kDroneConfig.mixer };   // only for its saturation flags (rate-I anti-windup), as ControlThread
    int   slot = 0;
    float sticks[3] = { 0, 0, 0 };      // roll, pitch, yaw, normalised
    float thr = 0.55f;
    float u_ctrl[3] = { 0, 0, 0 }, u_plant[3] = { 0, 0, 0 };
    float state_full[StateIdx::N], euler[3];
    float t = 0;

    explicit Sim(int s) : slot(s) { plant.fill(state_full, euler); }

    void tick()
    {
        g_sim_time_us += CONTROL_PERIOD_US;
        t += DT;
        plant.fill(state_full, euler);
        float in[InputIdx::N_INPUTS] = {};
        in[InputIdx::THRUST] = thr;
        in[InputIdx::ROLL_TGT] = sticks[0]; in[InputIdx::PITCH_TGT] = sticks[1]; in[InputIdx::YAW_RATE] = sticks[2];
        in[InputIdx::FLIGHT_MODE] = -1.0f;   // STABILIZE
        fsm.set_indi_sensor_health(SensorHealth::GOOD);
        fsm.set_active_controller(slot);
        float thrust;
        fsm.update(state_full, euler, in, true, true, plant.rpm, 0.0f, u_ctrl, thrust);
        // The real mixer's limit flags feed the rate integrators next tick. Without
        // this they read "limited" (MixerLimits' default) and PID/PID_PI rate I never runs.
        const float safety[2] = { euler[0], euler[1] };
        int32_t motor_out[4];
        mixer.update(u_ctrl, thrust, true, safety, motor_out);
        fsm.set_mixer_limits(mixer.limits());
        plant.step(u_ctrl, u_plant);
    }
    void run(float sec, const std::function<void()> &each = nullptr)
    {
        const int k = (int)lroundf(sec / DT);
        for (int i = 0; i < k; ++i) { tick(); if (each) each(); }
    }
    // Arm, push the throttle past the takeoff threshold, finish the spool-up, settle.
    void airborne() { run(3.0f); }
};

static const char *slot_name(int s)
{
    switch (kDroneConfig.controllers.switch_pos[s]) {
        case AttCtrl::PID:      return "PID";
        case AttCtrl::INDI:     return "INDI";
        case AttCtrl::PID_PI:   return "PID_PI";
        case AttCtrl::PID_JERK: return "PID_JERK";
        case AttCtrl::JINDI:    return "JINDI";
        default:                return "?";
    }
}
// The distinct controllers on the switch, as slot numbers.
static std::vector<int> slots()
{
    std::vector<int> v;
    for (int s = 0; s < 3; ++s) {
        bool dup = false;
        for (int o : v) dup |= kDroneConfig.controllers.switch_pos[o] == kDroneConfig.controllers.switch_pos[s];
        if (!dup) v.push_back(s);
    }
    return v;
}

// ── Margins: break the loop at the actuator ──────────────────────────────────
// Inject d = A sin(wt) at the plant input on one axis. With u_c the controller
// output and u_p = u_c + d what the plant receives, the loop transfer function
// at the actuator is L = -U_c / U_p (fundamental, over whole periods). All
// controller internals (filters, integrators, INDI's RPM torque feedback,
// PID_PI's strain inner loop) are inside L.
static std::complex<double> loop_at(int slot, int axis, float f, float amp)
{
    Sim s(slot);
    Model &m = s.plant.m;
    m.SNOISE = 0; m.GNOISE = 0; m.RPM_NOISE = 0; m.TDIST = 0;
    s.airborne();
    const double w = 2 * M_PI * f;
    const int per = (int)ceil(1.0 / (f * DT));
    const int settle = std::max(per * 2, (int)(1.5f / DT));
    const int ncyc = std::max(3, (int)ceil(2.0 * f));
    const int nmeas = (int)lround(ncyc / (f * DT));
    std::complex<double> Uc = 0, Up = 0;
    for (int k = 0; k < settle + nmeas; ++k) {
        const double tt = k * DT;
        s.plant.inject[axis] = amp * (float)sin(w * tt);
        s.tick();
        if (k >= settle) {
            const std::complex<double> e = std::exp(std::complex<double>(0, -w * tt));
            Uc += (double)s.u_ctrl[axis] * e;
            Up += (double)s.u_plant[axis] * e;
        }
        if (fabsf(s.plant.roll) > 1.0f || fabsf(s.plant.pitch) > 1.0f) return std::complex<double>(NAN, NAN);
    }
    return -Uc / Up;
}

struct Margins { double fc, pm, f180, gm_db, dm_ms, ms; bool ok; };

static Margins margins(int slot, int axis, bool print_bode)
{
    const float amp = envf("AMP", 0.01f);
    std::vector<double> fs, mag, ph;
    for (double f = 0.3; f <= 25.0; f *= 1.08) {
        const auto L = loop_at(slot, axis, (float)f, amp);
        if (std::isnan(L.real())) continue;
        fs.push_back(f); mag.push_back(std::abs(L)); ph.push_back(std::arg(L) * R2D);
    }
    // unwrap phase starting from the low-frequency value
    for (size_t i = 1; i < ph.size(); ++i) { while (ph[i] - ph[i - 1] > 180) ph[i] -= 360; while (ph[i] - ph[i - 1] < -180) ph[i] += 360; }
    Margins r = { NAN, NAN, NAN, NAN, NAN, 0, true };
    for (size_t i = 0; i < fs.size(); ++i) {
        const std::complex<double> L = std::polar(mag[i], ph[i] / R2D);
        r.ms = std::max(r.ms, 1.0 / std::abs(1.0 + L));
    }
    for (size_t i = 1; i < fs.size(); ++i) {
        if (std::isnan(r.fc) && mag[i - 1] >= 1.0 && mag[i] < 1.0) {   // first gain crossover
            const double a = (log(mag[i - 1])) / (log(mag[i - 1]) - log(mag[i]));
            r.fc = exp(log(fs[i - 1]) + a * (log(fs[i]) - log(fs[i - 1])));
            const double phc = ph[i - 1] + a * (ph[i] - ph[i - 1]);
            r.pm = 180.0 + phc;
            while (r.pm > 180) r.pm -= 360;
            while (r.pm < -180) r.pm += 360;
            r.dm_ms = r.pm / (360.0 * r.fc) * 1000.0;
        }
        // first phase crossing of -180 (mod 360) above the crossover
        const double p0 = ph[i - 1] + 180.0, p1 = ph[i] + 180.0;
        if (!std::isnan(r.fc) && fs[i] > r.fc && std::isnan(r.f180) &&
            floor(p0 / 360.0) != floor(p1 / 360.0)) {
            const double edge = 360.0 * std::max(floor(p0 / 360.0), floor(p1 / 360.0));
            const double a = (p0 - edge) / (p0 - p1);
            r.f180 = exp(log(fs[i - 1]) + a * (log(fs[i]) - log(fs[i - 1])));
            const double mg = exp(log(mag[i - 1]) + a * (log(mag[i]) - log(mag[i - 1])));
            r.gm_db = -20.0 * log10(mg);
        }
    }
    if (print_bode) {
        printf("      f[Hz]   |L|     phase[deg]\n");
        for (size_t i = 0; i < fs.size(); i += 2) printf("      %6.2f  %7.3f  %8.1f\n", fs[i], mag[i], ph[i]);
    }
    return r;
}

static void mode_margins(int only_axis)
{
    printf("Loop at the actuator (STABILIZE, angle + rate loops closed), %s\n", "plant/sensor model as printed by `att_sim model`");
    printf("%-8s %-5s %8s %8s %9s %9s %10s %6s\n", "ctrl", "axis", "fc[Hz]", "PM[deg]", "f180[Hz]", "GM[dB]", "DM[ms]", "Ms");
    const bool bode = getenv("BODE") != nullptr;
    for (int s : slots())
        for (int ax = 0; ax < 2; ++ax) {
            if (only_axis >= 0 && ax != only_axis) continue;
            const Margins r = margins(s, ax, bode);
            printf("%-8s %-5s %8.2f %8.1f %9.2f %9.1f %10.1f %6.2f\n", slot_name(s), ax ? "pitch" : "roll",
                   r.fc, r.pm, r.f180, r.gm_db, r.dm_ms, r.ms);
        }
}

// ── Robustness: time-domain limits ───────────────────────────────────────────
// A 5 deg step on one axis, then 6 s; "stable" means the attitude error decays
// (last-second peak < 1 deg and nothing ever past 45 deg).
static bool survives(int slot, int axis, int extra_dead, float k_scale)
{
    Sim s(slot);
    s.plant.m.TDIST = 0;
    s.airborne();
    s.plant.extra_dead = extra_dead; s.plant.k_scale = k_scale;
    s.sticks[axis] = 5.0f / (kDroneConfig.controllers.stick_max_angle_deg);
    bool ok = true; float late = 0; int k = 0;
    const float tgt = 5.0f / R2D;
    s.run(6.0f, [&] {
        const float e = (axis ? s.plant.pitch : s.plant.roll) - tgt;
        if (fabsf(s.plant.roll) > 0.8f || fabsf(s.plant.pitch) > 0.8f) ok = false;
        if (++k > (int)(5.0f / DT)) late = fmaxf(late, fabsf(e));
    });
    return ok && late * R2D < 1.0f;
}

static void mode_robust()
{
    printf("Time-domain limits (5 deg step, STABILIZE). Extra delay = added actuator dead time;\n"
           "K low/high = plant-gain scale at which it stops settling.\n");
    printf("%-8s %-5s %14s %10s %10s\n", "ctrl", "axis", "extra delay[ms]", "K low", "K high");
    for (int s : slots())
        for (int ax = 0; ax < 2; ++ax) {
            int d = 0;
            while (d < 80 && survives(s, ax, d + 1, 1.0f)) ++d;
            float lo = 1.0f, hi = 1.0f;
            while (lo > 0.1f && survives(s, ax, 0, lo - 0.05f)) lo -= 0.05f;
            while (hi < 6.0f && survives(s, ax, 0, hi + 0.1f)) hi += 0.1f;
            printf("%-8s %-5s %14.1f %10.2f %10.2f\n", slot_name(s), ax ? "pitch" : "roll",
                   d * DT * 1000.0f, lo, hi);
        }
}

// ── Steps ─────────────────────────────────────────────────────────────────────
static void mode_step()
{
    printf("20 deg step (STABILIZE), per controller and axis\n");
    printf("%-8s %-5s %9s %12s %12s %14s\n", "ctrl", "axis", "rise[ms]", "overshoot[%]", "settle2%[s]", "peak rate[dps]");
    const float A = 20.0f;
    for (int s : slots())
        for (int ax = 0; ax < 2; ++ax) {
            Sim sim(s);
            sim.airborne();
            sim.sticks[ax] = A / kDroneConfig.controllers.stick_max_angle_deg;
            float t10 = -1, t90 = -1, peak = 0, rate = 0, last_out = 0;
            const float t0 = sim.t;
            sim.run(4.0f, [&] {
                const float y = (ax ? sim.plant.pitch : sim.plant.roll) * R2D, tt = sim.t - t0;
                if (t10 < 0 && y > 0.1f * A) t10 = tt;
                if (t90 < 0 && y > 0.9f * A) t90 = tt;
                peak = fmaxf(peak, y);
                rate = fmaxf(rate, fabsf(ax ? sim.plant.q : sim.plant.p) * R2D);
                if (fabsf(y - A) > 0.02f * A) last_out = tt;
            });
            printf("%-8s %-5s %9.0f %12.1f %12.2f %14.0f\n", slot_name(s), ax ? "pitch" : "roll",
                   (t90 - t10) * 1000.0f, (peak - A) / A * 100.0f, last_out, rate);
        }
}

// ── Disturbance rejection and noise ──────────────────────────────────────────
// Hands-off hover with a random external torque on both axes (TDIST N·m rms,
// default 0.10 — the 2026-11-04 leaf-blower test measured 0.13 roll / 0.11
// pitch) and every sensor noise on. Reports the attitude error and how hard the
// controller works (command rms — mostly noise amplification above ~10 Hz).
static void mode_dist()
{
    const float td = envf("TDIST", 0.10f);
    printf("Hover, %.2f N·m rms external torque, sensor noise on, 30 s\n", td);
    printf("%-8s %14s %14s %12s %12s %12s %12s\n", "ctrl", "roll err[deg]", "pitch err[deg]", "u_r rms", "u_p rms", "u_r >12Hz", "u_p >12Hz");
    for (int s : slots()) {
        Sim sim(s);
        sim.plant.m.TDIST = td;
        sim.airborne();
        double er = 0, ep = 0, ur = 0, up = 0, hr = 0, hp = 0; int c = 0; bool bad = false;
        Biquad2pState lr = {}, lp = {};   // u minus its 12 Hz low-pass = the noise-driven part
        sim.run(30.0f, [&] {
            er += sim.plant.roll * sim.plant.roll; ep += sim.plant.pitch * sim.plant.pitch;
            ur += sim.u_ctrl[0] * sim.u_ctrl[0]; up += sim.u_ctrl[1] * sim.u_ctrl[1]; ++c;
            const float a = sim.u_ctrl[0] - lowpass2p(sim.u_ctrl[0], lr, 12.0f, DT);
            const float b = sim.u_ctrl[1] - lowpass2p(sim.u_ctrl[1], lp, 12.0f, DT);
            hr += a * a; hp += b * b;
            bad |= fabsf(sim.plant.roll) > 0.8f || fabsf(sim.plant.pitch) > 0.8f;
        });
        printf("%-8s %14.2f %14.2f %12.4f %12.4f %12.4f %12.4f%s\n", slot_name(s), sqrt(er / c) * R2D, sqrt(ep / c) * R2D,
               sqrt(ur / c), sqrt(up / c), sqrt(hr / c), sqrt(hp / c), bad ? "  DIVERGED" : "");
    }
}

// ── Replay logged sticks ──────────────────────────────────────────────────────
static void mode_replay(const char *in_csv, int slot, const char *out_csv)
{
    FILE *fi = fopen(in_csv, "r");
    if (!fi) { fprintf(stderr, "can't open %s\n", in_csv); exit(2); }
    std::vector<float> T, R, P, Y;
    char line[512];
    if (!fgets(line, sizeof(line), fi)) exit(2);   // header
    while (fgets(line, sizeof(line), fi)) {
        float t, r, p, y;
        if (sscanf(line, "%f,%f,%f,%f", &t, &r, &p, &y) == 4) { T.push_back(t); R.push_back(r); P.push_back(p); Y.push_back(y); }
    }
    fclose(fi);
    FILE *fo = fopen(out_csv, "w");
    fprintf(fo, "t,roll,pitch,roll_tgt,pitch_tgt,p,q,p_meas,q_meas,pdot,qdot,u_r,u_p,unm_r,unm_p,nu_r,nu_p\n");
    Sim s(slot);
    s.plant.m.TDIST = envf("TDIST", 0.0f);
    s.airborne();
    const float scale = kDroneConfig.controllers.stick_max_angle_deg / R2D;
    size_t j = 0;
    const float t0 = T.front();
    int k = 0;
    while (j + 1 < T.size()) {
        const float tt = t0 + k * DT;
        while (j + 1 < T.size() && T[j + 1] <= tt) ++j;
        s.sticks[0] = R[j]; s.sticks[1] = P[j]; s.sticks[2] = Y[j];
        s.tick();
        float di[10];
        s.fsm.get_indi_diag(di);
        if (k % 4 == 0)
            fprintf(fo, "%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f,%.3f,%.3f\n", tt,
                    s.plant.roll, s.plant.pitch, R[j] * scale, P[j] * scale, s.plant.p, s.plant.q,
                    s.state_full[StateIdx::P], s.state_full[StateIdx::Q],
                    s.state_full[StateIdx::P_DOT], s.state_full[StateIdx::Q_DOT], s.u_plant[0], s.u_plant[1],
                    di[0], di[1], di[6], di[7]);
        ++k;
        if (j + 1 >= T.size()) break;
    }
    fclose(fo);
}

static void print_model()
{
    Model m;
    printf("plant  K roll %.0f pitch %.0f rad/s²/unit, motor lag %.0f ms, dead %d ticks (%.1f ms)\n",
           m.K[0], m.K[1], m.TAU * 1000, m.DEAD, m.DEAD * DT * 1000);
    printf("rpm    %.2f N·m/unit through the Unmixer (rpm_lpf_hz %.0f), %d tick(s) late\n",
           m.GU, kDroneConfig.ang_accel_filt.rpm_lpf_hz, m.ENC_DELAY);
    printf("strain roll %.2f·p_dot %+.2f·p, %d ticks | pitch %.2f·q_dot %+.2f·q, %d ticks | noise %.2f rad/s²\n",
           m.SA[0], m.SC[0], m.SDEL[0], m.SA[1], m.SC[1], m.SDEL[1], m.SNOISE);
    printf("switch slots: [%s, %s, %s]\n", slot_name(0), slot_name(1), slot_name(2));
}

int main(int argc, char **argv)
{
    const std::string mode = argc > 1 ? argv[1] : "margins";
    print_model();
    if (mode == "model") return 0;
    if (mode == "margins") { mode_margins(argc > 2 ? atoi(argv[2]) : -1); return 0; }
    if (mode == "robust")  { mode_robust(); return 0; }
    if (mode == "step")    { mode_step(); return 0; }
    if (mode == "dist")    { mode_dist(); return 0; }
    if (mode == "replay" && argc > 4) { mode_replay(argv[2], atoi(argv[3]), argv[4]); return 0; }
    fprintf(stderr, "usage: att_sim margins [0|1] | robust | step | dist | model | replay <sticks.csv> <slot> <out.csv>\n");
    return 2;
}
