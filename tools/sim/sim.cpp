// Host flight simulation of the POS_HOLD guidance stack: the real
// FlightStateMachine (attitude controller, AltControl, PosControl,
// HeadingControl, TrajectoryTracker) flying a simple rigid-body quadrotor.
// Each scenario checks pass/fail limits and writes a CSV to the output
// directory. See tools/sim/README.md.
//
// The plant is deliberately simple (torque → angular acceleration, thrust
// along body −z, hover at the drone config's hover_thr, first-order motor lag,
// linear drag, a steady wind). It is good for catching logic errors, sign
// errors, target jumps and gross mistuning — not for final gains.
#include "src/controllers/FlightStateMachine.hpp"
#include "src/controllers/HeadingControl.hpp"
#include "src/FlightState.hpp"
#include "src/math/math.hpp"
#include "src/threads.hpp"
#include "src/uptime.hpp"
#include "configs/DroneConfig.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <functional>

uint64_t g_sim_time_us = 1000000;

static const float DT = CONTROL_DT_S;
static const float G  = 9.80665f;
static const float RAD2DEG = 57.29578f;
static const float NANF = NAN;

static std::string g_out_dir = ".";
static int g_failures = 0;

static void check(bool ok, const char *what, double value, double limit)
{
    printf("    [%s] %-58s %9.4f  (limit %g)\n", ok ? " ok " : "FAIL", what, value, limit);
    if (!ok) ++g_failures;
}

// ── Plant ────────────────────────────────────────────────────────────────────
struct Plant {
    float pos[3] = { 0, 0, 0 };    // NED; ground at D = 0
    float vel[3] = { 0, 0, 0 };
    float roll = 0, pitch = 0, yaw = 0;
    float p = 0, q = 0, r = 0;
    float p_dot = 0, q_dot = 0, r_dot = 0;
    float torque[3] = { 0, 0, 0 }, thrust = 0;     // after motor lag
    float wind[2] = { 0.25f, -0.15f };            // m/s² steady disturbance, N/E
    bool  stuck = false;                          // hold the vehicle in place (N/E) — "can't keep up" test

    static constexpr float K_RP = 400.0f, K_YAW = 40.0f;   // rad/s² per unit torque command
    static constexpr float MOTOR_LAG = 0.03f, DRAG = 0.15f;

    void step(const float cmd[3], float thr)
    {
        const float a = DT / (MOTOR_LAG + DT);
        for (int i = 0; i < 3; ++i) torque[i] += a * (cmd[i] - torque[i]);
        thrust += a * (thr - thrust);

        p_dot = K_RP * torque[0]; q_dot = K_RP * torque[1]; r_dot = K_YAW * torque[2];
        p += p_dot * DT; q += q_dot * DT; r += r_dot * DT;

        const float sr = sinf(roll), cr = cosf(roll), sp = sinf(pitch), cp = cosf(pitch);
        const float sy = sinf(yaw), cy = cosf(yaw);
        roll  += (p + (q * sr + r * cr) * sp / cp) * DT;
        pitch += (q * cr - r * sr) * DT;
        yaw    = wrap_pi(yaw + ((q * sr + r * cr) / cp) * DT);

        const float T = G * thrust / kDroneConfig.alt.hover_thr;   // hovers where this drone's config says it does
        const bool  airborne = pos[2] < -1.0e-4f;
        float acc[3] = { -T * (cr * sp * cy + sr * sy), -T * (cr * sp * sy - sr * cy), G - T * cr * cp };
        for (int i = 0; i < 3; ++i) acc[i] -= DRAG * vel[i];
        if (airborne) { acc[0] += wind[0]; acc[1] += wind[1]; }
        for (int i = 0; i < 3; ++i) { vel[i] += acc[i] * DT; }
        if (stuck) { vel[0] = vel[1] = 0.0f; }
        for (int i = 0; i < 3; ++i) { pos[i] += vel[i] * DT; }

        if (pos[2] >= 0.0f) {      // on the ground
            pos[2] = 0.0f;
            if (vel[2] > 0.0f) vel[2] = 0.0f;
            vel[0] = vel[1] = 0.0f;
            roll = pitch = 0.0f; p = q = r = 0.0f;
        }
    }

    void fill(float state_full[StateIdx::N], float euler[3]) const
    {
        memset(state_full, 0, sizeof(float) * StateIdx::N);
        const float sr = sinf(roll), cr = cosf(roll), sp = sinf(pitch), cp = cosf(pitch);
        const float sy = sinf(yaw), cy = cosf(yaw);
        // body→NED rotation, columns = body axes in NED
        const float R[3][3] = {
            { cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy },
            { cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy },
            { -sp,     sr * cp,                cr * cp                } };
        state_full[StateIdx::X] = pos[0]; state_full[StateIdx::Y] = pos[1]; state_full[StateIdx::Z_POS] = pos[2];
        for (int b = 0; b < 3; ++b)
            state_full[StateIdx::U + b] = R[0][b] * vel[0] + R[1][b] * vel[1] + R[2][b] * vel[2];
        state_full[StateIdx::P] = p; state_full[StateIdx::Q] = q; state_full[StateIdx::R] = r;
        state_full[StateIdx::P_DOT] = p_dot; state_full[StateIdx::Q_DOT] = q_dot; state_full[StateIdx::R_DOT] = r_dot;
        euler[0] = roll; euler[1] = pitch; euler[2] = yaw;
    }
};

// ── Simulation: plant + FlightStateMachine + pilot ───────────────────────────
struct Sim {
    Plant              plant;
    FlightStateMachine fsm{ kDroneConfig };
    float t = 0;
    // Pilot
    bool  armed = false, pos_valid = true;
    float thr = 0.0f, roll_stick = 0, pitch_stick = 0, yaw_stick = 0, mode_sw = 1.0f;   // +1 = POS_HOLD
    // Last tick
    float state_full[StateIdx::N], euler[3];
    float ctun[12], traj[8];
    float thrust_out = 0;
    FILE *csv = nullptr;
    // Optional degraded position/velocity measurement (0 = perfect): the
    // controller sees position and velocity `meas_delay` ticks old, refreshed
    // only every `meas_hold` ticks. Pessimistic stand-in for a slow, late
    // mocap link — the real estimator fills in between updates with the IMU.
    int   meas_delay = 0, meas_hold = 0;
    static constexpr int HIST = 256;
    float hist[HIST][6] = {};
    float held[6] = {};
    long  ticks = 0;

    explicit Sim(const char *name)
    {
        const std::string path = g_out_dir + "/" + name + ".csv";
        csv = fopen(path.c_str(), "w");
        if (csv) fprintf(csv, "t,n,e,d,vn,ve,vd,roll,pitch,yaw,pos_n_tgt,pos_e_tgt,roll_tgt,pitch_tgt,"
                              "traj_state,ref_n,ref_e,ref_d,yaw_tgt,yaw_rate_cmd,h_off,path_speed,thrust,phase\n");
        plant.fill(state_full, euler);
    }
    ~Sim() { if (csv) fclose(csv); }

    void tick()
    {
        g_sim_time_us += CONTROL_PERIOD_US;
        t += DT;
        plant.fill(state_full, euler);
        if (meas_hold > 0) {
            float *h = hist[ticks % HIST];
            for (int i = 0; i < 3; ++i) { h[i] = state_full[StateIdx::X + i]; h[3 + i] = state_full[StateIdx::U + i]; }
            if (ticks % meas_hold == 0 && ticks >= meas_delay)
                memcpy(held, hist[(ticks - meas_delay) % HIST], sizeof(held));
            for (int i = 0; i < 3; ++i) { state_full[StateIdx::X + i] = held[i]; state_full[StateIdx::U + i] = held[3 + i]; }
        }
        ++ticks;
        float input[InputIdx::N_INPUTS] = {};
        input[InputIdx::THRUST] = thr; input[InputIdx::ROLL_TGT] = roll_stick;
        input[InputIdx::PITCH_TGT] = pitch_stick; input[InputIdx::YAW_RATE] = yaw_stick;
        input[InputIdx::FLIGHT_MODE] = mode_sw;
        const uint32_t rpm[4] = { 0, 0, 0, 0 };
        float cmds[3];
        fsm.update(state_full, euler, input, armed, pos_valid, rpm, cmds, thrust_out);
        fsm.get_ctun_diag(ctun);
        fsm.get_traj_diag(traj);
        if (csv) fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.5f,%.5f,"
                              "%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f,%.4f,%d\n",
                         t, plant.pos[0], plant.pos[1], plant.pos[2], plant.vel[0], plant.vel[1], plant.vel[2],
                         plant.roll, plant.pitch, plant.yaw, ctun[0], ctun[2], ctun[8], ctun[9],
                         (int)traj[0], traj[1], traj[2], traj[3], traj[4], traj[5], traj[6], traj[7],
                         thrust_out, (int)fsm.phase());
        plant.step(cmds, thrust_out);
    }

    // Run for `seconds`, calling each(sim) after every tick.
    void run(float seconds, const std::function<void()> &each = nullptr)
    {
        const int n = (int)lroundf(seconds / DT);
        for (int i = 0; i < n; ++i) { tick(); if (each) each(); }
    }

    TrajResult cmd(TrajCmdType type, float p0 = NANF, float p1 = NANF, float p2 = NANF, float p3 = NANF,
                   float p4 = NANF, float p5 = NANF, float p6 = NANF)
    {
        TrajCommand c = { type, { p0, p1, p2, p3, p4, p5, p6 } };
        return fsm.traj_command(c, state_full, euler);
    }

    TrajState traj_state() const { return (TrajState)(int)traj[0]; }

    // Arm in POS_HOLD on the ground, climb to about `height` m, settle.
    void takeoff_to(float height)
    {
        armed = true; thr = 0.0f; mode_sw = 1.0f;
        run(0.5f);
        thr = 0.8f;
        for (int i = 0; i < 400 * 20 && plant.pos[2] > -height + 0.15f; ++i) tick();
        thr = 0.5f;
        run(4.0f);
    }
};

static float path_error(const Sim &s, float cn, float ce, float rN, float rE)
{
    float best = 1e9f;
    for (int i = 0; i < 3600; ++i) {
        const float th = 6.2831853f * i / 3600.0f;
        const float dn = cn + rN * cosf(th) - s.plant.pos[0], de = ce + rE * sinf(th) - s.plant.pos[1];
        best = fminf(best, sqrtf(dn * dn + de * de));
    }
    return best;
}

// ── Scenarios ────────────────────────────────────────────────────────────────

// Step 1: HeadingControl alone against a yaw-rate lag.
static void scenario_heading_unit()
{
    printf("heading_unit: HeadingControl step and wrap\n");
    const HeadingGains g = kDroneConfig.heading;
    for (int test = 0; test < 2; ++test) {
        HeadingControl hc(g);
        float yaw = (test == 0) ? 0.0f : 170.0f / RAD2DEG;
        const float des = (test == 0) ? 90.0f / RAD2DEG : -170.0f / RAD2DEG;
        float rate = 0, max_cmd = 0, overshoot = 0, travelled = 0;
        for (int k = 0; k < 400 * 8; ++k) {
            const float cmd = hc.update_heading(yaw, des, 0.0f);
            max_cmd = fmaxf(max_cmd, fabsf(cmd));
            rate += (cmd - rate) * DT / (0.14f + DT);
            yaw = wrap_pi(yaw + rate * DT);
            travelled += rate * DT;
            overshoot = fmaxf(overshoot, wrap_pi(yaw - des));   // both tests turn in the positive direction
        }
        const float want = (test == 0) ? 90.0f : 20.0f;
        check(fabsf(travelled * RAD2DEG - want) < 1.0f, test == 0 ? "90 deg step: degrees turned" : "170 -> -170 deg: degrees turned (short way = +20)",
              travelled * RAD2DEG, want);
        check(overshoot * RAD2DEG < 5.0f, "  overshoot [deg]", overshoot * RAD2DEG, 5.0);
        check(max_cmd <= 2.0f * g.max_rate + 1e-3f, "  peak yaw-rate command [rad/s]", max_cmd, 2.0 * g.max_rate);
    }
}

// Step 1: POS_HOLD yaw stick through the whole stack.
static void scenario_heading_stick()
{
    printf("heading_stick: POS_HOLD yaw stick turn and hold\n");
    Sim s("heading_stick");
    s.takeoff_to(1.0f);
    const float yaw0 = s.plant.yaw;
    float drift = 0;
    s.run(5.0f, [&] { drift = fmaxf(drift, fabsf(wrap_pi(s.plant.yaw - yaw0))); });
    check(drift * RAD2DEG < 5.0f, "hands-off heading drift over 5 s [deg]", drift * RAD2DEG, 5.0);

    s.yaw_stick = 0.5f;
    s.run(1.5f);
    s.yaw_stick = 0.0f;
    check(s.plant.r > 0.5f, "yaw rate at half stick [rad/s] (positive = nose right)", s.plant.r, 0.5);
    // Stick released: how far does the nose swing past where the target stops?
    float turned = 0, prev_yaw = s.plant.yaw, peak = 0;
    s.run(1.5f, [&] { turned += wrap_pi(s.plant.yaw - prev_yaw); prev_yaw = s.plant.yaw; peak = fmaxf(peak, turned); });
    const float yaw_rest = s.traj[4];             // HeadingControl's target once stopped
    const float past = wrap_pi(s.plant.yaw - yaw_rest) + (peak - turned);
    check(past * RAD2DEG < 3.0f, "overshoot past the heading target when the turn stops [deg]", past * RAD2DEG, 3.0);
    float after = 0;
    s.run(4.0f, [&] { after = fmaxf(after, fabsf(wrap_pi(s.plant.yaw - yaw_rest))); });
    check(after * RAD2DEG < 5.0f, "heading error after the turn stops [deg]", after * RAD2DEG, 5.0);
    const float hold_n = s.ctun[0], hold_e = s.ctun[2];
    const float perr = hypotf(s.plant.pos[0] - hold_n, s.plant.pos[1] - hold_e);
    check(perr < 0.15f, "position error after yawing [m]", perr, 0.15);
}

// Step 4: point moves, defaults, floor, override, falling behind.
static void scenario_point()
{
    printf("point: origin, point moves, floor, override, pause\n");
    Sim s("point");
    s.takeoff_to(1.0f);

    check(s.cmd(TrajCmdType::POINT, 1.0f, 0.0f) == TrajResult::NO_ORIGIN, "point before set-origin rejected (NO_ORIGIN)", 1, 1);
    check(s.cmd(TrajCmdType::CIRCLE, 1.0f) == TrajResult::NO_ORIGIN, "circle before set-origin rejected (NO_ORIGIN)", 1, 1);
    check(s.cmd(TrajCmdType::SET_ORIGIN) == TrajResult::ACCEPTED, "set-origin accepted", 1, 1);
    const float o[3] = { s.plant.pos[0], s.plant.pos[1], s.plant.pos[2] };

    // Point with nothing given: origin N/E at the current altitude → stays put.
    check(s.cmd(TrajCmdType::POINT) == TrajResult::ACCEPTED, "point with no parameters accepted", 1, 1);
    float moved = 0;
    s.run(4.0f, [&] { moved = fmaxf(moved, hypotf(s.plant.pos[0] - o[0], s.plant.pos[1] - o[1]));
                      moved = fmaxf(moved, fabsf(s.plant.pos[2] - o[2])); });
    check(moved < 0.10f, "  vehicle stays put [m]", moved, 0.10);
    check(s.traj_state() == TrajState::HOLD, "  state is HOLD", (int)s.traj_state(), 2);

    // 2 m North, altitude not given.
    const float yaw_before = s.plant.yaw;
    check(s.cmd(TrajCmdType::POINT, 2.0f, 0.0f) == TrajResult::ACCEPTED, "point 2 m North accepted", 1, 1);
    float max_n = -1e9f, max_lean = 0, max_yaw_dev = 0, max_e_dev = 0, max_speed = 0;
    s.run(12.0f, [&] { max_n = fmaxf(max_n, s.plant.pos[0] - o[0]);
                       max_speed = fmaxf(max_speed, hypotf(s.plant.vel[0], s.plant.vel[1]));
                       max_lean = fmaxf(max_lean, fmaxf(fabsf(s.plant.roll), fabsf(s.plant.pitch)));
                       max_e_dev = fmaxf(max_e_dev, fabsf(s.plant.pos[1] - o[1]));
                       max_yaw_dev = fmaxf(max_yaw_dev, fabsf(wrap_pi(s.plant.yaw - yaw_before))); });
    const float arrive = hypotf(s.plant.pos[0] - (o[0] + 2.0f), s.plant.pos[1] - o[1]);
    check(arrive < 0.15f, "  distance from the point after 12 s [m]", arrive, 0.15);
    check(max_n - 2.0f < 0.05f, "  overshoot past the point [m]", max_n - 2.0f, 0.05);
    check(max_speed < 1.15f * TrajectoryTracker::V_MOVE, "  peak speed [m/s] (reference V_MOVE)", max_speed, 1.15 * TrajectoryTracker::V_MOVE);
    check(max_e_dev < 0.15f, "  sideways deviation from the line [m]", max_e_dev, 0.15);
    check(max_lean * RAD2DEG < 15.0f, "  peak lean [deg]", max_lean * RAD2DEG, 15.0);
    check(max_yaw_dev * RAD2DEG < 5.0f, "  heading held [deg]", max_yaw_dev * RAD2DEG, 5.0);
    check(fabsf(s.plant.pos[2] - o[2]) < 0.10f, "  altitude held [m]", fabsf(s.plant.pos[2] - o[2]), 0.10);

    // Explicit D: 0.5 m above the origin, back over it.
    s.cmd(TrajCmdType::POINT, 0.0f, 0.0f, -0.5f);
    s.run(12.0f);
    check(fabsf(s.plant.pos[2] - (o[2] - 0.5f)) < 0.10f, "point with D = -0.5: altitude error [m]", fabsf(s.plant.pos[2] - (o[2] - 0.5f)), 0.10);

    // Throttle stick raises the whole trajectory; N/E unaffected.
    const float d_before = s.plant.pos[2];
    s.thr = 0.75f; s.run(1.0f); s.thr = 0.5f; s.run(3.0f);
    check(s.plant.pos[2] < d_before - 0.3f, "throttle up raises the hold point: change in D [m]", s.plant.pos[2] - d_before, -0.3);
    check(s.traj_state() == TrajState::HOLD, "  trajectory still active", (int)s.traj_state(), 2);
    check(hypotf(s.plant.pos[0] - o[0], s.plant.pos[1] - o[1]) < 0.15f, "  N/E unchanged [m]", hypotf(s.plant.pos[0] - o[0], s.plant.pos[1] - o[1]), 0.15);
    const float h_off = s.traj[6];
    s.cmd(TrajCmdType::POINT, 0.0f, 0.0f);
    s.run(0.01f);
    check(fabsf(s.traj[6]) < 1e-3f && fabsf(h_off) > 0.3f, "  new command zeroes the height offset [m]", s.traj[6], 0.0);
    float jump = 0; const float d_cmd = s.plant.pos[2];
    s.run(3.0f, [&] { jump = fmaxf(jump, fabsf(s.plant.pos[2] - d_cmd)); });
    check(jump < 0.10f, "  ... without the vehicle changing height [m]", jump, 0.10);

    // Point or circle below the floor: refused, and the hold in progress is untouched.
    const float n_before = s.plant.pos[0], e_before = s.plant.pos[1], d_hold = s.plant.pos[2];
    check(s.cmd(TrajCmdType::POINT, 1.0f, 0.0f, 5.0f) == TrajResult::BAD_PARAM, "point 5 m below origin (under the floor): BAD_PARAM", 1, 1);
    check(s.cmd(TrajCmdType::POINT, 1.0f, 0.0f, -o[2] - 0.05f) == TrajResult::BAD_PARAM, "point 5 cm above the ground (inside the margin): BAD_PARAM", 1, 1);
    check(s.cmd(TrajCmdType::CIRCLE, 1.0f, NANF, NANF, 5.0f) == TrajResult::BAD_PARAM, "circle 5 m below origin: BAD_PARAM", 1, 1);
    float drift = 0;
    s.run(4.0f, [&] { drift = fmaxf(drift, hypotf(s.plant.pos[0] - n_before, s.plant.pos[1] - e_before));
                      drift = fmaxf(drift, fabsf(s.plant.pos[2] - d_hold)); });
    check(drift < 0.05f, "  vehicle does not move [m]", drift, 0.05);
    check(s.traj_state() == TrajState::HOLD, "  the hold in progress carries on", (int)s.traj_state(), 2);
    check(s.cmd(TrajCmdType::POINT, 0.0f, 0.0f, -o[2] - 0.3f) == TrajResult::ACCEPTED, "point 0.3 m above the ground: accepted", 1, 1);
    float lowest_ref = -1e9f, lowest_veh = -1e9f; bool stayed_active = true;
    s.run(8.0f);

    // Throttle held down: the trajectory stops at the floor limit and the vehicle never lands.
    s.thr = 0.2f;
    s.run(4.0f, [&] { lowest_ref = fmaxf(lowest_ref, s.traj[3]); lowest_veh = fmaxf(lowest_veh, s.plant.pos[2]);
                      stayed_active = stayed_active && s.fsm.phase() == FlightPhase::ACTIVE; });
    s.thr = 0.5f;
    check(lowest_ref <= -TrajectoryTracker::FLOOR_MARGIN_M + 1e-4f, "throttle held down: lowest reference D [m] (floor at 0)", lowest_ref, -TrajectoryTracker::FLOOR_MARGIN_M);
    check(lowest_veh < -0.03f, "  vehicle never reaches the ground: lowest D [m]", lowest_veh, -0.03);
    check(stayed_active, "  landed detector did not trip (phase stays ACTIVE)", stayed_active, 1);
    s.thr = 0.75f; s.run(0.5f);
    const float d_at_floor = s.plant.pos[2];
    s.run(1.0f); s.thr = 0.5f;
    check(s.plant.pos[2] < d_at_floor - 0.2f, "  throttle up from the floor climbs straight away: change in D [m]", s.plant.pos[2] - d_at_floor, -0.2);
    s.cmd(TrajCmdType::POINT, 0.0f, 0.0f, -1.0f);
    s.run(8.0f);

    // Override mid-move: roll stick tap → trajectory off, stays off, no lean step.
    s.cmd(TrajCmdType::POINT, 3.0f, 0.0f);
    s.run(3.0f);
    const float roll_tgt_before = s.ctun[8], pitch_tgt_before = s.ctun[9];
    s.roll_stick = 0.3f;
    s.tick();
    const float lean_step = fmaxf(fabsf(s.ctun[8] - roll_tgt_before), fabsf(s.ctun[9] - pitch_tgt_before));
    check(s.traj_state() == TrajState::IDLE, "roll stick mid-move cancels the trajectory", (int)s.traj_state(), 0);
    check(lean_step * RAD2DEG < 2.0f, "  lean-target step at hand-back [deg]", lean_step * RAD2DEG, 2.0);
    s.run(0.3f); s.roll_stick = 0.0f;
    s.run(5.0f);
    check(s.traj_state() == TrajState::IDLE, "  stays cancelled after the stick is released", (int)s.traj_state(), 0);
    const float speed = hypotf(s.plant.vel[0], s.plant.vel[1]);
    check(speed < 0.05f, "  vehicle stopped in position hold [m/s]", speed, 0.05);
    check(s.plant.pos[0] - o[0] < 2.5f, "  ... short of the 3 m point: N [m]", s.plant.pos[0] - o[0], 2.5);

    // Falling behind: hold the vehicle in place mid-move.
    s.cmd(TrajCmdType::POINT, 0.0f, 0.0f);
    s.run(8.0f);
    s.cmd(TrajCmdType::POINT, 3.0f, 0.0f);
    s.run(1.0f);
    s.plant.stuck = true;
    float lead = 0;
    s.run(6.0f, [&] { lead = fmaxf(lead, hypotf(s.traj[1] - s.plant.pos[0], s.traj[2] - s.plant.pos[1])); });
    check(lead < TrajectoryTracker::ERR_PAUSE_M + 0.02f, "vehicle held in place: reference stops within [m]", lead, TrajectoryTracker::ERR_PAUSE_M);
    s.plant.stuck = false;
    s.run(14.0f);
    check(hypotf(s.plant.pos[0] - (o[0] + 3.0f), s.plant.pos[1] - o[1]) < 0.15f, "  released: completes the move, error [m]",
          hypotf(s.plant.pos[0] - (o[0] + 3.0f), s.plant.pos[1] - o[1]), 0.15);
}

// Step 5: one circle/ellipse run.
static void run_orbit(const char *name, float r, float focus, float dir, float speed, float start_n, float start_e)
{
    printf("%s: r=%.2f focus=%.2f dir=%+.0f speed=%.2f\n", name, r, focus, dir, speed);
    Sim s(name);
    s.takeoff_to(1.0f);
    s.cmd(TrajCmdType::SET_ORIGIN);
    const float o[2] = { s.plant.pos[0], s.plant.pos[1] };
    if (start_n != 0.0f || start_e != 0.0f) {      // fly somewhere else first
        s.cmd(TrajCmdType::POINT, start_n, start_e);
        s.run(hypotf(start_n, start_e) / 0.5f + 6.0f);
    }
    const float c = fabsf(focus);
    const float a = r + c, b = sqrtf(r * (r + 2 * c));
    const float rN = focus >= 0 ? a : b, rE = focus >= 0 ? b : a;

    check(s.cmd(TrajCmdType::CIRCLE, r, NANF, NANF, NANF, focus, dir, speed) == TrajResult::ACCEPTED, "circle accepted", 1, 1);

    // Approach: wait for ORBIT, watching for any jump in the position target.
    float max_tgt_step = 0, prev_tn = s.ctun[0], prev_te = s.ctun[2], t_orbit = -1;
    const float t_cmd = s.t;
    for (int i = 0; i < 400 * 40 && s.traj_state() != TrajState::ORBIT; ++i) {
        s.tick();
        max_tgt_step = fmaxf(max_tgt_step, hypotf(s.ctun[0] - prev_tn, s.ctun[2] - prev_te));
        prev_tn = s.ctun[0]; prev_te = s.ctun[2];
    }
    t_orbit = s.t - t_cmd;
    check(s.traj_state() == TrajState::ORBIT, "  reaches the path and starts the orbit after [s]", t_orbit, 40);
    const float entry_err = path_error(s, o[0], o[1], rN, rE);
    check(entry_err < 0.15f, "  distance from the path when the orbit starts [m]", entry_err, 0.15);

    // Orbit: skip the first lap's spin-up, then measure.
    float max_path = 0, max_yaw = 0, max_lean = 0, vmin = 1e9f, vmax = 0, max_alt = 0, turned = 0, prev_yaw = s.plant.yaw;
    const float d0 = s.plant.pos[2];
    int k = 0;
    s.run(45.0f, [&] {
        max_tgt_step = fmaxf(max_tgt_step, hypotf(s.ctun[0] - prev_tn, s.ctun[2] - prev_te));
        prev_tn = s.ctun[0]; prev_te = s.ctun[2];
        turned += wrap_pi(s.plant.yaw - prev_yaw); prev_yaw = s.plant.yaw;
        if (++k < 400 * 8) return;
        if (k % 8 == 0) max_path = fmaxf(max_path, path_error(s, o[0], o[1], rN, rE));
        const float course = atan2f(s.plant.vel[1], s.plant.vel[0]);
        max_yaw  = fmaxf(max_yaw, fabsf(wrap_pi(s.plant.yaw - course)));
        max_lean = fmaxf(max_lean, fmaxf(fabsf(s.plant.roll), fabsf(s.plant.pitch)));
        const float v = hypotf(s.plant.vel[0], s.plant.vel[1]);
        vmin = fminf(vmin, v); vmax = fmaxf(vmax, v);
        max_alt = fmaxf(max_alt, fabsf(s.plant.pos[2] - d0));
    });
    check(max_tgt_step < 0.02f, "  largest per-tick jump in the position target [m]", max_tgt_step, 0.02);
    check(max_path < 0.10f, "  path error after spin-up [m]", max_path, 0.10);
    check(max_yaw * RAD2DEG < 10.0f, "  nose vs direction of travel [deg]", max_yaw * RAD2DEG, 10.0);
    check(max_lean * RAD2DEG < 30.0f, "  peak lean [deg]", max_lean * RAD2DEG, 30.0);
    check(max_alt < 0.10f, "  altitude deviation [m]", max_alt, 0.10);
    check(turned * dir > 3.0f, "  rotation sense: yaw turned [rad], signed by commanded direction", turned * dir, 3.0);
    printf("           speed range on the path: %.2f .. %.2f m/s\n", vmin, vmax);
    if (c > 0.0f) check(vmin < 0.85f * vmax, "  slows for the tight ends: min/max speed", vmin / vmax, 0.85);

    // Stop command → plain position hold.
    check(s.cmd(TrajCmdType::STOP) == TrajResult::ACCEPTED, "  stop accepted", 1, 1);
    const float roll_tgt_before = s.ctun[8], pitch_tgt_before = s.ctun[9];
    s.tick();
    const float lean_step = fmaxf(fabsf(s.ctun[8] - roll_tgt_before), fabsf(s.ctun[9] - pitch_tgt_before));
    check(lean_step * RAD2DEG < 2.0f, "  lean-target step at hand-back [deg]", lean_step * RAD2DEG, 2.0);
    s.run(6.0f);
    check(hypotf(s.plant.vel[0], s.plant.vel[1]) < 0.05f, "  stopped in position hold [m/s]", hypotf(s.plant.vel[0], s.plant.vel[1]), 0.05);
}

// Gain check: hold position and fly a move with a slow, late position
// measurement. Looks for the loop going unstable, not for fine performance.
static void scenario_degraded_link()
{
    printf("degraded_link: POS_HOLD + point move with position 100 ms late, refreshed at 23 Hz\n");
    Sim s("degraded_link");
    s.takeoff_to(1.0f);
    s.meas_delay = 40;     // 100 ms
    s.meas_hold  = 17;     // 400 / 17 = 23.5 Hz
    s.run(3.0f);
    const float n0 = s.plant.pos[0], e0 = s.plant.pos[1];
    // Shove it: 0.5 m/s sideways.
    s.plant.vel[1] += 0.5f;
    float peak = 0, late = 0, lean = 0; int k = 0;
    s.run(10.0f, [&] { const float r = hypotf(s.plant.pos[0] - n0, s.plant.pos[1] - e0);
                       peak = fmaxf(peak, r); if (++k > 400 * 6) late = fmaxf(late, r);
                       lean = fmaxf(lean, fmaxf(fabsf(s.plant.roll), fabsf(s.plant.pitch))); });
    check(peak < 0.40f, "0.5 m/s shove: peak excursion [m]", peak, 0.40);
    check(late < 0.05f, "  back within 5 cm and staying there after 6 s: max error 6-10 s [m]", late, 0.05);
    check(lean * RAD2DEG < 20.0f, "  peak lean [deg]", lean * RAD2DEG, 20.0);
    s.cmd(TrajCmdType::SET_ORIGIN);
    const float o[2] = { s.plant.pos[0], s.plant.pos[1] };
    s.cmd(TrajCmdType::POINT, 1.0f, 0.0f);
    float max_n = -1e9f, max_speed = 0;
    s.run(10.0f, [&] { max_n = fmaxf(max_n, s.plant.pos[0] - o[0]); max_speed = fmaxf(max_speed, hypotf(s.plant.vel[0], s.plant.vel[1])); });
    check(max_n - 1.0f < 0.08f, "1 m point move: overshoot past the point [m]", max_n - 1.0f, 0.08);
    check(max_speed < 1.5f * TrajectoryTracker::V_MOVE, "  peak speed [m/s]", max_speed, 1.5 * TrajectoryTracker::V_MOVE);
    check(hypotf(s.plant.pos[0] - (o[0] + 1.0f), s.plant.pos[1] - o[1]) < 0.05f, "  final error [m]", hypotf(s.plant.pos[0] - (o[0] + 1.0f), s.plant.pos[1] - o[1]), 0.05);
}

// Step 6: command acceptance rules and cancel conditions.
static void scenario_commands()
{
    printf("commands: acceptance rules and cancel conditions\n");
    Sim s("commands");
    // On the ground, disarmed.
    s.run(0.1f);
    check(s.cmd(TrajCmdType::SET_ORIGIN) == TrajResult::ACCEPTED, "set-origin on the ground, disarmed: accepted", 1, 1);
    check(s.cmd(TrajCmdType::POINT, 1.0f) == TrajResult::NOT_READY, "point while disarmed: NOT_READY", 1, 1);
    s.takeoff_to(1.0f);
    check(s.cmd(TrajCmdType::CIRCLE) == TrajResult::BAD_PARAM, "circle with no radius: BAD_PARAM", 1, 1);
    check(s.cmd(TrajCmdType::CIRCLE, 0.0f) == TrajResult::BAD_PARAM, "circle radius 0: BAD_PARAM", 1, 1);
    check(s.cmd(TrajCmdType::CIRCLE, -1.0f) == TrajResult::BAD_PARAM, "circle radius -1: BAD_PARAM", 1, 1);
    check(s.cmd(TrajCmdType::CIRCLE, 50.0f) == TrajResult::BAD_PARAM, "circle radius 50: BAD_PARAM", 1, 1);
    check(s.cmd(TrajCmdType::CIRCLE, 1.0f, NANF, NANF, NANF, 12.0f) == TrajResult::BAD_PARAM, "circle focus 12: BAD_PARAM", 1, 1);
    check(s.cmd(TrajCmdType::POINT, INFINITY) == TrajResult::BAD_PARAM, "point N = inf: BAD_PARAM", 1, 1);
    check(s.cmd(TrajCmdType::POINT, 100.0f) == TrajResult::BAD_PARAM, "point N = 100: BAD_PARAM", 1, 1);
    check(s.traj_state() == TrajState::IDLE, "nothing started by the rejected commands", (int)s.traj_state(), 0);

    s.mode_sw = 0.0f; s.run(1.0f);            // ALT_HOLD
    check(s.cmd(TrajCmdType::POINT, 1.0f) == TrajResult::NOT_READY, "point in ALT_HOLD: NOT_READY", 1, 1);
    s.mode_sw = 1.0f; s.run(2.0f);

    s.pos_valid = false; s.run(0.1f);
    check(s.cmd(TrajCmdType::POINT, 1.0f) == TrajResult::NOT_READY, "point with no valid position: NOT_READY", 1, 1);
    check(s.cmd(TrajCmdType::SET_ORIGIN) == TrajResult::NOT_READY, "set-origin with no valid position: NOT_READY", 1, 1);
    s.pos_valid = true; s.run(0.1f);

    // Circle given only a radius, with the origin on the ground: must stay at the current height.
    const float d_before = s.plant.pos[2];
    check(s.cmd(TrajCmdType::CIRCLE, 1.0f) == TrajResult::ACCEPTED, "circle with only a radius: accepted", 1, 1);
    s.run(20.0f);
    check(s.traj_state() == TrajState::ORBIT, "  orbiting", (int)s.traj_state(), 3);
    check(fabsf(s.plant.pos[2] - d_before) < 0.10f, "  at the height it was commanded from [m]", fabsf(s.plant.pos[2] - d_before), 0.10);
    check(s.cmd(TrajCmdType::SET_ORIGIN) == TrajResult::NOT_READY, "set-origin during a trajectory: NOT_READY", 1, 1);
    check(s.traj_state() == TrajState::ORBIT, "  still orbiting", (int)s.traj_state(), 3);

    // Position lost mid-orbit → cancelled.
    s.pos_valid = false; s.run(0.05f);
    check(s.traj_state() == TrajState::IDLE, "position lost mid-orbit: cancelled", (int)s.traj_state(), 0);
    s.pos_valid = true; s.run(3.0f);
    check(s.traj_state() == TrajState::IDLE, "  not resumed when the position returns", (int)s.traj_state(), 0);

    // Mode switch mid-orbit → cancelled, and not resumed on switching back.
    s.cmd(TrajCmdType::CIRCLE, 1.0f); s.run(6.0f);
    s.mode_sw = 0.0f; s.run(0.5f); s.mode_sw = 1.0f; s.run(1.0f);
    check(s.traj_state() == TrajState::IDLE, "leaving POS_HOLD mid-trajectory: cancelled, not resumed", (int)s.traj_state(), 0);

    // Yaw and pitch sticks cancel too.
    s.cmd(TrajCmdType::POINT, 0.0f, 0.0f); s.run(1.0f);
    s.yaw_stick = 0.2f; s.run(0.05f); s.yaw_stick = 0.0f;
    check(s.traj_state() == TrajState::IDLE, "yaw stick cancels", (int)s.traj_state(), 0);
    s.run(2.0f);
    s.cmd(TrajCmdType::POINT, 0.0f, 0.0f); s.run(1.0f);
    s.pitch_stick = -0.2f; s.run(0.05f); s.pitch_stick = 0.0f;
    check(s.traj_state() == TrajState::IDLE, "pitch stick cancels", (int)s.traj_state(), 0);
    s.run(2.0f);
    // Sticks inside the deadband do not.
    s.cmd(TrajCmdType::POINT, 0.0f, 0.0f); s.run(1.0f);
    s.roll_stick = 0.08f; s.pitch_stick = -0.08f; s.yaw_stick = 0.08f; s.run(1.0f);
    check(s.traj_state() != TrajState::IDLE, "sticks inside the deadband do not cancel", (int)s.traj_state(), 1);
    s.roll_stick = s.pitch_stick = s.yaw_stick = 0.0f;

    // Land in POS_HOLD with a trajectory cancelled: normal landing, ground idle.
    s.cmd(TrajCmdType::STOP);
    s.thr = 0.2f;
    for (int i = 0; i < 400 * 30 && s.fsm.phase() == FlightPhase::ACTIVE; ++i) s.tick();
    check(s.fsm.phase() == FlightPhase::GROUND_IDLE, "landing in POS_HOLD after a trajectory: reaches ground idle", (int)s.fsm.phase(), 1);
    check(s.cmd(TrajCmdType::POINT, 1.0f) == TrajResult::NOT_READY, "point in ground idle: NOT_READY", 1, 1);
    s.armed = false; s.run(0.5f); s.thr = 0.0f;
    // Origin survives the disarm.
    s.takeoff_to(1.0f);
    check(s.cmd(TrajCmdType::POINT, 0.5f, 0.0f) == TrajResult::ACCEPTED, "origin survives disarm: point accepted after re-arming", 1, 1);
}

int main(int argc, char **argv)
{
    if (argc > 1) g_out_dir = argv[1];
    scenario_heading_unit();
    scenario_heading_stick();
    scenario_point();
    run_orbit("circle_r1_v05_cw",   1.0f,  0.0f,  1.0f, 0.5f, 0.0f, 0.0f);
    run_orbit("circle_r1_v05_ccw",  1.0f,  0.0f, -1.0f, 0.5f, 0.0f, 0.0f);
    run_orbit("circle_r1_v10_cw",   1.0f,  0.0f,  1.0f, 1.0f, 0.0f, 0.0f);
    run_orbit("circle_r1_v10_ccw",  1.0f,  0.0f, -1.0f, 1.0f, 0.0f, 0.0f);
    run_orbit("ellipse_north",      0.75f, 0.5f,  1.0f, 1.0f, 0.0f, 0.0f);
    run_orbit("ellipse_east",       0.75f, -0.5f, -1.0f, 1.0f, 0.0f, 0.0f);
    run_orbit("circle_from_3m_off", 1.0f,  0.0f,  1.0f, 0.5f, 3.0f, 2.0f);
    scenario_commands();
    scenario_degraded_link();
    printf("\n%s (%d failed checks)\n", g_failures ? "FAILED" : "ALL PASSED", g_failures);
    return g_failures ? 1 : 0;
}
