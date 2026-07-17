/*
 * threads.cpp — BPRL thread function definitions.
 *
 * All eight flight-controller threads live here.  Each thread receives its
 * update period through the arg pointer (a const sysinterval_t *), set in
 * the rate sequencer block in main.cpp.
 *
 * Shared state (g_state, g_imu, g_can_imu, …) is also defined here and
 * exported via threads.hpp for use by Coms drivers (e.g. CAN.cpp).
 */

#include "src/threads.hpp"
#include "src/FlightState.hpp"
#include "src/coms/SPI.hpp"
#include "src/coms/CAN.hpp"
#include "src/coms/I2C.hpp"
#include "src/coms/PWM.hpp"
#include "src/coms/DShot.hpp"
#include "src/coms/Radio.hpp"
#include "src/controllers/FlightStateMachine.hpp"
#include "src/controllers/MotorMixer.hpp"
#include "src/state_estimator/StateManager.hpp"
#include "src/logging/Logger.hpp"
#include "src/logging/LogMessages.hpp"
#include "src/usb_serial.hpp"
#include "src/coms/CalFlash.hpp"
#include "src/coms/MAVLink.hpp"
#include "src/math/math.hpp"
#include "src/diagnostics/ThreadTiming.hpp"
#include "chprintf.h"
#include "memstreams.h"
#include "ff.h"
#include <cstring>
#include <cstdio>
#include <cmath>

/* ── Shared state definitions ────────────────────────────────────────────── */

MUTEX_DECL(state_mtx);
float   g_state[StateIdx::N] = {};   // 19-element EKF state vector
float   g_euler[3]           = {};   // [roll, pitch, yaw] derived from quaternion
float   g_input[InputIdx::N_INPUTS] = {};
int32_t g_output[4]          = {};
float   g_ctrl[4]            = {};   // [roll_tq, pitch_tq, yaw_tq, thrust] — active controller outputs
float   g_indi_diag[8]       = {};   // [unmix_roll, unmix_pitch, delta_roll, delta_pitch, cmd_roll, cmd_pitch, accel_cmd_roll, accel_cmd_pitch] — INDI shadow diagnostics, always populated
float   g_ctun_diag[12]      = {};   // TEMP: [pos_n_tgt, pos_n_err, pos_e_tgt, pos_e_err, vel_n_tgt, vel_n_err, vel_e_tgt, vel_e_err, roll_tgt, pitch_tgt, climb_rate_tgt, climb_rate_err] — pos-hold NE + alt-hold shadow tuning diagnostics
bool    g_armed              = false;
int     g_flight_mode        = 0;    // FlightMode enum value (0=STABILIZE, 1=ALT_HOLD, 2=POS_HOLD)
bool    g_use_indi           = false; // attitude controller switch from radio (false=PID, true=INDI)

MUTEX_DECL(imu_mtx);
IMURaw g_imu[3] = {};

MUTEX_DECL(can_imu_mtx);
CANIMURaw g_can_imu = {1.0f};  // q0=1: identity quaternion so angles show 0/0/0 before first frame

MUTEX_DECL(strainRate_mtx);
StrainRateRaw g_strain_rate = {};

MUTEX_DECL(mocap_mtx);
MocapRaw  g_mocap   = {};

MUTEX_DECL(baro_mtx);
BaroRaw   g_baro    = {};

MUTEX_DECL(esc_mtx);
uint32_t g_rpm_gated[4] = {};

// Persistent per-motor gate state for rpm_gate() — only ControlThread (the
// site that publishes g_rpm_gated) needs to own this.
static RpmGateState s_rpm_gate[4];

/* ── Calibration data loaded from flash at boot ──────────────────────────── */
static CalibData g_cal = {};
static bool      g_cal_valid = false;

/* ── Motor test (always built) ───────────────────────────────────────────── */
MUTEX_DECL(motor_test_mtx);
bool    g_motor_test_active = false;
int32_t g_motor_test_cmd[4] = {};

/* Serialises all USB CDC writes between USBCmdThread and DebugThread.
 * chprintf is NOT atomic — it puts characters one at a time, so concurrent
 * callers interleave at the byte level and produce garbage lines.
 * USBCmdThread uses chMtxLock (must send a complete response before releasing).
 * DebugThread uses chMtxTryLock so it skips a $TEL tick rather than blocking
 * during a potentially long log download. */
static MUTEX_DECL(s_usb_write_mtx);

/* ── Controller instances (ControlThread only) ───────────────────────────── */
static FlightStateMachine flight_sm;
static MotorMixer         mixer;

/* ── State estimator (StateEstThread only) ───────────────────────────────── */
static StateManager state_mgr;

/* ── Thread working areas ────────────────────────────────────────────────── */
static THD_WORKING_AREA(waSPI,      2048);
static THD_WORKING_AREA(waCAN,      2048);
static THD_WORKING_AREA(waStateEst, 6144);  // enlarged for StateManager method frames
static THD_WORKING_AREA(waI2C,      1024);
static THD_WORKING_AREA(waControl,  2048);
static THD_WORKING_AREA(waRadio,    1024);
static THD_WORKING_AREA(waHeartbeat, 1024);
static THD_WORKING_AREA(waLog,      8192);  // 8 KB: FatFS + ring-read stack
static THD_WORKING_AREA(waUSBCmd,   4096);  // 4 KB: FatFS log access + line parser
static THD_WORKING_AREA(waMAVLink,  2048);  // MAVLink parser + heartbeat sender
#ifdef BPRL_DEBUG
static THD_WORKING_AREA(waDebug,    2048);
#endif

/* DMA-safe (non-cacheable) buffer for log download via USB */
static uint8_t __attribute__((section(".nocache"))) s_usb_dl_buf[2048];

#ifdef BPRL_DEBUG
/* CAN INS message rate counters — incremented by StateEstThread, read by DebugThread */
static volatile uint32_t s_can_quat_cnt = 0;
static volatile uint32_t s_can_rate_cnt = 0;

/* Per-lane EKF state (radians) — written by StateEstThread under state_mtx,
 * read by DebugThread under the same mutex to emit $EKFL. */
static float s_lane_roll[3] = {};
static float s_lane_pitch[3] = {};
static float s_lane_yaw[3] = {};
static float s_lane_p[3] = {};
static float s_lane_q[3] = {};
static float s_lane_r[3] = {};
static int   s_primary_lane = 0;
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * SPIThread — 1 kHz  NORMALPRIO+30
 * Reads raw accel+gyro from all three on-board IMUs.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(SPIThread, arg)
{
    chRegSetThreadName("spi");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    // Load persistent calibration biases before IMU init; zero biases if absent.
    g_cal_valid = cal_load(g_cal);
    if (!g_cal_valid) memset(&g_cal, 0, sizeof(g_cal));

    spi_drv_init();   // init all three IMUs (may sleep 100 ms each)

    const int tid = TIMING_REGISTER("spi", period);

    systime_t next = chVTGetSystemTime();
    while (true) {
        TIMING_TICK_BEGIN(tid);
        float a[3], g[3];

        if (imu1.read(a, g)) {
            // ROTATION_ROLL_180_YAW_135 → NED z-down: [(y-x)/√2, (y+x)/√2, -z]
            static constexpr float RS = 0.70710678f;
            const float ra[3] = { RS*(a[1]-a[0]), RS*(a[1]+a[0]), -a[2] };
            const float rg[3] = { RS*(g[1]-g[0]), RS*(g[1]+g[0]), -g[2] };
            chMtxLock(&imu_mtx);
            for (int k = 0; k < 3; k++) {
                g_imu[0].accel[k] = ra[k] - g_cal.accel_bias[0][k];
                g_imu[0].gyro[k]  = rg[k] - g_cal.gyro_bias[0][k];
            }
            g_imu[0].valid = true;
            chMtxUnlock(&imu_mtx);
        }
        if (imu2.read(a, g)) {
            // Instance 0 (CS=PC15): ROTATION_YAW_90 → NED z-down: [-y,+x,+z]
            const float ra[3] = {-a[1], +a[0], +a[2]};
            const float rg[3] = {-g[1], +g[0], +g[2]};
            chMtxLock(&imu_mtx);
            for (int k = 0; k < 3; k++) {
                g_imu[1].accel[k] = ra[k] - g_cal.accel_bias[1][k];
                g_imu[1].gyro[k]  = rg[k] - g_cal.gyro_bias[1][k];
            }
            g_imu[1].valid = true;
            chMtxUnlock(&imu_mtx);
        }
        if (imu3.read(a, g)) {
            // Instance 1 (CS=PC13): ROTATION_PITCH_180_YAW_90 → NED z-down: [-y,-x,-z]
            const float ra[3] = {-a[1], -a[0], -a[2]};
            const float rg[3] = {-g[1], -g[0], -g[2]};
            chMtxLock(&imu_mtx);
            for (int k = 0; k < 3; k++) {
                g_imu[2].accel[k] = ra[k] - g_cal.accel_bias[2][k];
                g_imu[2].gyro[k]  = rg[k] - g_cal.gyro_bias[2][k];
            }
            g_imu[2].valid = true;
            chMtxUnlock(&imu_mtx);
        }

        float baro_p, baro_t, baro_alt;
        if (baro1.read(baro_p, baro_t, baro_alt)) {
            chMtxLock(&baro_mtx);
            g_baro.pressure_pa   = baro_p;
            g_baro.temperature_c = baro_t;
            g_baro.alt_m         = baro_alt;
            g_baro.has_new       = true;
            g_baro.valid         = true;
            chMtxUnlock(&baro_mtx);
        }

        TIMING_TICK_END(tid);
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * CANThread — event-driven  NORMALPRIO+28
 * Blocks on a semaphore signaled by the FDCAN1 ISR (new message / message
 * lost / bus-off), then drains RxFIFO0 and dispatches. The wait has a
 * timeout purely so Bus_Off gets checked (and self-healed) periodically
 * even in the freak case the ISR itself never fires. See CAN.cpp for why
 * this doesn't use ChibiOS's canReceiveTimeout()/CAND1.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(CANThread, arg)
{
    (void)arg;
    chRegSetThreadName("can");

    const int tid = TIMING_REGISTER("can", 0);  // event-driven, no fixed period

    while (true) {
        bprl_can_wait_rx(TIME_MS2I(200));

        TIMING_TICK_BEGIN(tid);
        CANRxFrame rxf;
        while (bprl_can_poll(rxf)) {
            can_dispatch(rxf);
        }

        if (can_is_bus_off()) {
            can_hw_reinit();
        }
        TIMING_TICK_END(tid);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * I2CThread — 500 Hz  NORMALPRIO+20
 * Calls each registered I2C device's poll function once per tick.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(I2CThread, arg)
{
    chRegSetThreadName("i2c");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);
    const int tid = TIMING_REGISTER("i2c", period);

    systime_t next = chVTGetSystemTime();
    while (true) {
        TIMING_TICK_BEGIN(tid);
        i2c_poll_all();
        TIMING_TICK_END(tid);
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}


/* ══════════════════════════════════════════════════════════════════════════
 * StateEstThread — 625 Hz  NORMALPRIO+25
 * Runs the 3-lane EKF, fuses g_imu[] and g_can_imu, writes g_state[].
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(StateEstThread, arg)
{
    chRegSetThreadName("est");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);
    const float dt_nominal = static_cast<float>(period)
                           / static_cast<float>(CH_CFG_ST_FREQUENCY);

    state_mgr.init();

    // Ticks since last IMX5 quaternion — clears valid after CAN_TIMEOUT_TICKS.
    uint32_t can_stale_ticks = 0;
    static constexpr uint32_t CAN_TIMEOUT_TICKS = 625;  // 1 s at 625 Hz

    const int tid = TIMING_REGISTER("est", period);

    systime_t next = chVTGetSystemTime();
    uint32_t last_tick_us = TIME_I2US(chVTGetSystemTimeX());
    while (true) {
        TIMING_TICK_BEGIN(tid);
        // Measured elapsed time since the previous tick, not a fixed nominal
        // dt — keeps EKF integration correct under scheduler jitter (same
        // approach PID::update() already uses). Clamped to +/-2x nominal so
        // a single missed-deadline outlier can't corrupt the integration.
        const uint32_t now_us = TIME_I2US(chVTGetSystemTimeX());
        float dt = static_cast<float>(now_us - last_tick_us) * 1.0e-6f;
        last_tick_us = now_us;
        dt = constrain_float(dt, dt_nominal * 0.5f, dt_nominal * 2.0f);

        // Snapshot CAN IMU and clear consumed-this-tick flags atomically.
        CANIMURaw can_snap;
        chMtxLock(&can_imu_mtx);
        can_snap = g_can_imu;
        g_can_imu.has_new_quat  = false;
        g_can_imu.has_new_rates = false;
        chMtxUnlock(&can_imu_mtx);

        // Timeout: if no IMX5 data (quat or rates) arrives for CAN_TIMEOUT_TICKS, mark link invalid.
        if (can_snap.has_new_quat || can_snap.has_new_rates) {
            can_stale_ticks = 0;
        } else if (++can_stale_ticks > CAN_TIMEOUT_TICKS) {
            chMtxLock(&can_imu_mtx);
            g_can_imu.valid = false;
            chMtxUnlock(&can_imu_mtx);
            can_snap.valid = false;
        }

        IMURaw imu_snap[3];
        chMtxLock(&imu_mtx);
        memcpy(imu_snap, g_imu, sizeof(g_imu));
        chMtxUnlock(&imu_mtx);

        MocapRaw mocap_snap;
        chMtxLock(&mocap_mtx);
        mocap_snap = g_mocap;
        g_mocap.has_new_pos = false;
        g_mocap.has_new_vel = false;
        g_mocap.has_new_yaw = false;
        chMtxUnlock(&mocap_mtx);

        BaroRaw baro_snap;
        chMtxLock(&baro_mtx);
        baro_snap = g_baro;
        g_baro.has_new = false;
        chMtxUnlock(&baro_mtx);

        uint32_t rpm_snap[4];
        chMtxLock(&esc_mtx);
        memcpy(rpm_snap, g_rpm_gated, sizeof(rpm_snap));
        chMtxUnlock(&esc_mtx);

        // Run all EKF lanes and derive outputs.
        state_mgr.update(dt, imu_snap, can_snap, mocap_snap, baro_snap, rpm_snap, now_us);
#ifdef BPRL_DEBUG
        if (can_snap.has_new_quat)  s_can_quat_cnt++;
        if (can_snap.has_new_rates) s_can_rate_cnt++;
#endif

        chMtxLock(&state_mtx);
        state_mgr.get_state(g_state);          // copies full 19-element state
        g_euler[0] = state_mgr.roll();         // Euler angles derived from quaternion
        g_euler[1] = state_mgr.pitch();
        g_euler[2] = state_mgr.yaw();
#ifdef BPRL_DEBUG
        for (int li = 0; li < StateManager::NUM_LANES; ++li) {
            state_mgr.get_lane_euler(li, s_lane_roll[li], s_lane_pitch[li], s_lane_yaw[li]);
            state_mgr.get_lane_pqr  (li, s_lane_p[li],   s_lane_q[li],     s_lane_r[li]);
        }
        s_primary_lane = state_mgr.primary_lane();
#endif
        chMtxUnlock(&state_mtx);

        TIMING_TICK_END(tid);
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}


/* ══════════════════════════════════════════════════════════════════════════
 * ControlThread — 400 Hz  NORMALPRIO+22
 * Cascade PID → MotorMixer → motor output.
 * ══════════════════════════════════════════════════════════════════════════ */

static THD_FUNCTION(ControlThread, arg)
{
    chRegSetThreadName("ctrl");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);
    const int tid = TIMING_REGISTER("ctrl", period);

    systime_t next = chVTGetSystemTime();
    while (true) {
        TIMING_TICK_BEGIN(tid);
        /* ── Motor test bypass: skip PID/mixer, drive ESCs directly ──────── */
        {
            bool    test_active;
            int32_t test_cmd[4];
            chMtxLock(&motor_test_mtx);
            test_active = g_motor_test_active;
            if (test_active) memcpy(test_cmd, g_motor_test_cmd, sizeof(test_cmd));
            chMtxUnlock(&motor_test_mtx);

            if (test_active) {
                motor_output_write(test_cmd);
                chMtxLock(&state_mtx);
                memset(g_output, 0, sizeof(g_output));
                chMtxUnlock(&state_mtx);
                TIMING_TICK_END(tid);
                next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
                continue;
            }
        }

        /* ── Full cascade: EKF state → FlightStateMachine → mixer → DShot ─ */
        float ctrl_full[StateIdx::N];
        float euler[3];
        float input[InputIdx::N_INPUTS];
        bool  armed;
        bool  use_indi;
        chMtxLock(&state_mtx);
        memcpy(ctrl_full, g_state,  sizeof(ctrl_full));
        memcpy(euler,     g_euler,  sizeof(euler));
        memcpy(input,     g_input,  sizeof(input));
        armed    = g_armed;
        use_indi = g_use_indi;
        chMtxUnlock(&state_mtx);

        flight_sm.set_use_indi(use_indi);

        ESCTelemetry sm_telem[4];
        dshot_get_telemetry(sm_telem);
        uint32_t rpm[4];
        for (int i = 0; i < 4; i++) {
            const uint32_t raw = sm_telem[i].valid ? sm_telem[i].erpm / 7U : 0U;
            rpm[i] = rpm_gate(s_rpm_gate[i], raw);
        }
        chMtxLock(&esc_mtx);
        memcpy(g_rpm_gated, rpm, sizeof(g_rpm_gated));
        chMtxUnlock(&esc_mtx);

        float torque_cmds[3];
        float thrust;
        flight_sm.update(ctrl_full, euler, input, armed, rpm, torque_cmds, thrust);

        float indi_diag[8];
        flight_sm.get_indi_diag(indi_diag);

        float ctun_diag[12];
        flight_sm.get_ctun_diag(ctun_diag);

        // MotorMixer disarm check uses state[0]=roll, state[1]=pitch.
        const float safety_state[2] = { euler[0], euler[1] };
        int32_t motor_out[4];
        mixer.update(torque_cmds, thrust, armed, safety_state, motor_out);
        motor_output_write(motor_out);

        chMtxLock(&state_mtx);
        g_ctrl[0] = torque_cmds[0];
        g_ctrl[1] = torque_cmds[1];
        g_ctrl[2] = torque_cmds[2];
        g_ctrl[3] = thrust;
        memcpy(g_indi_diag, indi_diag, sizeof(g_indi_diag));
        memcpy(g_ctun_diag, ctun_diag, sizeof(g_ctun_diag));
        memcpy(g_output, motor_out, sizeof(g_output));
        g_flight_mode = (int)flight_sm.mode();
        chMtxUnlock(&state_mtx);

        TIMING_TICK_END(tid);
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * RadioThread — 100 Hz  NORMALPRIO+10
 * Reads RC input and writes g_input / g_armed.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(RadioThread, arg)
{
    chRegSetThreadName("radio");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);
    const int tid = TIMING_REGISTER("radio", period);

    systime_t next = chVTGetSystemTime();
    while (true) {
        TIMING_TICK_BEGIN(tid);
        radio_input_update();

        chMtxLock(&state_mtx);
        g_input[InputIdx::THRUST]      = radio_thr();
        g_input[InputIdx::ROLL_TGT]    = radio_roll();
        g_input[InputIdx::PITCH_TGT]   = radio_pitch();
        g_input[InputIdx::YAW_RATE]    = radio_yaw();
        g_input[InputIdx::FLIGHT_MODE] = radio_flight_mode();
        g_input[InputIdx::INDI_STK]    = radio_indi();
        g_armed    = radio_armed();
        g_use_indi = radio_use_indi();
        chMtxUnlock(&state_mtx);

        TIMING_TICK_END(tid);
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * HeartbeatThread — 1 Hz  NORMALPRIO-5
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(HeartbeatThread, arg)
{
    (void)arg;
    chRegSetThreadName("heartbeat");

    /* Simplified heartbeat: LED blink + DShot diagnostics over USB.
     * IMU/EKF output removed since SPIThread and StateEstThread are not
     * running in the motor-test configuration.
     *
     * Every 2 s:
     *   $DSHOT,<ms>,tc=<tim1_dma_tc>/<tim4_dma_tc>,cc=<tim1_cc>/<tim4_cc>,
     *           edges=<m0>/<m1>/<m2>/<m3>
     *
     * tc increasing → DShot DMA is firing → signal is being sent.
     * edges > 0     → ESC is responding with GCR telemetry.
     * tc = 0        → DShot is NOT running (check dshot_init failure).
     */
    // NB: rates.heartbeat (passed as arg) is unused — this thread hardcodes
    // its own 200 ms tick below. Registered with that literal so the timing
    // report's period matches what actually runs, not the configured-but-
    // ignored rate.
    const int tid = TIMING_REGISTER("heartbeat", TIME_MS2I(200));

    uint32_t tick = 0;
    systime_t next = chVTGetSystemTime();
    while (true) {
        TIMING_TICK_BEGIN(tid);
        /* LED: 200 ms flash every 2 s (every 10th 200 ms tick). */
        if (tick % 10 == 0) palSetLine(LINE_LED_ACTIVITY);
        if (tick % 10 == 1) palClearLine(LINE_LED_ACTIVITY);
        TIMING_TICK_END(tid);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, TIME_MS2I(200)));
        tick++;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * USBCmdThread — event-driven  NORMALPRIO-20  (always built)
 * Blocks waiting for commands on USB CDC (SDU1).  Handles motor test
 * and SD log access.  Sleeps until a byte arrives — near-zero overhead
 * when the drone is not plugged in to a PC.
 * ══════════════════════════════════════════════════════════════════════════ */

static void usb_log_list(void)
{
    DIR      dir;
    FILINFO  fno;
    if (f_opendir(&dir, "/LOGS") != FR_OK) {
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "LOG,ERR,no_sd\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        return;
    }
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        if (fno.fattrib & AM_DIR) continue;
        /* Each chprintf is individually locked — FatFS I/O between entries is
         * NOT under the mutex so DebugThread is not stalled by SD card reads. */
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "LOG,FILE,%s,%lu\r\n", fno.fname, (uint32_t)fno.fsize);
        chMtxUnlock(&s_usb_write_mtx);
    }
    f_closedir(&dir);
    chMtxLock(&s_usb_write_mtx);
    chprintf((BaseSequentialStream *)&SDU1, "LOG,LIST,END\r\n");
    chMtxUnlock(&s_usb_write_mtx);
}

static void usb_log_get(const char *fname)
{
    char path[32];
    snprintf(path, sizeof(path), "/LOGS/%s", fname);

    FILINFO fno;
    if (f_stat(path, &fno) != FR_OK) {
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "LOG,ERR,notfound\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        return;
    }

    /* Open before sending the size header so we can report open errors cleanly
     * without having already committed the SIZE handshake to the client. */
    FIL fil;
    if (f_open(&fil, path, FA_READ) != FR_OK) {
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "LOG,ERR,open_failed\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        return;
    }

    /* Hold the mutex for the ENTIRE SIZE header + binary payload.
     * DebugThread uses chMtxTryLock, so it skips $TEL ticks rather than
     * injecting bytes into the middle of the binary stream. */
    chMtxLock(&s_usb_write_mtx);
    chprintf((BaseSequentialStream *)&SDU1, "LOG,SIZE,%lu\r\n", (uint32_t)fno.fsize);
    UINT br;
    while (f_read(&fil, s_usb_dl_buf, sizeof(s_usb_dl_buf), &br) == FR_OK && br > 0) {
        chnWriteTimeout((BaseChannel *)&SDU1, s_usb_dl_buf, br, TIME_MS2I(2000));
    }
    chMtxUnlock(&s_usb_write_mtx);
    f_close(&fil);
}

static void usb_log_erase(void)
{
    DIR     dir;
    FILINFO fno;
    char    path[32];
    int     erased = 0;

    /* Identify the currently-open log file so we do not delete it. */
    const char *cur     = logger.current_path();
    const char *cur_fn  = nullptr;
    if (cur) {
        cur_fn = strrchr(cur, '/');
        if (cur_fn) cur_fn++;  // skip the '/'
    }

    if (f_opendir(&dir, "/LOGS") != FR_OK) {
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "LOG,ERR,no_sd\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        return;
    }
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        if (fno.fattrib & AM_DIR) continue;
        if (cur_fn && strcmp(fno.fname, cur_fn) == 0) continue;  // skip active file
        snprintf(path, sizeof(path), "/LOGS/%s", fno.fname);
        if (f_unlink(path) == FR_OK) erased++;
    }
    f_closedir(&dir);
    chMtxLock(&s_usb_write_mtx);
    chprintf((BaseSequentialStream *)&SDU1, "LOG,ERASED,%d\r\n", erased);
    chMtxUnlock(&s_usb_write_mtx);
}

/* Staging buffer for CAL,set commands; written to flash by CAL,commit. */
static CalibData s_cal_stage = {};

static void usb_cmd_dispatch(const char *line)
{
    if (strcmp(line, "BOOT") == 0) {
        /* Reboot into the CubePilot bootloader for firmware upload. */
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "BOOT,OK\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        chThdSleepMilliseconds(50);   /* flush before reset */
        NVIC_SystemReset();
        /* unreachable */
    } else if (strncmp(line, "MT,", 3) == 0) {
        const char *rest = line + 3;
        if (strcmp(rest, "stop") == 0) {
            chMtxLock(&motor_test_mtx);
            g_motor_test_active = false;
            memset(g_motor_test_cmd, 0, sizeof(g_motor_test_cmd));
            chMtxUnlock(&motor_test_mtx);
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "MT,OK,stopped\r\n");
            chMtxUnlock(&s_usb_write_mtx);
            return;
        }
        int motor = -1, pct = -1;
        if (sscanf(rest, "%d,%d", &motor, &pct) != 2 ||
            motor < 0 || motor > 3 || pct < 0 || pct > 100) {
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "MT,ERR,bad_args\r\n");
            chMtxUnlock(&s_usb_write_mtx);
            return;
        }
        chMtxLock(&state_mtx);
        bool armed = g_armed;
        chMtxUnlock(&state_mtx);
        if (armed) {
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "MT,ERR,armed\r\n");
            chMtxUnlock(&s_usb_write_mtx);
            return;
        }
        int32_t dv = pct * 10;  // 0–100% → 0–1000
        chMtxLock(&motor_test_mtx);
        g_motor_test_active = true;
        memset(g_motor_test_cmd, 0, sizeof(g_motor_test_cmd));
        g_motor_test_cmd[motor] = dv;
        chMtxUnlock(&motor_test_mtx);
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "MT,OK,%d,%d\r\n", motor, pct);
        chMtxUnlock(&s_usb_write_mtx);

    } else if (strncmp(line, "LOG,", 4) == 0) {
        const char *rest = line + 4;
        if (strcmp(rest, "status") == 0) {
            static const char *const err_str[] = {
                "not_tried", "sdcStart", "sdcConnect", "f_mount", "f_open", "ok"
            };
            uint8_t e  = logger.last_init_err();
            uint8_t ff = logger.last_ff_err();
            const char *es = (e <= 5) ? err_str[e] : "unknown";
            chMtxLock(&s_usb_write_mtx);
            if (logger.is_ready()) {
                chprintf((BaseSequentialStream *)&SDU1,
                         "LOG,STATUS,ready,file=%s,expand_err=%u\r\n",
                         logger.current_path(), (unsigned)logger.expand_err());
            } else if (ff != 0) {
                chprintf((BaseSequentialStream *)&SDU1,
                         "LOG,STATUS,not_ready,last_err=%u(%s),ff=%u\r\n",
                         (unsigned)e, es, (unsigned)ff);
            } else {
                chprintf((BaseSequentialStream *)&SDU1,
                         "LOG,STATUS,not_ready,last_err=%u(%s)\r\n", (unsigned)e, es);
            }
            chMtxUnlock(&s_usb_write_mtx);
        } else if (strcmp(rest, "list") == 0) {
            usb_log_list();
        } else if (strncmp(rest, "get,", 4) == 0) {
            usb_log_get(rest + 4);
        } else if (strcmp(rest, "erase") == 0) {
            usb_log_erase();
        } else {
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "LOG,ERR,unknown_cmd\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        }
    } else if (strncmp(line, "CAL,", 4) == 0) {
        const char *rest = line + 4;
        if (strncmp(rest, "set,", 4) == 0) {
            // CAL,set,<i>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>
            int   idx = -1;
            float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
            if (sscanf(rest + 4, "%d,%f,%f,%f,%f,%f,%f",
                       &idx, &gx, &gy, &gz, &ax, &ay, &az) == 7
                && idx >= 0 && idx <= 2)
            {
                s_cal_stage.gyro_bias[idx][0] = gx;
                s_cal_stage.gyro_bias[idx][1] = gy;
                s_cal_stage.gyro_bias[idx][2] = gz;
                s_cal_stage.accel_bias[idx][0] = ax;
                s_cal_stage.accel_bias[idx][1] = ay;
                s_cal_stage.accel_bias[idx][2] = az;
                chMtxLock(&s_usb_write_mtx);
                chprintf((BaseSequentialStream *)&SDU1, "CAL,SET,%d,OK\r\n", idx);
                chMtxUnlock(&s_usb_write_mtx);
            } else {
                chMtxLock(&s_usb_write_mtx);
                chprintf((BaseSequentialStream *)&SDU1, "CAL,ERR,bad_args\r\n");
                chMtxUnlock(&s_usb_write_mtx);
            }
        } else if (strcmp(rest, "commit") == 0) {
            bool ok = cal_save(s_cal_stage);
            if (ok) {
                // Apply immediately so current session benefits without reboot
                g_cal = s_cal_stage;
                g_cal_valid = true;
            }
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1,
                     ok ? "CAL,OK\r\n" : "CAL,ERR,write_failed\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        } else if (strcmp(rest, "clear") == 0) {
            cal_clear();
            memset(&g_cal, 0, sizeof(g_cal));
            g_cal_valid = false;
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "CAL,OK\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        } else if (strcmp(rest, "query") == 0) {
            CalibData stored = {};
            bool valid = cal_load(stored);
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1,
                "CAL,DATA,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\r\n",
                (double)stored.gyro_bias[0][0], (double)stored.gyro_bias[0][1],
                (double)stored.gyro_bias[0][2], (double)stored.gyro_bias[1][0],
                (double)stored.gyro_bias[1][1], (double)stored.gyro_bias[1][2],
                (double)stored.gyro_bias[2][0], (double)stored.gyro_bias[2][1],
                (double)stored.gyro_bias[2][2],
                (double)stored.accel_bias[0][0], (double)stored.accel_bias[0][1],
                (double)stored.accel_bias[0][2], (double)stored.accel_bias[1][0],
                (double)stored.accel_bias[1][1], (double)stored.accel_bias[1][2],
                (double)stored.accel_bias[2][0], (double)stored.accel_bias[2][1],
                (double)stored.accel_bias[2][2], (int)valid);
            chMtxUnlock(&s_usb_write_mtx);
        } else {
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "CAL,ERR,unknown_cmd\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        }
    } else if (strcmp(line, "DSHOT,diag") == 0) {
        DShotDiag d = {};
        dshot_get_diag(&d);
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
            "DSHOT,DIAG,dma_tc=%u/%u,cc_isr=%u/%u,"
            "edges=%u/%u/%u/%u,"
            "e0=%u,%u,%u,%u,%u,"
            "e1=%u,%u,%u,%u,%u,"
            "e2=%u,%u,%u,%u,%u,"
            "e3=%u,%u,%u,%u,%u\r\n",
            (unsigned)d.dma_tc[0], (unsigned)d.dma_tc[1],
            (unsigned)d.cc_isr[0], (unsigned)d.cc_isr[1],
            (unsigned)d.edge_cnt[0], (unsigned)d.edge_cnt[1],
            (unsigned)d.edge_cnt[2], (unsigned)d.edge_cnt[3],
            (unsigned)d.edges[0][0], (unsigned)d.edges[0][1],
            (unsigned)d.edges[0][2], (unsigned)d.edges[0][3],
            (unsigned)d.edges[0][4],
            (unsigned)d.edges[1][0], (unsigned)d.edges[1][1],
            (unsigned)d.edges[1][2], (unsigned)d.edges[1][3],
            (unsigned)d.edges[1][4],
            (unsigned)d.edges[2][0], (unsigned)d.edges[2][1],
            (unsigned)d.edges[2][2], (unsigned)d.edges[2][3],
            (unsigned)d.edges[2][4],
            (unsigned)d.edges[3][0], (unsigned)d.edges[3][1],
            (unsigned)d.edges[3][2], (unsigned)d.edges[3][3],
            (unsigned)d.edges[3][4]);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "CAN,status") == 0) {
        // Read FDCAN1 diagnostic registers directly (no driver API needed).
        // PSR: protocol status (ACT, LEC, EP, EW, BO).
        // ECR: error counters (TEC[7:0], REC[14:8], RP[15]).
        // RXF0S: RxFIFO0 fill level (F0FL[6:0]) and overflow flag (RF0L[25]).
        const uint32_t psr   = FDCAN1->PSR;
        const uint32_t ecr   = FDCAN1->ECR;
        const uint32_t rxf0s = FDCAN1->RXF0S;
        const uint32_t cccr  = FDCAN1->CCCR;
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "CAN,STATUS,psr=0x%08x,ecr=0x%08x,rxf0s=0x%08x,cccr=0x%08x\r\n",
                 (unsigned)psr, (unsigned)ecr, (unsigned)rxf0s, (unsigned)cccr);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "CAN,diag") == 0) {
        CANDiag d = {};
        can_get_diag(d);
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "CAN,DIAG,total_rx=%lu,dispatched=%lu,"
                 "last_sid=0x%03x,last_eid=0x%08lx,last_eff=%u,"
                 "last_dlc=%u,last_data=%02x%02x%02x%02x%02x%02x%02x%02x,"
                 "msg_lost=%lu,reinit_count=%lu\r\n",
                 (uint32_t)d.total_rx, (uint32_t)d.dispatched,
                 (unsigned)d.last_sid, (uint32_t)d.last_eid, (unsigned)d.last_eff,
                 (unsigned)d.last_dlc,
                 d.last_data[0], d.last_data[1], d.last_data[2], d.last_data[3],
                 d.last_data[4], d.last_data[5], d.last_data[6], d.last_data[7],
                 (uint32_t)d.msg_lost, (uint32_t)d.reinit_count);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "MAV,diag") == 0) {
        MavlinkDiag d = {};
        mavlink_get_diag(d);
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "MAV,DIAG,bytes_rx=%lu,frames_ok=%lu,frames_bad_crc=%lu,"
                 "heartbeat_rx=%lu,param_req_rx=%lu,vision_pos_rx=%lu,"
                 "vision_speed_rx=%lu,unknown_rx=%lu\r\n",
                 (uint32_t)d.bytes_rx, (uint32_t)d.frames_ok, (uint32_t)d.frames_bad_crc,
                 (uint32_t)d.heartbeat_rx, (uint32_t)d.param_req_rx, (uint32_t)d.vision_pos_rx,
                 (uint32_t)d.vision_speed_rx, (uint32_t)d.unknown_rx);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "CAN,scan,start") == 0) {
        can_scan_start();
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "CAN,SCAN,started\r\n");
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "CAN,scan,stop") == 0) {
        can_scan_stop();
        CANScanEntry entries[CAN_SCAN_MAX];
        int n = can_scan_get(entries, CAN_SCAN_MAX);
        chMtxLock(&s_usb_write_mtx);
        for (int i = 0; i < n; i++) {
            chprintf((BaseSequentialStream *)&SDU1,
                     "CAN,SCAN,id=%s0x%03lx,count=%lu\r\n",
                     entries[i].is_ext ? "EXT:" : "",
                     (uint32_t)entries[i].id,
                     (uint32_t)entries[i].count);
        }
        chprintf((BaseSequentialStream *)&SDU1, "CAN,SCAN,END\r\n");
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "CAN,regdump") == 0) {
        CANRegEntry regs[12];
        int n = can_read_regs(regs, 12);
        chMtxLock(&s_usb_write_mtx);
        for (int i = 0; i < n; i++) {
            chprintf((BaseSequentialStream *)&SDU1,
                     "CAN,REG,%s=0x%08lx\r\n",
                     regs[i].name, (uint32_t)regs[i].value);
        }
        chprintf((BaseSequentialStream *)&SDU1, "CAN,REG,END\r\n");
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "STRAIN_RATE,read") == 0) {
        chMtxLock(&strainRate_mtx);
        StrainRateRaw snap = g_strain_rate;
        chMtxUnlock(&strainRate_mtx);
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "STRAIN_RATE,%d,%d,%d,%d,%u\r\n",
                 (int)snap.val[0], (int)snap.val[1],
                 (int)snap.val[2], (int)snap.val[3],
                 (unsigned)snap.valid);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "I2C,scan") == 0) {
        // Probe every valid I2C address (0x08–0x77) and report which ones ACK.
        // Use a 2 ms timeout per probe so a stuck device can't hang the scan
        // forever and starve the I2CThread of bus access.
        uint8_t acked[16] = {};  // bitmask: bit (addr&7) of byte (addr>>3)
        uint8_t dummy[1];
        i2cAcquireBus(&I2CD2);
        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            if (i2cMasterReceiveTimeout(&I2CD2, addr, dummy, 1, TIME_MS2I(2)) == MSG_OK) {
                acked[addr >> 3] |= (uint8_t)(1U << (addr & 7U));
            }
        }
        i2cReleaseBus(&I2CD2);
        chMtxLock(&s_usb_write_mtx);
        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            if (acked[addr >> 3] & (1U << (addr & 7U))) {
                chprintf((BaseSequentialStream *)&SDU1,
                         "I2C,SCAN,0x%02x,ack\r\n", (unsigned)addr);
            }
        }
        chprintf((BaseSequentialStream *)&SDU1, "I2C,SCAN,END\r\n");
        chMtxUnlock(&s_usb_write_mtx);
#ifdef BPRL_TIMING
    } else if (strcmp(line, "TIM,status") == 0) {
        static char rep_buf[1536];
        size_t len = timing_format_report(rep_buf, sizeof(rep_buf));
        chMtxLock(&s_usb_write_mtx);
        chnWriteTimeout((BaseChannel *)&SDU1, (uint8_t *)rep_buf, len, TIME_MS2I(200));
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "TIM,reset") == 0) {
        timing_reset();
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "TIM,OK,reset\r\n");
        chMtxUnlock(&s_usb_write_mtx);
#endif
    }
}

static THD_FUNCTION(USBCmdThread, arg)
{
    chRegSetThreadName("usbcmd");
    (void)arg;

    static char   s_line[64];
    static uint8_t s_len = 0;

    const int tid = TIMING_REGISTER("usbcmd", 0);  // event-driven, no fixed period

    while (true) {
        msg_t byte = chnGetTimeout((BaseChannel *)&SDU1, TIME_MS2I(50));
        if (byte == MSG_TIMEOUT || byte == MSG_RESET) continue;

        char c = (char)byte;
        if (c == '\n' || c == '\r') {
            if (s_len > 0) {
                s_line[s_len] = '\0';
                TIMING_TICK_BEGIN(tid);
                usb_cmd_dispatch(s_line);
                TIMING_TICK_END(tid);
                s_len = 0;
            }
        } else if (s_len < (uint8_t)(sizeof(s_line) - 1U)) {
            s_line[s_len++] = c;
        } else {
            s_len = 0;  // line too long — reset
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * DebugThread — 10 Hz  NORMALPRIO-10  [BPRL_DEBUG only]
 * Streams a $TEL telemetry line over USB CDC at 10 Hz.
 * Format:
 *   $TEL,<time_ms>,<roll°>,<pitch°>,<yaw°>,<p>,<q>,<r>,<thr>,
 *        <rc_roll>,<rc_pitch>,<rc_yaw>,<armed>,
 *        <rpm0>,<rpm1>,<rpm2>,<rpm3>,
 *        <imu0_v>,<imu1_v>,<imu2_v>,<can_v>,<can_quat_hz>,<can_rate_hz>,
 *        <flight_mode>,         ← FlightMode enum (0=STABILIZE, 1=ALT_HOLD, 2=POS_HOLD)
 *        <use_indi>             ← active attitude controller (0=PID, 1=INDI)
 * ══════════════════════════════════════════════════════════════════════════ */
#ifdef BPRL_DEBUG
static THD_FUNCTION(DebugThread, arg)
{
    chRegSetThreadName("dbg");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    uint32_t prev_quat_cnt = 0, prev_rate_cnt = 0;
    uint32_t can_quat_hz   = 0, can_rate_hz   = 0;
    int      rate_tick     = 0;

    const int tid = TIMING_REGISTER("debug", period);

    systime_t next = chVTGetSystemTime();
    while (true) {
        TIMING_TICK_BEGIN(tid);
        /* ── Update CAN INS rate estimate once per second (10 ticks) ───── */
        if (++rate_tick >= 10) {
            uint32_t qc = s_can_quat_cnt;
            uint32_t rc = s_can_rate_cnt;
            can_quat_hz   = qc - prev_quat_cnt;
            can_rate_hz   = rc - prev_rate_cnt;
            prev_quat_cnt = qc;
            prev_rate_cnt = rc;
            rate_tick     = 0;
        }

        /* ── Snapshot flight state ──────────────────────────────────────── */
        float roll, pitch, yaw, thr, rc_roll, rc_pitch, rc_yaw;
        float p, q, r;
        float pos_x, pos_y, pos_z, vel_u, vel_v, vel_w;
        float lane_roll[3], lane_pitch[3], lane_yaw[3];
        float lane_p[3], lane_q[3], lane_r[3];
        int   primary_lane;
        bool  armed;
        int   flight_mode;
        bool  use_indi;
        chMtxLock(&state_mtx);
        roll     = g_euler[0];
        pitch    = g_euler[1];
        yaw      = g_euler[2];
        p        = g_state[StateIdx::P];
        q        = g_state[StateIdx::Q];
        r        = g_state[StateIdx::R];
        pos_x    = g_state[StateIdx::X];
        pos_y    = g_state[StateIdx::Y];
        pos_z    = g_state[StateIdx::Z_POS];
        vel_u    = g_state[StateIdx::U];
        vel_v    = g_state[StateIdx::V];
        vel_w    = g_state[StateIdx::W];
        thr      = g_input[InputIdx::THRUST];
        rc_roll  = g_input[InputIdx::ROLL_TGT];
        rc_pitch = g_input[InputIdx::PITCH_TGT];
        rc_yaw   = g_input[InputIdx::YAW_RATE];
        armed    = g_armed;
        flight_mode = g_flight_mode;
        use_indi = g_use_indi;
        for (int li = 0; li < 3; ++li) {
            lane_roll[li]  = s_lane_roll[li];
            lane_pitch[li] = s_lane_pitch[li];
            lane_yaw[li]   = s_lane_yaw[li];
            lane_p[li]     = s_lane_p[li];
            lane_q[li]     = s_lane_q[li];
            lane_r[li]     = s_lane_r[li];
        }
        primary_lane = s_primary_lane;
        chMtxUnlock(&state_mtx);

        /* ── Snapshot full IMU data ─────────────────────────────────────── */
        float imu_ax[3], imu_ay[3], imu_az[3];
        float imu_gx[3], imu_gy[3], imu_gz[3];
        bool  imu_v[3];
        chMtxLock(&imu_mtx);
        for (int _i = 0; _i < 3; _i++) {
            imu_ax[_i] = g_imu[_i].accel[0];
            imu_ay[_i] = g_imu[_i].accel[1];
            imu_az[_i] = g_imu[_i].accel[2];
            imu_gx[_i] = g_imu[_i].gyro[0];
            imu_gy[_i] = g_imu[_i].gyro[1];
            imu_gz[_i] = g_imu[_i].gyro[2];
            imu_v[_i]  = g_imu[_i].valid;
        }
        chMtxUnlock(&imu_mtx);

        float can_p_snap, can_q_snap, can_r_snap;
        float can_qw, can_qx, can_qy, can_qz;
        bool can_v;
        chMtxLock(&can_imu_mtx);
        can_v      = g_can_imu.valid;
        can_p_snap = g_can_imu.p;
        can_q_snap = g_can_imu.q;
        can_r_snap = g_can_imu.r;
        can_qw     = g_can_imu.q0;
        can_qx     = g_can_imu.q1;
        can_qy     = g_can_imu.q2;
        can_qz     = g_can_imu.q3;
        chMtxUnlock(&can_imu_mtx);

        /* ── Snapshot mocap ground truth ────────────────────────────────── */
        float mocap_x, mocap_y, mocap_z, mocap_vx, mocap_vy, mocap_vz;
        bool  mocap_v;
        chMtxLock(&mocap_mtx);
        mocap_x  = g_mocap.x;
        mocap_y  = g_mocap.y;
        mocap_z  = g_mocap.z;
        mocap_vx = g_mocap.vx;
        mocap_vy = g_mocap.vy;
        mocap_vz = g_mocap.vz;
        mocap_v  = g_mocap.valid;
        chMtxUnlock(&mocap_mtx);

        // Quaternion → ZYX Euler (deg) for the CAN IMX5 lane in $EKFL.
        // Always computed from the stored quaternion so the display shows the
        // last known attitude instead of zeroing out during brief dropouts.
        // can_v = false just means the data is stale, not that q0-q3 are gone.
        float roll_r  = atan2f(2.0f*(can_qw*can_qx + can_qy*can_qz),
                               1.0f - 2.0f*(can_qx*can_qx + can_qy*can_qy));
        float pitch_r = asinf(fmaxf(-1.0f, fminf(1.0f,
                               2.0f*(can_qw*can_qy - can_qz*can_qx))));
        float yaw_r   = atan2f(2.0f*(can_qw*can_qz + can_qx*can_qy),
                               1.0f - 2.0f*(can_qy*can_qy + can_qz*can_qz));
        float can_roll_deg  = roll_r  * 57.2958f;
        float can_pitch_deg = pitch_r * 57.2958f;
        float can_yaw_deg   = yaw_r   * 57.2958f;

        /* ── Snapshot fault-gated RPM (published by ControlThread) ─────── */
        uint32_t rpm[4];
        chMtxLock(&esc_mtx);
        memcpy(rpm, g_rpm_gated, sizeof(rpm));
        chMtxUnlock(&esc_mtx);

        /* ── Emit $TEL line over USB ────────────────────────────────────── */
        /* Format into a local buffer first (non-blocking), then send with a
         * real timeout.  This avoids the TIME_INFINITE block that chprintf
         * directly on SDU1 uses when the USB output queue is full, and removes
         * the USB_ACTIVE guard (chnWriteTimeout handles a disconnected port
         * gracefully by returning an error rather than hanging). */
        {
            static char tel_buf[256];
            MemoryStream ms;
            msObjectInit(&ms, (uint8_t *)tel_buf, sizeof(tel_buf) - 1, 0);
            chprintf((BaseSequentialStream *)&ms,
                "$TEL,%lu,"
                "%.2f,%.2f,%.2f,"
                "%.4f,%.4f,%.4f,"
                "%.3f,%.3f,%.3f,%.3f,%d,"
                "%lu,%lu,%lu,%lu,"
                "%d,%d,%d,%d,%lu,%lu,"
                "%d,%d\r\n",
                (uint32_t)TIME_I2MS(chVTGetSystemTime()),
                (double)(roll  * 57.2958f),
                (double)(pitch * 57.2958f),
                (double)(yaw   * 57.2958f),
                (double)p, (double)q, (double)r,
                (double)thr,
                (double)rc_roll, (double)rc_pitch, (double)rc_yaw,
                (int)armed,
                rpm[0], rpm[1], rpm[2], rpm[3],
                (int)imu_v[0], (int)imu_v[1], (int)imu_v[2],
                (int)can_v,
                can_quat_hz, can_rate_hz,
                flight_mode, (int)use_indi);
            size_t tlen = ms.eos;
            if (tlen > 0 && chMtxTryLock(&s_usb_write_mtx)) {
                chnWriteTimeout((BaseChannel *)&SDU1,
                                (uint8_t *)tel_buf, tlen, TIME_MS2I(50));
                chMtxUnlock(&s_usb_write_mtx);
            }
        }

        /* ── Emit $EKFL per-lane line over USB ─────────────────────────── */
        /* Format:
         *   $EKFL,<ms>,<primary>,
         *         <roll0°>,<pitch0°>,<yaw0°>,<p0>,<q0>,<r0>,
         *         <roll1°>,<pitch1°>,<yaw1°>,<p1>,<q1>,<r1>,
         *         <roll2°>,<pitch2°>,<yaw2°>,<p2>,<q2>,<r2>,
         *         <can_roll°>,<can_pitch°>,<can_yaw°>,<can_p>,<can_q>,<can_r>  ← IMX5 INS (raw, from g_can_imu)
         */
        {
            static char ekfl_buf[256];
            MemoryStream ekfl_ms;
            msObjectInit(&ekfl_ms, (uint8_t *)ekfl_buf, sizeof(ekfl_buf) - 1, 0);
            chprintf((BaseSequentialStream *)&ekfl_ms,
                "$EKFL,%lu,%d,"
                "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,"
                "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,"
                "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,"
                "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\r\n",
                (uint32_t)TIME_I2MS(chVTGetSystemTime()), primary_lane,
                (double)(lane_roll[0]*57.2958f), (double)(lane_pitch[0]*57.2958f),
                (double)(lane_yaw[0]*57.2958f),
                (double)lane_p[0], (double)lane_q[0], (double)lane_r[0],
                (double)(lane_roll[1]*57.2958f), (double)(lane_pitch[1]*57.2958f),
                (double)(lane_yaw[1]*57.2958f),
                (double)lane_p[1], (double)lane_q[1], (double)lane_r[1],
                (double)(lane_roll[2]*57.2958f), (double)(lane_pitch[2]*57.2958f),
                (double)(lane_yaw[2]*57.2958f),
                (double)lane_p[2], (double)lane_q[2], (double)lane_r[2],
                (double)can_roll_deg, (double)can_pitch_deg, (double)can_yaw_deg,
                (double)can_p_snap, (double)can_q_snap, (double)can_r_snap);
            size_t elen = ekfl_ms.eos;
            if (elen > 0 && chMtxTryLock(&s_usb_write_mtx)) {
                chnWriteTimeout((BaseChannel *)&SDU1,
                                (uint8_t *)ekfl_buf, elen, TIME_MS2I(50));
                chMtxUnlock(&s_usb_write_mtx);
            }
        }

        /* ── Emit $IMU line over USB (raw sensor + CAN IMU rates) ─────── */
        /* Format: $IMU,<ms>,<ax0>,<ay0>,<az0>,<gx0>,<gy0>,<gz0>,<v0>, ...×3,
         *               <can_p>,<can_q>,<can_r>,<can_v>                       */
        {
            static char imu_buf[300];
            MemoryStream imu_ms;
            msObjectInit(&imu_ms, (uint8_t *)imu_buf, sizeof(imu_buf) - 1, 0);
            chprintf((BaseSequentialStream *)&imu_ms,
                "$IMU,%lu,"
                "%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%d,"
                "%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%d,"
                "%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%d,"
                "%.5f,%.5f,%.5f,%d\r\n",
                (uint32_t)TIME_I2MS(chVTGetSystemTime()),
                (double)imu_ax[0], (double)imu_ay[0], (double)imu_az[0],
                (double)imu_gx[0], (double)imu_gy[0], (double)imu_gz[0], (int)imu_v[0],
                (double)imu_ax[1], (double)imu_ay[1], (double)imu_az[1],
                (double)imu_gx[1], (double)imu_gy[1], (double)imu_gz[1], (int)imu_v[1],
                (double)imu_ax[2], (double)imu_ay[2], (double)imu_az[2],
                (double)imu_gx[2], (double)imu_gy[2], (double)imu_gz[2], (int)imu_v[2],
                (double)can_p_snap, (double)can_q_snap, (double)can_r_snap, (int)can_v);
            size_t ilen = imu_ms.eos;
            if (ilen > 0 && chMtxTryLock(&s_usb_write_mtx)) {
                chnWriteTimeout((BaseChannel *)&SDU1,
                                (uint8_t *)imu_buf, ilen, TIME_MS2I(50));
                chMtxUnlock(&s_usb_write_mtx);
            }
        }

        /* ── Emit $POS line over USB (EKF position/velocity + mocap truth) */
        /* Format: $POS,<ms>,
         *   <ekf_x>,<ekf_y>,<ekf_z>,          NED position (m)
         *   <ekf_u>,<ekf_v>,<ekf_w>,          body-frame velocity (m/s)
         *   <mocap_x>,<mocap_y>,<mocap_z>,    NED position (m)
         *   <mocap_vx>,<mocap_vy>,<mocap_vz>, NED velocity (m/s)
         *   <mocap_valid>                                                  */
        {
            static char pos_buf[256];
            MemoryStream pos_ms;
            msObjectInit(&pos_ms, (uint8_t *)pos_buf, sizeof(pos_buf) - 1, 0);
            chprintf((BaseSequentialStream *)&pos_ms,
                "$POS,%lu,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,%d\r\n",
                (uint32_t)TIME_I2MS(chVTGetSystemTime()),
                (double)pos_x, (double)pos_y, (double)pos_z,
                (double)vel_u, (double)vel_v, (double)vel_w,
                (double)mocap_x, (double)mocap_y, (double)mocap_z,
                (double)mocap_vx, (double)mocap_vy, (double)mocap_vz,
                (int)mocap_v);
            size_t plen = pos_ms.eos;
            if (plen > 0 && chMtxTryLock(&s_usb_write_mtx)) {
                chnWriteTimeout((BaseChannel *)&SDU1,
                                (uint8_t *)pos_buf, plen, TIME_MS2I(50));
                chMtxUnlock(&s_usb_write_mtx);
            }
        }

        TIMING_TICK_END(tid);
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}
#endif // BPRL_DEBUG

/* ══════════════════════════════════════════════════════════════════════════
 * LogThread — 50 Hz  NORMALPRIO-15
 * Logs 9 message types per tick: ATT, LIN, RCIN, OUTP, RPMS, STRN, IMU1, IMU2, IMU3.
 * Output files are compatible with UAV Log Viewer (plot.ardupilot.org).
 * Retries logger.init() every 5 s until an SD card is inserted.
 * ══════════════════════════════════════════════════════════════════════════ */

/* ══════════════════════════════════════════════════════════════════════════
 * MAVLinkThread — 100 Hz, NORMALPRIO-8
 * Slimmed-down MAVLink on TELEM2 (USART3, 115200 baud).
 * Sends heartbeat at 1 Hz; receives VISION_POSITION_ESTIMATE and
 * VISION_SPEED_ESTIMATE and writes them into g_mocap for the EKF.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(MAVLinkThread, arg)
{
    (void)arg;
    mavlink_comms_init();

    // Hardcodes its own 10 ms poll below (no ThreadRates entry) — registered
    // with that literal so the timing report reflects the real period.
    const int tid = TIMING_REGISTER("mavlink", TIME_MS2I(10));

    while (true) {
        TIMING_TICK_BEGIN(tid);
        mavlink_comms_update();
        TIMING_TICK_END(tid);
        chThdSleepMilliseconds(10);  // 100 Hz poll — sufficient to drain 115200 baud
    }
}

static THD_FUNCTION(LogThread, arg)
{
    chRegSetThreadName("log");
    const LogRates *rates = static_cast<const LogRates *>(arg);

    while (!logger.init()) {
        chThdSleepMilliseconds(5000);
    }

    const int tid = TIMING_REGISTER_SOFT("log", rates->period);

    systime_t next = chVTGetSystemTime();

    while (true) {
        TIMING_TICK_BEGIN(tid);
        /* Timestamp in microseconds (millisecond precision via TIME_I2MS). */
        const uint64_t t_us = (uint64_t)TIME_I2MS(chVTGetSystemTime()) * 1000ULL;

        /* ── State + controller snapshot (one mutex hold) ─────────────── */
        float euler[3], state[StateIdx::N], inp[InputIdx::N_INPUTS], ctrl[4], indi_diag[8], ctun_diag[12];
        bool  armed;
        chMtxLock(&state_mtx);
        memcpy(euler,     g_euler,     sizeof(euler));
        memcpy(state,     g_state,     sizeof(state));
        memcpy(inp,       g_input,     sizeof(inp));
        memcpy(ctrl,      g_ctrl,      sizeof(ctrl));
        memcpy(indi_diag, g_indi_diag, sizeof(indi_diag));
        memcpy(ctun_diag, g_ctun_diag, sizeof(ctun_diag));
        armed = g_armed;
        chMtxUnlock(&state_mtx);

        /* ── Strain rate snapshot ─────────────────────────────────────── */
        StrainRateRaw strain = {};
        chMtxLock(&strainRate_mtx);
        strain = g_strain_rate;
        chMtxUnlock(&strainRate_mtx);

        /* ── RPM snapshot (fault-gated, published by ControlThread) ───── */
        uint32_t rpm_log[4];
        chMtxLock(&esc_mtx);
        memcpy(rpm_log, g_rpm_gated, sizeof(rpm_log));
        chMtxUnlock(&esc_mtx);

        /* ── Barometer snapshot ───────────────────────────────────────── */
        BaroRaw baro_snap_log = {};
        chMtxLock(&baro_mtx);
        baro_snap_log = g_baro;
        chMtxUnlock(&baro_mtx);

        /* ── Mocap snapshot ──────────────────────────────────────────── */
        MocapRaw mocap_snap_log = {};
        chMtxLock(&mocap_mtx);
        mocap_snap_log = g_mocap;
        chMtxUnlock(&mocap_mtx);

        /* ── IMU snapshot ────────────────────────────────────────────── */
        IMURaw imu_snap_log[3];
        chMtxLock(&imu_mtx);
        memcpy(imu_snap_log, g_imu, sizeof(imu_snap_log));
        chMtxUnlock(&imu_mtx);

        /* ── ATT — angular states ─────────────────────────────────────── */
        {
            LogMsgATT msg = {};
            msg.time_us = t_us;
            msg.roll    = euler[0];
            msg.pitch   = euler[1];
            msg.yaw     = euler[2];
            msg.p       = state[StateIdx::P];
            msg.q       = state[StateIdx::Q];
            msg.r       = state[StateIdx::R];
            msg.p_dot   = state[StateIdx::P_DOT];
            msg.q_dot   = state[StateIdx::Q_DOT];
            msg.r_dot   = state[StateIdx::R_DOT];
            logger.write(LOG_MSG_ATT, msg);
        }

        /* ── LIN — linear states ──────────────────────────────────────── */
        {
            LogMsgLIN msg = {};
            msg.time_us = t_us;
            msg.x       = state[StateIdx::X];
            msg.y       = state[StateIdx::Y];
            msg.z       = state[StateIdx::Z_POS];
            msg.u       = state[StateIdx::U];
            msg.v       = state[StateIdx::V];
            msg.w       = state[StateIdx::W];
            msg.u_dot   = state[StateIdx::U_DOT];
            msg.v_dot   = state[StateIdx::V_DOT];
            msg.w_dot   = state[StateIdx::W_DOT];
            logger.write(LOG_MSG_LIN, msg);
        }

        /* ── RCIN — RC stick inputs ───────────────────────────────────── */
        {
            LogMsgRCIN msg = {};
            msg.time_us    = t_us;
            msg.roll_stk   = inp[InputIdx::ROLL_TGT];
            msg.pitch_stk  = inp[InputIdx::PITCH_TGT];
            msg.yaw_stk    = inp[InputIdx::YAW_RATE];
            msg.thr_stk    = inp[InputIdx::THRUST];
            msg.flight_mode = inp[InputIdx::FLIGHT_MODE];
            msg.indi_stk   = inp[InputIdx::INDI_STK];
            msg.armed      = (uint8_t)armed;
            logger.write(LOG_MSG_RCIN, msg);
        }

        /* ── OUTP — controller outputs entering MotorMixer ───────────── */
        {
            LogMsgOUTP msg = {};
            msg.time_us  = t_us;
            msg.roll_tq  = ctrl[0];
            msg.pitch_tq = ctrl[1];
            msg.yaw_tq   = ctrl[2];
            msg.throttle = ctrl[3];
            logger.write(LOG_MSG_OUTP, msg);
        }

        /* ── INDI — shadow INDI controller diagnostics (always logged) ── */
        {
            LogMsgINDI msg = {};
            msg.time_us     = t_us;
            msg.unmix_roll  = indi_diag[0];
            msg.unmix_pitch = indi_diag[1];
            msg.delta_roll  = indi_diag[2];
            msg.delta_pitch = indi_diag[3];
            msg.cmd_roll    = indi_diag[4];
            msg.cmd_pitch   = indi_diag[5];
            msg.accel_roll  = indi_diag[6];
            msg.accel_pitch = indi_diag[7];
            logger.write(LOG_MSG_INDI, msg);
        }

#if LOG_CTUN_ENABLED
        /* ── CTUN — TEMP pos-hold NE tuning diagnostics ───────────────── */
        {
            LogMsgCTUN msg = {};
            msg.time_us   = t_us;
            msg.pos_n_tgt = ctun_diag[0];
            msg.pos_n_err = ctun_diag[1];
            msg.pos_e_tgt = ctun_diag[2];
            msg.pos_e_err = ctun_diag[3];
            msg.vel_n_tgt = ctun_diag[4];
            msg.vel_n_err = ctun_diag[5];
            msg.vel_e_tgt = ctun_diag[6];
            msg.vel_e_err = ctun_diag[7];
            msg.roll_tgt  = ctun_diag[8];
            msg.pitch_tgt = ctun_diag[9];
            msg.climb_rate_tgt = ctun_diag[10];
            msg.climb_rate_err = ctun_diag[11];
            logger.write(LOG_MSG_CTUN, msg);
        }
#endif

        /* ── RPMS — per-motor mechanical RPM ─────────────────────────── */
        {
            LogMsgRPMS msg = {};
            msg.time_us = t_us;
            msg.rpm0 = (int32_t)rpm_log[0];
            msg.rpm1 = (int32_t)rpm_log[1];
            msg.rpm2 = (int32_t)rpm_log[2];
            msg.rpm3 = (int32_t)rpm_log[3];
            logger.write(LOG_MSG_RPMS, msg);
        }

        /* ── STRN — strain rate sensor ────────────────────────────────── */
        {
            LogMsgSTRN msg = {};
            msg.time_us = t_us;
            msg.s0      = strain.val[0];
            msg.s1      = strain.val[1];
            msg.s2      = strain.val[2];
            msg.s3      = strain.val[3];
            msg.valid   = (uint8_t)strain.valid;
            logger.write(LOG_MSG_STRN, msg);
        }

        /* ── IMU1/IMU2/IMU3 — per-IMU raw accel + gyro ──────────────────── */
        {
            static constexpr uint8_t ids[3] = { LOG_MSG_IMU1, LOG_MSG_IMU2, LOG_MSG_IMU3 };
            for (uint8_t i = 0; i < 3; i++) {
                LogMsgIMU msg = {};
                msg.time_us = t_us;
                msg.ax    = imu_snap_log[i].accel[0];
                msg.ay    = imu_snap_log[i].accel[1];
                msg.az    = imu_snap_log[i].accel[2];
                msg.gx    = imu_snap_log[i].gyro[0];
                msg.gy    = imu_snap_log[i].gyro[1];
                msg.gz    = imu_snap_log[i].gyro[2];
                msg.valid = (uint8_t)imu_snap_log[i].valid;
                logger.write(ids[i], msg);
            }
        }

        /* ── BARO — barometric pressure/temperature/altitude ─────────── */
        {
            LogMsgBARO msg = {};
            msg.time_us     = t_us;
            msg.pressure_pa = baro_snap_log.pressure_pa;
            msg.temp_c      = baro_snap_log.temperature_c;
            msg.alt_m       = baro_snap_log.alt_m;
            msg.valid       = (uint8_t)baro_snap_log.valid;
            logger.write(LOG_MSG_BARO, msg);
        }

        /* ── MOCP — raw mocap position/velocity estimate, pre-EKF ─────── */
        {
            LogMsgMOCP msg = {};
            msg.time_us = t_us;
            msg.x       = mocap_snap_log.x;
            msg.y       = mocap_snap_log.y;
            msg.z       = mocap_snap_log.z;
            msg.vx      = mocap_snap_log.vx;
            msg.vy      = mocap_snap_log.vy;
            msg.vz      = mocap_snap_log.vz;
            msg.valid   = (uint8_t)mocap_snap_log.valid;
            logger.write(LOG_MSG_MOCP, msg);
        }

        logger.flush();

        /* If a write error killed the logger, attempt a clean restart.
         * 2-second cooldown prevents hammering a truly dead card. */
        if (!logger.is_ready()) {
            logger.close();
            chThdSleepMilliseconds(2000);
            logger.init();  // internally retries sdcConnect/f_mount up to 5×
        }

        TIMING_TICK_END(tid);
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, rates->period));
    }
}

/* ── Thread launcher (called from main) ──────────────────────────────────── */
void threads_start(const ThreadRates &rates)
{
    // Priority ordering (highest first):
    //   SPIThread       +30  1 kHz IMU reads
    //   CANThread       +28  event-driven CAN RX
    //   StateEstThread  +25  625 Hz EKF
    //   ControlThread   +22  400 Hz PID/mixer/DShot
    //   I2CThread       +20  500 Hz aux-sensor polling
    //   RadioThread     +10  100 Hz RC input
    //   HeartbeatThread  -5  LED + DShot diag
    //   MAVLinkThread    -8  100 Hz MAVLink on TELEM2 (vision position)
    //   DebugThread     -10  10 Hz $TEL/$EKFL USB stream
    //   LogThread       -15  50 Hz SD card logging
    //   USBCmdThread    -20  event-driven USB commands

    chThdCreateStatic(waSPI,       sizeof(waSPI),       NORMALPRIO + 30, SPIThread,       (void *)&rates.spi);
    chThdCreateStatic(waCAN,       sizeof(waCAN),       NORMALPRIO + 28, CANThread,       nullptr);
    chThdCreateStatic(waI2C,       sizeof(waI2C),       NORMALPRIO + 20, I2CThread,       (void *)&rates.i2c);
    chThdCreateStatic(waStateEst,  sizeof(waStateEst),  NORMALPRIO + 25, StateEstThread,  (void *)&rates.est);
    chThdCreateStatic(waControl,   sizeof(waControl),   NORMALPRIO + 22, ControlThread,   (void *)&rates.control);
    chThdCreateStatic(waRadio,     sizeof(waRadio),     NORMALPRIO + 10, RadioThread,     (void *)&rates.radio);
    chThdCreateStatic(waHeartbeat, sizeof(waHeartbeat), NORMALPRIO -  5, HeartbeatThread, (void *)&rates.heartbeat);
    chThdCreateStatic(waMAVLink,   sizeof(waMAVLink),   NORMALPRIO -  8, MAVLinkThread,   nullptr);
#ifdef BPRL_DEBUG
    chThdCreateStatic(waDebug,     sizeof(waDebug),     NORMALPRIO - 10, DebugThread,     (void *)&rates.debug);
#endif
    chThdCreateStatic(waUSBCmd,    sizeof(waUSBCmd),    NORMALPRIO - 20, USBCmdThread,    nullptr);

    chThdCreateStatic(waLog, sizeof(waLog), NORMALPRIO - 15, LogThread, (void *)&rates.log);
}
