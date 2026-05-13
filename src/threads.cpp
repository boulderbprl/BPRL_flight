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
#include "src/controllers/AttitudeController.hpp"
#include "src/controllers/MotorMixer.hpp"
#include "src/state_estimator/StateManager.hpp"
#include "src/logging/Logger.hpp"
#include "src/logging/LogMessages.hpp"
#include "chprintf.h"
#include <cstring>

/* ── Shared state definitions ────────────────────────────────────────────── */

MUTEX_DECL(state_mtx);
float   g_state[StateIdx::N] = {};   // 19-element EKF state vector
float   g_euler[3]           = {};   // [roll, pitch, yaw] derived from quaternion
float   g_input[4]           = {};
int32_t g_output[4]          = {};
bool    g_armed              = false;

MUTEX_DECL(imu_mtx);
IMURaw g_imu[3] = {};

MUTEX_DECL(can_imu_mtx);
CANIMURaw g_can_imu = {};

MUTEX_DECL(mocap_mtx);
MocapRaw  g_mocap   = {};

MUTEX_DECL(esc_mtx);
ESCTelemetry g_esc_telem[4] = {};

/* ── Controller instances (ControlThread only) ───────────────────────────── */
static AttitudeController att_ctrl;
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
#ifdef BPRL_DEBUG
static THD_WORKING_AREA(waDebug,    1024);
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * SPIThread — 1 kHz  NORMALPRIO+30
 * Reads raw accel+gyro from all three on-board IMUs.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(SPIThread, arg)
{
    chRegSetThreadName("spi");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    spi_drv_init();   // init all three IMUs (may sleep 100 ms each)

    systime_t next = chVTGetSystemTime();
    while (true) {
        float a[3], g[3];

        if (imu1.read(a, g)) {
            chMtxLock(&imu_mtx);
            memcpy(g_imu[0].accel, a, sizeof(a));
            memcpy(g_imu[0].gyro,  g, sizeof(g));
            g_imu[0].valid = true;
            chMtxUnlock(&imu_mtx);
        }
        if (imu2.read(a, g)) {
            chMtxLock(&imu_mtx);
            memcpy(g_imu[1].accel, a, sizeof(a));
            memcpy(g_imu[1].gyro,  g, sizeof(g));
            g_imu[1].valid = true;
            chMtxUnlock(&imu_mtx);
        }
        if (imu3.read(a, g)) {
            chMtxLock(&imu_mtx);
            memcpy(g_imu[2].accel, a, sizeof(a));
            memcpy(g_imu[2].gyro,  g, sizeof(g));
            g_imu[2].valid = true;
            chMtxUnlock(&imu_mtx);
        }

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * CANThread — event-driven  NORMALPRIO+28
 * Blocks on the CAN RxFIFO and dispatches frames as they arrive.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(CANThread, arg)
{
    (void)arg;
    chRegSetThreadName("can");

    while (true) {
        CANRxFrame rxf;
        if (canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &rxf,
                              TIME_MS2I(5)) == MSG_OK) {
            can_dispatch(rxf);
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * I2CThread — 100 Hz  NORMALPRIO+22
 * Calls each registered I2C device's poll function once per tick.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(I2CThread, arg)
{
    chRegSetThreadName("i2c");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        i2c_poll_all();
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}


/* ══════════════════════════════════════════════════════════════════════════
 * StateEstThread — 500 Hz  NORMALPRIO+25
 * Runs the 3-lane EKF, fuses g_imu[] and g_can_imu, writes g_state[].
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(StateEstThread, arg)
{
    chRegSetThreadName("est");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);
    const float dt = static_cast<float>(period)
                   / static_cast<float>(CH_CFG_ST_FREQUENCY);

    state_mgr.init();

    systime_t next = chVTGetSystemTime();
    while (true) {
        // Snapshot CAN IMU and clear consumed-this-tick flags atomically.
        CANIMURaw can_snap;
        chMtxLock(&can_imu_mtx);
        can_snap = g_can_imu;
        g_can_imu.has_new_quat  = false;
        g_can_imu.has_new_rates = false;
        chMtxUnlock(&can_imu_mtx);

        IMURaw imu_snap[3];
        chMtxLock(&imu_mtx);
        memcpy(imu_snap, g_imu, sizeof(g_imu));
        chMtxUnlock(&imu_mtx);

        MocapRaw mocap_snap;
        chMtxLock(&mocap_mtx);
        mocap_snap = g_mocap;
        g_mocap.has_new = false;
        chMtxUnlock(&mocap_mtx);

        // Run all EKF lanes and derive outputs.
        state_mgr.update(dt, imu_snap, can_snap, mocap_snap);

        chMtxLock(&state_mtx);
        state_mgr.get_state(g_state);          // copies full 19-element state
        g_euler[0] = state_mgr.roll();         // Euler angles derived from quaternion
        g_euler[1] = state_mgr.pitch();
        g_euler[2] = state_mgr.yaw();
        chMtxUnlock(&state_mtx);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}


/* ══════════════════════════════════════════════════════════════════════════
 * ControlThread — 500 Hz  NORMALPRIO+20
 * Cascade PID → MotorMixer → motor output.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(ControlThread, arg)
{
    chRegSetThreadName("ctrl");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        float state[StateIdx::N], euler[3], input[4];
        bool  armed;
        chMtxLock(&state_mtx);
        memcpy(state, g_state, sizeof(state));
        memcpy(euler, g_euler, sizeof(euler));
        memcpy(input, g_input, sizeof(input));
        armed = g_armed;
        chMtxUnlock(&state_mtx);

        // Build 6-element [roll, pitch, yaw, p, q, r] for the cascade controller.
        // AttitudeController and MotorMixer keep their existing index assumptions
        // (0=roll, 1=pitch, 2=yaw, 3=p, 4=q, 5=r) — no interface changes needed.
        const float ctrl_state[6] = {
            euler[0], euler[1], euler[2],
            state[StateIdx::P], state[StateIdx::Q], state[StateIdx::R]
        };

        float   cmds[3];
        att_ctrl.update(ctrl_state, input, cmds);
        float   thr = att_ctrl.compute_throttle(ctrl_state, input);

        int32_t out[4];
        mixer.update(cmds, thr, armed, ctrl_state, out);
        motor_output_write(out);

        chMtxLock(&state_mtx);
        memcpy(g_output, out, sizeof(out));
        chMtxUnlock(&state_mtx);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * RadioThread — 50 Hz  NORMALPRIO+10
 * Reads RC input and writes g_input / g_armed.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(RadioThread, arg)
{
    chRegSetThreadName("radio");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        radio_input_update();

        chMtxLock(&state_mtx);
        g_input[InputIdx::THRUST]    = radio_thr();
        g_input[InputIdx::ROLL_TGT]  = radio_roll();
        g_input[InputIdx::PITCH_TGT] = radio_pitch();
        g_input[InputIdx::YAW_RATE]  = radio_yaw();
        g_armed = radio_armed();
        if (!g_armed) { att_ctrl.reset_all(); }
        chMtxUnlock(&state_mtx);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * HeartbeatThread — 5 Hz  NORMALPRIO-5
 * LED heartbeat, SD log flush (future), watchdog pat (future).
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(HeartbeatThread, arg)
{
    chRegSetThreadName("heartbeat");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        palToggleLine(LINE_LED_ACTIVITY);
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * DebugThread — 10 Hz  NORMALPRIO-10  [BPRL_DEBUG only]
 * Prints a status line over USART3 (Telem1 / debug connector).
 * Build with -DBPRL_DEBUG to enable; omit for zero overhead in flight.
 * ══════════════════════════════════════════════════════════════════════════ */
#ifdef BPRL_DEBUG
static THD_FUNCTION(DebugThread, arg)
{
    chRegSetThreadName("dbg");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        float roll, pitch, yaw, thr;
        int32_t m0, m1, m2, m3;
        bool    armed;

        chMtxLock(&state_mtx);
        roll  = g_euler[0];
        pitch = g_euler[1];
        yaw   = g_euler[2];
        thr   = g_input[InputIdx::THRUST];
        m0 = g_output[0]; m1 = g_output[1];
        m2 = g_output[2]; m3 = g_output[3];
        armed = g_armed;
        chMtxUnlock(&state_mtx);

        chprintf((BaseSequentialStream *)&SD3,
            "armed=%d r=%.2f p=%.2f y=%.2f thr=%.2f m=[%ld,%ld,%ld,%ld]\r\n",
            (int)armed,
            (double)roll, (double)pitch, (double)yaw, (double)thr,
            m0, m1, m2, m3);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}
#endif // BPRL_DEBUG

/* ══════════════════════════════════════════════════════════════════════════
 * LogThread — 100 Hz IMU / 50 Hz state  NORMALPRIO-15
 * Snapshots g_imu[] and g_state[], buffers to ring, flushes to SD card.
 * Retries logger.init() every 5 s until an SD card is inserted.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(LogThread, arg)
{
    chRegSetThreadName("log");
    const LogRates *rates = static_cast<const LogRates *>(arg);

    /* state_div: log state every Nth IMU tick (e.g. 2 for 50 Hz when IMU = 100 Hz) */
    const uint32_t state_div = (rates->state > 0) ?
                               (uint32_t)(rates->imu / rates->state) : 1U;

    /* Retry until SD card is inserted and mounted. */
    while (!logger.init()) {
        chThdSleepMilliseconds(5000);
    }

    systime_t next = chVTGetSystemTime();
    uint32_t  tick = 0U;

    while (true) {
        const uint32_t t_ms = TIME_I2MS(chVTGetSystemTime());

        /* ── IMU snapshot (every tick → 100 Hz) ───────────────────────── */
        {
            LogMsgIMU msg = {};
            msg.time_ms = t_ms;

            chMtxLock(&imu_mtx);
            msg.ax0 = g_imu[0].accel[0]; msg.ay0 = g_imu[0].accel[1]; msg.az0 = g_imu[0].accel[2];
            msg.gx0 = g_imu[0].gyro[0];  msg.gy0 = g_imu[0].gyro[1];  msg.gz0 = g_imu[0].gyro[2];
            msg.valid0 = (uint8_t)g_imu[0].valid;
            msg.ax1 = g_imu[1].accel[0]; msg.ay1 = g_imu[1].accel[1]; msg.az1 = g_imu[1].accel[2];
            msg.gx1 = g_imu[1].gyro[0];  msg.gy1 = g_imu[1].gyro[1];  msg.gz1 = g_imu[1].gyro[2];
            msg.valid1 = (uint8_t)g_imu[1].valid;
            msg.ax2 = g_imu[2].accel[0]; msg.ay2 = g_imu[2].accel[1]; msg.az2 = g_imu[2].accel[2];
            msg.gx2 = g_imu[2].gyro[0];  msg.gy2 = g_imu[2].gyro[1];  msg.gz2 = g_imu[2].gyro[2];
            msg.valid2 = (uint8_t)g_imu[2].valid;
            chMtxUnlock(&imu_mtx);

            chMtxLock(&can_imu_mtx);
            msg.qw      = g_can_imu.q0; msg.qx = g_can_imu.q1;
            msg.qy      = g_can_imu.q2; msg.qz = g_can_imu.q3;
            msg.can_p   = g_can_imu.p;  msg.can_q = g_can_imu.q; msg.can_r = g_can_imu.r;
            msg.can_valid = (uint8_t)g_can_imu.valid;
            chMtxUnlock(&can_imu_mtx);

            logger.write(LOG_MSG_IMU, msg);
        }

        /* ── State snapshot (every state_div ticks → 50 Hz) ───────────── */
        if (state_div == 0U || tick % state_div == 0U) {
            LogMsgState msg = {};
            msg.time_ms = t_ms;

            chMtxLock(&state_mtx);
            msg.roll    = g_euler[0];
            msg.pitch   = g_euler[1];
            msg.yaw     = g_euler[2];
            msg.p       = g_state[StateIdx::P];
            msg.q       = g_state[StateIdx::Q];
            msg.r       = g_state[StateIdx::R];
            msg.z_pos   = g_state[StateIdx::Z_POS];
            msg.z_vel   = g_state[StateIdx::W];
            msg.z_accel = g_state[StateIdx::W_DOT];
            msg.thr     = g_input[InputIdx::THRUST];
            msg.armed   = (uint8_t)g_armed;
            chMtxUnlock(&state_mtx);

            logger.write(LOG_MSG_STATE, msg);
        }

        /* ── Flush ring buffer to SD card ─────────────────────────────── */
        logger.flush();

        tick++;
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, rates->imu));
    }
}

/* ── Thread launcher (called from main) ──────────────────────────────────── */
void threads_start(const ThreadRates &rates)
{
    chThdCreateStatic(waSPI,      sizeof(waSPI),      NORMALPRIO + 30, SPIThread,      (void *)&rates.spi);
    chThdCreateStatic(waCAN,      sizeof(waCAN),      NORMALPRIO + 28, CANThread,      nullptr);
    chThdCreateStatic(waStateEst, sizeof(waStateEst), NORMALPRIO + 25, StateEstThread, (void *)&rates.est);
    chThdCreateStatic(waI2C,      sizeof(waI2C),      NORMALPRIO + 22, I2CThread,      (void *)&rates.i2c);
    chThdCreateStatic(waControl,  sizeof(waControl),  NORMALPRIO + 20, ControlThread,  (void *)&rates.control);
    chThdCreateStatic(waRadio,    sizeof(waRadio),    NORMALPRIO + 10, RadioThread,    (void *)&rates.radio);
    chThdCreateStatic(waHeartbeat, sizeof(waHeartbeat), NORMALPRIO -  5, HeartbeatThread, (void *)&rates.heartbeat);
    chThdCreateStatic(waLog,      sizeof(waLog),      NORMALPRIO - 15, LogThread,      (void *)&rates.log);
#ifdef BPRL_DEBUG
    chThdCreateStatic(waDebug,    sizeof(waDebug),    NORMALPRIO - 10, DebugThread,    (void *)&rates.debug);
#endif
}
