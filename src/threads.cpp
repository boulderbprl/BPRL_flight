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
#include "src/coms/Radio.hpp"
#include "src/controllers/AttitudeController.hpp"
#include "src/controllers/MotorMixer.hpp"
#include "src/logging/Logger.hpp"
#include "src/logging/LogMessages.hpp"
#include "chprintf.h"
#include <cstring>

/* ── Shared state definitions ────────────────────────────────────────────── */

MUTEX_DECL(state_mtx);
float   g_state[9]  = {};
float   g_input[4]  = {};
int32_t g_output[4] = {};
bool    g_armed     = false;

MUTEX_DECL(imu_mtx);
IMURaw g_imu[3] = {};

MUTEX_DECL(can_imu_mtx);
CANIMURaw g_can_imu = {};

/* ── Controller instances (ControlThread only) ───────────────────────────── */
static AttitudeController att_ctrl;
static MotorMixer         mixer;

/* ── Thread working areas ────────────────────────────────────────────────── */
static THD_WORKING_AREA(waSPI,      2048);
static THD_WORKING_AREA(waCAN,      2048);
static THD_WORKING_AREA(waStateEst, 2048);
static THD_WORKING_AREA(waI2C,      1024);
static THD_WORKING_AREA(waControl,  2048);
static THD_WORKING_AREA(waRadio,    1024);
static THD_WORKING_AREA(waHouse,    1024);
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
 * CANThread — 1 kHz  NORMALPRIO+28
 * Drains the CAN RxFIFO and dispatches frames to registered callbacks.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(CANThread, arg)
{
    chRegSetThreadName("can");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        CANRxFrame rxf;
        while (canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &rxf,
                                 TIME_IMMEDIATE) == MSG_OK) {
            can_dispatch(rxf);
        }
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * StateEstThread — 500 Hz  NORMALPRIO+25
 * Fuses g_imu[] and g_can_imu into g_state[].
 *
 * Phase 1 (now):    pass through g_can_imu if valid, else gyro-only.
 * Phase 2 (future): complementary filter on accel + gyro.
 * Phase 3 (future): EKF / weighted average across all sources.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(StateEstThread, arg)
{
    chRegSetThreadName("est");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        CANIMURaw can_snap;
        chMtxLock(&can_imu_mtx);
        can_snap = g_can_imu;
        chMtxUnlock(&can_imu_mtx);

        IMURaw imu_snap[3];
        chMtxLock(&imu_mtx);
        memcpy(imu_snap, g_imu, sizeof(g_imu));
        chMtxUnlock(&imu_mtx);

        float roll = 0, pitch = 0, yaw = 0, p = 0, q = 0, r = 0;

        if (can_snap.valid) {
            roll  = can_snap.roll;
            pitch = can_snap.pitch;
            yaw   = can_snap.yaw;
            p     = can_snap.p;
            q     = can_snap.q;
            r     = can_snap.r;
        } else if (imu_snap[0].valid) {
            // TODO: integrate gyro, correct with accelerometer tilt.
            p = imu_snap[0].gyro[0];
            q = imu_snap[0].gyro[1];
            r = imu_snap[0].gyro[2];
        }

        chMtxLock(&state_mtx);
        g_state[StateIdx::ROLL]  = roll;
        g_state[StateIdx::PITCH] = pitch;
        g_state[StateIdx::YAW]   = yaw;
        g_state[StateIdx::P]     = p;
        g_state[StateIdx::Q]     = q;
        g_state[StateIdx::R]     = r;
        chMtxUnlock(&state_mtx);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
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
 * ControlThread — 500 Hz  NORMALPRIO+20
 * Cascade PID → MotorMixer → motor output.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(ControlThread, arg)
{
    chRegSetThreadName("ctrl");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        float state[9], input[4];
        bool  armed;
        chMtxLock(&state_mtx);
        memcpy(state, g_state, sizeof(state));
        memcpy(input, g_input, sizeof(input));
        armed = g_armed;
        chMtxUnlock(&state_mtx);

        float   cmds[3];
        att_ctrl.update(state, input, cmds);
        float   thr = att_ctrl.compute_throttle(state, input);

        int32_t out[4];
        mixer.update(cmds, thr, armed, state, out);
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
 * HouseThread — 5 Hz  NORMALPRIO-5
 * LED heartbeat, SD log flush (future), watchdog pat (future).
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(HouseThread, arg)
{
    chRegSetThreadName("house");
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
        roll  = g_state[StateIdx::ROLL];
        pitch = g_state[StateIdx::PITCH];
        yaw   = g_state[StateIdx::YAW];
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
            msg.roll = g_can_imu.roll; msg.pitch = g_can_imu.pitch; msg.yaw = g_can_imu.yaw;
            msg.p    = g_can_imu.p;    msg.q     = g_can_imu.q;    msg.r   = g_can_imu.r;
            msg.can_valid = (uint8_t)g_can_imu.valid;
            chMtxUnlock(&can_imu_mtx);

            logger.write(LOG_MSG_IMU, msg);
        }

        /* ── State snapshot (every state_div ticks → 50 Hz) ───────────── */
        if (state_div == 0U || tick % state_div == 0U) {
            LogMsgState msg = {};
            msg.time_ms = t_ms;

            chMtxLock(&state_mtx);
            msg.roll    = g_state[StateIdx::ROLL];
            msg.pitch   = g_state[StateIdx::PITCH];
            msg.yaw     = g_state[StateIdx::YAW];
            msg.p       = g_state[StateIdx::P];
            msg.q       = g_state[StateIdx::Q];
            msg.r       = g_state[StateIdx::R];
            msg.z_pos   = g_state[StateIdx::Z_POS];
            msg.z_vel   = g_state[StateIdx::Z_VEL];
            msg.z_accel = g_state[StateIdx::Z_ACCEL];
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
    chThdCreateStatic(waCAN,      sizeof(waCAN),      NORMALPRIO + 28, CANThread,      (void *)&rates.can);
    chThdCreateStatic(waStateEst, sizeof(waStateEst), NORMALPRIO + 25, StateEstThread, (void *)&rates.est);
    chThdCreateStatic(waI2C,      sizeof(waI2C),      NORMALPRIO + 22, I2CThread,      (void *)&rates.i2c);
    chThdCreateStatic(waControl,  sizeof(waControl),  NORMALPRIO + 20, ControlThread,  (void *)&rates.control);
    chThdCreateStatic(waRadio,    sizeof(waRadio),    NORMALPRIO + 10, RadioThread,    (void *)&rates.radio);
    chThdCreateStatic(waHouse,    sizeof(waHouse),    NORMALPRIO -  5, HouseThread,    (void *)&rates.house);
    chThdCreateStatic(waLog,      sizeof(waLog),      NORMALPRIO - 15, LogThread,      (void *)&rates.log);
#ifdef BPRL_DEBUG
    chThdCreateStatic(waDebug,    sizeof(waDebug),    NORMALPRIO - 10, DebugThread,    (void *)&rates.debug);
#endif
}
