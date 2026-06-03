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
#include "src/usb_serial.hpp"
#include "src/coms/CalFlash.hpp"
#include "chprintf.h"
#include "memstreams.h"
#include "ff.h"
#include <cstring>
#include <cstdio>

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

/* ── Calibration data loaded from flash at boot ──────────────────────────── */
static CalibData g_cal = {};
static bool      g_cal_valid = false;

/* ── Motor test (always built) ───────────────────────────────────────────── */
MUTEX_DECL(motor_test_mtx);
bool     g_motor_test_active = false;
uint16_t g_motor_test_cmd[4] = {};

/* Serialises all USB CDC writes between USBCmdThread and DebugThread.
 * chprintf is NOT atomic — it puts characters one at a time, so concurrent
 * callers interleave at the byte level and produce garbage lines.
 * USBCmdThread uses chMtxLock (must send a complete response before releasing).
 * DebugThread uses chMtxTryLock so it skips a $TEL tick rather than blocking
 * during a potentially long log download. */
static MUTEX_DECL(s_usb_write_mtx);

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
static THD_WORKING_AREA(waUSBCmd,   4096);  // 4 KB: FatFS log access + line parser
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

    systime_t next = chVTGetSystemTime();
    while (true) {
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

    // Ticks since last IMX5 quaternion — clears valid after 100 ticks (200 ms at 500 Hz).
    uint32_t can_stale_ticks = 0;
    static constexpr uint32_t CAN_TIMEOUT_TICKS = 100;

    systime_t next = chVTGetSystemTime();
    while (true) {
        // Snapshot CAN IMU and clear consumed-this-tick flags atomically.
        CANIMURaw can_snap;
        chMtxLock(&can_imu_mtx);
        can_snap = g_can_imu;
        g_can_imu.has_new_quat  = false;
        g_can_imu.has_new_rates = false;
        chMtxUnlock(&can_imu_mtx);

        // Timeout: if no quaternion arrives for CAN_TIMEOUT_TICKS, mark link invalid.
        if (can_snap.has_new_quat) {
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
        g_mocap.has_new = false;
        chMtxUnlock(&mocap_mtx);

        // Run all EKF lanes and derive outputs.
        state_mgr.update(dt, imu_snap, can_snap, mocap_snap);
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

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}


/* ══════════════════════════════════════════════════════════════════════════
 * ControlThread — 400 Hz  NORMALPRIO+20
 * Cascade PID → MotorMixer → motor output.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(ControlThread, arg)
{
    chRegSetThreadName("ctrl");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        /* ── Motor test bypass: skip PID/mixer, drive ESCs directly ──────── */
        {
            bool     test_active;
            uint16_t test_cmd[4];
            chMtxLock(&motor_test_mtx);
            test_active = g_motor_test_active;
            if (test_active) memcpy(test_cmd, g_motor_test_cmd, sizeof(test_cmd));
            chMtxUnlock(&motor_test_mtx);

            if (test_active) {
                dshot_write(test_cmd);
                chMtxLock(&state_mtx);
                memset(g_output, 0, sizeof(g_output));
                chMtxUnlock(&state_mtx);
                next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
                continue;
            }
        }

        /* ── Disarmed idle path (no radio, no state estimator running) ──────
         * With only ControlThread + USBCmdThread active, g_armed is always
         * false and all state is zero.  Skip the full PID/mixer path and
         * send DShot 0 (disarm/idle) directly so the ESC always receives a
         * valid, uninterrupted stream of packets at 400 Hz.  The motor-test
         * bypass above handles actual spin-up via USB commands. */
        {
            static const uint16_t kIdle[4] = {0U, 0U, 0U, 0U};
            dshot_write(kIdle);
        }

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
    uint32_t tick = 0;
    systime_t next = chVTGetSystemTime();
    while (true) {
        /* LED: 200 ms flash every 2 s (every 10th 200 ms tick). */
        if (tick % 10 == 0) palSetLine(LINE_LED_ACTIVITY);
        if (tick % 10 == 1) palClearLine(LINE_LED_ACTIVITY);

        /* Print DShot status every 2 s (every 10th tick at 5 Hz). */
        if (tick % 10 == 0) {
            DShotDiag d = {};
            dshot_get_diag(&d);
            char buf[128];
            int n = chsnprintf(buf, sizeof(buf),
                "$DSHOT,%lu,tc=%u/%u,cc=%u/%u,edges=%u/%u/%u/%u\r\n",
                (uint32_t)TIME_I2MS(chVTGetSystemTime()),
                (unsigned)d.dma_tc[0], (unsigned)d.dma_tc[1],
                (unsigned)d.cc_isr[0], (unsigned)d.cc_isr[1],
                (unsigned)d.edge_cnt[0], (unsigned)d.edge_cnt[1],
                (unsigned)d.edge_cnt[2], (unsigned)d.edge_cnt[3]);
            chnWriteTimeout((BaseChannel *)&SDU1, (uint8_t *)buf, (size_t)n, TIME_MS2I(50));
        }

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
        uint16_t dv = (pct == 0) ? 0U
                                  : (uint16_t)(48U + (uint32_t)pct * 1999U / 100U);
        if (dv > 2047U) dv = 2047U;
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
        if (strcmp(rest, "list") == 0) {
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
    }
}

static THD_FUNCTION(USBCmdThread, arg)
{
    chRegSetThreadName("usbcmd");
    (void)arg;

    static char   s_line[64];
    static uint8_t s_len = 0;

    while (true) {
        msg_t byte = chnGetTimeout((BaseChannel *)&SDU1, TIME_MS2I(50));
        if (byte == MSG_TIMEOUT || byte == MSG_RESET) continue;

        char c = (char)byte;
        if (c == '\n' || c == '\r') {
            if (s_len > 0) {
                s_line[s_len] = '\0';
                usb_cmd_dispatch(s_line);
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
 *        <imu0_v>,<imu1_v>,<imu2_v>,<can_v>,<can_quat_hz>,<can_rate_hz>
 * ══════════════════════════════════════════════════════════════════════════ */
#ifdef BPRL_DEBUG
static THD_FUNCTION(DebugThread, arg)
{
    chRegSetThreadName("dbg");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    uint32_t prev_quat_cnt = 0, prev_rate_cnt = 0;
    uint32_t can_quat_hz   = 0, can_rate_hz   = 0;
    int      rate_tick     = 0;

    systime_t next = chVTGetSystemTime();
    while (true) {
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
        float lane_roll[3], lane_pitch[3], lane_yaw[3];
        float lane_p[3], lane_q[3], lane_r[3];
        int   primary_lane;
        bool  armed;
        chMtxLock(&state_mtx);
        roll     = g_euler[0];
        pitch    = g_euler[1];
        yaw      = g_euler[2];
        p        = g_state[StateIdx::P];
        q        = g_state[StateIdx::Q];
        r        = g_state[StateIdx::R];
        thr      = g_input[InputIdx::THRUST];
        rc_roll  = g_input[InputIdx::ROLL_TGT];
        rc_pitch = g_input[InputIdx::PITCH_TGT];
        rc_yaw   = g_input[InputIdx::YAW_RATE];
        armed    = g_armed;
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

        /* ── Snapshot IMU validity ──────────────────────────────────────── */
        bool imu_v[3];
        chMtxLock(&imu_mtx);
        imu_v[0] = g_imu[0].valid;
        imu_v[1] = g_imu[1].valid;
        imu_v[2] = g_imu[2].valid;
        chMtxUnlock(&imu_mtx);

        bool can_v;
        chMtxLock(&can_imu_mtx);
        can_v = g_can_imu.valid;
        chMtxUnlock(&can_imu_mtx);

        /* ── Snapshot ESC telemetry ─────────────────────────────────────── */
        ESCTelemetry telem[4];
        dshot_get_telemetry(telem);
        uint32_t rpm[4];
        for (int i = 0; i < 4; i++) {
            rpm[i] = telem[i].valid ? telem[i].erpm / 7U : 0U;
        }

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
                "%d,%d,%d,%d,%lu,%lu\r\n",
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
                can_quat_hz, can_rate_hz);
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
         *         0,0,0,0,0,0          ← IMX5 INS (placeholder)
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
                "0.00,0.00,0.00,0.0000,0.0000,0.0000\r\n",
                (uint32_t)TIME_I2MS(chVTGetSystemTime()), primary_lane,
                (double)(lane_roll[0]*57.2958f), (double)(lane_pitch[0]*57.2958f),
                (double)(lane_yaw[0]*57.2958f),
                (double)lane_p[0], (double)lane_q[0], (double)lane_r[0],
                (double)(lane_roll[1]*57.2958f), (double)(lane_pitch[1]*57.2958f),
                (double)(lane_yaw[1]*57.2958f),
                (double)lane_p[1], (double)lane_q[1], (double)lane_r[1],
                (double)(lane_roll[2]*57.2958f), (double)(lane_pitch[2]*57.2958f),
                (double)(lane_yaw[2]*57.2958f),
                (double)lane_p[2], (double)lane_q[2], (double)lane_r[2]);
            size_t elen = ekfl_ms.eos;
            if (elen > 0 && chMtxTryLock(&s_usb_write_mtx)) {
                chnWriteTimeout((BaseChannel *)&SDU1,
                                (uint8_t *)ekfl_buf, elen, TIME_MS2I(50));
                chMtxUnlock(&s_usb_write_mtx);
            }
        }

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
    /*
     * Minimal configuration: only the motor output thread and the USB debug
     * interface.  All other threads are intentionally omitted.
     *
     * Why this matters for bidirectional DShot:
     *   SPIThread runs SPI DMA transfers at 1 kHz.  Each SPI DMA completion
     *   ISR fires at a priority above the TIM1_CC / TIM4 edge-capture IRQs,
     *   preempting them mid-GCR-frame.  At 750 kHz GCR a single 5 µs ISR
     *   preemption can overwrite CCR before we read it.  Removing all sources
     *   of high-frequency ISR contention lets the edge-capture ISRs run
     *   uninterrupted within the 1.33 µs GCR bit window.
     *
     * ControlThread: calls dshot_write() at 500 Hz.  Handles both the motor-
     *   test bypass (USB MT commands) and the normal PID/mixer path.  All
     *   shared state (g_state, g_euler, g_input) defaults to zero — safe
     *   because with g_armed=false the mixer outputs zeros, and the motor-test
     *   bypass bypasses the mixer entirely.
     *
     * USBCmdThread: MT,* (motor test), DSHOT,diag, BOOT, CAL,* commands.
     *   Event-driven; near-zero overhead when no USB host is connected.
     */
    chThdCreateStatic(waControl,   sizeof(waControl),   NORMALPRIO + 20, ControlThread,   (void *)&rates.control);
    chThdCreateStatic(waUSBCmd,    sizeof(waUSBCmd),    NORMALPRIO - 20, USBCmdThread,    nullptr);
    chThdCreateStatic(waHeartbeat, sizeof(waHeartbeat), NORMALPRIO -  5, HeartbeatThread, (void *)&rates.heartbeat);
#ifdef BPRL_DEBUG
    chThdCreateStatic(waDebug,     sizeof(waDebug),     NORMALPRIO - 10, DebugThread,     (void *)&rates.debug);
#endif
}
