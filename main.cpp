/*
 * main.cpp — BPRL Standalone ChibiOS Flight Controller
 * Target: STM32H7xx (CubeBlue H7 / CubeOrange+) at 400 MHz
 * Build:  make BOARD=CubeBlueH7  (or CubeOrangePlus)
 * Flash:  make flash BOARD=CubeBlueH7  PORT=/dev/ttyACM0
 *
 * ── Thread architecture ──────────────────────────────────────────────────────
 *
 *   SPIThread      1 kHz  prio+30  Reads 3x on-board IMUs via SPI
 *   CANThread      1 kHz  prio+28  CAN bus dispatcher (expandable device table)
 *   StateEstThread 500 Hz prio+25  Sensor fusion → attitude estimate → g_state
 *   I2CThread      100 Hz prio+22  I2C dispatcher (expandable device table)
 *   ControlThread  500 Hz prio+20  Cascade PID → motor mixing → motor output
 *   RadioThread     50 Hz prio+10  RC input (PWM now; SBUS/PPM via interface)
 *   HouseThread      5 Hz prio-5   SD log flush, LED heartbeat, watchdog
 *   DebugThread     10 Hz prio-10  UART3 state print [build with -DBPRL_DEBUG]
 *
 * ── Sensor fusion data flow ──────────────────────────────────────────────────
 *
 *   SPIThread  ──▶ g_imu[3]     (3x raw accel/gyro, imu_mtx)
 *   CANThread  ──▶ g_can_imu    (external IMU Euler+rates, can_imu_mtx)
 *                      │
 *                      ▼
 *   StateEstThread ──▶ g_state[] (fused attitude, state_mtx)
 *                      │
 *              ┌───────┴───────┐
 *              ▼               ▼
 *   ControlThread         DebugThread / HouseThread
 *
 * ── Adding new CAN devices ───────────────────────────────────────────────────
 *   1. Write a handler: void my_device_can(const CANRxFrame &f, void *ctx)
 *   2. In main(): bprl_can_register(MY_CAN_ID, my_device_can, nullptr);
 *   CANThread dispatches all received frames to registered handlers.
 *
 * ── Adding new I2C devices ───────────────────────────────────────────────────
 *   1. Write a poll function: void my_device_i2c(void *ctx)
 *   2. In main(): bprl_i2c_register(MY_I2C_ADDR, my_device_i2c, nullptr);
 *   I2CThread calls each registered poller every 100 Hz.
 *
 * ── Motor output interface ────────────────────────────────────────────────────
 *   Currently: PWM via TIM1 (LINE_FMU_CH1-4, 400 Hz).
 *   Future DShot: replace motor_output_write() body only — ControlThread
 *   and the rest of the firmware stay unchanged.
 *
 * ── Radio input interface ─────────────────────────────────────────────────────
 *   Currently: PWM capture via ICU on TIM8/LINE_RC_INPUT (stub — enable
 *   HAL_USE_ICU in halconf.h and fill in radio_input_update()).
 *   Future SBUS: redirect LINE_RC_INPUT to USART6, change RadioThread
 *   period to 14 ms, decode SBUS framing in radio_input_update().
 *   Future PPM: single ICU channel, decode pulse train in the callback.
 *
 * ── Debug UART ────────────────────────────────────────────────────────────────
 *   Compile with -DBPRL_DEBUG to enable DebugThread.
 *   In Makefile: UDEFS += -DBPRL_DEBUG
 *   Disable before flight: remove -DBPRL_DEBUG from UDEFS.
 *   Uses USART3 (PD8/PD9, 115200 baud, Telem1 connector on Cube).
 */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "src/AttitudeController.hpp"
#include "src/MotorMixer.hpp"
#include "src/FlightState.hpp"
#include <cstring>
#include <cmath>

/* ── Shared flight state ─────────────────────────────────────────────────────
 * All fields are in SI units matching StateIdx / InputIdx in FlightState.hpp.
 * Access only under state_mtx.                                               */
static MUTEX_DECL(state_mtx);
static float   g_state[9] = {};   // StateIdx::*  — roll…z_accel (rad, rad/s, m, m/s, m/s²)
static float   g_input[4] = {};   // InputIdx::*  — thrust[0,1], roll/pitch/yaw[-1,1]
static int32_t g_output[4] = {};  // motor PWM µs [FR, RL, FL, RR]
static bool    g_armed      = false;

/* ── Raw IMU data (SPIThread → StateEstThread) ───────────────────────────────
 * One entry per on-board IMU.  valid=false until the IMU driver is added.   */
struct IMURaw {
    float accel[3];   // m/s² in board frame (X fwd, Y right, Z down)
    float gyro[3];    // rad/s in board frame
    bool  valid;
};
static MUTEX_DECL(imu_mtx);
static IMURaw g_imu[3] = {};   // [0]=ICM-20948 primary, [1]=ICM-20948 ext, [2]=ICM-20602

/* ── External CAN IMU (CANThread → StateEstThread) ──────────────────────────
 * Euler angles and body rates from external IMU on CAN bus.
 * valid=false until the CAN IMU callback is registered and data arrives.    */
struct CANIMURaw {
    float roll, pitch, yaw;   // rad
    float p, q, r;             // rad/s
    bool  valid;
};
static MUTEX_DECL(can_imu_mtx);
static CANIMURaw g_can_imu = {};

/* ── CAN device registration table ──────────────────────────────────────────
 * Register handlers in main() before threads start.  CANThread dispatches
 * every received frame to the matching callback.                             */
#define MAX_CAN_DEVICES 8
typedef void (*CANCallback)(const CANRxFrame &frame, void *ctx);
struct CANDevice {
    uint32_t    id;
    CANCallback callback;
    void       *ctx;
};
static CANDevice can_table[MAX_CAN_DEVICES];
static int       num_can_devices = 0;

void bprl_can_register(uint32_t id, CANCallback cb, void *ctx)
{
    if (num_can_devices < MAX_CAN_DEVICES) {
        can_table[num_can_devices++] = {id, cb, ctx};
    }
}

/* ── I2C device registration table ──────────────────────────────────────────
 * Each registered device's poll() is called once per I2CThread tick.        */
#define MAX_I2C_DEVICES 8
typedef void (*I2CCallback)(void *ctx);
struct I2CDevice {
    uint8_t     addr;
    I2CCallback poll;
    void       *ctx;
};
static I2CDevice i2c_table[MAX_I2C_DEVICES];
static int        num_i2c_devices = 0;

void bprl_i2c_register(uint8_t addr, I2CCallback poll_fn, void *ctx)
{
    if (num_i2c_devices < MAX_I2C_DEVICES) {
        i2c_table[num_i2c_devices++] = {addr, poll_fn, ctx};
    }
}

/* ── Motor output interface ──────────────────────────────────────────────────
 * Swap the body of motor_output_write() for DShot without touching
 * ControlThread.                                                              */
static void motor_output_init(void)
{
    // TODO: configure TIM1 for 400 Hz PWM on LINE_FMU_CH1..4.
    // Example (once HAL_USE_PWM is enabled in halconf.h):
    //   pwmStart(&PWMD1, &pwm_cfg);
    //   pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    //   ... repeat for CH2-4
    //
    // For DShot (future): configure TIM1 DMA, write DShot bitstream instead.
}

static void motor_output_write(const int32_t pwm_us[4])
{
    // TODO: write pwm_us[0..3] to TIM1 CH1..4.
    // Example:
    //   for (int i = 0; i < 4; i++) {
    //       pwmEnableChannel(&PWMD1, i,
    //           PWM_FRACTION_TO_WIDTH(&PWMD1, 20000, (uint32_t)pwm_us[i]));
    //   }
    (void)pwm_us;  // suppress unused warning until driver is wired up
}

/* ── Radio input interface ───────────────────────────────────────────────────
 * Replace radio_input_update() body when switching to SBUS / PPM.           */
static float _rc_thr  = 0.0f;
static float _rc_roll = 0.0f, _rc_pitch = 0.0f, _rc_yaw = 0.0f;
static bool  _rc_armed = false;

static void radio_input_init(void)
{
    // TODO: configure ICU for PWM capture (HAL_USE_ICU in halconf.h),
    //       or configure USART6 for SBUS (inverted UART, 100000 baud, 8E2).
}

static void radio_input_update(void)
{
    // TODO: read and normalise RC channel values.
    //
    // PWM: read ICU-captured pulse widths:
    //   uint32_t thr_us = icu_lld_get_width(&ICUD8);   // 1000-2000 µs
    //   _rc_thr   = (thr_us - 1000.0f) / 1000.0f;      // [0, 1]
    //   _rc_roll  = (ch2_us - 1500.0f) / 500.0f;       // [-1, 1]
    //   _rc_pitch = (ch3_us - 1500.0f) / 500.0f;
    //   _rc_yaw   = (ch4_us - 1500.0f) / 500.0f;
    //   _rc_armed = (arm_us > 1700);
    //
    // SBUS: parse the 25-byte frame from USART6, map channels 1-5.
    (void)0;
}

/* ── Controller instances (used by ControlThread only) ──────────────────────*/
static AttitudeController att_ctrl;
static MotorMixer         mixer;

/* ── Thread stacks ───────────────────────────────────────────────────────────*/
static THD_WORKING_AREA(waSPI,       2048);
static THD_WORKING_AREA(waCAN,       2048);
static THD_WORKING_AREA(waStateEst,  2048);
static THD_WORKING_AREA(waI2C,       1024);
static THD_WORKING_AREA(waControl,   2048);
static THD_WORKING_AREA(waRadio,     1024);
static THD_WORKING_AREA(waHouse,     1024);
#ifdef BPRL_DEBUG
static THD_WORKING_AREA(waDebug,     1024);
#endif

/* ══════════════════════════════════════════════════════════════════════════════
 * Thread: SPIThread — 1 kHz on-board IMU reads
 * ══════════════════════════════════════════════════════════════════════════════
 * Reads raw accel+gyro from all three on-board IMUs over SPI.
 * Writes to g_imu[].  StateEstThread fuses the results.
 *
 * IMUs:
 *   g_imu[0] = ICM-20948 primary  (SPI1, LINE_IMU1_CS, PC2)
 *   g_imu[1] = ICM-20948 ext      (SPI4, LINE_IMU2_CS, PE4)
 *   g_imu[2] = ICM-20602 ext      (SPI4, LINE_IMU3_CS, PC13)
 *
 * TODO: enable HAL_USE_SPI in halconf.h, add SPI1/SPI4 in mcuconf.h,
 *       instantiate ICMxxxx driver objects and call their read() methods.    */
static THD_FUNCTION(SPIThread, arg)
{
    (void)arg;
    chRegSetThreadName("spi");
    systime_t next             = chVTGetSystemTime();
    const sysinterval_t period = TIME_US2I(1000); // 1 kHz

    while (true) {
        // TODO: call IMU driver read functions here.
        // Example (once ICM20602 driver is written in src/imu/):
        //   float a[3], g[3];
        //   if (icm[0].read(a, g)) {
        //       chMtxLock(&imu_mtx);
        //       memcpy(g_imu[0].accel, a, sizeof(a));
        //       memcpy(g_imu[0].gyro,  g, sizeof(g));
        //       g_imu[0].valid = true;
        //       chMtxUnlock(&imu_mtx);
        //   }

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Thread: CANThread — 1 kHz CAN bus dispatcher
 * ══════════════════════════════════════════════════════════════════════════════
 * Drains the CAN receive mailbox and dispatches frames to registered handlers.
 * New CAN devices: call bprl_can_register() in main() before threads start.
 *
 * External IMU (original FreeRTOS protocol, 500 kbps):
 *   ID 0x01: bytes 0-1 = roll  (int16, ×10000 → rad)
 *   ID 0x02: bytes 0-1 = pitch (int16, ×10000 → rad)
 *   ID 0x03: bytes 0-1 = yaw   (int16, ×10000 → rad)
 *   ID 0x04: bytes 0-1 = p, 2-3 = q, 4-5 = r  (int16, ×1000 → rad/s)
 *
 * TODO: enable HAL_USE_CAN in halconf.h and STM32_FDCAN1_ENABLED in mcuconf.h,
 *       then call canStart(&CAND1, &can_cfg) in main().                      */
static THD_FUNCTION(CANThread, arg)
{
    (void)arg;
    chRegSetThreadName("can");
    systime_t next             = chVTGetSystemTime();
    const sysinterval_t period = TIME_US2I(1000); // 1 kHz polling cadence

    while (true) {
        // TODO: receive CAN frames and dispatch to registered handlers.
        // Example (once CAN driver is configured):
        //   CANRxFrame rxf;
        //   while (canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &rxf,
        //                            TIME_IMMEDIATE) == MSG_OK) {
        //       for (int i = 0; i < num_can_devices; i++) {
        //           if (can_table[i].id == rxf.SID) {
        //               can_table[i].callback(rxf, can_table[i].ctx);
        //           }
        //       }
        //   }

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Thread: StateEstThread — 500 Hz sensor fusion
 * ══════════════════════════════════════════════════════════════════════════════
 * Fuses data from g_imu[0..2] (on-board IMUs) and g_can_imu (external CAN
 * IMU) into a single attitude estimate written to g_state[].
 *
 * Fusion strategy (to be expanded as drivers come online):
 *   Phase 1 (now):       pass through g_can_imu if valid, else zeros.
 *   Phase 2 (SPI IMUs):  complementary filter on accel+gyro from g_imu[].
 *   Phase 3 (full):      weighted average / EKF across all sources.           */
static THD_FUNCTION(StateEstThread, arg)
{
    (void)arg;
    chRegSetThreadName("est");
    systime_t next             = chVTGetSystemTime();
    const sysinterval_t period = TIME_US2I(2000); // 500 Hz

    while (true) {
        // ── Snapshot raw sensor data ──────────────────────────────────────────
        CANIMURaw can_snap;
        chMtxLock(&can_imu_mtx);
        can_snap = g_can_imu;
        chMtxUnlock(&can_imu_mtx);

        IMURaw imu_snap[3];
        chMtxLock(&imu_mtx);
        memcpy(imu_snap, g_imu, sizeof(g_imu));
        chMtxUnlock(&imu_mtx);

        // ── Sensor fusion ─────────────────────────────────────────────────────
        // TODO: replace with complementary filter / EKF once IMU drivers exist.
        //
        // For now: prefer CAN IMU (external, pre-processed) if present,
        // else fall back to on-board IMU 0 (once it has valid data).
        float roll = 0, pitch = 0, yaw = 0, p = 0, q = 0, r = 0;

        if (can_snap.valid) {
            roll  = can_snap.roll;
            pitch = can_snap.pitch;
            yaw   = can_snap.yaw;
            p     = can_snap.p;
            q     = can_snap.q;
            r     = can_snap.r;
        } else if (imu_snap[0].valid) {
            // TODO: integrate gyro and correct with accelerometer tilt.
            p = imu_snap[0].gyro[0];
            q = imu_snap[0].gyro[1];
            r = imu_snap[0].gyro[2];
        }

        // ── Write fused attitude to shared state ──────────────────────────────
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

/* ══════════════════════════════════════════════════════════════════════════════
 * Thread: I2CThread — 100 Hz I2C device poller
 * ══════════════════════════════════════════════════════════════════════════════
 * Calls each registered I2C device's poll function once per tick.
 * New devices: call bprl_i2c_register() in main() before threads start.
 *
 * TODO: enable HAL_USE_I2C in halconf.h, call i2cStart(&I2CD1, &i2c_cfg)
 *       in main(), then register magnetometer, barometer, etc.               */
static THD_FUNCTION(I2CThread, arg)
{
    (void)arg;
    chRegSetThreadName("i2c");
    systime_t next             = chVTGetSystemTime();
    const sysinterval_t period = TIME_MS2I(10); // 100 Hz

    while (true) {
        for (int i = 0; i < num_i2c_devices; i++) {
            i2c_table[i].poll(i2c_table[i].ctx);
        }
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Thread: ControlThread — 500 Hz cascade PID + motor output
 * ══════════════════════════════════════════════════════════════════════════════
 * 1. Snapshots g_state and g_input under a short mutex lock.
 * 2. Runs AttitudeController (cascade PID → torque commands).
 * 3. Runs MotorMixer (torque + thrust → 4 × PWM µs).
 * 4. Writes outputs via motor_output_write() (swap for DShot when ready).    */
static THD_FUNCTION(ControlThread, arg)
{
    (void)arg;
    chRegSetThreadName("ctrl");
    systime_t next             = chVTGetSystemTime();
    const sysinterval_t period = TIME_US2I(2000); // 500 Hz

    while (true) {
        // Snapshot shared state under the shortest possible lock
        float state[9], input[4];
        bool  armed;
        chMtxLock(&state_mtx);
        memcpy(state, g_state, sizeof(state));
        memcpy(input, g_input, sizeof(input));
        armed = g_armed;
        chMtxUnlock(&state_mtx);

        // Cascade PID: attitude error → rate target → torque commands
        float   cmds[3];
        att_ctrl.update(state, input, cmds);
        float   thr = att_ctrl.compute_throttle(state, input);

        // Motor mixing: normalised torques + throttle → 4 × PWM µs
        int32_t out[4];
        mixer.update(cmds, thr, armed, state, out);

        // Write to hardware (PWM now; replace body for DShot)
        motor_output_write(out);

        chMtxLock(&state_mtx);
        memcpy(g_output, out, sizeof(out));
        chMtxUnlock(&state_mtx);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Thread: RadioThread — 50 Hz RC input
 * ══════════════════════════════════════════════════════════════════════════════
 * Calls radio_input_update() then copies normalised sticks into g_input.
 * Rate/interface change for SBUS: set period to TIME_MS2I(14), change
 * radio_input_update() to decode SBUS framing from USART6.                    */
static THD_FUNCTION(RadioThread, arg)
{
    (void)arg;
    chRegSetThreadName("radio");
    systime_t next             = chVTGetSystemTime();
    const sysinterval_t period = TIME_MS2I(20); // 50 Hz (change to 14ms for SBUS)

    while (true) {
        radio_input_update();

        chMtxLock(&state_mtx);
        g_input[InputIdx::THRUST]    = _rc_thr;
        g_input[InputIdx::ROLL_TGT]  = _rc_roll;
        g_input[InputIdx::PITCH_TGT] = _rc_pitch;
        g_input[InputIdx::YAW_RATE]  = _rc_yaw;
        g_armed = _rc_armed;
        if (!g_armed) {
            att_ctrl.reset_all();
        }
        chMtxUnlock(&state_mtx);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Thread: HouseThread — 5 Hz housekeeping
 * ══════════════════════════════════════════════════════════════════════════════
 * LED heartbeat, SD card log flush, watchdog pat (future).                    */
static THD_FUNCTION(HouseThread, arg)
{
    (void)arg;
    chRegSetThreadName("house");

    while (true) {
        palToggleLine(LINE_LED_ACTIVITY);
        // TODO: flush SD card log buffer here once Logger class is added.
        chThdSleepMilliseconds(200); // 5 Hz
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Thread: DebugThread — 10 Hz UART3 state print  [BPRL_DEBUG only]
 * ══════════════════════════════════════════════════════════════════════════════
 * Prints a human-readable status line over USART3 (Telem1 / debug connector).
 * Enable: add -DBPRL_DEBUG to UDEFS in Makefile.
 * Disable before flight: remove -DBPRL_DEBUG (zero code and zero CPU overhead
 * in the non-debug build).                                                     */
#ifdef BPRL_DEBUG
static THD_FUNCTION(DebugThread, arg)
{
    (void)arg;
    chRegSetThreadName("dbg");
    systime_t next             = chVTGetSystemTime();
    const sysinterval_t period = TIME_MS2I(100); // 10 Hz

    while (true) {
        float roll, pitch, yaw, thr;
        int32_t m0, m1, m2, m3;
        bool    armed;

        chMtxLock(&state_mtx);
        roll  = g_state[StateIdx::ROLL];
        pitch = g_state[StateIdx::PITCH];
        yaw   = g_state[StateIdx::YAW];
        thr   = g_input[InputIdx::THRUST];
        m0    = g_output[0];  m1 = g_output[1];
        m2    = g_output[2];  m3 = g_output[3];
        armed = g_armed;
        chMtxUnlock(&state_mtx);

        chprintf((BaseSequentialStream *)&SD3,
            "armed=%d r=%.2f p=%.2f y=%.2f thr=%.2f "
            "m=[%ld,%ld,%ld,%ld]\r\n",
            (int)armed,
            (double)roll, (double)pitch, (double)yaw, (double)thr,
            m0, m1, m2, m3);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}
#endif // BPRL_DEBUG

/* ══════════════════════════════════════════════════════════════════════════════
 * main
 * ══════════════════════════════════════════════════════════════════════════════*/
int main(void)
{
    halInit();
    chSysInit();

#ifdef BPRL_DEBUG
    // Initialise USART3 for debug output (PD8=TX, PD9=RX, 115200 8N1)
    static const SerialConfig uart3_cfg = {115200, 0, USART_CR2_STOP1_BITS, 0};
    sdStart(&SD3, &uart3_cfg);
    chprintf((BaseSequentialStream *)&SD3, "\r\nBPRL boot [" BOARD_NAME "]\r\n");
#endif

    // ── Hardware output init ─────────────────────────────────────────────────
    motor_output_init();
    radio_input_init();

    // ── Register CAN devices ──────────────────────────────────────────────────
    // External CAN IMU (FreeRTOS-compatible protocol):
    //   bprl_can_register(0x01, ext_imu_can_cb, nullptr);
    //   bprl_can_register(0x02, ext_imu_can_cb, nullptr);
    //   ...
    // Additional devices (ESC telemetry, GPS, etc.) added here.

    // ── Register I2C devices ──────────────────────────────────────────────────
    // Magnetometer, barometer, rangefinder, etc.:
    //   bprl_i2c_register(0x1E, compass_poll, nullptr);  // HMC5883
    //   bprl_i2c_register(0x76, baro_poll, nullptr);     // MS5611

    // ── Start threads ─────────────────────────────────────────────────────────
    chThdCreateStatic(waSPI,       sizeof(waSPI),      NORMALPRIO + 30, SPIThread,       NULL);
    chThdCreateStatic(waCAN,       sizeof(waCAN),      NORMALPRIO + 28, CANThread,       NULL);
    chThdCreateStatic(waStateEst,  sizeof(waStateEst), NORMALPRIO + 25, StateEstThread,  NULL);
    chThdCreateStatic(waI2C,       sizeof(waI2C),      NORMALPRIO + 22, I2CThread,       NULL);
    chThdCreateStatic(waControl,   sizeof(waControl),  NORMALPRIO + 20, ControlThread,   NULL);
    chThdCreateStatic(waRadio,     sizeof(waRadio),    NORMALPRIO + 10, RadioThread,     NULL);
    chThdCreateStatic(waHouse,     sizeof(waHouse),    NORMALPRIO -  5, HouseThread,     NULL);
#ifdef BPRL_DEBUG
    chThdCreateStatic(waDebug,     sizeof(waDebug),    NORMALPRIO - 10, DebugThread,     NULL);
#endif

    // Main thread becomes idle background; spin with a long sleep
    while (true) {
        chThdSleepMilliseconds(1000);
    }
    return 0;
}
