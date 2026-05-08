/*
 * main.cpp — BPRL Standalone ChibiOS Flight Controller
 * Target:  STM32H7xx (CubeBlue H7 / CubeOrange+) at 400 MHz
 * Build:   make BOARD=CubeBlueH7          (or CubeOrangePlus)
 * Flash:   make flash BOARD=CubeBlueH7  PORT=/dev/ttyACM0
 * Debug:   make BOARD=CubeBlueH7 UDEFS_EXTRA=-DBPRL_DEBUG
 *
 * ── What lives where ────────────────────────────────────────────────────────
 *
 *   main.cpp               Init calls, thread rate sequencer, scheduler start.
 *   src/threads.cpp/.hpp   All eight thread function bodies + shared state.
 *   src/coms/SPI.*         ICM-20948/20602 SPI bus drivers.
 *   src/coms/CAN.*         FDCAN1 driver, IMX5 callback, device registration.
 *   src/coms/I2C.*         I2C device table (compass, baro — future).
 *   src/coms/PWM.*         Motor PWM output stub (TIM1, future DShot).
 *   src/coms/Radio.*       RC radio input stub (ICU/SBUS, future).
 *   src/controllers/       PID, AttitudeController, MotorMixer.
 *
 * ── Adding a CAN device ─────────────────────────────────────────────────────
 *   1. Write: void my_cb(const CANRxFrame &f, void *ctx) in src/coms/CAN.cpp
 *      or anywhere that includes src/coms/CAN.hpp.
 *   2. Add: bprl_can_register(MY_ID, my_cb, nullptr);  below.
 *
 * ── Adding an I2C device ────────────────────────────────────────────────────
 *   1. Write: void my_poll(void *ctx)
 *   2. Add: bprl_i2c_register(MY_ADDR, my_poll, nullptr);  below.
 *
 * ── Switching motor protocol to DShot ───────────────────────────────────────
 *   Replace the body of motor_output_write() in src/coms/PWM.cpp only.
 *
 * ── Switching radio to SBUS ─────────────────────────────────────────────────
 *   Fill in src/coms/Radio.cpp, then change kRates.radio to TIME_MS2I(14).
 */

#include "ch.h"
#include "hal.h"
#include "src/threads.hpp"
#include "src/coms/CAN.hpp"
#include "src/coms/I2C.hpp"
#include "src/coms/PWM.hpp"
#include "src/coms/Radio.hpp"
#include "chprintf.h"

int main(void)
{
    halInit();
    chSysInit();

#ifdef BPRL_DEBUG
    static const SerialConfig uart3_cfg = {115200, 0, USART_CR2_STOP1_BITS, 0};
    sdStart(&SD3, &uart3_cfg);
    chprintf((BaseSequentialStream *)&SD3, "\r\nBPRL boot [" BOARD_NAME "]\r\n");
#endif

    /* ── Hardware init ──────────────────────────────────────────────────────
     * SPI IMU init runs inside SPIThread (needs chThdSleep during ICM reset).
     * All other drivers are safe to call here after chSysInit().           */
    motor_output_init();
    radio_input_init();
    can_drv_init();    // starts FDCAN1 + registers IMX5 callbacks (IDs 0x01-0x04)
    i2c_drv_init();    // placeholder — add compass/baro calls inside i2c_drv_init()

    /* ── Additional CAN devices ─────────────────────────────────────────────
     * bprl_can_register(MY_ID, my_callback, nullptr);                      */

    /* ── Additional I2C devices ─────────────────────────────────────────────
     * bprl_i2c_register(0x1E, compass_poll, nullptr);  // HMC5883
     * bprl_i2c_register(0x76, baro_poll,    nullptr);  // MS5611           */

    /* ══════════════════════════════════════════════════════════════════════
     * Thread rate sequencer
     * Adjust these values to change each thread's update rate.
     * Rates are passed to threads_start() and copied by each thread at
     * startup — no need to store them after threads_start() returns.
     * ══════════════════════════════════════════════════════════════════════ */
    static const ThreadRates kRates = {
        /* .spi     = */ TIME_US2I(1000),  // SPIThread      1000 Hz
        /* .can     = */ TIME_US2I(1000),  // CANThread      1000 Hz
        /* .est     = */ TIME_US2I(2000),  // StateEstThread  500 Hz
        /* .i2c     = */ TIME_MS2I(10),    // I2CThread       100 Hz
        /* .control = */ TIME_US2I(2000),  // ControlThread   500 Hz
        /* .radio   = */ TIME_MS2I(20),    // RadioThread      50 Hz  (14 ms for SBUS)
        /* .house   = */ TIME_MS2I(200),   // HouseThread       5 Hz
        /* .debug   = */ TIME_MS2I(100),   // DebugThread      10 Hz  [BPRL_DEBUG only]
    };

    threads_start(kRates);

    /* Main thread becomes low-priority idle background. */
    while (true) {
        chThdSleepMilliseconds(1000);
    }
    return 0;
}
