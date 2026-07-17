/*
 * main.cpp — BPRL Standalone ChibiOS Flight Controller
 * Target:  STM32H7xx (CubeBlue H7 / CubeOrange+) at 400 MHz
 * Build:   make BOARD=CubeBlueH7          (or CubeOrangePlus)
 * Flash:   make flash BOARD=CubeBlueH7  PORT=/dev/ttyACM0
 * Debug:   make BOARD=CubeBlueH7 UDEFS_EXTRA=-DBPRL_DEBUG
 * Timing:  make BOARD=CubeBlueH7 UDEFS_EXTRA=-DBPRL_TIMING   (see src/diagnostics/ThreadTiming.hpp)
 *
 * ── What lives where ────────────────────────────────────────────────────────
 *
 *   main.cpp               Init calls, thread rate sequencer, scheduler start.
 *   src/threads.cpp/.hpp   All eight thread function bodies + shared state.
 *   src/coms/SPI.*         ICM-20948/20602 SPI bus drivers.
 *   src/coms/CAN.*         FDCAN1 driver, IMX5 callback, device registration.
 *   src/coms/I2C.*         I2C peripheral driver (I2CD1, 400 kHz).
 *   src/sensors/StrainRate.*  Strain rate sensor, CAN/I2C switchable via STRAIN_RATE_INTERFACE.
 *   src/coms/PWM.*         Motor PWM output stub (TIM1, future DShot).
 *   src/coms/Radio.*       RC radio input (SBUS on SBUSo / CRSF on TELEM1).
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
 * ── Switching radio protocol ────────────────────────────────────────────────
 *   Change RADIO_PROTOCOL in src/coms/Radio.hpp (or pass -DRADIO_PROTOCOL=...
 *   via UDEFS_EXTRA). SBUS=SBUSo port (USART6), CRSF=TELEM1 (USART3).
 */

#include "ch.h"
#include "hal.h"
#include "src/threads.hpp"
#include "src/coms/CAN.hpp"
#include "src/coms/I2C.hpp"
#include "src/sensors/StrainRate.hpp"
#include "src/coms/PWM.hpp"
#include "src/coms/Radio.hpp"
#include "src/usb_serial.hpp"
#include "chprintf.h"

int main(void)
{
    halInit();

    /* Start IWDG with ~32 s timeout for crash recovery.
     * 0x5555 unlocks PR/RLR; 0xCCCC starts the timer; 0xAAAA reloads.
     * LSI ≈ 32 kHz, /256 → 8 ms/tick; RLR=0xFFF=4095 → 32.76 s. */
    IWDG1->KR  = 0x5555U;
    IWDG1->PR  = 0x06U;    /* /256 */
    IWDG1->RLR = 0xFFFU;   /* ~32 s */
    IWDG1->KR  = 0xCCCCU;  /* start */
    IWDG1->KR  = 0xAAAAU;  /* reload */

    /* Staged LED diagnostic: blink count tells us how far boot got.
     *   3 fast blinks  = halInit() done, pre-RTOS code running
     *   +5 slow blinks = chSysInit() + usb_serial_init() succeeded
     *   Then HeartbeatThread takes over at ~0.5 Hz.
     */
#define BLINK_TICK  4000000U   /* ~50 ms at 400 MHz */
#define BLINK_SLOW 32000000U   /* ~400 ms at 400 MHz */
#define BLINK_GAP    400000U   /* ~5 ms gap */
    for (int i = 0; i < 3; i++) {               /* 3 fast: halInit OK */
        palSetLine(LINE_LED_ACTIVITY);
        { volatile uint32_t n = BLINK_TICK; while (n--) {} }
        palClearLine(LINE_LED_ACTIVITY);
        { volatile uint32_t n = BLINK_GAP;  while (n--) {} }
    }

    chSysInit();

    for (int i = 0; i < 5; i++) {               /* 5 slow: chSysInit OK */
        palSetLine(LINE_LED_ACTIVITY);
        chThdSleepMilliseconds(400);
        palClearLine(LINE_LED_ACTIVITY);
        chThdSleepMilliseconds(100);
    }

    usb_serial_init();
    chThdSleepMilliseconds(1500);   /* wait for host USB enumeration */

    /* ══════════════════════════════════════════════════════════════════════
     * Thread rate sequencer
     * ══════════════════════════════════════════════════════════════════════ */
    static const ThreadRates kRates = {
        /* .spi     = */ TIME_US2I(1000),
        /* .est     = */ TIME_US2I(1600),  // TEMP BISECTION TEST (uncommitted): 625 Hz (16 ticks at the 100us/10kHz system tick — nearest achievable above 600 Hz) — was 1700 (~588 Hz), 2500 (400 Hz), 2000 (500 Hz), originally 1250 (800 Hz)
        /* .i2c     = */ TIME_US2I(2000),  // 500 Hz — matches Teensy ADC sample rate
        /* .control = */ TIME_US2I(2500),  // 400 Hz — matches ArduPilot default
        /* .radio   = */ TIME_MS2I(10),
        /* .heartbeat = */ TIME_MS2I(500),
        /* .debug   = */ TIME_MS2I(100),
        /* .log     = */ { TIME_MS2I(20) },   // 50 Hz
    };

    motor_output_init();
    can_drv_init();        // start FDCAN1, register IMX5 callbacks
    i2c_drv_init();        // start I2CD2 at 400 kHz
    strain_rate_init();    // register CAN or I2C based on STRAIN_RATE_INTERFACE
    radio_input_init();    // start USART3 CRSF receiver at 420000 baud
    threads_start(kRates);

    /* Main thread: low-priority idle, feeds IWDG. */
    while (true) {
        IWDG1->KR = 0xAAAAU;   /* kick watchdog every second */
        chThdSleepMilliseconds(1000);
    }
    return 0;
}
