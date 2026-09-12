/*
 * main.cpp — BPRL Standalone ChibiOS Flight Controller
 * Target:  STM32H7xx (CubeBlue H7 / CubeOrange+) at 400 MHz
 * Build:   make DRONE=Drone2              (or Drone1)
 * Flash:   make flash DRONE=Drone2  PORT=/dev/ttyACM0
 * Debug:   make DRONE=Drone2 UDEFS_EXTRA=-DBPRL_DEBUG
 * Timing:  make DRONE=Drone2 UDEFS_EXTRA=-DBPRL_TIMING   (see src/diagnostics/ThreadTiming.hpp)
 *
 * ── What lives where ────────────────────────────────────────────────────────
 *
 *   main.cpp               Init calls, thread rate sequencer, scheduler start.
 *   src/threads.cpp/.hpp   All seven thread function bodies + shared state.
 *   src/coms/SPI.*         On-board IMU + barometer SPI bus drivers (chip set is
 *                          board-conditional — ICM-45686 x3 on Drone1/CubeOrangePlus,
 *                          ICM-20948 x2 + ICM-20602 on Drone2/CubeBlueH7).
 *   src/coms/CAN.*         FDCAN1 driver, IMX5 callback, device registration.
 *   src/coms/I2C.*         I2C peripheral driver (I2CD1, 400 kHz).
 *   src/sensors/StrainRate.*  Strain rate sensor, CAN/I2C switchable via STRAIN_RATE_INTERFACE.
 *   src/sensors/EncoderRPM.*  Shaft-angle encoder RPM nodes, CAN 0x70/0x71 (Feather M4 + AS5047P).
 *   src/coms/PWM.*, DShot.* Motor output — bidirectional DShot 600 by default (MOTOR_PROTOCOL in PWM.hpp).
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
 *   via UDEFS_EXTRA). SBUS=SBUSo port (USART6), CRSF=TELEM1 (USART2).
 */

#include "ch.h"
#include "hal.h"
#include "src/threads.hpp"
#include "src/coms/CAN.hpp"
#include "src/coms/I2C.hpp"
#include "src/sensors/StrainRate.hpp"
#include "src/sensors/EncoderRPM.hpp"
#include "src/coms/PWM.hpp"
#include "src/coms/Radio.hpp"
#include "src/usb_serial.hpp"
#include "configs/DroneConfig.hpp"
#include "chprintf.h"
#include <cstdarg>

/* ══════════════════════════════════════════════════════════════════════════
 * TEMPORARY — Cube Blue hardware bring-up diagnostic (remove once confirmed
 * stable). Neither Cube board has a usable status LED, so the old staged
 * LED-blink diagnostic below is silent on real hardware and was never a
 * valid signal. This prints boot-stage text instead, over the same built-in
 * USB port (SDU1, micro USB / OTG_FS) used to flash — no Telem UART needed.
 *
 * Trade-off vs. a wired UART: SDU1 doesn't exist as a channel until
 * usb_serial_init() runs and the host enumerates it, so anything that hangs
 * before that point — including the clock/PWR bring-up in stm32_clock_init()
 * now wired up via __early_init() (see board.c) — produces NO output at all,
 * just a Cube that never shows up as a serial port. That silence is still
 * informative (narrows the hang to at-or-before usb_serial_init()); it just
 * can't distinguish clock init vs. chSysInit() vs. USB bring-up itself the
 * way per-stage UART prints could. If it comes to that, a wired UART on
 * either Telem port is the fallback for finer resolution.
 *
 * chprintf()'s normal SDU1 write blocks with an infinite timeout — first
 * boot showed the device enumerate, then silently reset on the IWDG ~30 s
 * later every time, which is what an indefinite block looks like (nothing
 * downstream of it, including the IWDG-feed loop, ever runs again). Bounded
 * to 100 ms so a slow/absent host can't wedge the watchdog feed. */
static void DBG_PRINTF(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = chvsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        chnWriteTimeout((BaseAsynchronousChannel *)&SDU1, (const uint8_t *)buf,
                         (size_t)n, TIME_MS2I(100));
    }
}

int main(void)
{
    /* CM4 is never released from reset on this dual-core part — this
     * firmware only runs on the M7 side (see board.h). Hold it explicitly
     * rather than relying on option-byte defaults: an auto-booting CM4 with
     * no firmware of its own would execute whatever garbage sits in the
     * unflashed program region and can write to shared D2-domain
     * peripherals (I2C2 lives there) with nothing stopping it. */
#if defined(STM32H757xx) || defined(STM32H747xx) || defined(STM32H755xx) || defined(STM32H745xx)
    RCC->GCR &= ~RCC_GCR_BOOT_C2;
#endif

    halInit();

    /* Start IWDG with ~32 s timeout for crash recovery.
     * 0x5555 unlocks PR/RLR; 0xCCCC starts the timer; 0xAAAA reloads.
     * LSI ≈ 32 kHz, /256 → 8 ms/tick; RLR=0xFFF=4095 → 32.76 s. */
    IWDG1->KR  = 0x5555U;
    IWDG1->PR  = 0x06U;    /* /256 */
    IWDG1->RLR = 0xFFFU;   /* ~32 s */
    IWDG1->KR  = 0xCCCCU;  /* start */
    IWDG1->KR  = 0xAAAAU;  /* reload */

    /* Staged LED diagnostic — kept harmless for CubeOrangePlus/future boards,
     * but neither Cube board has a usable status LED, so it's silent here.
     * DBG_PRINTF() below (once SDU1 exists) is the real signal now. */
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
    DBG_PRINTF("\r\n\r\nBOOT: usb_serial_init() OK — halInit()/chSysInit() also"
               " succeeded, since we got this far\r\n");

    /* ══════════════════════════════════════════════════════════════════════
     * Thread rate sequencer
     *
     * Not `static`: kRates.log now derives from kDroneConfig (a runtime
     * value, not a compile-time constant), so a function-local `static`
     * here would need the C++ thread-safe-init guard (__cxa_guard_acquire/
     * release) that this embedded target's minimal runtime doesn't provide.
     * A plain local is fine — main() never returns, so it lives for the
     * program's whole lifetime, which is all threads_start()'s per-thread
     * pointers into it need.
     * ══════════════════════════════════════════════════════════════════════ */
    const ThreadRates kRates = {
        /* .spi     = */ TIME_US2I(1000),
        /* .i2c     = */ TIME_US2I(5000),  // 200 Hz
        /* .control = */ TIME_US2I(2500),  // 400 Hz — matches ArduPilot default
        /* .radio   = */ TIME_MS2I(10),
        /* .heartbeat = */ TIME_MS2I(500),
        /* .debug   = */ TIME_MS2I(100),
        /* .log     = */ { TIME_MS2I(1000.0f / kDroneConfig.logging.log_rate_hz) },  // from DroneConfig
    };

    // ── IMPORTANT: none of the calls below were ever board-conditional in
    // git history. The Cube Blue bring-up's "disable everything" (this
    // whole block was commented out at HEAD, commit 42448c9, before this
    // bring-up session even started) therefore disabled every peripheral —
    // including motor_output_init() — for EVERY board, not just
    // CubeBlueH7. Restored full peripheral init for CubeOrangePlus/Orqa
    // below, matching the last confirmed-working configuration (commit
    // 61c453a, confirmed flying on CubeOrangePlus) plus encoder_rpm_init(),
    // a real feature added since. Cube Blue keeps its own deliberate,
    // still-in-progress staged config. ──
#if defined(BPRL_BOARD_CUBEBLUE)
    // Cube Blue bring-up, stage 1: root cause found and fixed (linker
    // script had the app linked at 0x08000000 — flash address zero, on top
    // of the board's own 128 KB bootloader reservation — so the bootloader
    // never jumped to the app at all; see boards/CubeBlueH7/STM32H743xI.ld).
    // Confirmed booting + USB CDC TX/RX both working, all sensors reading
    // correctly (python3 tools/hw_status.py). motor_output_init() and
    // ControlThread (src/threads.cpp) stay disabled on purpose until this
    // stage is confirmed stable — ControlThread is what actually calls
    // motor_output_write() outside of a USB "MT," test command. i2c_drv_init()/
    // strain_rate_init() stay off too — this board's sensors are all SPI.
    // motor_output_init();
    can_drv_init();        // start FDCAN1, register IMX5 callbacks
    // i2c_drv_init();
    // strain_rate_init();
    encoder_rpm_init();    // register CAN 0x70/0x71 shaft-angle encoder nodes
    radio_input_init();    // start USART3 CRSF receiver at 420000 baud
#else
    motor_output_init();
    can_drv_init();        // start FDCAN1, register IMX5 callbacks
    i2c_drv_init();        // start I2CD2 at 400 kHz
    strain_rate_init();    // register CAN or I2C based on STRAIN_RATE_INTERFACE
    encoder_rpm_init();    // register CAN 0x70/0x71 shaft-angle encoder nodes
    radio_input_init();    // start USART3 CRSF receiver at 420000 baud
#endif
    threads_start(kRates);
    DBG_PRINTF("BOOT: threads_start() returned\r\n");

    /* Main thread: low-priority idle, feeds IWDG. */
    uint32_t alive = 0;
    while (true) {
        IWDG1->KR = 0xAAAAU;   /* kick watchdog every second */
        chThdSleepMilliseconds(1000);
        DBG_PRINTF("ALIVE %lu\r\n", (unsigned long)alive++);
    }
    return 0;
}
