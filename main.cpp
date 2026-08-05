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
#include "src/coms/PWM.hpp"
#include "src/coms/Radio.hpp"
#include "src/usb_serial.hpp"
#include "configs/DroneConfig.hpp"
#include "chprintf.h"

/* DIAGNOSTIC: chSysHalt() (fired by a failed chDbgAssert) disables interrupts
 * before running CH_CFG_SYSTEM_HALT_HOOK (see cfg/chconf.h), so USB can't
 * actually flush anything written to SDU1 from inside that hook. Instead the
 * hook stashes the panic reason here — placed in the same .nocache region
 * s_usb_dl_buf already uses, which is NOLOAD and untouched by the C runtime's
 * .bss zero-fill, so it survives a warm/IWDG reset — and main() prints it
 * on the next boot, before anything else. g_panic_magic distinguishes a real
 * stashed panic from cold-boot SRAM garbage. Remove once resolved. */
#define PANIC_MAGIC 0xDEAD10CCU
extern "C" {
    volatile unsigned int __attribute__((section(".nocache"))) g_panic_magic;
    char                  __attribute__((section(".nocache"))) g_panic_reason[128];
}

/* DIAGNOSTIC: no LASTPANIC ever printed despite a repeatable hang means no
 * chDbgAssert fired — so this is very likely a hard fault (bad memory/
 * register access), whose default handler is a silent infinite loop that
 * never runs CH_CFG_SYSTEM_HALT_HOOK. These override the weak default
 * fault handlers (see vectors.S) to stash the fault type + CFSR/HFSR/
 * MMFAR/BFAR into the same cross-reset buffer, then reset immediately
 * instead of waiting out the 32s IWDG so we can iterate faster. Manual hex
 * formatting only — no chprintf/snprintf, to stay safe from fault context.
 * Remove once resolved. */
static void hex32(char *out, unsigned int v)
{
    static const char digits[] = "0123456789ABCDEF";
    for (int i = 7; i >= 0; i--) {
        out[7 - i] = digits[(v >> (i * 4)) & 0xF];
    }
    out[8] = '\0';
}

static void stash_fault(const char *name)
{
    char cfsr[9], hfsr[9], mmfar[9], bfar[9];
    hex32(cfsr,  SCB->CFSR);
    hex32(hfsr,  SCB->HFSR);
    hex32(mmfar, SCB->MMFAR);
    hex32(bfar,  SCB->BFAR);

    int n = 0;
    for (const char *p = name; *p && n < 40; p++) g_panic_reason[n++] = *p;
    g_panic_reason[n++] = ',';
    for (const char *p = "CFSR=0x";  *p; p++) g_panic_reason[n++] = *p;
    for (const char *p = cfsr;       *p; p++) g_panic_reason[n++] = *p;
    for (const char *p = ",HFSR=0x"; *p; p++) g_panic_reason[n++] = *p;
    for (const char *p = hfsr;       *p; p++) g_panic_reason[n++] = *p;
    for (const char *p = ",MMFAR=0x"; *p; p++) g_panic_reason[n++] = *p;
    for (const char *p = mmfar;      *p; p++) g_panic_reason[n++] = *p;
    for (const char *p = ",BFAR=0x"; *p; p++) g_panic_reason[n++] = *p;
    for (const char *p = bfar;       *p; p++) g_panic_reason[n++] = *p;
    g_panic_reason[n] = '\0';
    g_panic_magic = PANIC_MAGIC;

    NVIC_SystemReset();
}

extern "C" {
    void HardFault_Handler(void)  { stash_fault("HardFault");  }
    void MemManage_Handler(void)  { stash_fault("MemManage");  }
    void BusFault_Handler(void)   { stash_fault("BusFault");   }
    void UsageFault_Handler(void) { stash_fault("UsageFault"); }
}

/* DIAGNOSTIC: print a "BOOT,STAGE,<label>" line straight to SDU1 from main()'s
 * own context, before any thread (and therefore before USBCmdThread or any
 * scheduling/priority problem) exists. Watch with e.g.:
 *   python3 -c "import serial,sys; s=serial.Serial('/dev/ttyACM0',115200,timeout=1)
 *   [print(l) for l in iter(lambda: s.readline().decode(errors='replace'), '')]"
 * or any serial terminal (screen, minicom) at 115200 8N1.
 * Whichever STAGE line is the last one printed pinpoints the hang. Remove
 * once the USBCmdThread-not-responding investigation is resolved. */
static void stage_print(const char *label)
{
    chprintf((BaseSequentialStream *)&SDU1, "BOOT,STAGE,%s\r\n", label);
    chThdSleepMilliseconds(20);   /* let the USB CDC IN endpoint actually drain */
}

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
    stage_print("usb_ready");

    if (g_panic_magic == PANIC_MAGIC) {
        chprintf((BaseSequentialStream *)&SDU1, "BOOT,LASTPANIC,%s\r\n", g_panic_reason);
        chThdSleepMilliseconds(20);
        g_panic_magic = 0;   /* consumed — don't reprint next boot */
    }

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
        /* .i2c     = */ TIME_US2I(2000),  // 500 Hz — matches Teensy ADC sample rate
        /* .control = */ TIME_US2I(5000),  // DIAGNOSTIC: was 2500 (400 Hz) — doubled to 200 Hz
                                            // to test whether ControlThread is overrunning its
                                            // budget (merged EKF + 3 shadow controllers) and
                                            // starving lower-priority threads incl. USBCmdThread
        /* .radio   = */ TIME_MS2I(10),
        /* .heartbeat = */ TIME_MS2I(500),
        /* .debug   = */ TIME_MS2I(100),
        /* .log     = */ { TIME_MS2I(1000.0f / kDroneConfig.logging.log_rate_hz) },  // from DroneConfig
    };

    motor_output_init();
    stage_print("motor_output_init_done");
    can_drv_init();        // start FDCAN1, register IMX5 callbacks
    stage_print("can_drv_init_done");
    i2c_drv_init();        // start I2CD2 at 400 kHz
    stage_print("i2c_drv_init_done");
    strain_rate_init();    // register CAN or I2C based on STRAIN_RATE_INTERFACE
    stage_print("strain_rate_init_done");
    radio_input_init();    // start USART3 CRSF receiver at 420000 baud
    stage_print("radio_input_init_done");
    threads_start(kRates);
    stage_print("threads_start_done");

    /* Main thread: low-priority idle, feeds IWDG. */
    while (true) {
        IWDG1->KR = 0xAAAAU;   /* kick watchdog every second */
        chThdSleepMilliseconds(1000);
    }
    return 0;
}
