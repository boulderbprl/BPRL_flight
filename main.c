/*
 * main.c - CubeBlue H7 flight controller
 * ChibiOS/RT on STM32H753
 *
 * This is the minimal skeleton. Add your threads here and replace
 * the placeholder logic with your ported FreeRTOS task code.
 *
 * Key API differences from FreeRTOS (see comments below):
 *   - Higher priority NUMBER = higher priority (opposite of FreeRTOS)
 *   - chThdSleepMilliseconds()  vs  vTaskDelay(pdMS_TO_TICKS())
 *   - chThdSleepUntilWindowed() vs  vTaskDelayUntil()  [use for periodic tasks]
 *   - chMtxLock() / chMtxUnlock() vs xSemaphoreTake() / xSemaphoreGive()
 *   - chSemWait() / chSemSignal() vs xSemaphoreTake() / xSemaphoreGive()
 *   - chMBPostTimeout() / chMBFetchTimeout() vs xQueueSend() / xQueueReceive()
 */

#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Thread working areas (stack space)                                        */
/* Size in bytes. Must be sufficient for the thread's worst-case stack usage.*/
/* Start conservatively (2048) and reduce after profiling.                   */
/*===========================================================================*/

/* Example: 500 Hz attitude control loop */
static THD_WORKING_AREA(waFlightControl, 2048);

/* Example: 1 kHz sensor read thread */
static THD_WORKING_AREA(waSensorRead, 2048);

/*===========================================================================*/
/* Thread functions                                                           */
/*===========================================================================*/

/*
 * Flight control thread - runs your control loop.
 * Port your FreeRTOS flight control task here.
 */
static THD_FUNCTION(FlightControlThread, arg) {
    (void)arg;
    chRegSetThreadName("flight_ctrl");

    /* Record wake time for precise periodic execution (equivalent to
     * vTaskDelayUntil in FreeRTOS). */
    systime_t deadline = chVTGetSystemTime();
    const sysinterval_t period = TIME_MS2I(2); /* 2 ms = 500 Hz */

    while (true) {
        /*
         * TODO: Replace with your control loop body.
         * Read sensor data, run attitude estimation, compute outputs.
         */

        /* Sleep until next period. This compensates for execution time
         * and keeps the loop rate accurate, same as vTaskDelayUntil(). */
        deadline = chThdSleepUntilWindowed(deadline, chTimeAddX(deadline, period));
    }
}

/*
 * Sensor read thread - runs your IMU/baro/GPS read logic.
 * Port your FreeRTOS sensor task here.
 */
static THD_FUNCTION(SensorReadThread, arg) {
    (void)arg;
    chRegSetThreadName("sensor_read");

    systime_t deadline = chVTGetSystemTime();
    const sysinterval_t period = TIME_MS2I(1); /* 1 ms = 1 kHz */

    while (true) {
        /*
         * TODO: Replace with your sensor read body.
         * Read IMU over SPI, barometer over I2C, etc.
         */

        deadline = chThdSleepUntilWindowed(deadline, chTimeAddX(deadline, period));
    }
}

/*===========================================================================*/
/* main()                                                                     */
/*===========================================================================*/

int main(void) {

    /*
     * These two calls are always required first, in this order.
     * halInit()  - initialises the ChibiOS HAL (clocks, GPIO, peripherals)
     * chSysInit() - initialises the RT kernel and starts the scheduler
     */
    halInit();
    chSysInit();

    /*
     * Create threads.
     *
     * chThdCreateStatic() arguments:
     *   1. Working area (stack) pointer
     *   2. Working area size
     *   3. Priority (NORMALPRIO = 64, range 1-255, HIGHER = more urgent)
     *   4. Thread function
     *   5. Argument passed to thread function (NULL if unused)
     *
     * Priority convention (NOTE: OPPOSITE to FreeRTOS):
     *   LOWPRIO    = 1
     *   NORMALPRIO = 64
     *   HIGHPRIO   = 127
     *   Higher number = higher priority = preempts lower number threads
     *
     * Sensor read runs at higher priority than control to ensure
     * fresh data is always available when the control loop runs.
     */
    chThdCreateStatic(waSensorRead,    sizeof(waSensorRead),
                      NORMALPRIO + 20, SensorReadThread,    NULL);

    chThdCreateStatic(waFlightControl, sizeof(waFlightControl),
                      NORMALPRIO + 10, FlightControlThread, NULL);

    /*
     * The main thread becomes the lowest-priority active thread after
     * chSysInit(). You can use it as a background/housekeeping task,
     * or just loop here.
     */
    while (true) {
        chThdSleepMilliseconds(1000);
    }

    return 0; /* Never reached */
}
