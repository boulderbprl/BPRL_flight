/*
 * boards/CubeBlueH7/board.c
 * CubeOrange+ — STM32H743ZI board-level initialisation
 *
 * Called by ChibiOS halInit() before the scheduler starts.
 */

#include "hal.h"
#include "board.h"

void boardInit(void)
{
    /* Power up the 3.3V sensor rail (IMUs, barometer) */
    palSetLine(LINE_SENSOR_PWR_EN);
}
