/*
 * board.c - CubeBlue H7 (STM32H753)
 *
 * Board-level initialization called by ChibiOS halInit() before
 * the scheduler starts. Use this for any hardware that must be
 * configured before threads run.
 *
 * The full GPIO init table (palSetupPads) should be populated in
 * board.h as you bring up each peripheral. For now this is a
 * minimal skeleton that gets the system booting.
 */

#include "hal.h"
#include "board.h"

/*===========================================================================*/
/* Board-specific initialization                                              */
/*===========================================================================*/

void boardInit(void) {
    /*
     * Called at the end of halInit(). Add any board-specific hardware
     * init here that isn't covered by the PAL/GPIO init table.
     *
     * Examples:
     *   - Assert/deassert reset lines to external ICs
     *   - Configure power enable pins
     *   - Enable external oscillators that need a startup delay
     */
}
