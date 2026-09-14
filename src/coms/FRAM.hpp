#pragma once
#include "hal.h"

/*
 * Driver for Cypress/Fujitsu FM25-series SPI FRAM ("Ramtron") — byte-
 * addressable, non-volatile, no erase cycle and no busy-wait, unlike
 * internal flash sector programming. Present on Cube-family carrier boards
 * (CubeOrangePlus, CubeBlueH7) per ArduPilot's own CubeOrange hwdef.inc:
 *   SPIDEV ramtron SPI2 DEVID10 FRAM_CS MODE3 8*MHZ 8*MHZ
 *   PB13=SPI2_SCK, PB14=SPI2_MISO, PB15=SPI2_MOSI (AF5), PD10=FRAM_CS
 * NOT present on Orqa (different carrier board design) — CalFlash.cpp keeps
 * using internal flash on that board; see its board-conditional dispatch.
 *
 * Protocol (transcribed from ArduPilot's AP_RAMTRON driver — same op-codes
 * across the whole Cypress/Fujitsu FM25 family):
 *   RDID  (0x9F) -> read manufacturer/device ID; used at init() to identify
 *                   the populated part (and therefore its size and whether
 *                   it takes a 2- or 3-byte address — smaller parts use 2).
 *   WREN  (0x06) -> write-enable, must precede every WRITE; auto-clears
 *                   after the write completes.
 *   READ  (0x03) + address -> read bytes back-to-back from that offset.
 *   WRITE (0x02) + address -> write bytes immediately (no erase, no
 *                   busy-wait — unlike NOR flash, FRAM writes complete
 *                   synchronously within the SPI transaction itself).
 * A wrong/absent chip fails safe: init() only succeeds if RDID matches a
 * known part, exactly like this codebase's SPI IMU WHOAMI checks.
 *
 * Call fram_drv_init() once from spi_drv_init() (inside SPIThread, for the
 * same chThdSleepMilliseconds-during-init reason documented in SPI.hpp).
 * fram_read()/fram_write() may be called from any thread afterward — they
 * acquire the SPI2 bus themselves.
 */

bool     fram_drv_init(void);
bool     fram_ready(void);
uint32_t fram_size(void);                                       // bytes, 0 if not ready
bool     fram_read(uint32_t offset, uint8_t *buf, uint32_t size);
bool     fram_write(uint32_t offset, const uint8_t *buf, uint32_t size);
