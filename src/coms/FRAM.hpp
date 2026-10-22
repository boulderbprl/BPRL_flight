#pragma once
#include "hal.h"

/*
 * Driver for the Cypress/Fujitsu FM25-series SPI FRAM ("Ramtron") on the
 * Cube carrier boards — byte-addressable, non-volatile, no erase cycle and
 * no busy-wait. Ported from ArduPilot's AP_RAMTRON, wired exactly as
 * ArduPilot's CubeOrange hwdef.inc (shared by CubeOrangePlus and Cube Blue):
 *   SPIDEV ramtron SPI2 DEVID10 FRAM_CS MODE3 8*MHZ 8*MHZ
 *   PB13=SPI2_SCK, PB14=SPI2_MISO, PB15=SPI2_MOSI (AF5), PD10=FRAM_CS
 *
 * Why: the IMU calibration used to live only in internal flash at
 * 0x081E0000, which the Cube bootloader's CHIP_ERASE wipes on every
 * firmware upload. ArduPilot keeps all its parameters (INS_ACCOFFS etc.) in
 * this FRAM for the same reason — see CalFlash.cpp.
 *
 * History: a 2026-09-14 FRAM attempt coincided with a Cube Blue crash loop
 * that survived reverting it (see README). Bring this up with a full power
 * cycle between tests; a missing/unresponsive chip fails init() and
 * CalFlash falls back to internal flash rather than halting (ArduPilot
 * panics instead — deliberately not copied).
 *
 * Protocol (AP_RAMTRON op-codes, same across the FM25 family):
 *   RDID  (0x9F) -> identify the part (size, 2- or 3-byte addressing)
 *   WREN  (0x06) -> write-enable, must precede every WRITE
 *   READ  (0x03) + address -> read back-to-back from that offset
 *   WRITE (0x02) + address -> write immediately (no erase/busy-wait)
 *
 * fram_drv_init() must run from a thread (SPIThread), before cal_load().
 * fram_read()/fram_write() take the SPI2 bus lock for the whole
 * transaction and may be called from any thread afterward.
 */

#if defined(BPRL_BOARD_CUBEBLUE) || defined(BPRL_BOARD_CUBEORANGEPLUS)
#define BPRL_HAS_FRAM 1

bool     fram_drv_init(void);
bool     fram_ready(void);
uint32_t fram_size(void);                                       // bytes, 0 if not ready
const char *fram_part_name(void);                               // "none" if not ready
bool     fram_read(uint32_t offset, uint8_t *buf, uint32_t size);   // size <= 128
bool     fram_write(uint32_t offset, const uint8_t *buf, uint32_t size);  // size <= 128

#endif
