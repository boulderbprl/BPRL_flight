/*
 * CubeBlueH7 application linker script.
 *
 * ── Bootloader flash reservation (THE Cube Blue bug) ────────────────────
 * This board ships with the same BL5 Cube/ArduPilot bootloader family as
 * CubeOrangePlus (confirmed live: tools/flash_upload.py reports board
 * ID=0x8c, BL rev=5, flash=1966080 B — i.e. 2097152 - 1966080 = 131072 B =
 * exactly 128 KB reserved at the bottom of flash for the bootloader itself,
 * identical to CubeOrangePlus's own 128 KB reservation). This file used to
 * define flash0 as starting at 0x08000000 — the true flash base, i.e. where
 * the BOOTLOADER lives, not the app — with a comment claiming "this board
 * is flashed at 0x08000000 directly, no bootloader offset". That was wrong:
 * there unavoidably IS a bootloader (it's what tools/flash_upload.py talks
 * to, and what enumerates as "Hex/ProfiCNC CubeOrangeRevB-BL" / 2dae:1017),
 * and it physically writes the flashed app image starting at 0x08020000,
 * not 0x08000000, exactly like CubeOrangePlus. Linking the app to think it
 * starts at 0x08000000 while it's actually loaded at 0x08020000 makes every
 * absolute address the linker bakes in — starting with the vector table's
 * own reset-handler entry — wrong by 128 KB. The bootloader reads that
 * corrupted reset vector, sees it's outside the valid app range, and
 * concludes there's no valid application: it never jumps, and stays
 * resident forever. That's the entire "enumerates as 2dae:1017 and never
 * boots into the app, on every single power cycle, with byte-identical
 * firmware" symptom seen bringing this board up — not a USB bug, not a
 * clock bug: the app was never being entered at all. Fixed by matching
 * CubeOrangePlus's scheme exactly (see boards/CubeOrangePlus/
 * STM32H743xI_app.ld) — 128 KB BL reservation, app at 0x08020000, same
 * 128 KB calibration sector reserved at the top of flash for consistency
 * even if unused here yet.
 *
 * Memory layout:
 *   Bootloader  : 0x08000000  128 K   (reserved, not ours)
 *   Application : 0x08020000  1792 K  (this firmware)
 *   Calibration : 0x081E0000  128 K   (Bank 2 sector 7 — persistent cal data)
 *
 * ── NOCACHE_RAM placement ────────────────────────────────────────────────
 * Also moved from SRAM3 to the last 16 KB of AXI SRAM to match
 * STM32_NOCACHE_RBAR/RASR in cfg/mcuconf.h.
 *
 * Why: mcuconf.h's non-cacheable MPU region (region 6) was moved from SRAM3
 * (0x30040000) to AXI SRAM (0x2407C000) when SD logging was fixed — SDMMC1
 * IDMA can only reliably reach AXI SRAM on STM32H743. The generic ChibiOS
 * linker script (used directly by this board before this file existed)
 * keeps NOCACHE_RAM aliased to SRAM3, which the MPU no longer marks
 * non-cacheable. That mismatch meant Logger's SD buffers (s_fs, s_file,
 * s_flush_buf) landed in SRAM3 while still cacheable — the Cortex-M7
 * default memory map treats the whole 0x20000000-0x3FFFFFFF range as
 * cacheable unless an MPU region says otherwise — while SDMMC DMA reads and
 * writes that physical memory directly, bypassing the cache entirely.
 * Classic DMA/cache coherency bug. CubeOrangePlus already has this fix in
 * its own STM32H743xI_app.ld; this gives CubeBlueH7 the same fix.
 */

MEMORY
{
    flash0 (rx) : org = 0x08020000, len = 1792k      /* App flash (after 128 KB BL) */
    cal    (r)  : org = 0x081E0000, len = 128k        /* Calibration sector (Bank2 S7) */
    flash1 (rx) : org = 0x08000000, len = 1M         /* Bank 1 (not directly used)  */
    flash2 (rx) : org = 0x08100000, len = 1M         /* Bank 2 (not directly used)  */
    flash3 (rx) : org = 0x00000000, len = 0
    flash4 (rx) : org = 0x00000000, len = 0
    flash5 (rx) : org = 0x00000000, len = 0
    flash6 (rx) : org = 0x00000000, len = 0
    flash7 (rx) : org = 0x00000000, len = 0
    ram0   (wx) : org = 0x24000000, len = 496k       /* AXI SRAM (cacheable) */
    ram0nc (wx) : org = 0x2407C000, len = 16k        /* AXI SRAM — non-cacheable via MPU (SDMMC DMA) */
    ram1   (wx) : org = 0x30000000, len = 256k       /* AHB SRAM1+SRAM2 */
    ram2   (wx) : org = 0x30000000, len = 288k       /* AHB SRAM1+SRAM2+SRAM3 */
    ram3   (wx) : org = 0x30040000, len = 32k        /* AHB SRAM3 */
    ram4   (wx) : org = 0x38000000, len = 64k        /* AHB SRAM4 */
    ram5   (wx) : org = 0x20000000, len = 128k       /* DTCM-RAM */
    ram6   (wx) : org = 0x00000000, len = 64k        /* ITCM-RAM */
    ram7   (wx) : org = 0x38800000, len = 4k         /* BCKP SRAM */
}

REGION_ALIAS("VECTORS_FLASH",     flash0);
REGION_ALIAS("VECTORS_FLASH_LMA", flash0);
REGION_ALIAS("XTORS_FLASH",       flash0);
REGION_ALIAS("XTORS_FLASH_LMA",   flash0);
REGION_ALIAS("TEXT_FLASH",        flash0);
REGION_ALIAS("TEXT_FLASH_LMA",    flash0);
REGION_ALIAS("RODATA_FLASH",      flash0);
REGION_ALIAS("RODATA_FLASH_LMA",  flash0);
REGION_ALIAS("VARIOUS_FLASH",     flash0);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash0);
REGION_ALIAS("RAM_INIT_FLASH_LMA",flash0);

REGION_ALIAS("MAIN_STACK_RAM",    ram5);
REGION_ALIAS("PROCESS_STACK_RAM", ram5);
REGION_ALIAS("DATA_RAM",          ram0);
REGION_ALIAS("DATA_RAM_LMA",      flash0);
REGION_ALIAS("BSS_RAM",           ram0);
REGION_ALIAS("HEAP_RAM",          ram0);
REGION_ALIAS("NOCACHE_RAM",       ram0nc);
REGION_ALIAS("ETH_RAM",           ram3);

INCLUDE rules_stacks.ld

SECTIONS
{
    .nocache (NOLOAD) : ALIGN(4)
    {
        __nocache_base__ = .;
        *(.nocache)
        *(.nocache.*)
        *(.bss.__nocache_*)
        . = ALIGN(4);
        __nocache_end__ = .;
    } > ram0nc

    .eth (NOLOAD) : ALIGN(4)
    {
        __eth_base__ = .;
        *(.eth)
        *(.eth.*)
        *(.bss.__eth_*)
        . = ALIGN(4);
        __eth_end__ = .;
    } > ETH_RAM
}

INCLUDE rules_code.ld
INCLUDE rules_data.ld
INCLUDE rules_memory.ld
