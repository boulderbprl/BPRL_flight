# SD Card Logging

Binary flight-data logger using FatFS over ChibiOS SDMMC1.

## Hardware

| Signal | MCU pin | AF   |
|--------|---------|------|
| SDMMC1_D0 | PC8  | AF12 |
| SDMMC1_D1 | PC9  | AF12 |
| SDMMC1_D2 | PC10 | AF12 |
| SDMMC1_D3 | PC11 | AF12 |
| SDMMC1_CK | PC12 | AF12 |
| SDMMC1_CMD | PD2 | AF12 |

Card format: **FAT32**, any capacity.  The CubeOrangePlus microSD slot has no
card-detect or write-protect signals wired to the MCU — presence is determined
by whether `sdcConnect()` succeeds.

## Clock configuration

SDMMC1 clock source: `PLL1_Q = 50 MHz` (`STM32_SDMMCSEL_PLL1_Q_CK`).

| Phase | Target | CLKDIV | Actual |
|-------|--------|--------|--------|
| Init  | 400 kHz | 63 | ~397 kHz |
| Data  | 25 MHz  |  1 | 25 MHz   |

`STM32_SDC_MAX_CLOCK 25000000` in `cfg/mcuconf.h` prevents CLKDIV=0 (bypass
mode), which PLL1_Q=50 MHz would otherwise trigger for a 50 MHz HS request.
Bypass mode produces a noisier clock that many cards reject.

## DMA memory layout

SDMMC1 IDMA (AHB2 master) can only reliably access **AXI SRAM** (0x24000000)
on STM32H743 — not SRAM3 (0x30040000).  This matches the restriction enforced
by ArduPilot's bouncebuffer (`mem_is_dma_safe` rejects SRAM3 for filesystem
DMA on H7).

The `.nocache` linker section occupies the **last 16 KB of AXI SRAM**
(`0x2407C000–0x2407FFFF`), marked non-cacheable by MPU region 6
(`STM32_NOCACHE_RBAR = 0x2407C000`, `MPU_RASR_SIZE_16K` in `cfg/mcuconf.h`).
All SDMMC DMA buffers land here:

| Variable | Purpose | Size |
|----------|---------|------|
| `__nocache_sd1_buf` | ChibiOS SDC internal (CID/CSD reads) | 512 B |
| `s_fs` | FatFS filesystem object (incl. sector window `win[]`) | ~600 B |
| `s_file` | FatFS open-file object | ~576 B |
| `s_flush_buf` | Write staging buffer | 4096 B |

## File format

Log files are written to `/LOGS/LOGxxxx.BIN` (auto-incrementing index).
Each file begins with a sequence of `FMT` records that describe every message
type (ArduPilot-compatible schema), followed by the data stream.

Every record is three bytes of header followed by a packed struct body:

```
[0xA3][0x95][msg_id][... body ...]
```

Files can be opened directly in [UAV Log Viewer](https://plot.ardupilot.org).

### Message types

| ID | Name | Fields |
|----|------|--------|
| 0x01 | ATT  | TimeUS, rate_hz, roll, pitch, yaw, p, q, r, p_dot, q_dot, r_dot |
| 0x02 | LIN  | TimeUS, ax, ay, az, vx, vy, vz, x, y, z |
| 0x03 | RCIN | TimeUS, throttle, roll, pitch, yaw, armed |
| 0x04 | OUTP | TimeUS, m1–m4 (motor commands) |
| 0x05 | RPMS | TimeUS, rpm1–rpm4, v1–v4 (ESC telemetry) |
| 0x06 | STRN | TimeUS, strain_rate, valid |

See `LogMessages.hpp` for struct definitions and ArduPilot format codes.

## Robustness

**Initialization** (`Logger::init()`) runs up to **5 attempts** with 100 ms
between each.  After `sdcConnect()` and `f_mount()` succeed, the file is opened
and the schema header is written before `init()` returns `true`.

**Sync interval**: `f_sync()` is called every 5 flushes (~100 ms at 50 Hz),
bounding the data lost to an unclean power-off.

**Runtime recovery**: if a flush fails (write error or card removed), LogThread
calls `logger.close()`, waits 2 s, and retries `logger.init()`.  Logging
resumes automatically if the card becomes accessible again.

**Init error codes** (visible via `LOG,status` USB command):

| Code | Meaning |
|------|---------|
| 0 | Not yet attempted |
| 1 | `sdcStart()` failed |
| 2 | `sdcConnect()` failed |
| 3 | `f_mount()` failed (see `last_ff_err` for FatFS `FRESULT`) |
| 4 | `f_open()` failed |
| 5 | Success |

## Source files

| File | Role |
|------|------|
| `Logger.hpp` | Class declaration, ring-buffer constants |
| `Logger.cpp` | Implementation: init, flush, close, ring buffer |
| `LogMessages.hpp` | Packed structs and FMT metadata for each message type |
