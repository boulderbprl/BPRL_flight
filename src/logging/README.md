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

Card format: **FAT32**, any capacity.  The Cube microSD slot (same PCB on
both `BOARD=orange` and `BOARD=blue`) has no card-detect or write-protect
signals wired to the MCU — presence is determined by whether `sdcConnect()`
succeeds.

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
| 0x09 | ATT  | time_us, roll, pitch, yaw, p, q, r, p_dot, q_dot, r_dot |
| 0x0A | LIN  | time_us, x, y, z, u, v, w, u_dot, v_dot, w_dot |
| 0x05 | RCIN | time_us, roll_stk, pitch_stk, yaw_stk, thr_stk, flight_mode, indi_stk, armed |
| 0x06 | OUTP | time_us, roll_tq, pitch_tq, yaw_tq, throttle |
| 0x07 | RPMS | time_us, rpm0–rpm3 |
| 0x08 | STRN | time_us, s0–s3, valid |
| 0x0B/0x0C/0x0D | IMU1/IMU2/IMU3 | time_us, ax, ay, az, gx, gy, gz, valid |
| 0x0E | INDI | time_us, unmix_roll, unmix_pitch, delta_roll, delta_pitch, cmd_roll, cmd_pitch, accel_roll, accel_pitch |
| 0x0F | BARO | time_us, pressure_pa, temp_c, alt_m, valid |

No message carries a rate field — every one logs at the fixed 50 Hz `LogThread` period, so it would only ever record a constant. See `LogMessages.hpp` for struct definitions and ArduPilot format codes.

Each `kLogDefs[]` entry's `name`/`fmt`/`labels` strings must fit the FMT record's fixed fields (`name` ≤4 chars, `fmt` ≤15, `labels` ≤63 — one byte short of the declared 4/16/64-byte fields since `strncpy` reserves a byte for the null terminator). A `static_assert` right after the table checks every entry at build time, so an over-length string fails the build instead of being silently truncated by `strncpy` at runtime (which happened once before the check existed).

## Robustness

**Initialization** (`Logger::init()`) runs up to **5 attempts** with 100 ms
between each.  After `sdcConnect()` and `f_mount()` succeed, the file is opened
and the schema header is written before `init()` returns `true`.

**Sync interval**: `f_sync()` is called every 5 flushes (~100 ms at 50 Hz),
bounding the data lost to an unclean power-off.

**File pre-allocation**: right after the file is created (still empty —
`f_expand()` requires `objsize == 0`), `init()` calls `f_expand(&s_file,
PRE_ALLOC_SIZE, 1)` to reserve a contiguous 10 MB cluster chain in one shot.
This means `f_write()` never needs to grow the FAT chain mid-flight — that
incremental growth, plus `f_sync()`'s own directory/FAT write, is what was
causing periodic multi-ms-to-hundreds-of-ms `LogThread` stalls (found via
`BPRL_TIMING` profiling, see the root README's [Timing and
Utilization](../../README.md#timing-and-utilization)). `close()` calls
`f_truncate()` to trim the file back to the actual bytes written before its
final `f_sync()`, so a short flight doesn't leave a mostly-empty 10 MB file.
Best-effort: if the card can't offer a contiguous run of that size (fragmented
or nearly-full card), logging still works via normal incremental growth, just
with the stalls pre-allocation exists to avoid — check `expand_err()`
(`FR_OK`/`0` = succeeded) via `LOG,status`'s `expand_err=` field rather than
assuming it worked. Even with pre-allocation confirmed working, some residual
stall rate remains: SD cards don't have a bounded worst-case write latency
(internal wear-leveling/garbage collection), so this mitigates rather than
eliminates the tail. It's a bounded, non-corrupting cost, though — the 32 KB
ring buffer absorbs any single stall without losing data, since `LogThread` is
both the sole producer and sole consumer of its own buffer.

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

`LOG,status` also reports `expand_err=` when the logger is ready — the FatFS
`FRESULT` from the `f_expand()` pre-allocation call described above (`0` =
`FR_OK`, succeeded; nonzero, commonly `FR_DENIED`, means it silently fell back
to incremental growth this boot).

## Source files

| File | Role |
|------|------|
| `Logger.hpp` | Class declaration, ring-buffer constants |
| `Logger.cpp` | Implementation: init, flush, close, ring buffer |
| `LogMessages.hpp` | Packed structs and FMT metadata for each message type |
