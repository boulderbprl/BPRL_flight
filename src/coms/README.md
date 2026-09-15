# BPRL Communications Drivers

All peripheral drivers live in `src/coms/`. Register CAN devices and I2C devices in `main.cpp` before calling `threads_start()`.

---

## CAN — FDCAN1 (`CAN.hpp/.cpp`)

**Clock:** PLL2Q = 80 MHz → 1 Mbit/s (BRP=5, TSEG1=13, TSEG2=2, 87.5% sample point)

Classical CAN, standard 11-bit IDs, RxFIFO0 accept-all filter (16 elements, overwrite-on-full).

This is a **direct register-level FDCAN1 driver**, not ChibiOS's `HAL_USE_CAN` (`HAL_USE_CAN` is `FALSE` in `cfg/halconf.h`). That driver's `canStart()` never programs `RXF0C` — the register that gives RxFIFO0 an actual address/size in message RAM — so on STM32H7 it silently receives nothing forever: frames get ACKed on the wire (no bus errors, looks perfectly healthy) and then have nowhere to be stored. ArduPilot doesn't hit this because it has its own FDCAN driver (`AP_HAL_ChibiOS/CANFDIface.cpp`) and never goes through that ChibiOS layer either. Full writeup is in the comment at the top of `CAN.cpp`.

### RX path — interrupt-driven

`FDCAN1_IT0` (our own `OSAL_IRQ_HANDLER(STM32_FDCAN1_IT0_HANDLER)`, not ChibiOS's) fires on new-message / message-lost / bus-off and signals a binary semaphore — no register or FIFO work happens in ISR context. `CANThread` blocks on that semaphore (`bprl_can_wait_rx()`, 200 ms timeout), drains RxFIFO0 with `bprl_can_poll()`, and calls `can_dispatch()` per frame, routing it to the registered callback in O(n) time.

### Self-healing

Every time `CANThread` wakes (on real traffic or the 200 ms timeout), it checks `PSR.BO` (Bus_Off) and calls `can_hw_reinit()` — a full hardware reconfigure — if set, so a transient bus fault recovers without a reflash. Two counters track this instead of it happening silently:

- `msg_lost` — RxFIFO0 overflowed (CANThread fell behind the bus)
- `reinit_count` — Bus_Off recoveries so far; climbing steadily means something on the bus (wiring, termination, a bad node) is causing real errors, not just this firmware

Check both with `python3 tools/can_tools.py can-diag`. Other diagnostics: `can-status` (raw PSR/ECR/RXF0S/CCCR), `can-regdump` (full FDCAN1 register dump), `can-scan` (per-ID traffic breakdown over a time window).

### Registered devices

| CAN ID | Device | Content | Rate |
|---|---|---|---|
| `0x01` | IMX5 quaternion | NED→Body [W,X,Y,Z], 4×int16 ÷ 10000 | 200 Hz |
| `0x02` | IMX5 p + ax | int16 ÷ 1000 (rad/s), int16 ÷ 100 (m/s²) | 100 Hz |
| `0x03` | IMX5 q + ay | same encoding | 100 Hz |
| `0x04` | IMX5 r + az | same encoding | 100 Hz |
| `0x69` | Strain rate sensor | 4 signed int16 values, one per arm (FR/RL/FL/RR) | 100 Hz — this is the override interface (`STRAIN_RATE_INTERFACE=STRAIN_RATE_CAN`, see `src/sensors/StrainRate.*`); I2C is the **default** |

`imx5_can_cb()` timestamps the quaternion (`0x01`) and rate (`0x02`) frames into `g_can_imu.quat_timestamp_us`/`rates_timestamp_us` on arrival — `StateManager` uses these to age-gate and forward-propagate the measurements rather than fusing/blending them as if they arrived instantaneously (see the root README's [State Estimation](../../README.md#3-state-estimation-ekf) section).

### Adding a device

```cpp
// in main.cpp, before threads_start():
bprl_can_register(0x10, my_callback, nullptr);
```

---

## DShot — Bidirectional DShot 600 (`DShot.hpp/.cpp`)

> ⚠️ **If DShot ever regresses on CubeOrangePlus/Orqa, start at the "DSHOT
> REGRESSION CHECKPOINT" comment at the top of `DShot.cpp`.** It documents
> two changes made on 2026-09-14 while debugging CubeBlueH7's *separate*
> standard-PWM output (`PWM.cpp`, `MOTOR_PROTO_PWM`) — DShot.cpp got touched
> only as a side effect (wrapped in `#if MOTOR_PROTOCOL == MOTOR_PROTO_DSHOT`
> so it compiles to nothing on a PWM-protocol board), and `cfg/mcuconf.h`/
> `cfg/halconf.h` gained board-conditional `STM32_PWM_USE_TIM1`/`TIM4`/
> `HAL_USE_PWM` flags for ChibiOS's own PWM driver. Both are explicitly
> gated off for DShot boards and should be no-ops for them, but the
> checkpoint comment gives the exact revert steps if that gate is ever
> wrong.

Motor output protocol is selected at compile time via `MOTOR_PROTOCOL` in `PWM.hpp`:

```cpp
#define MOTOR_PROTO_DSHOT  0   // bidirectional DShot 600 (default)
#define MOTOR_PROTO_PWM    1   // standard servo PWM
```

`PWM.hpp/.cpp` is a thin wrapper around this choice — `motor_output_write(val[4])` accepts normalized commands (**0 = disarm**, **1–1000 = 0.1–100% throttle**) and forwards to `dshot_write()`. All the actual timer/DMA/GCR-decode work lives in `DShot.hpp/.cpp`, which has two board-specific implementations behind `#if defined(BPRL_BOARD_ORQA)` sharing common DShot600 bit-timing/GCR-decode/frame-construction code:

### Cube boards (default — `BPRL_BOARD_CUBEBLUE` / `BPRL_BOARD_CUBEORANGEPLUS`)

TIM1 carries 3 motors sharing one timer via ArduPilot's CC2 cross-capture trick (only TIM1 has a usable 3-channel-BIDIR configuration on this MCU); TIM4 carries the 4th motor alone, no sharing needed.

| Motor (logical) | Pin | Timer / Channel |
|---|---|---|
| 0 | PE11 | TIM1 CH2 |
| 1 | PE9  | TIM1 CH1 |
| 2 | PD13 | TIM4 CH2 |
| 3 | PE13 | TIM1 CH3 |

**⚠️ These pins are the Cube carrier board's AUX outputs (AUX2-5), not MAIN.** Specifically: motor 0 (FR)→AUX3, motor 1 (RL)→AUX4, motor 2 (FL)→AUX5, motor 3 (RR)→AUX2. On real Cube/Pixhawk-family hardware, **MAIN 1-8 is driven by a separate IO co-processor** (a small STM32F1-class chip) that the main FMU talks to over a dedicated serial link — `PE9`/`PE11`/`PE13`/`PD13` above are direct FMU timer pins, which only ever reach the AUX connector, not MAIN. Plugging ESCs into MAIN 1-4 while running `MOTOR_PROTO_PWM`/`MOTOR_PROTO_DSHOT` gets you a real board that boots, a real signal generated correctly in firmware, and zero volts at the ESC — nothing drives those pins from this path, so the ESCs sit there beeping their own "no signal" alert forever.
>
> **Update**: on CubeBlueH7, AUX itself later turned out to be unreliable at the hardware level (bench-confirmed, and independently reproduced under stock ArduPilot firmware on the same unit) — see the **IOMCU** section below for the driver that talks to the IO co-processor directly and drives MAIN 1-4 instead. If you're on AUX and it's working fine, no need to switch; if AUX is flaky like it was here, `MOTOR_PROTO_IOMCU` is the fix, not a wiring change.

- **TX** (burst DMA, 400 Hz): TIM1 via `TIM1_UP→DMAR`, DCR burst of 4 CCRs (DBL=3, FIFO+INCR4) covers CH1/CH2/CH3 simultaneously (CH4 column always 0, never CC4E-enabled — pure padding to reach a DMA-supported burst size). TIM4 via `TIM4_UP→DMAR`, DBL=0 (single CCR, CH2 only).
- **RX — GCR telemetry:** TIM1's dedicated IC stream rotates across CH1/CH2/CH3 (~133 Hz each motor, via the CC2 cross-capture trick — CC2S selects TI2 direct for motor 0 or TI1 cross for motor 1, so only 2 distinct DMAMUX capture IDs cover 3 motors). TIM4 captures every frame (400 Hz, no rotation needed for a single motor).

### Drone3 / Orqa QuadCore H7 (`BPRL_BOARD_ORQA`)

This board's main "ESC" JST-GH connector (pads MOT1-4, where a standard 4-in-1 ESC plugs in) is wired to only **two** timers via their CH1/CH2 pair — TIM4 (MOT1/MOT2) and TIM2 (MOT3/MOT4) — not the four separate timers ArduPilot's `hwdef.dat` flags `BIDIR` (those are one channel per timer across FOUR timers, spanning both the ESC connector and a separate secondary connector — the wrong physical pads for a standard 4-in-1 ESC on the main connector).

| Motor (logical) | Pin | Timer / Channel | MOT pad |
|---|---|---|---|
| 0 | PD12 | TIM4 CH1 | MOT1 |
| 1 | PD13 | TIM4 CH2 | MOT2 |
| 2 | PA1  | TIM2 CH2 | MOT3 |
| 3 | PA0  | TIM2 CH1 | MOT4 |

- **TX** (burst DMA, 400 Hz): one DMAR burst-of-4 stream per timer — same DBL=3/FIFO+INCR4 mechanism as the Cube boards' TIM1 above, just with CH1/CH2 as the 2 real motors and CH3/CH4 as dummy zero padding (two independent DMA streams both listening on the same `TIMx_UP` request line was tried first and doesn't work — DMAMUX delivers each hardware request to only one subscribed stream at a time, round-robin, not a broadcast, so each stream saw only every-other update event and the ESC never received a complete frame).
- **RX — GCR telemetry:** each timer's two motors take turns — one dedicated IC stream per timer, alternating slot 0 (CH1)/slot 1 (CH2) every DShot cycle. No cross-capture trick needed (unlike the Cube boards' TIM1) since both channels here have their own direct DMAMUX capture request IDs.

Both boards' `dshot_write(throttle[4])` takes throttle values in **physical DShot lane order** — the mixer/telemetry/test-command layers translate to/from *logical* `[FR,RL,FL,RR]` order via `MotorMixerConfig::motor_map` before/after calling into this driver; see the root README's [MotorMixer](../../README.md#motormixer) section.

### Common bit timing / GCR decode

- Bit period: 1.67 µs (DShot 600); each bit is encoded as a high pulse of 625 ns (0) or 1250 ns (1)
- GCR decode: 21-bit transition-marking → 4×5-bit symbols → 16-bit frame → eRPM = 60,000,000 / period_µs

### DShot value mapping

| `val` | DShot command | Effect |
|---|---|---|
| 0 | 0 | Disarm / stop |
| 1 | 48 | Minimum throttle |
| 1000 | 2047 | Full throttle |

---

## IOMCU — Cube carrier boards' MAIN 1-4 (`IOMCU.hpp/.cpp`, `MOTOR_PROTO_IOMCU`)

Every Cube/Pixhawk-family carrier board has a **second MCU** (a factory-present, separate STM32F1-class "IO co-processor") that drives the **MAIN 1-8** connector — electrically independent of the FMU's own AUX outputs (`DShot`/`PWM` above, TIM1/TIM4). This driver talks to that already-running chip over a UART; it does **not** implement or flash any firmware onto it — whatever's already on the IO co-processor (from its last ArduPilot session, or the factory) is what this driver talks to.

**Why this exists**: CubeBlueH7's AUX outputs were bench-confirmed unreliable — only one of four channels ever connects, and even that one can't be throttle-controlled — and the same failure was independently reproduced under **stock ArduPilot firmware** on the same hardware, ruling out this project's firmware as the cause. ArduPilot's own MAIN 1-4 test on the same board worked correctly. `MOTOR_PROTOCOL == MOTOR_PROTO_IOMCU` (set in `configs/Drone2/config.mk`) is the result.

**Scope is deliberately minimal** — PWM output only. No RC passthrough, no failsafe mixing, no DShot-via-IOMCU, no telemetry/status readback, no safety-switch state machine, no firmware CRC-check/reflash. ArduPilot's own `AP_IOMCU`/`iofirmware` implement all of that; none of it is needed here, and pulling it in would multiply the risk surface for no benefit on this board. `MOTOR_PROTO_PWM` (AUX, unchanged) remains the fallback if AUX is ever fixed at the hardware level or a different unit is used.

**Interface**: USART6, PC6=TX/PC7=RX, AF7 — reserved for exactly this purpose by comments already in this codebase before this driver existed (`cfg/mcuconf.h`'s `STM32_SERIAL_USE_USART6` and `src/coms/SBUS.cpp` both call it out as "FMU<->IOMCU bridge; not used by firmware"). 1,500,000 baud, 8N1.

**Protocol**: register-based request/response packets (not copied from ArduPilot's source — reimplemented from its wire format). `byte0 = count(6 bits) | code(2 bits)<<6`, `byte1 = crc`, `byte2 = page`, `byte3 = offset`, then `count` little-endian `uint16` registers (max 22, so max packet 48 bytes). CRC-8, poly 0x07, init 0, no reflection, no final XOR, computed with the `crc` byte itself zeroed. A write's reply is always a fixed 4-byte ACK. Reads use a compact 4-byte request (CRC over just those 4 bytes) — the modern IO firmware accepts this.

**Two silent-failure gates that are easy to under-scope** (both are one-time writes in `iomcu_init()`, `PAGE_SETUP`=50):
- `CHANNEL_MASK` (offset 27) defaults to **0** on the IO's own boot — miss this and every channel stays at zero output forever, with every packet still ACKing `CODE_SUCCESS`. No error, ever.
- Safety: either `FORCE_SAFETY_OFF` (offset 12, magic value `22027`) or `IGNORE_SAFETY` (offset 20, channel bitmask) — without one of these the IO silently zeroes PWM output regardless of what's written to `PAGE_DIRECT_PWM`, gated on the (possibly-never-pressed) hardware safety button.

**200 ms IO-side watchdog**: if no `PAGE_DIRECT_PWM` write arrives for 200 ms, the IO zeroes every output itself. `IOMCUThread` (`src/threads.cpp`) must keep sending well above 5 Hz for as long as motor output is wanted — it targets ~400 Hz, bounded in practice by the actual UART round-trip.

**Why `motor_output_write()` doesn't touch the UART directly**: a write+ACK transaction is a blocking round-trip (bounded by a 10 ms timeout in the worst case). `ControlThread` runs the 400 Hz control loop and must never block on that — injecting UART jitter into attitude control is not acceptable. So `motor_output_write()` (`PWM.cpp`'s `MOTOR_PROTO_IOMCU` branch) only calls `iomcu_set_pwm()` — an O(1) mutex lock + memcpy + unlock, no UART access at all. `IOMCUThread` is a **separate, dedicated thread** that owns `SD6` exclusively, reads the published values, and performs the actual send/ACK cycle at its own pace — decoupling control-loop timing from IO-comms timing entirely. This mirrors how `EncoderRPM.hpp`/`StrainGauge.hpp` publish sensor data into a `*Raw` struct for other threads to read, just in the write direction.

**Bench bring-up order**: `IOMCU,status` (USB command) reads `PAGE_CONFIG` and should report `protocol_version=4,protocol_version2=10` — if this fails, the UART/pin/CRC plumbing itself is wrong and nothing past this point will work either; don't move on to motor testing until it passes.

**Output-channel mapping**: `g_iomcu_cmd.pwm_us[i]` drives MAIN `(i+1)` — a fresh assignment (MAIN has no pre-existing physical-lane convention the way AUX's `DShot.cpp` pin table does). If a motor ends up on the wrong physical corner, that's a `MotorMixerConfig::motor_map` fix in the drone config, not a change here — same as the existing AUX/DShot output paths.

---

## SPI (`SPI.hpp/.cpp`)

Two SPI buses drive the on-board IMUs plus (on the Cube boards) the barometer. **The IMU chip set — and on Drone3, the IMU count and whether there's an SPI barometer at all — is board-conditional** — `DRONE=Drone1`/`DRONE=Drone2`/`DRONE=Drone3` select `-DBPRL_BOARD_CUBEORANGEPLUS`/`-DBPRL_BOARD_CUBEBLUE`/`-DBPRL_BOARD_ORQA` via `configs/<Drone>/config.mk`, and `SPI.hpp`/`SPI.cpp` `#if` on that to pick both the driver classes and the CS pins/SPI mode:

| Bus | Peripheral | CS pin | `DRONE=Drone1` device | `DRONE=Drone2` device | `DRONE=Drone3` device | Role |
|---|---|---|---|---|---|---|
| SPI1 | SPID1 | PG1 (Drone1) / PC2 (Drone2) / PA4 (Drone3) | ICM-45686 | ICM-20649 | ICM-42688 | Primary IMU (`imu1`) |
| SPI4 | SPID4 | PC15 (Drone1) / PE4 (Drone2) / PE11 (Drone3) | ICM-45686 | ICM-20948 | ICM-42688 | External IMU (`imu2`) |
| SPI4 | SPID4 | PC13 | ICM-45686 | ICM-20602 | *(not present)* | Backup IMU (`imu3`) |
| SPI1 | SPID1 | PD7 | MS5611 | MS5611 (same pin on the Cube boards) | *(no SPI baro — see I2C below)* | Barometer (`baro1`) |

**`DRONE=Drone1` (CubeOrangePlus, default):** all three IMU instances are driven by the **same** `ICM45686` class (`src/coms/IMUs/ICM45686.hpp/.cpp`) — confirmed all three slots on this board are physically populated with ICM-45686 (each passes its WHOAMI check on init). Other CubeOrangePlus hardware revisions instead populate the SPI4 slots (`imu2`/`imu3`) with ICM-42688 — the separate `ICM42688.hpp/.cpp` class exists in the tree to support that variant, not instantiated on this board.

**`DRONE=Drone2` (CubeBlueH7):** `imu1` uses `ICM20649.hpp/.cpp`, `imu2` uses `ICM20948.hpp/.cpp`, `imu3` uses `ICM20602.hpp/.cpp` — all three are classic MPU-9250-family parts with a register-based DLPF instead of the 45686/42688's analog AAF stage, no FIFO/oversampling, and SPI **MODE3** (CPOL=1, CPHA=1) rather than MODE0 (confirmed against ArduPilot's `hwdef.dat` for this chip pairing on Cube-family hardware). CS pins here (PC2/PE4/PC13) come from `boards/CubeBlueH7/board.h`'s documented pin map and are now WHOAMI-confirmed against two physical units. `imu1` was originally assumed to be an ICM-20948 like `imu2` — it's actually an ICM-20649 (WHOAMI 0xE1, not 0xEA), a pin/protocol-compatible "high-g" sibling that needs its own driver class since it takes a different `GYRO_CFG1` FS_SEL encoding and accelerometer scale factor for the same nominal settings; see the root README's [ICM-20649 / ICM-20948 / ICM-20602](../../README.md#8-imu-drivers) section for the full story. A wrong CS/chip pairing still fails safe (each chip class has a distinct WHOAMI, so a mismatch just leaves that `g_imu[i].valid` false). The per-IMU axis-rotation math in `SPIThread` (`src/threads.cpp`) is bench-derived (an earlier CubeOrange-`hwdef.inc` transcription was tested and found wrong — this board's IMU mounting isn't the same as CubeOrange's) — see the root README's [ICM-20649 / ICM-20948 / ICM-20602](../../README.md#8-imu-drivers) section for the derivation; re-verify against `$TEL`/`$IMU` telemetry before flying on it.

**`DRONE=Drone3` (Orqa QuadCore H7):** only **two** on-board IMUs, both `ICM42688.hpp/.cpp` (`imu1`/`imu2`) — `g_imu[2]` is never written and stays `valid=false` (`StateManager` already treats an invalid lane as absent). No SPI barometer on this board at all — its DPS310 is I2C-only, polled from `I2CThread` instead (see the I2C section below). See the root README's [ICM-42688](../../README.md#8-imu-drivers) section for axis-rotation details.

`spi_drv_init()` must run inside `SPIThread` (power-on/reset sequences use `chThdSleepMilliseconds`). `imu2`/`imu3` share SPI4, and `imu1`/`baro1` share SPI1, each with `spiAcquireBus`/`spiReleaseBus` for mutual exclusion.

SPI clock: ~781 kHz for init, 6.25–12.5 MHz for burst reads/conversions (per-device divider, across the Cube boards). On `DRONE=Drone1` the ICM-45686 runs at ~1.6kHz fast-sampling ODR (2x its 800Hz base rate) and drains/averages every FIFO packet queued since the last `SPIThread` tick (capped at 8) — a real oversample-then-decimate step, not just a faster poll. This rate is a deliberate middle ground between the 800Hz base rate (no oversampling) and ArduPilot's own 3.2kHz default for this chip — noise reduction scales with `1/√N` averaged samples, so the first doubling captures roughly the first ~21% of the available reduction for about half the `SPIThread` cost of going all the way to 3.2kHz (measured ~9% vs. ~20% utilization of `SPIThread`'s 1kHz budget). `DRONE=Drone2`'s ICM-20649/20948/20602 have no equivalent FIFO-averaging step — one sample per `SPIThread` tick. It uses 32-byte aligned DMA buffers with `cacheBufferFlush`/`cacheBufferInvalidate` for H7 D-cache coherency; the MS5611 driver is register/command based rather than FIFO-burst, since each pressure/temperature conversion takes multiple milliseconds — `MS5611::read()` is a small state machine called once per `SPIThread` tick, returning a completed sample roughly every 6 ticks. See `src/coms/Baro/MS5611.hpp` and the root README's [MS5611 Barometer](../../README.md#ms5611-barometer) section for the fusion/mocap-priority behavior.

---

## I2C — I2C2 (`I2C.hpp/.cpp`)

**Pins:** PB10 (SCL) / PB11 (SDA), AF4, 400 kHz Fast Mode. `I2CThread` polls all registered devices at 200 Hz via `i2c_poll_all()`.

**Bus recovery:** `i2c_drv_init()` bit-bangs up to 9 SCL clocks (plus a STOP condition) as plain GPIO before starting `I2CD2` — this unsticks a slave left holding SDA low mid-transaction (e.g. after a reset during a live transfer), which otherwise leaves the peripheral seeing `BUSY` forever with SCL never toggling. `i2c_drv_reset()` runs the same recovery sequence and restarts `I2CD2` after a timeout-induced locked state; `STM32_I2C_DMA_ERROR_HOOK` is non-fatal (`cfg/mcuconf.h`) so a DMA error lets the 5 ms software timeout expire and `i2c_drv_reset()` recover cleanly instead of halting the system.

Current use: strain rate sensor's I2C interface (`STRAIN_RATE_INTERFACE=STRAIN_RATE_I2C`, the **default** — CAN is the override; see `src/sensors/StrainRate.*`). No magnetometer is present. On the Cube boards the barometer (MS5611) is on SPI, not I2C — see the SPI section above. On `DRONE=Drone3` (Orqa QuadCore H7) it's the opposite: the DPS310 barometer (`src/coms/Baro/DPS310.hpp/.cpp`, I2C2 address 0x77) is this board's only barometer, registered via `bprl_i2c_register()` the same way the strain-rate sensor is and polled by `I2CThread` at the same 200 Hz — see the root README's [DPS310 Barometer](../../README.md#8-imu-drivers) section.

### Adding a device

```cpp
// in main.cpp, before threads_start():
bprl_i2c_register(MY_ADDR, my_poll, nullptr);
```

---

## PWM / Radio (`Radio.hpp/.cpp`)

`radio_thr()`, `radio_roll()`, `radio_pitch()`, `radio_yaw()`, `radio_flight_mode()`, and `radio_indi()` return normalized RC channel values (`[0,1]` or `[-1,1]`, see `Radio.hpp`); channel indices come from `DroneConfig::rc_map` (see `configs/DroneConfig.hpp`) rather than being hardcoded. `radio_armed()` reads the dedicated arm-switch channel (threshold at center). `radio_switch_position()` buckets `radio_indi()` into 0/1/2 (low/mid/high) — a thresholded int built on top of the raw accessor, same pattern as `radio_flight_mode()`'s raw value vs. `FlightStateMachine`'s thresholded mode selection; `FlightStateMachine::set_active_controller()` maps that to a controller-list index.

Both `SBUS.hpp/.cpp` and `CRSF.hpp/.cpp` receiver protocol drivers exist and are compiled; the active one is selected at compile time via `RADIO_PROTOCOL` in `Radio.hpp` (default: CRSF).

`motor_output_write()` selects DShot 600 or standard servo PWM (1000–2000 µs) based on the `MOTOR_PROTOCOL` define in `PWM.hpp`. The MotorMixer and ControlThread always produce 0–1000 normalized commands and are unaffected by this choice.

---

## MAVLink — TELEM2 (`MAVLink.hpp/.cpp`)

Parses MAVLink on the TELEM2 UART. Currently used solely to ingest motion-capture position/velocity into `g_mocap`:

| Message | Writes | Flag set |
|---|---|---|
| `VISION_POSITION_ESTIMATE` | `g_mocap.{x,y,z}` | `has_new_pos` |
| `VISION_SPEED_ESTIMATE` | `g_mocap.{vx,vy,vz}` | `has_new_vel` |

The two are handled independently (separate MAVLink messages, not guaranteed to arrive together or at the same rate) — see `StateManager::update()`'s comment on why position and velocity mocap fusion are gated separately rather than combined into one call.

---

## CalFlash (`CalFlash.hpp/.cpp`)

Persistent IMU calibration storage — per-IMU gyro and accelerometer bias, written to STM32H743 internal flash Bank2 sector 7 (`0x081E0000`) via a direct register-level flash driver (bypasses ChibiOS's `HAL_USE_EFL`). Validated with a magic number, version field, and CRC32 on load; `cal_load()`/`cal_save()`/`cal_clear()` are the public entry points. `SPIThread` loads this at boot (before IMU init) and falls back to zero biases if the stored data is absent or fails validation.
