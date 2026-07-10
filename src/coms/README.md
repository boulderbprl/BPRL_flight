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
| `0x69` | Strain rate sensor | 4 signed int16 values, one per arm (FR/RL/FL/RR) | 100 Hz — this is the **default** interface (`STRAIN_RATE_INTERFACE=STRAIN_RATE_CAN`, see `src/sensors/StrainRate.*`); I2C is the override |

### Adding a device

```cpp
// in main.cpp, before threads_start():
bprl_can_register(0x10, my_callback, nullptr);
```

---

## DShot — Bidirectional DShot 600 (`PWM.hpp/.cpp`)

Motor output is selected at compile time via `MOTOR_PROTOCOL` in `PWM.hpp`:

```cpp
#define MOTOR_PROTO_DSHOT  0   // bidirectional DShot 600 (default)
#define MOTOR_PROTO_PWM    1   // standard servo PWM
```

`motor_output_write(val[4])` accepts normalized commands: **0 = disarm**, **1–1000 = 0.1–100% throttle**.

### Motor pin mapping

| Motor | Position | Pin | Timer / Channel |
|---|---|---|---|
| 0 | FR | PE11 | TIM1 CH2 |
| 1 | RL | PE9  | TIM1 CH1 |
| 2 | FL | PD13 | TIM4 CH2 |
| 3 | RR | PE13 | TIM1 CH3 |

### TX (burst DMA, 400 Hz)

- TIM1: burst DMA via `TIM1_UP→DMAR`, DCR burst of 4 CCRs covers CH1/CH2/CH3 simultaneously
- TIM4: burst DMA via `TIM4_UP→DMAR`, DBL=0 (single CCR, CH2 only)
- Bit period: 1.67 µs (DShot 600); each bit is encoded as a high pulse of 625 ns (0) or 1250 ns (1)

### RX — GCR telemetry (bidirectional)

Each ESC returns an eRPM frame on the same wire after the TX burst ends:

- TIM1: input-capture DMA rotates across CH1/CH2/CH3 at ~133 Hz each motor
- TIM4: input-capture DMA captures TIM4/CH2 every frame (400 Hz)
- GCR decode: 21-bit transition-marking → 4×5-bit symbols → 16-bit frame → eRPM = 60,000,000 / period_µs

### DShot value mapping

| `val` | DShot command | Effect |
|---|---|---|
| 0 | 0 | Disarm / stop |
| 1 | 48 | Minimum throttle |
| 1000 | 2047 | Full throttle |

---

## SPI (`SPI.hpp/.cpp`)

Two SPI buses drive the three on-board IMUs plus the barometer.

| Bus | Peripheral | CS pin | Device | Role |
|---|---|---|---|---|
| SPI1 | SPID1 | PG1  | ICM-45686 | Primary IMU (`imu1`) |
| SPI4 | SPID4 | PC15 | ICM-42688 (probed/read as ICM-45686) | External IMU (`imu2`) |
| SPI4 | SPID4 | PC13 | ICM-42688 (probed/read as ICM-45686) | Backup IMU (`imu3`) |
| SPI1 | SPID1 | PD7  | MS5611 | Barometer (`baro1`) |

All three IMU instances are driven by the **same** `ICM45686` class (`src/coms/IMUs/ICM45686.hpp/.cpp`) — `imu2`/`imu3` are physically ICM-42688 chips, but that driver reads them too because the two parts share a compatible WHOAMI/register/FIFO layout at the settings used here. The separate `ICM42688.hpp/.cpp` class exists in the tree but is never instantiated (dead code), as are the older `ICM20948.hpp/.cpp`/`ICM20602.hpp/.cpp` classes from an earlier hardware revision.

`spi_drv_init()` must run inside `SPIThread` (power-on/reset sequences use `chThdSleepMilliseconds`). `imu2`/`imu3` share SPI4, and `imu1`/`baro1` share SPI1, each with `spiAcquireBus`/`spiReleaseBus` for mutual exclusion.

SPI clock: ~781 kHz for init, 6.25–12.5 MHz for burst reads/conversions (per-device divider). The IMU driver uses 32-byte aligned DMA buffers with `cacheBufferFlush`/`cacheBufferInvalidate` for H7 D-cache coherency; the MS5611 driver is register/command based rather than FIFO-burst, since each pressure/temperature conversion takes multiple milliseconds — `MS5611::read()` is a small state machine called once per `SPIThread` tick, returning a completed sample roughly every 6 ticks. See `src/coms/Baro/MS5611.hpp` and the root README's [MS5611 Barometer](../../README.md#ms5611-barometer) section for the fusion/mocap-priority behavior.

---

## I2C — I2C2 (`I2C.hpp/.cpp`)

**Pins:** PB10 (SCL) / PB11 (SDA), AF4, 400 kHz Fast Mode. `I2CThread` polls all registered devices at 500 Hz via `i2c_poll_all()`.

**Bus recovery:** `i2c_drv_init()` bit-bangs up to 9 SCL clocks (plus a STOP condition) as plain GPIO before starting `I2CD2` — this unsticks a slave left holding SDA low mid-transaction (e.g. after a reset during a live transfer), which otherwise leaves the peripheral seeing `BUSY` forever with SCL never toggling. `i2c_drv_reset()` runs the same recovery sequence and restarts `I2CD2` after a timeout-induced locked state; `STM32_I2C_DMA_ERROR_HOOK` is non-fatal (`cfg/mcuconf.h`) so a DMA error lets the 5 ms software timeout expire and `i2c_drv_reset()` recover cleanly instead of halting the system.

Current use: strain rate sensor's I2C **fallback** interface only (`STRAIN_RATE_INTERFACE=STRAIN_RATE_I2C` override — CAN is the default; see `src/sensors/StrainRate.*`). No magnetometer is present. The barometer (MS5611) is on SPI, not I2C — see the SPI section above.

### Adding a device

```cpp
// in main.cpp, before threads_start():
bprl_i2c_register(MY_ADDR, my_poll, nullptr);
```

---

## PWM / Radio (`Radio.hpp/.cpp`)

`radio_thr()`, `radio_roll()`, `radio_pitch()`, `radio_yaw()` return normalized RC channel values. `radio_armed()` reads a dedicated arm-switch channel: `PARSER.channel(4) > 992u` (channel 5, threshold at center) — this used to be a stub returning `false` unconditionally, it is now a real implementation.

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
