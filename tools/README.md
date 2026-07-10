# BPRL Tools

Ground-station utilities for the BPRL flight controller. Connect via USB CDC (`/dev/ttyACMx`, VID:PID `0483:5740`). All scripts auto-detect the port.

## Dependencies

```bash
pip install pyserial rich
```

---

## Script overview

| Script | Subcommands | DEBUG build? |
|---|---|---|
| `telemetry.py` | `telemetry`, `ekf-status`, `imu-compare`, `pos-vel` | Required |
| `motor_test.py` | `motor-test` | No |
| `calibrate.py` | `calibrate` | Required |
| `can_tools.py` | `can-status`, `can-diag`, `can-regdump`, `can-scan` | No |
| `mav_tools.py` | `mav-diag` | No |
| `strain_rate.py` | `strain-rate` | No |
| `dshot_tools.py` | `dshot-diag` | No |
| `i2c_tools.py` | `i2c-scan` | No |
| `logs.py` | `logs list/download/decode/erase`, `log-status` | No |
| `flash_upload.py` | *(positional firmware path)* | — |

`dshot_decode_test.py` is a standalone dev/test script (no argparse subcommands) — not part of the main CLI family above.

All scripts accept `--port /dev/ttyACMx` and `--baud N` global options.

---

## telemetry.py

> Requires `-DBPRL_DEBUG` firmware build.

Parses the `$TEL`, `$EKFL`, and `$IMU` 10 Hz streams emitted by `DebugThread`.

| Subcommand | Description |
|---|---|
| `telemetry` | Live attitude/rate/RPM/IMU/CAN dashboard |
| `ekf-status` | Per-lane EKF roll/pitch/yaw/p/q/r table (all three onboard lanes + IMX5) |
| `imu-compare` | Side-by-side raw accel and gyro from all three onboard IMUs |
| `pos-vel` | Live XYZ position/velocity vs. mocap ground truth |

```bash
python3 tools/telemetry.py telemetry
python3 tools/telemetry.py ekf-status
python3 tools/telemetry.py imu-compare
python3 tools/telemetry.py pos-vel
```

---

## motor_test.py

> Works on any firmware build. Remove props before use.

Interactive motor test with live RPM feedback from bidirectional DShot GCR telemetry.

```bash
python3 tools/motor_test.py motor-test
```

Commands at the `>` prompt:

| Command | Effect |
|---|---|
| `motor <0-3> <0-100>` | Spin one motor at the given throttle %. |
| `stop` | Stop all motors. |
| `quit` | Stop all motors and exit. |

Motor layout (top view): `FL[2]  FR[0]` / `RL[1]  RR[3]`

---

## calibrate.py

> Requires `-DBPRL_DEBUG` firmware build (needs the `$IMU` stream).

Collects static IMU data, computes gyro and accelerometer biases, and writes them to flash.

```bash
python3 tools/calibrate.py calibrate [--duration 30]
```

Place the drone level on a flat surface before running. The script collects `--duration` seconds of `$IMU` samples (default 30 s), prints the computed biases, and asks for confirmation before writing to flash via `CAL,set` and `CAL,commit`.

---

## can_tools.py

> Works on any firmware build.

| Subcommand | Description |
|---|---|
| `can-status` | Read FDCAN1 PSR / ECR / RXF0S registers and decode error flags |
| `can-diag` | `msg_lost` (RxFIFO0 overflow count) and `reinit_count` (Bus_Off recoveries so far) |
| `can-regdump` | Full FDCAN1 register dump |
| `can-scan [--duration N]` | Record all CAN IDs seen for N seconds, display Hz breakdown |

```bash
python3 tools/can_tools.py can-status
python3 tools/can_tools.py can-diag
python3 tools/can_tools.py can-regdump
python3 tools/can_tools.py can-scan --duration 2
```

`can-scan` marks each ID as registered (in the device table) or unknown. Unknown IDs appearing at high rate indicate a device that needs `bprl_can_register()`.

---

## mav_tools.py

> Works on any firmware build.

| Subcommand | Description |
|---|---|
| `mav-diag [--watch] [--interval N]` | Read MAVLinkThread's SD3/TELEM2 RX counters (bytes, frames parsed OK/bad-CRC, per-message-type counts) |

```bash
python3 tools/mav_tools.py mav-diag
python3 tools/mav_tools.py mav-diag --watch --interval 1
```

Diagnoses the RX side of the MAVLink/TELEM2 link independent of the radio and any ground-side bridge — tells you whether bytes are reaching SD3 at all, whether frames are failing CRC (dialect mismatch), and whether `VISION_POSITION_ESTIMATE`/`VISION_SPEED_ESTIMATE` specifically are showing up.

---

## i2c_tools.py

> Works on any firmware build.

| Subcommand | Description |
|---|---|
| `i2c-scan` | Probe all I2C addresses and display a grid of responding devices (also the default with no subcommand) |

```bash
python3 tools/i2c_tools.py i2c-scan
```

---

## strain_rate.py

> Works on any firmware build. In development.

Live display of the strain-rate sensor (CAN ID 0x69): 4 signed int16 values representing strain rate on each arm.

```bash
python3 tools/strain_rate.py strain-rate
```

Polls `STRAIN_RATE,read` at ~5 Hz and shows a live panel. The `valid` flag reflects whether the sensor is actively sending CAN frames.

---

## dshot_tools.py

> Works on any firmware build.

One-shot snapshot of bidirectional DShot telemetry state: DMA TC counts, input-capture ISR counts, and raw edge timestamps for each motor.

```bash
python3 tools/dshot_tools.py dshot-diag
```

Useful for diagnosing DShot issues: no DMA fires → DMA init problem; DMA fires but no ISR → NVIC/DIER problem; ISR fires but no edges → ESC not responding or wiring issue.

---

## logs.py

> Works on any firmware build. SD card must be inserted.

| Subcommand | Description |
|---|---|
| `logs list` | List log files on SD card with sizes |
| `logs download [FILE]` | Download a log file (default: latest completed) |
| `logs download FILE --decode` | Download and immediately decode to CSV |
| `logs decode FILE.bin` | Offline: decode a binary log to CSV files |
| `logs erase` | Erase all completed log files |
| `log-status` | Query SD card logger status (ready / error code) |

```bash
python3 tools/logs.py logs list
python3 tools/logs.py logs download
python3 tools/logs.py logs download LOG0042.BIN
python3 tools/logs.py logs decode LOG0042.BIN
python3 tools/logs.py logs erase
python3 tools/logs.py log-status
```

`logs decode` is offline — no serial port needed. It reads the ArduPilot DataFlash schema from the file header (FMT records) and writes one CSV per message type:

| CSV file | Contents |
|---|---|
| `<stem>_att.csv` | TimeUS, Roll/Pitch/Yaw (rad), P/Q/R (rad/s), Pdot/Qdot/Rdot (rad/s²) |
| `<stem>_lin.csv` | TimeUS, X/Y/Z position (m NED), U/V/W velocity (m/s body), Udot/Vdot/Wdot accel (m/s²) |
| `<stem>_rcin.csv` | TimeUS, RollStk/PitchStk/YawStk/ThrStk (normalized), FlightMode (raw switch value), Armed |
| `<stem>_outp.csv` | TimeUS, RollTq/PitchTq/YawTq (normalized torque [-1,1]), Thr |
| `<stem>_rpms.csv` | TimeUS, RPM0–RPM3 (mechanical RPM, int32) |
| `<stem>_strn.csv` | TimeUS, S0–S3 (int16 strain-rate), Valid |
| `<stem>_imu1.csv` / `_imu2.csv` / `_imu3.csv` | TimeUS, AccX/AccY/AccZ (m/s²), GyrX/GyrY/GyrZ (rad/s), Valid — one per on-board IMU |
| `<stem>_indi.csv` | TimeUS, UnmixR/UnmixP (N·m), DeltaR/DeltaP (N·m), CmdR/CmdP (normalized), AccR/AccP (rad/s² INDI-commanded accel) |
| `<stem>_baro.csv` | TimeUS, Press (Pa), Temp (°C), Alt (m, positive up), Valid |

All 11 message types log at the fixed 50 Hz `LogThread` period — there's no per-record rate field.

The `.bin` files are also compatible with [UAV Log Viewer](https://plot.ardupilot.org) — open the file directly in the browser for interactive plots.

---

## flash_upload.py

Uploads a compiled `.bin` firmware image to a CubeBlue H7 or CubeOrange+ using the ChibiOS bootloader protocol over USB.

```bash
# Recommended: use Makefile targets
make flash BOARD=CubeBlueH7
make flash BOARD=CubeOrangePlus PORT=/dev/ttyACM0

# Or directly
python3 tools/flash_upload.py build/BPRL.bin
python3 tools/flash_upload.py --port /dev/ttyACM0 build/BPRL.bin
```

If the board is already running firmware, the script sends a reboot-to-bootloader command automatically. Sequence: detect port → reboot if needed → erase → program in 252-byte chunks → CRC-32 verify → reboot.

---

## Quick reference

```bash
# Flash (default board: BOARD=CubeBlueH7)
make flash BOARD=CubeOrangePlus

# Debug build + flash
make BOARD=CubeOrangePlus UDEFS_EXTRA=-DBPRL_DEBUG && make flash BOARD=CubeOrangePlus

# Telemetry (debug build required)
python3 tools/telemetry.py telemetry

# Motor test
python3 tools/motor_test.py motor-test

# Logs
python3 tools/logs.py logs list
python3 tools/logs.py logs download
python3 tools/logs.py logs decode LOG0042.BIN

# CAN diagnostics
python3 tools/can_tools.py can-scan --duration 2

# IMU calibration (debug build required)
python3 tools/calibrate.py calibrate
```
