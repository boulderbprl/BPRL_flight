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
| `telemetry.py` | `telemetry`, `ekf-status`, `imu-compare` | Required |
| `motor_test.py` | `motor-test` | No |
| `calibrate.py` | `calibrate` | Required |
| `can_tools.py` | `can-status`, `can-scan` | No |
| `strain_rate.py` | `strain-rate` | No |
| `dshot_tools.py` | `dshot-diag` | No |
| `logs.py` | `logs list/download/decode/erase`, `log-status` | No |
| `flash_upload.py` | *(positional firmware path)* | — |

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

```bash
python3 tools/telemetry.py telemetry
python3 tools/telemetry.py ekf-status
python3 tools/telemetry.py imu-compare
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
| `can-scan [--duration N]` | Record all CAN IDs seen for N seconds, display Hz breakdown |

```bash
python3 tools/can_tools.py can-status
python3 tools/can_tools.py can-scan --duration 2
```

`can-scan` marks each ID as registered (in the device table) or unknown. Unknown IDs appearing at high rate indicate a device that needs `bprl_can_register()`.

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

| CSV file | Rate | Contents |
|---|---|---|
| `<stem>_att.csv` | 50 Hz | TimeUS, Roll/Pitch/Yaw (rad), P/Q/R (rad/s), Pdot/Qdot/Rdot (rad/s²) |
| `<stem>_lin.csv` | 50 Hz | TimeUS, X/Y/Z position (m NED), U/V/W velocity (m/s body), Udot/Vdot/Wdot accel (m/s²) |
| `<stem>_rcin.csv` | 50 Hz | TimeUS, RollStk/PitchStk/YawStk/ThrStk (normalized), Armed |
| `<stem>_outp.csv` | 50 Hz | TimeUS, RollTq/PitchTq/YawTq (normalized torque [-1,1]), Thr |
| `<stem>_rpms.csv` | 50 Hz | TimeUS, RPM0–RPM3 (mechanical RPM, int32) |
| `<stem>_strn.csv` | 50 Hz | TimeUS, S0–S3 (int16 strain-rate), Valid |

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
# Flash
make flash BOARD=CubeBlueH7

# Debug build + flash
make BOARD=CubeBlueH7 UDEFS_EXTRA=-DBPRL_DEBUG && make flash BOARD=CubeBlueH7

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
