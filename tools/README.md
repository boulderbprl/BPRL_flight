# BPRL Tools

Ground-station utilities for the BPRL flight controller.

| Script | Purpose | Build required |
|---|---|---|
| [flash_upload.py](#flash_uploadpy) | Flash firmware to the Cube over USB | — |
| [bprl.py](#bprlpy) | Motor test, telemetry, SD log access | any (telemetry: DEBUG) |

---

## Dependencies

```bash
pip install pyserial rich
```

`pyserial` is required by both scripts. `rich` is required only by `bprl.py`.

---

## flash_upload.py

Uploads a compiled `.bin` firmware image to a CubeBlue H7 or CubeOrange+ using the PX4/ArduPilot ChibiOS bootloader protocol over USB.

### How it works

The uploader speaks to the ChibiOS bootloader that lives in protected flash. When you connect the Cube over USB it appears as `/dev/ttyACMx`. If the board is already running flight firmware the script sends a MAVLink `COMMAND_LONG` reboot-to-bootloader command automatically — you do not need to hold a button or power-cycle.

Sequence: **detect port → send reboot request if needed → erase flash → program in 252-byte chunks → CRC-32 verify → reboot into firmware**.

### Usage

**Recommended: use the Makefile targets from the project root.**

```bash
# Build and flash CubeBlue H7 (auto-detects /dev/ttyACM*)
make flash BOARD=CubeBlueH7

# Specify port explicitly
make flash BOARD=CubeBlueH7 PORT=/dev/ttyACM0

# CubeOrange+
make flash BOARD=CubeOrangePlus PORT=/dev/ttyACM0

# Debug build (enables USB telemetry + extra output)
make BOARD=CubeBlueH7 UDEFS_EXTRA=-DBPRL_DEBUG
make flash BOARD=CubeBlueH7 PORT=/dev/ttyACM0
```

**Or call the script directly:**

```bash
python3 tools/flash_upload.py build/BPRL.bin
python3 tools/flash_upload.py --port /dev/ttyACM0 build/BPRL.bin
python3 tools/flash_upload.py --port /dev/ttyACM0 --baud 115200 build/BPRL.bin
```

### Options

| Flag | Default | Description |
|---|---|---|
| `firmware` | *(required)* | Path to the `.bin` file, e.g. `build/BPRL.bin` |
| `--port` | auto-detect | Serial port. Searches `/dev/serial/by-id/usb-*Cube*` then `/dev/ttyACM*` |
| `--baud` | 115200 | Bootloader baud rate (rarely needs changing) |

### Port detection order

When `--port` is omitted the script scans in this order and uses the first match:

1. `/dev/serial/by-id/usb-*Hex*`
2. `/dev/serial/by-id/usb-*Cube*`
3. `/dev/serial/by-id/usb-*ArduPilot*`
4. `/dev/ttyACM*` (fallback)

### Troubleshooting

**"Waiting for board…" hangs indefinitely**

- Unplug and re-plug the USB cable. The script retries as soon as it sees the port appear.
- On Linux, check whether ModemManager is interfering:
  ```bash
  sudo systemctl stop ModemManager
  ```
  The script will warn you if ModemManager is installed.
- Confirm the port exists: `ls /dev/ttyACM*`

**"firmware exceeds flash" error**

The `.bin` is larger than the 2 MB application flash. This should not happen in a normal build — check `build/BPRL.map` for unexpectedly large sections.

**Upload succeeds but board does not boot**

Verify the correct `BOARD=` was passed. Flashing a CubeBlue H7 build onto a CubeOrange+ (or vice-versa) will produce a board that boots but immediately faults because the clock and peripheral register layouts differ.

**ST-Link alternative**

If the USB bootloader is broken or unavailable:
```bash
make flash-stlink BOARD=CubeBlueH7
```
This uses OpenOCD with an ST-Link probe connected to the debug connector.

---

## bprl.py

A unified ground tool for interacting with the flight controller over USB CDC (`/dev/ttyACMx`, VID:PID `0483:5740`). It auto-detects the port.

> **Motor test and log commands work on any firmware build.**
> The `telemetry` subcommand requires a debug build (`-DBPRL_DEBUG`).

```bash
python3 tools/bprl.py [--port /dev/ttyACMx] <subcommand>
```

### Global options

| Flag | Default | Description |
|---|---|---|
| `--port` | auto-detect | Override the serial port |
| `--baud` | 115200 | Baud rate (CDC ignores this, but pyserial requires it) |

---

### `telemetry` — Live sensor dashboard

> Requires a firmware built with `-DBPRL_DEBUG`.

```bash
python3 tools/bprl.py telemetry
```

Connects to the drone and displays a live dashboard updating at ~10 Hz. The firmware streams a `$TEL` CSV line each tick; this command parses those lines and renders them in the terminal.

**What is displayed:**

| Panel | Fields |
|---|---|
| Attitude | Roll, Pitch, Yaw in degrees |
| Body rates | P, Q, R in rad/s (from EKF state) |
| RC inputs | Throttle [0–1], Roll/Pitch/Yaw targets [−1–1] |
| IMU status | Valid/absent flag for each of the 3 on-board IMUs |
| CAN INS (IMX5) | Connected flag + measured quaternion rate (expected 200 Hz) and body-rate message rate (expected 100 Hz) |
| Motor RPM | Mechanical RPM for each motor (eRPM ÷ 14 pole pairs); green ● = valid GCR frame received |

**Reading the IMU status panel:**

- `ICM-20948 pri` — primary SPI1 IMU (always expected valid when soldered)
- `ICM-20948 ext` — external SPI4 IMU
- `ICM-20602` — backup SPI4 IMU
- `CAN INS (IMX5)` — external Inertial Sense IMX5 over FDCAN1. The Hz values show measured incoming frame rates. If `quat=0 Hz` the IMX5 is not sending; the EKF falls back to on-board gyro/accel only.

Press `Ctrl-C` to exit.

---

### `motor-test` — Interactive motor test

> Works on any firmware build. Props must be removed first.

```bash
python3 tools/bprl.py motor-test
```

Sends motor test commands to the firmware and displays live RPM feedback. While motor test is active the firmware's ControlThread bypasses the PID controller and drives the ESC directly at the commanded DShot value.

**Motor layout (top view):**

```
    FL [2]       FR [0]
         \       /
         [  body  ]
         /       \
    RL [1]       RR [3]
```

**Commands at the `>` prompt:**

| Command | Effect |
|---|---|
| `motor <0-3> <0-100>` | Spin one motor at the given throttle %. All others are stopped. |
| `stop` or `s` | Stop all motors and exit test mode. |
| `quit` or `q` | Stop all motors and exit the script. |

**Examples:**

```
> motor 0 15      # spin FR motor at 15% throttle
> motor 2 25      # spin FL motor at 25% throttle
> stop            # stop all
```

**Safety rules enforced by the firmware:**

- The drone must be **disarmed** (arm switch in disarm position). If armed, the command is rejected with `MT,ERR,armed`.
- Only one motor runs at a time. Each `motor` command zeroes the other three ESC outputs first.
- `quit` sends a `MT,stop` command before exiting so motors always stop even if the terminal is closed abruptly.

**RPM display:**

RPM values come from the bidirectional DShot GCR telemetry built into `src/coms/DShot.cpp`. The ESCs return electrical RPM; the display converts to mechanical RPM using 14 pole pairs. A `●` means at least one valid GCR frame has been decoded for that motor since it started spinning. `○` means no valid frame yet (motor may be spinning but telemetry not decoded — check ESC BLHeli_32 bidirectional DShot setting).

> If no `$TEL` lines are arriving (non-DEBUG build) the RPM column shows `---`. Motor test still works — you just can't see RPM live without a DEBUG build.

---

### `logs list` — List SD card log files

> Works on any firmware build. SD card must be inserted.

```bash
python3 tools/bprl.py logs list
```

Requests a directory listing of `/LOGS/` on the SD card and prints a table with filename and size. The highest-numbered file is the currently-open log (being written now); all others are completed.

**Example output:**

```
         SD Card Log Files
┌─────────────┬────────────────────┐
│ Filename    │ Size               │
├─────────────┼────────────────────┤
│ LOG0001.BIN │   1.2 MB  (1,258,496 B) │
│ LOG0042.BIN │   4.7 MB  (4,927,488 B) │
└─────────────┴────────────────────┘
  Latest: LOG0042.BIN
```

---

### `logs download` — Download a log file

> Works on any firmware build.

```bash
# Download the latest completed log (second-to-highest-numbered file)
python3 tools/bprl.py logs download

# Download a specific file
python3 tools/bprl.py logs download LOG0001.BIN
```

Saves the file to the current directory with a progress bar. Transfer speed depends on USB CDC throughput (typically 200–400 KB/s for full-speed USB).

**Protocol detail:** The firmware sends `LOG,SIZE,<bytes>` then immediately streams raw binary. The script preserves any bytes already read past the size line so no data is lost at the start of the binary stream.

The default (no filename given) downloads the **second-to-latest** log file, i.e., the most recent completed flight. The highest-numbered file is still being written and may be incomplete; use `logs download LOG<NNNN>.BIN` to explicitly request it if needed.

---

### `logs erase` — Erase completed log files

> Works on any firmware build.

```bash
python3 tools/bprl.py logs erase
```

Asks for confirmation, then deletes all log files in `/LOGS/` **except the currently-open one** (the firmware's active log for this boot). Useful before a flight to clear old data and free SD card space.

```
Erase all completed log files from SD card? [y/N]: y
Erased 41 file(s).
```

---

### `logs decode` — Decode a binary log to CSV

> Offline — no drone or serial port needed.

```bash
python3 tools/bprl.py logs decode LOG0042.BIN
```

Parses the binary log format (`[0xA3][0x95][msg_id][packed struct]` records) and writes two CSV files alongside the source file:

| Output file | Contents | Rate |
|---|---|---|
| `LOG0042_imu.csv` | Raw accel/gyro from 3 on-board IMUs + CAN IMU quaternion and body rates | 100 Hz |
| `LOG0042_state.csv` | Fused flight state: roll, pitch, yaw, p/q/r, Z position/velocity/accel, throttle, arm status | 50 Hz |

**IMU CSV columns:**

```
TimeMS, AX0,AY0,AZ0,GX0,GY0,GZ0,V0,  ← ICM-20948 primary  (m/s², rad/s, valid)
        AX1,AY1,AZ1,GX1,GY1,GZ1,V1,  ← ICM-20948 external
        AX2,AY2,AZ2,GX2,GY2,GZ2,V2,  ← ICM-20602
        QW,QX,QY,QZ,CanP,CanQ,CanR,CV ← IMX5 CAN INS (quaternion + body rates)
```

**State CSV columns:**

```
TimeMS, Roll, Pitch, Yaw,   ← rad (EKF output via StateManager)
        P, Q, R,            ← rad/s
        ZPos, ZVel, ZAcc,   ← m, m/s, m/s² (NED down axis)
        Thr,                ← [0, 1]
        Armed               ← 0 or 1
```

Time is in milliseconds since boot (`chVTGetSystemTime` converted to ms).

---

## Quick-reference card

```bash
# Flash firmware
make flash BOARD=CubeBlueH7

# Flash debug build (enables $TEL telemetry stream)
make BOARD=CubeBlueH7 UDEFS_EXTRA=-DBPRL_DEBUG && make flash BOARD=CubeBlueH7

make BOARD=CubeOrangePlus UDEFS_EXTRA=-DBPRL_DEBUG && make flash BOARD=CubeOrangePlus

# Live telemetry dashboard (debug build required)
python3 tools/bprl.py telemetry

# Test FL motor at 20% throttle then stop
python3 tools/bprl.py motor-test
> motor 2 20
> stop

# See what logs are on the SD card
python3 tools/bprl.py logs list

# Download latest completed flight log
python3 tools/bprl.py logs download

# Decode it to CSV
python3 tools/bprl.py logs decode LOG0042.BIN

# Erase old logs before a flight
python3 tools/bprl.py logs erase
```
