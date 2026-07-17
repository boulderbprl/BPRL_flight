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
| *(raw serial, see [Timing](#timing--schedulability-bprl_timing-build) below)* | `TIM,status`, `TIM,reset` | Requires `-DBPRL_TIMING` |

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
| `<stem>_rcin.csv` | TimeUS, RollStk/PitchStk/YawStk/ThrStk (normalized), FlightMode (raw switch value), IndiStk (raw switch value, >0.33=INDI), Armed |
| `<stem>_outp.csv` | TimeUS, RollTq/PitchTq/YawTq (normalized torque [-1,1]), Thr |
| `<stem>_rpms.csv` | TimeUS, RPM0–RPM3 (mechanical RPM, int32) |
| `<stem>_strn.csv` | TimeUS, S0–S3 (int16 strain-rate), Valid |
| `<stem>_imu1.csv` / `_imu2.csv` / `_imu3.csv` | TimeUS, AccX/AccY/AccZ (m/s²), GyrX/GyrY/GyrZ (rad/s), Valid — one per on-board IMU |
| `<stem>_indi.csv` | TimeUS, UnmixR/UnmixP (N·m), DeltaR/DeltaP (N·m), CmdR/CmdP (normalized), AccR/AccP (rad/s² INDI-commanded accel) |
| `<stem>_baro.csv` | TimeUS, Press (Pa), Temp (°C), Alt (m, positive up), Valid |

All 11 message types log at the fixed 50 Hz `LogThread` period — there's no per-record rate field.

The `.bin` files are also compatible with [UAV Log Viewer](https://plot.ardupilot.org) — open the file directly in the browser for interactive plots.

---

## Timing / schedulability (`BPRL_TIMING` build)

> Requires `-DBPRL_TIMING` firmware build. Testing/bench only — disable for flight builds (adds per-tick timestamp reads on every instrumented thread).

Per-thread execution-time and CPU-utilization instrumentation, added to check whether the ChibiOS thread set (SPI, CAN, I2C, StateEst, Control, Radio, Heartbeat, MAVLink, Debug, Log, USBCmd) is actually schedulable at its configured rates and priorities, rather than assuming it. See `src/diagnostics/ThreadTiming.hpp` for the implementation and `threads_start()` in `src/threads.cpp` for the current priority ordering.

Build and flash:

```bash
make BOARD=orange UDEFS_EXTRA=-DBPRL_TIMING
make flash BOARD=orange
# combine with debug telemetry if needed:
make BOARD=blue UDEFS_EXTRA="-DBPRL_DEBUG -DBPRL_TIMING"
```

There's no dedicated `tools/*.py` wrapper yet — query the two commands directly over the USB serial port, e.g. with pyserial's bundled terminal:

```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 115200
# then type: TIM,status        (dump the report)
#            TIM,reset         (zero out min/max/avg/miss counters, start a fresh window)
# Ctrl+] to exit
```

or a short one-off script:

```python
import serial, time
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
time.sleep(0.2)
ser.write(b"TIM,status\r\n")
time.sleep(0.3)
print(ser.read(4096).decode(errors="replace"))
```

Sample output — one `$THD` line per registered thread, plus a `$CPU` totals line. This is a real capture (625 Hz EKF, 1.6 kHz IMU oversampling ODR, 400 Hz control, armed/loaded):

```
$THD,can,period_us=event,exec_avg_us=1,exec_max_us=100,n=39120
$THD,i2c,period_us=2000,exec_avg_us=0,exec_max_us=100,util_pct=0.0,misses=0,n=39119
$THD,est,period_us=1600,exec_avg_us=876,exec_max_us=1400,util_pct=54.7,misses=0,n=48899
$THD,ctrl,period_us=2500,exec_avg_us=7,exec_max_us=100,util_pct=0.3,misses=0,n=31296
$THD,radio,period_us=10000,exec_avg_us=5,exec_max_us=100,util_pct=0.0,misses=0,n=7824
$THD,heartbeat,period_us=200000,exec_avg_us=1,exec_max_us=100,util_pct=0.0,misses=0,n=392
$THD,mavlink,period_us=10000,exec_avg_us=0,exec_max_us=100,util_pct=0.0,misses=0,n=7575
$THD,usbcmd,period_us=event,exec_avg_us=916,exec_max_us=1900,n=6
$THD,log,period_us=20000,exec_avg_us=4352,exec_max_us=43700,util_pct=21.7,misses=153,n=3909
$THD,spi,period_us=1000,exec_avg_us=93,exec_max_us=500,util_pct=9.3,misses=0,n=78167
$CPU,util_pct=86.2,util_pct_hrt=64.3,n_threads=10
```

Reading it:

| Field | Meaning |
|---|---|
| `period_us=event` | An event-driven thread (`can`, `usbcmd`) — exec time is still tracked but excluded from `util_pct`/`util_pct_hrt` since it has no fixed period. |
| `exec_avg_us` / `exec_max_us` | Running average / worst-case time spent in that thread's per-tick work (excludes the sleep-until-next-period call). |
| `util_pct` | `exec_avg_us / period_us`, i.e. this thread's slice of CPU time. |
| `misses` | Ticks where `exec_us` exceeded the thread's own period — a direct sign that thread is not keeping up with its own rate. |
| `$CPU,util_pct` | Sum of `util_pct` over every rate-tracked thread — the Liu & Layland `Σ(C_i/T_i)` figure. Above ~70% is a soft warning for a task set this size; approaching 100% or `misses>0` anywhere means it's not schedulable as configured. |
| `$CPU,util_pct_hrt` | Same sum, but skipping threads registered with `TIMING_REGISTER_SOFT` (currently just `log`) — a truer figure for hard-deadline schedulability when the task set also has a soft-deadline, I/O-bound thread whose worst-case latency isn't CPU-bound (see the root README's [Timing and Utilization](../README.md#timing-and-utilization)). In the capture above, `util_pct` is 86.2% but `util_pct_hrt` is 64.3% — `log`'s SD-card-write tail alone accounts for the rest. |

Run it under realistic load (armed, radio connected, CAN/mocap link up) — idle-bench numbers will understate `est`/`ctrl`/`spi` load significantly.

---

## flash_upload.py

Uploads a compiled `.bin` firmware image to a CubeBlue H7 or CubeOrange+ using the ChibiOS bootloader protocol over USB.

```bash
# Recommended: use Makefile targets
make flash BOARD=blue
make flash BOARD=orange PORT=/dev/ttyACM0

# Or directly
python3 tools/flash_upload.py build/BPRL.bin
python3 tools/flash_upload.py --port /dev/ttyACM0 build/BPRL.bin
```

If the board is already running firmware, the script sends a reboot-to-bootloader command automatically. Sequence: detect port → reboot if needed → erase → program in 252-byte chunks → CRC-32 verify → reboot.

---

## Quick reference

```bash
# Flash (default board: BOARD=orange)
make flash BOARD=orange

# Debug build + flash
make BOARD=orange UDEFS_EXTRA=-DBPRL_DEBUG && make flash BOARD=orange

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

# Timing/schedulability build + query (see Timing section above)
make BOARD=blue UDEFS_EXTRA=-DBPRL_TIMING && make flash BOARD=blue
python3 -m serial.tools.miniterm /dev/ttyACM0 115200   # then type: TIM,status
```
