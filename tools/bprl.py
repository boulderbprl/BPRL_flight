#!/usr/bin/env python3
"""
BPRL Ground Tool — motor test, telemetry, and SD log access over USB CDC.

Usage:
    python3 tools/bprl.py telemetry              # live attitude / rate / IMU dashboard
    python3 tools/bprl.py motor-test             # spin individual motors, view RPM
    python3 tools/bprl.py logs list              # list log files on SD card
    python3 tools/bprl.py logs download [FILE]   # download a log file (default: latest)
    python3 tools/bprl.py logs decode FILE.bin   # decode binary log to CSV
    python3 tools/bprl.py logs erase             # erase completed log files

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate for VCP (default 115200, ignored by USB CDC)

Motor test and log commands work on any build.
Telemetry requires a -DBPRL_DEBUG build.

Dependencies: pip install pyserial rich
"""

import argparse
import glob
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed.  Run: pip install pyserial rich")
    sys.exit(1)

try:
    from rich.console import Console
    from rich.layout import Layout
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    from rich import print as rprint
    from rich.progress import Progress, BarColumn, TransferSpeedColumn, DownloadColumn
except ImportError:
    print("ERROR: rich not installed.  Run: pip install pyserial rich")
    sys.exit(1)

console = Console()

# ── Serial port auto-detection ────────────────────────────────────────────────

BPRL_VID = 0x0483
BPRL_PID = 0x5740

def find_port() -> Optional[str]:
    for p in serial.tools.list_ports.comports():
        if p.vid == BPRL_VID and p.pid == BPRL_PID:
            return p.device
    candidates = glob.glob("/dev/ttyACM*")
    return candidates[0] if candidates else None


def open_port(port: Optional[str], baud: int) -> serial.Serial:
    if port is None:
        port = find_port()
    if port is None:
        console.print("[red]ERROR: No BPRL device found. Use --port to specify.")
        sys.exit(1)
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        console.print(f"[green]Connected: {port}")
        # Give the USB CDC state machine time to finish its transition and
        # flush any leftover bytes from a previous session.
        time.sleep(0.15)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        console.print(f"[red]ERROR: Cannot open {port}: {e}")
        sys.exit(1)


# ── Shared telemetry state ────────────────────────────────────────────────────

@dataclass
class TelState:
    time_ms:      float = 0.0
    roll:         float = 0.0   # degrees
    pitch:        float = 0.0
    yaw:          float = 0.0
    p:            float = 0.0   # rad/s
    q:            float = 0.0
    r:            float = 0.0
    thr:          float = 0.0
    rc_roll:      float = 0.0
    rc_pitch:     float = 0.0
    rc_yaw:       float = 0.0
    armed:        bool  = False
    rpm:          list  = field(default_factory=lambda: [0, 0, 0, 0])
    rpm_valid:    list  = field(default_factory=lambda: [False]*4)
    imu_valid:    list  = field(default_factory=lambda: [False, False, False])
    can_valid:    bool  = False
    can_quat_hz:  int   = 0
    can_rate_hz:  int   = 0
    received_any: bool  = False  # True once first $TEL line parsed
    usb_rx_any:   bool  = False  # True once ANY bytes arrive from firmware
    last_rx:      float = field(default_factory=time.monotonic)
    lines_rx:     int   = 0      # total lines received (for diagnostic display)


def parse_tel_line(line: str, state: TelState) -> bool:
    """Parse a $TEL CSV line into state.  Returns True on success."""
    if not line.startswith("$TEL,"):
        return False
    try:
        parts = line[5:].split(",")
        if len(parts) < 22:
            return False
        state.time_ms      = float(parts[0])
        state.roll         = float(parts[1])
        state.pitch        = float(parts[2])
        state.yaw          = float(parts[3])
        state.p            = float(parts[4])
        state.q            = float(parts[5])
        state.r            = float(parts[6])
        state.thr          = float(parts[7])
        state.rc_roll      = float(parts[8])
        state.rc_pitch     = float(parts[9])
        state.rc_yaw       = float(parts[10])
        state.armed        = bool(int(parts[11]))
        state.rpm          = [int(parts[12]), int(parts[13]),
                              int(parts[14]), int(parts[15])]
        state.rpm_valid    = [r > 0 for r in state.rpm]
        state.imu_valid    = [bool(int(parts[16])), bool(int(parts[17])),
                              bool(int(parts[18]))]
        state.can_valid    = bool(int(parts[19]))
        state.can_quat_hz  = int(parts[20])
        state.can_rate_hz  = int(parts[21])
        state.received_any = True
        state.usb_rx_any   = True
        state.last_rx      = time.monotonic()
        return True
    except (ValueError, IndexError):
        return False


# ── Background serial reader ──────────────────────────────────────────────────

class SerialReader:
    """Reads lines from serial in a background thread, calls callbacks."""
    def __init__(self, ser: serial.Serial):
        self._ser     = ser
        self._lock    = threading.Lock()
        self._lines   = []
        self._stop    = threading.Event()
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", errors="replace").strip()
                if text:
                    with self._lock:
                        self._lines.append(text)

    def pop_lines(self) -> list:
        with self._lock:
            lines, self._lines = self._lines, []
        return lines

    def stop(self):
        self._stop.set()


# ── Telemetry display ─────────────────────────────────────────────────────────

MOTOR_LABELS = ["FR[0]", "RL[1]", "FL[2]", "RR[3]"]
IMU_LABELS   = ["ICM-20948 pri", "ICM-20948 ext", "ICM-20602    "]


def _valid(v: bool) -> Text:
    return Text("● valid  ", style="green") if v else Text("○ absent ", style="dim")


def build_telemetry_panel(s: TelState) -> Panel:
    age   = time.monotonic() - s.last_rx
    stale = age > 1.0

    # Header
    arm_text = Text("● ARMED  ", style="bold red") if s.armed \
               else Text("● DISARMED", style="bold green")
    t_sec    = s.time_ms / 1000.0

    # Attitude / rates / RC grid
    grid = Table.grid(padding=(0, 2))
    grid.add_column(min_width=18)
    grid.add_column(min_width=18)
    grid.add_column(min_width=18)
    grid.add_row("[bold]ATTITUDE[/bold]",
                 "[bold]BODY RATES (rad/s)[/bold]",
                 "[bold]RC INPUTS[/bold]")
    grid.add_row(f"Roll:  {s.roll:+7.2f}°",
                 f"P: {s.p:+8.4f}",
                 f"Thr:   {s.thr:5.3f}")
    grid.add_row(f"Pitch: {s.pitch:+7.2f}°",
                 f"Q: {s.q:+8.4f}",
                 f"Roll:  {s.rc_roll:+5.3f}")
    grid.add_row(f"Yaw:   {s.yaw:+7.2f}°",
                 f"R: {s.r:+8.4f}",
                 f"Pitch: {s.rc_pitch:+5.3f}")
    grid.add_row("", "", f"Yaw:   {s.rc_yaw:+5.3f}")

    # IMU status
    imu_grid = Table.grid(padding=(0, 2))
    imu_grid.add_column(min_width=20)
    imu_grid.add_column()
    for i, lbl in enumerate(IMU_LABELS):
        imu_grid.add_row(f"  IMU{i} {lbl}:", _valid(s.imu_valid[i]))
    can_line = Text()
    can_line.append(_valid(s.can_valid))
    if s.can_valid:
        can_line.append(f"quat={s.can_quat_hz:3d} Hz  rate={s.can_rate_hz:3d} Hz",
                        style="cyan")
    imu_grid.add_row("  CAN INS (IMX5):    ", can_line)

    # Motor RPM
    rpm_grid = Table.grid(padding=(0, 2))
    rpm_grid.add_column(min_width=24)
    rpm_grid.add_column(min_width=24)
    for row in ((0, 1), (2, 3)):
        cells = []
        for i in row:
            dot  = "●" if s.rpm_valid[i] else "○"
            col  = "green" if s.rpm_valid[i] else "dim"
            cells.append(Text(f"  {MOTOR_LABELS[i]}: {s.rpm[i]:6d} rpm {dot}",
                              style=col))
        rpm_grid.add_row(*cells)

    if not s.usb_rx_any:
        stale_note = "  [dim](no USB data — check connection)[/dim]"
    elif not s.received_any:
        stale_note = "  [dim](USB ok, waiting for $TEL — requires -DBPRL_DEBUG build)[/dim]"
    elif stale:
        stale_note = "  [yellow](stale — lost contact)[/yellow]"
    else:
        stale_note = ""
    diag = f"  [dim]lines_rx={s.lines_rx}[/dim]"
    title = (f"BPRL Debug  {arm_text}    "
             f"t={t_sec:8.1f} s{stale_note}{diag}")

    from rich.columns import Columns
    body = Table.grid()
    body.add_row(grid)
    body.add_row(Panel(imu_grid,  title="IMU Status", border_style="dim"))
    body.add_row(Panel(rpm_grid,  title="Motor RPM",  border_style="dim"))

    return Panel(body, title=title, border_style="blue")


def cmd_telemetry(ser: serial.Serial, _args):
    state  = TelState()
    reader = SerialReader(ser)

    try:
        with Live(build_telemetry_panel(state), refresh_per_second=10,
                  console=console) as live:
            while True:
                for line in reader.pop_lines():
                    state.lines_rx += 1
                    if not parse_tel_line(line, state):
                        # Non-$TEL line still proves the firmware is connected
                        state.usb_rx_any = True
                live.update(build_telemetry_panel(state))
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()


# ── Motor test ────────────────────────────────────────────────────────────────

def build_motor_panel(state: TelState, active_motor: int, active_pct: int) -> Panel:
    rpm_grid = Table.grid(padding=(0, 2))
    rpm_grid.add_column(min_width=26)
    rpm_grid.add_column(min_width=26)
    for row in ((0, 1), (2, 3)):
        cells = []
        for i in row:
            dot = "●" if state.rpm[i] > 0 else "○"
            col = ("bold green" if i == active_motor and active_pct > 0
                   else ("green" if state.rpm[i] > 0 else "dim"))
            cells.append(Text(f"  {MOTOR_LABELS[i]}: {state.rpm[i]:6d} rpm {dot}",
                              style=col))
        rpm_grid.add_row(*cells)

    note = ""
    if active_motor >= 0 and active_pct > 0:
        note = f"  Active: {MOTOR_LABELS[active_motor]} @ {active_pct}%"

    body = Table.grid()
    body.add_row(rpm_grid)
    if note:
        body.add_row(Text(note, style="bold yellow"))

    return Panel(body,
                 title="[bold red]BPRL Motor Test  ⚠  REMOVE PROPS BEFORE TESTING[/bold red]",
                 border_style="red")


def send_cmd(ser: serial.Serial, cmd: str):
    ser.write((cmd + "\n").encode())
    ser.flush()


def cmd_motor_test(ser: serial.Serial, _args):
    state        = TelState()
    active_motor = -1
    active_pct   = 0
    reader       = SerialReader(ser)

    console.print("[bold red]⚠  MOTOR TEST MODE — REMOVE PROPS BEFORE PROCEEDING[/bold red]")
    console.print("Commands: [cyan]motor <0-3> <pct%>[/cyan]  |  "
                  "[cyan]stop[/cyan]  |  [cyan]quit[/cyan]")
    console.print("[dim]Tip: 'motor 0 15' spins FR motor at 15%[/dim]")

    try:
        while True:
            # Refresh RPM display
            for line in reader.pop_lines():
                parse_tel_line(line, state)

            # Build a quick status line (no Live to keep input prompt clean)
            rpm_parts = []
            for i, lbl in enumerate(MOTOR_LABELS):
                dot = "●" if state.rpm[i] > 0 else "○"
                rpm_parts.append(f"{lbl}:{state.rpm[i]:6d}rpm{dot}")
            console.print("  " + "   ".join(rpm_parts), highlight=False, end="\r")

            try:
                cmd = input("\n> ").strip().lower()
            except EOFError:
                break

            if cmd in ("quit", "q", "exit"):
                send_cmd(ser, "MT,stop")
                break

            if cmd in ("stop", "s"):
                send_cmd(ser, "MT,stop")
                active_motor = -1
                active_pct   = 0
                console.print("[green]Motors stopped.")
                continue

            parts = cmd.split()
            if len(parts) == 3 and parts[0] == "motor":
                try:
                    motor = int(parts[1])
                    pct   = int(parts[2])
                except ValueError:
                    console.print("[yellow]Usage: motor <0-3> <0-100>")
                    continue
                if motor < 0 or motor > 3:
                    console.print("[yellow]Motor index must be 0-3.")
                    continue
                if pct < 0 or pct > 100:
                    console.print("[yellow]Throttle must be 0-100%.")
                    continue
                send_cmd(ser, f"MT,{motor},{pct}")
                # Read one line of response
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    for line in reader.pop_lines():
                        if line.startswith("MT,"):
                            if "ERR,armed" in line:
                                console.print("[bold red]REJECTED: Drone is armed! Disarm first.")
                            elif "OK" in line:
                                active_motor = motor
                                active_pct   = pct
                                console.print(
                                    f"[green]{MOTOR_LABELS[motor]} spinning at {pct}%")
                            else:
                                console.print(f"[yellow]Response: {line}")
                            break
                    else:
                        time.sleep(0.05)
                        continue
                    break
                continue

            console.print("[dim]Unknown command. Try: motor 0 20 | stop | quit")
    except KeyboardInterrupt:
        pass
    finally:
        send_cmd(ser, "MT,stop")
        reader.stop()
        console.print("\n[green]Motor test ended. All motors stopped.")


# ── Log commands ──────────────────────────────────────────────────────────────

def _read_line(ser: serial.Serial, timeout: float = 5.0) -> Optional[str]:
    """Read one line; discard any bytes after the newline (text-only contexts)."""
    line, _ = _read_line_ex(ser, timeout)
    return line


def _read_line_ex(ser: serial.Serial, timeout: float = 5.0):
    """Read one line and return (line, remainder_bytes) so callers can
    preserve bytes already read past the newline (e.g. binary downloads)."""
    deadline = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            if b"\n" in buf:
                idx  = buf.index(b"\n")
                line = buf[:idx].decode("ascii", errors="replace").strip()
                return line, buf[idx + 1:]
        time.sleep(0.01)
    return None, b""


def cmd_logs_list(ser: serial.Serial, _args):
    ser.write(b"LOG,list\n")
    ser.flush()
    files = []
    while True:
        line = _read_line(ser, timeout=5.0)
        if line is None:
            console.print("[red]Timeout waiting for log list.")
            break
        if line.startswith("LOG,ERR,"):
            console.print(f"[red]Error: {line}")
            break
        if line == "LOG,LIST,END":
            break
        if line.startswith("LOG,FILE,"):
            parts = line.split(",")
            if len(parts) >= 4:
                fname = parts[2]
                fsize = int(parts[3])
                files.append((fname, fsize))

    if not files:
        console.print("[yellow]No log files found (or no SD card).")
        return

    table = Table(title="SD Card Log Files", show_header=True)
    table.add_column("Filename", style="cyan")
    table.add_column("Size", justify="right")
    for fname, fsize in files:
        mb = fsize / 1_048_576
        table.add_row(fname, f"{mb:.1f} MB  ({fsize:,} B)")
    console.print(table)
    if files:
        console.print(f"  [dim]Latest: {files[-1][0]}")


def cmd_logs_download(ser: serial.Serial, args):
    fname = args.file
    if fname is None:
        # Ask for list first to find latest
        ser.write(b"LOG,list\n")
        ser.flush()
        files = []
        while True:
            line = _read_line(ser, timeout=5.0)
            if line is None or line == "LOG,LIST,END":
                break
            if line.startswith("LOG,FILE,"):
                parts = line.split(",")
                if len(parts) >= 4:
                    files.append(parts[2])
        if not files:
            console.print("[red]No log files found.")
            return
        # The highest-numbered file is the currently-open log (being written now).
        # Default to the second-latest so we get a complete file.  If there's
        # only one file, download it anyway (the user asked explicitly).
        fname = files[-2] if len(files) >= 2 else files[-1]
        console.print(f"[dim]Downloading latest completed log: {fname}")

    ser.write(f"LOG,get,{fname}\n".encode())
    ser.flush()

    # Read until we see LOG,SIZE or LOG,ERR — skip any $TEL lines that arrive
    # before the firmware's response (can happen in a DEBUG build).
    size_line = None
    leftover  = b""
    deadline  = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        line, leftover = _read_line_ex(ser, timeout=deadline - time.monotonic())
        if line is None:
            break
        if line.startswith("LOG,SIZE,") or line.startswith("LOG,ERR,"):
            size_line = line
            break
        # else: $TEL or other debug output — discard and keep waiting

    if size_line is None:
        console.print("[red]Timeout waiting for LOG,SIZE response.")
        return
    if size_line.startswith("LOG,ERR,"):
        console.print(f"[red]Error: {size_line}")
        return

    total = int(size_line.split(",")[2])
    out_path = Path(fname)
    console.print(f"Downloading {fname} ({total / 1_048_576:.1f} MB) → {out_path}")

    with Progress(
        "[progress.description]{task.description}",
        BarColumn(),
        DownloadColumn(),
        TransferSpeedColumn(),
        console=console,
    ) as progress:
        task = progress.add_task(fname, total=total)
        received = 0
        with open(out_path, "wb") as f:
            # Write any bytes already buffered past the size newline
            if leftover:
                f.write(leftover)
                received += len(leftover)
                progress.update(task, advance=len(leftover))
            ser.timeout = 2.0
            while received < total:
                want  = min(2048, total - received)
                chunk = ser.read(want)
                if not chunk:
                    console.print(f"\n[red]Timeout: received {received}/{total} bytes.")
                    break
                f.write(chunk)
                received += len(chunk)
                progress.update(task, advance=len(chunk))
        ser.timeout = 0.1

    if received == total:
        console.print(f"[green]Saved {out_path} ({received:,} bytes).")
    else:
        console.print(f"[yellow]Partial download: {received}/{total} bytes.")


def cmd_logs_erase(ser: serial.Serial, _args):
    answer = input("Erase all completed log files from SD card? [y/N]: ").strip().lower()
    if answer != "y":
        console.print("[dim]Aborted.")
        return
    ser.write(b"LOG,erase\n")
    ser.flush()
    # Skip any $TEL lines while waiting for the erase response.
    deadline = time.monotonic() + 10.0
    line = None
    while time.monotonic() < deadline:
        candidate = _read_line(ser, timeout=deadline - time.monotonic())
        if candidate is None:
            break
        if candidate.startswith("LOG,ERASED,") or candidate.startswith("LOG,ERR,"):
            line = candidate
            break
    if line is None:
        console.print("[red]Timeout waiting for erase response.")
        return
    if line.startswith("LOG,ERASED,"):
        n = int(line.split(",")[2])
        console.print(f"[green]Erased {n} file(s).")
    elif line.startswith("LOG,ERR,"):
        console.print(f"[red]Error: {line}")
    else:
        console.print(f"[yellow]{line}")


# ── Log decoder (offline — no serial needed) ──────────────────────────────────

SYNC = bytes([0xA3, 0x95])

LOG_MSG_FMT   = 0x80
LOG_MSG_IMU   = 0x01
LOG_MSG_STATE = 0x02

# Struct formats (little-endian, packed) — must match LogMessages.hpp
_IMU_FMT   = "<I6fB6fB6fB7fB"   # time_ms + 3×(6 floats+valid) + 4+3 floats + can_valid
_STATE_FMT = "<I10fB"            # time_ms + 10 floats + armed

_IMU_FIELDS = [
    "TimeMS",
    "AX0","AY0","AZ0","GX0","GY0","GZ0","V0",
    "AX1","AY1","AZ1","GX1","GY1","GZ1","V1",
    "AX2","AY2","AZ2","GX2","GY2","GZ2","V2",
    "QW","QX","QY","QZ","CanP","CanQ","CanR","CV",
]
_STATE_FIELDS = [
    "TimeMS","Roll","Pitch","Yaw","P","Q","R",
    "ZPos","ZVel","ZAcc","Thr","Armed",
]

_FMT_BODY   = 71  # bytes after the 3-byte header for a FMT record
_IMU_BODY   = struct.calcsize(_IMU_FMT)
_STATE_BODY = struct.calcsize(_STATE_FMT)

_BODY_SIZES = {
    LOG_MSG_FMT:   _FMT_BODY,
    LOG_MSG_IMU:   _IMU_BODY,
    LOG_MSG_STATE: _STATE_BODY,
}


def cmd_logs_decode(_, args):
    src = Path(args.decode_file)
    if not src.exists():
        console.print(f"[red]File not found: {src}")
        return

    stem       = src.stem
    imu_path   = src.with_name(f"{stem}_imu.csv")
    state_path = src.with_name(f"{stem}_state.csv")

    imu_rows   = []
    state_rows = []
    skipped    = 0

    data = src.read_bytes()
    pos  = 0
    n    = len(data)

    while pos < n - 3:
        # Scan for sync bytes
        if data[pos] != 0xA3 or data[pos + 1] != 0x95:
            pos += 1
            skipped += 1
            continue

        msg_id   = data[pos + 2]
        body_len = _BODY_SIZES.get(msg_id)
        if body_len is None:
            pos += 1
            continue

        start = pos + 3
        end   = start + body_len
        if end > n:
            break

        body = data[start:end]
        pos  = end

        if msg_id == LOG_MSG_IMU:
            try:
                vals = struct.unpack(_IMU_FMT, body)
                imu_rows.append(vals)
            except struct.error:
                pass

        elif msg_id == LOG_MSG_STATE:
            try:
                vals = struct.unpack(_STATE_FMT, body)
                state_rows.append(vals)
            except struct.error:
                pass

    import csv

    with open(imu_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(_IMU_FIELDS)
        w.writerows(imu_rows)
    console.print(f"[green]Decoded {len(imu_rows):,} IMU records   → {imu_path}")

    with open(state_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(_STATE_FIELDS)
        w.writerows(state_rows)
    console.print(f"[green]Decoded {len(state_rows):,} STATE records → {state_path}")

    if skipped:
        console.print(f"[dim]Skipped {skipped} non-sync bytes.")


# ── Argument parsing and dispatch ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL ground tool — motor test, telemetry, SD log access")
    parser.add_argument("--port",  default=None, help="Serial port (auto-detected)")
    parser.add_argument("--baud",  type=int, default=115200)

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("telemetry",   help="Live telemetry dashboard (DEBUG build required)")

    sub.add_parser("motor-test",  help="Interactive motor test with RPM feedback")

    logs_p = sub.add_parser("logs", help="SD card log commands")
    logs_sub = logs_p.add_subparsers(dest="logs_cmd", required=True)
    logs_sub.add_parser("list",    help="List log files on SD card")

    dl_p = logs_sub.add_parser("download", help="Download a log file")
    dl_p.add_argument("file", nargs="?", default=None,
                      help="Filename (default: latest completed log)")

    dec_p = logs_sub.add_parser("decode", help="Decode a local binary log to CSV")
    dec_p.add_argument("decode_file", help="Path to .BIN file")

    logs_sub.add_parser("erase",   help="Erase completed log files from SD card")

    args = parser.parse_args()

    # Commands that don't need serial
    if args.command == "logs" and args.logs_cmd == "decode":
        cmd_logs_decode(None, args)
        return

    # All other commands need a serial connection
    ser = open_port(args.port, args.baud)

    try:
        if args.command == "telemetry":
            cmd_telemetry(ser, args)

        elif args.command == "motor-test":
            cmd_motor_test(ser, args)

        elif args.command == "logs":
            if args.logs_cmd == "list":
                cmd_logs_list(ser, args)
            elif args.logs_cmd == "download":
                cmd_logs_download(ser, args)
            elif args.logs_cmd == "erase":
                cmd_logs_erase(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
