#!/usr/bin/env python3
"""
BPRL Ground Tool — motor test, telemetry, and SD log access over USB CDC.

Usage:
    python3 tools/bprl.py telemetry              # live attitude / rate / IMU dashboard
    python3 tools/bprl.py ekf-status             # per-lane EKF roll/pitch/yaw/p/q/r table
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
import queue
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


# ── IMU sample parser ($IMU stream) ──────────────────────────────────────────

@dataclass
class ImuSample:
    time_ms: int
    accel:   list   # [3][3] body-frame m/s²  (imu × axis)
    gyro:    list   # [3][3] body-frame rad/s
    valid:   list   # [3] bool


def parse_imu_line(line: str) -> Optional[ImuSample]:
    """Parse a $IMU CSV line.
    Format: $IMU,<ms>,ax0,ay0,az0,gx0,gy0,gz0,v0(0xWW), ...×3
    Returns None on malformed input.
    """
    if not line.startswith("$IMU,"):
        return None
    try:
        parts = line[5:].split(",")
        if len(parts) < 22:
            return None
        ms = int(parts[0])
        accel, gyro, valid = [], [], []
        for i in range(3):
            base = 1 + i * 7
            accel.append([float(parts[base]),
                          float(parts[base + 1]),
                          float(parts[base + 2])])
            gyro.append([float(parts[base + 3]),
                         float(parts[base + 4]),
                         float(parts[base + 5])])
            vf = parts[base + 6]
            valid.append(bool(int(vf.split("(")[0])))
        return ImuSample(time_ms=ms, accel=accel, gyro=gyro, valid=valid)
    except (ValueError, IndexError):
        return None


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
IMU_LABELS   = ["IMU0 (pri)   ", "IMU1 (ext)   ", "IMU2         "]


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
    resp_q       = queue.Queue()   # firmware MT,OK / MT,ERR lines
    stop_bg      = threading.Event()

    console.print("[bold red]⚠  MOTOR TEST MODE — REMOVE PROPS BEFORE PROCEEDING[/bold red]")
    console.print("Commands: [cyan]motor <0-3> <pct%>[/cyan]  |  "
                  "[cyan]stop[/cyan]  |  [cyan]quit[/cyan]")
    console.print("[dim]Tip: 'motor 0 15' spins FR motor at 15%[/dim]\n")

    with Live(build_motor_panel(state, active_motor, active_pct),
              refresh_per_second=10, console=console, vertical_overflow="visible") as live:

        def bg_loop():
            while not stop_bg.is_set():
                for line in reader.pop_lines():
                    if line.startswith("MT,"):
                        resp_q.put(line)
                    else:
                        parse_tel_line(line, state)
                live.update(build_motor_panel(state, active_motor, active_pct))
                time.sleep(0.05)

        bg_t = threading.Thread(target=bg_loop, daemon=True)
        bg_t.start()

        try:
            while True:
                try:
                    cmd = input("> ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    break

                if cmd in ("quit", "q", "exit"):
                    send_cmd(ser, "MT,stop")
                    break

                if cmd in ("stop", "s"):
                    send_cmd(ser, "MT,stop")
                    active_motor = -1
                    active_pct   = 0
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
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline:
                        try:
                            resp = resp_q.get(timeout=0.1)
                        except queue.Empty:
                            continue
                        if "ERR,armed" in resp:
                            console.print("[bold red]REJECTED: Drone is armed! Disarm first.")
                        elif "OK" in resp:
                            active_motor = motor
                            active_pct   = pct
                        break
                    continue

                if cmd:
                    console.print("[dim]Unknown command. Try: motor 0 20 | stop | quit")

        except KeyboardInterrupt:
            pass
        finally:
            stop_bg.set()
            bg_t.join(timeout=1.0)

    send_cmd(ser, "MT,stop")
    reader.stop()
    console.print("[green]Motor test ended. All motors stopped.")


# ── EKF per-lane state ────────────────────────────────────────────────────

LANE_LABELS = ["Lane 0", "Lane 1", "Lane 2", "IMX5 INS"]


@dataclass
class EkfLaneState:
    time_ms:      float = 0.0
    primary:      int   = 0
    roll:         list  = field(default_factory=lambda: [0.0] * 4)   # degrees
    pitch:        list  = field(default_factory=lambda: [0.0] * 4)
    yaw:          list  = field(default_factory=lambda: [0.0] * 4)
    p:            list  = field(default_factory=lambda: [0.0] * 4)   # rad/s
    q:            list  = field(default_factory=lambda: [0.0] * 4)
    r:            list  = field(default_factory=lambda: [0.0] * 4)
    received_any: bool  = False
    usb_rx_any:   bool  = False
    last_rx:      float = field(default_factory=time.monotonic)
    lines_rx:     int   = 0


def parse_ekfl_line(line: str, state: EkfLaneState) -> bool:
    """Parse a $EKFL line into state.  Returns True on success.
    Format: $EKFL,<ms>,<primary>,<r0°>,<p0°>,<y0°>,<p0>,<q0>,<r0>,  ...×3 onboard lanes,  <r_ins°>,<p_ins°>,<y_ins°>,<p_ins>,<q_ins>,<r_ins>
    """
    if not line.startswith("$EKFL,"):
        return False
    try:
        parts = line[6:].split(",")
        if len(parts) < 26:          # 1(ms) + 1(primary) + 4×6 = 26
            return False
        state.time_ms = float(parts[0])
        state.primary = int(parts[1])
        for i in range(4):
            base = 2 + i * 6
            state.roll[i]  = float(parts[base])
            state.pitch[i] = float(parts[base + 1])
            state.yaw[i]   = float(parts[base + 2])
            state.p[i]     = float(parts[base + 3])
            state.q[i]     = float(parts[base + 4])
            state.r[i]     = float(parts[base + 5])
        state.received_any = True
        state.usb_rx_any   = True
        state.last_rx      = time.monotonic()
        return True
    except (ValueError, IndexError):
        return False


def build_ekf_panel(s: EkfLaneState) -> Panel:
    age   = time.monotonic() - s.last_rx
    stale = age > 1.0
    t_sec = s.time_ms / 1000.0

    if not s.usb_rx_any:
        stale_note = "  [dim](no USB data)[/dim]"
    elif not s.received_any:
        stale_note = "  [dim](waiting for $EKFL — requires -DBPRL_DEBUG build)[/dim]"
    elif stale:
        stale_note = "  [yellow](stale)[/yellow]"
    else:
        stale_note = ""

    tbl = Table(show_header=True, header_style="bold", box=None, padding=(0, 2))
    tbl.add_column("", min_width=12)
    for i, lbl in enumerate(LANE_LABELS):
        pri_tag = " [bold green](pri)[/bold green]" if (i < 3 and i == s.primary) else ""
        ins_tag = " [dim](zeros until CAN enabled)[/dim]" if i == 3 else ""
        tbl.add_column(f"{lbl}{pri_tag}{ins_tag}", min_width=18, justify="right")

    def _row(label, values, fmt, unit=""):
        cells = [f"[bold]{label}[/bold]"]
        for i, v in enumerate(values):
            style = "dim" if (i == 3 and v == 0.0) else ""
            cells.append(f"[{style}]{v:{fmt}}{unit}[/{style}]" if style
                         else f"{v:{fmt}}{unit}")
        tbl.add_row(*cells)

    _row("Roll",  s.roll,  "+8.2f", "°")
    _row("Pitch", s.pitch, "+8.2f", "°")
    _row("Yaw",   s.yaw,   "+8.2f", "°")
    tbl.add_row("")
    _row("p",     s.p,     "+9.4f", "")
    _row("q",     s.q,     "+9.4f", "")
    _row("r",     s.r,     "+9.4f", "")

    diag = f"  [dim]lines_rx={s.lines_rx}[/dim]"
    title = (f"BPRL EKF Lanes    t={t_sec:8.1f} s{stale_note}{diag}")
    return Panel(tbl, title=title, border_style="cyan")


def cmd_ekf_status(ser: serial.Serial, _args):
    state  = EkfLaneState()
    reader = SerialReader(ser)

    try:
        with Live(build_ekf_panel(state), refresh_per_second=10,
                  console=console) as live:
            while True:
                for line in reader.pop_lines():
                    state.lines_rx += 1
                    if not parse_ekfl_line(line, state):
                        state.usb_rx_any = True
                live.update(build_ekf_panel(state))
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()


# ── CAN status ────────────────────────────────────────────────────────────────

_LEC_NAMES = ["NoError","Stuff","Form","Ack","Bit1","Bit0","CRC","NoChange"]
_ACT_NAMES = ["Synchronizing","Idle","Receiver","Transmitter"]

def _decode_psr(psr: int) -> str:
    lec  = _LEC_NAMES[psr & 0x7]
    act  = _ACT_NAMES[(psr >> 3) & 0x3]
    ep   = bool(psr & (1 << 5))
    ew   = bool(psr & (1 << 6))
    bo   = bool(psr & (1 << 7))
    pxe  = bool(psr & (1 << 14))
    flags = []
    if bo:  flags.append("BUS-OFF")
    if ep:  flags.append("ErrorPassive")
    if ew:  flags.append("ErrorWarning")
    if pxe: flags.append("ProtoExcept")
    flag_str = " ".join(flags) if flags else "OK"
    return f"ACT={act}  LEC={lec}  {flag_str}"

def _decode_ecr(ecr: int) -> str:
    tec = ecr & 0xFF
    rec = (ecr >> 8) & 0x7F
    rp  = bool(ecr & (1 << 15))
    cel = (ecr >> 16) & 0xFF
    return f"TEC={tec}  REC={rec}{'(passive)' if rp else ''}  CEL={cel}"

def _decode_rxf0s(rxf0s: int) -> str:
    fill = rxf0s & 0x7F
    full = bool(rxf0s & (1 << 24))
    lost = bool(rxf0s & (1 << 25))
    return f"fill={fill}{'  FULL' if full else ''}{'  MSGS_LOST' if lost else ''}"

def cmd_can_status(ser: serial.Serial, _args):
    import re
    ser.write(b"CAN,status\r\n")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        m = re.match(
            r"CAN,STATUS,psr=(0x[0-9a-fA-F]+),ecr=(0x[0-9a-fA-F]+),"
            r"rxf0s=(0x[0-9a-fA-F]+),cccr=(0x[0-9a-fA-F]+)", line)
        if m:
            psr   = int(m.group(1), 16)
            ecr   = int(m.group(2), 16)
            rxf0s = int(m.group(3), 16)
            cccr  = int(m.group(4), 16)
            console.print(f"[bold]FDCAN1 Status[/bold]")
            console.print(f"  PSR  0x{psr:08x}  →  {_decode_psr(psr)}")
            console.print(f"  ECR  0x{ecr:08x}  →  {_decode_ecr(ecr)}")
            console.print(f"  RXF0S 0x{rxf0s:08x}  →  {_decode_rxf0s(rxf0s)}")
            console.print(f"  CCCR 0x{cccr:08x}  →  INIT={cccr&1}  MON={(cccr>>5)&1}  TEST={(cccr>>7)&1}")
            console.print()
            console.print("[dim]ACT=Idle + LEC=NoError/NoChange + TEC=0/REC=0 → bus healthy, IMX5 not transmitting[/dim]")
            console.print("[dim]ACT=Synchronizing → baud rate mismatch or no termination[/dim]")
            console.print("[dim]LEC=Ack + TEC>0 → FC transmitting but no ack (only one node, or wrong baud)[/dim]")
            console.print("[dim]LEC=Bit1/Form/CRC + REC>0 → IMX5 transmitting at different baud rate[/dim]")
            return
    console.print("[red]No CAN,STATUS response — firmware may not support this command[/red]")


# ── DShot diagnostic ──────────────────────────────────────────────────────────

def cmd_dshot_health(ser: serial.Serial, _args):
    """
    Live DShot health monitor.  Parses the $DSHOT heartbeat line (printed every
    2 s by HeartbeatThread) and shows rates, edge counts, and ESC arm status.

    $DSHOT,<ms>,tc=<t1>/<t4>,cc=<c1>/<c4>,edges=<m0>/<m1>/<m2>/<m3>
    tc   = DMA TX-complete count  → should grow at ~400/s (1 per frame)
    cc   = IC-complete count      → should grow at ~400/s (TIM1 rotates 3 slots)
    edges = last IC edge count per motor (0 = ESC not responding)
    """
    import re

    console.print("[bold cyan]DShot Health Monitor[/bold cyan]  (Ctrl-C to exit)")
    console.print("Waiting for $DSHOT heartbeat (prints every 2 s)...\n")

    prev_tc = [None, None]
    prev_cc = [None, None]
    prev_ms = None
    arm_detected = [False, False, False, False]

    MOTOR = ["M0 FR", "M1 RL", "M2 FL", "M3 RR"]
    ESC_WATCHDOG_S = 30.0   # ESC resets if no valid signal for this long

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line.startswith("$DSHOT,"):
                continue

            # Parse: $DSHOT,<ms>,tc=T1/T4,cc=C1/C4,edges=E0/E1/E2/E3
            m = re.match(
                r"\$DSHOT,(\d+),tc=(\d+)/(\d+),cc=(\d+)/(\d+),edges=(\d+)/(\d+)/(\d+)/(\d+)",
                line
            )
            if not m:
                continue

            ms     = int(m.group(1))
            tc1    = int(m.group(2));  tc4   = int(m.group(3))
            cc1    = int(m.group(4));  cc4   = int(m.group(5))
            edges  = [int(m.group(6+i)) for i in range(4)]

            dt_s   = ((ms - prev_ms) / 1000.0) if prev_ms is not None else 2.0
            dt_s   = max(dt_s, 0.01)

            tc1_hz = ((tc1 - prev_tc[0]) / dt_s) if prev_tc[0] is not None else 0.0
            tc4_hz = ((tc4 - prev_tc[1]) / dt_s) if prev_tc[1] is not None else 0.0
            cc1_hz = ((cc1 - prev_cc[0]) / dt_s) if prev_cc[0] is not None else 0.0
            cc4_hz = ((cc4 - prev_cc[1]) / dt_s) if prev_cc[1] is not None else 0.0

            prev_tc = [tc1, tc4];  prev_cc = [cc1, cc4];  prev_ms = ms

            # ── Print snapshot ────────────────────────────────────────────────
            console.rule(f"[dim]t={ms/1000:.1f}s[/dim]")

            # TX rate
            tc1_ok = 350 < tc1_hz < 450
            tc4_ok = 350 < tc4_hz < 450
            tc1_col = "green" if tc1_ok else "red"
            tc4_col = "green" if tc4_ok else "red"
            console.print(
                f"  TX rate  TIM1=[{tc1_col}]{tc1_hz:5.0f}[/{tc1_col}] Hz  "
                f"TIM4=[{tc4_col}]{tc4_hz:5.0f}[/{tc4_col}] Hz  "
                f"(expected ~400)"
            )
            if not tc1_ok or not tc4_ok:
                console.print("  [red]→ TX rate wrong: DShot frames not being sent at 400 Hz.[/red]")
                console.print("  [red]  Check dshot_init() DMA alloc and timer CEN start.[/red]")

            # IC / decode rate
            cc1_ok = cc1_hz > 50   # at least 50 decodes/s (some may be timeouts)
            cc1_col = "green" if cc1_ok else "yellow"
            console.print(
                f"  IC rate  TIM1=[{cc1_col}]{cc1_hz:5.0f}[/{cc1_col}] Hz  "
                f"TIM4=[green]{cc4_hz:5.0f}[/green] Hz  "
                f"(TIM1 expects ~133/s per motor slot; TIM4 ~400/s)"
            )

            # Edge counts per motor
            console.print()
            all_zero = all(e == 0 for e in edges)
            for i, name in enumerate(MOTOR):
                e = edges[i]
                if e == 0:
                    status = "[red]NO EDGES[/red]"
                    if not arm_detected[i]:
                        status += " — ESC not arming yet"
                elif e < 10:
                    status = f"[yellow]PARTIAL ({e}/21)[/yellow]"
                elif e >= 21:
                    status = f"[green]FULL ({e}/21) — decoding active[/green]"
                    arm_detected[i] = True
                else:
                    status = f"[cyan]{e}/21[/cyan]"
                console.print(f"    {name}: edges={e:2d}  {status}")

            # Diagnosis
            console.print()
            if all_zero and cc1_ok:
                console.print(
                    "[yellow]ESC silent: IC is running but no edges arrive.[/yellow]\n"
                    "  Possible causes:\n"
                    "  1. ESC not in bidirectional DShot mode (configure in BLHeli/AM32)\n"
                    "  2. Signal not reaching ESC (probe AUX 2-5 with scope)\n"
                    "  3. ESC not yet armed — try MT,0,0 or wait for arming beep"
                )
            elif any(arm_detected):
                armed_names = [MOTOR[i] for i in range(4) if arm_detected[i]]
                console.print(f"[green bold]ESC RESPONDING: {', '.join(armed_names)}[/green bold]")
                console.print("  GCR edges arriving — RPM telemetry active.")
    except KeyboardInterrupt:
        pass


def cmd_dshot_diag(ser: serial.Serial, _args):
    import re
    ser.write(b"DSHOT,diag\r\n")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if not line.startswith("DSHOT,DIAG,"):
            continue
        # parse all key=value tokens after the prefix
        rest = line[len("DSHOT,DIAG,"):]
        kv   = dict(tok.split("=") for tok in rest.split(",") if "=" in tok)

        dma0   = int(kv.get("dma_tc",  "0/0").split("/")[0])
        dma1   = int(kv.get("dma_tc",  "0/0").split("/")[1])
        isr0   = int(kv.get("cc_isr",  "0/0").split("/")[0])
        isr1   = int(kv.get("cc_isr",  "0/0").split("/")[1])
        ecnts  = [int(x) for x in kv.get("edges", "0/0/0/0").split("/")]

        # re-parse edge timestamps from e0..e3 fields
        raw_edges = []
        for key in ("e0", "e1", "e2", "e3"):
            if key in kv:
                raw_edges.append([int(x) for x in kv[key].split("/")])
            else:
                raw_edges.append([0] * 5)

        console.print("\n[bold]DShot Bidirectional Diagnostic[/bold]")
        console.print(f"  DMA TC fires  : TIM1={dma0}  TIM4={dma1}")
        console.print(f"  CC ISR decode : TIM1={isr0}  TIM4={isr1}")
        console.print()

        motor_names = ["M0 FR (TIM1/CC3)", "M1 RL (TIM1/CC1)", "M2 FL (TIM4/CC2)", "M3 RR (TIM1/CC2)"]
        for i, name in enumerate(motor_names):
            ec = ecnts[i] if i < len(ecnts) else 0
            es = raw_edges[i] if i < len(raw_edges) else [0]*5
            # Interpret first edge: should be ~6000+ ticks (≈30µs at 200 MHz)
            first_us = es[0] / 200.0 if es[0] > 0 else 0.0
            # Interpret spacing: should be ~267 ticks (1.33 µs per GCR bit)
            spacing = (es[1] - es[0]) if ec >= 2 and es[1] > es[0] else 0
            status = "[green]OK[/green]" if ec >= 2 else "[red]NO EDGES[/red]" if ec == 0 else "[yellow]PARTIAL[/yellow]"
            console.print(f"  {name}: edges={ec}/21  {status}")
            if ec > 0:
                console.print(f"    First edge: {es[0]} ticks ({first_us:.1f} µs after TX)  spacing: {spacing} ticks (~{spacing/267:.2f}× expected)")
                console.print(f"    Raw[0..4]: {es}")

        console.print()
        # Interpret results
        if dma0 == 0 and dma1 == 0:
            console.print("[red]FAIL: DMA TC never fires — DShot TX not completing (check DMAMUX or DMA init)[/red]")
        elif isr0 == 0 and isr1 == 0:
            console.print("[red]FAIL: CC ISR never decodes — input capture never triggered (check NVIC/DIER)[/red]")
        elif all(ec == 0 for ec in ecnts):
            console.print("[yellow]INFO: ISR fires but no edges captured — ESC not responding")
            console.print("  → Is the motor spinning? (ESC only responds after arming)")
            console.print("  → Check bidirectional DShot wiring (pull-up resistor on motor wire?)[/yellow]")
        elif all(ec >= 2 for ec in ecnts):
            console.print("[green]INFO: Edges captured on all motors — GCR decode is failing[/green]")
            console.print("  → Check bit_ticks (~267 expected), CRC inversion, edge polarity")
        else:
            console.print("[yellow]INFO: Partial edge capture — some motors responding, some not[/yellow]")
        return
    console.print("[red]No DSHOT,DIAG response — flash latest firmware and retry[/red]")


# ── Calibration ───────────────────────────────────────────────────────────────

def cmd_calibrate(ser: serial.Serial, args):
    duration = getattr(args, "duration", 30)
    reader   = SerialReader(ser)

    console.print("\n[bold]IMU Static Bias Calibration[/bold]")
    console.print("Place the drone on a [bold]level surface[/bold] and do not move it.")
    try:
        input("Press [Enter] to begin collection...")
    except EOFError:
        pass

    console.print(f"Collecting {duration} s of IMU data...")

    # Accumulators: [imu][axis]
    sums_gyro  = [[0.0]*3 for _ in range(3)]
    sums_accel = [[0.0]*3 for _ in range(3)]
    counts     = [0, 0, 0]
    t_end      = time.monotonic() + duration

    with Progress(transient=True) as prog:
        task = prog.add_task("Collecting...", total=duration)
        while time.monotonic() < t_end:
            for line in reader.pop_lines():
                s = parse_imu_line(line)
                if s is None:
                    continue
                for i in range(3):
                    if not s.valid[i]:
                        continue
                    counts[i] += 1
                    for k in range(3):
                        sums_gyro[i][k]  += s.gyro[i][k]
                        sums_accel[i][k] += s.accel[i][k]
            elapsed = duration - (t_end - time.monotonic())
            prog.update(task, completed=min(elapsed, duration))
            time.sleep(0.05)

    reader.stop()

    if min(counts) < 10:
        console.print(f"[red]Too few samples (got {counts}). Check connection.")
        return

    # Compute biases
    gyro_bias  = [[sums_gyro[i][k]  / counts[i] for k in range(3)] for i in range(3)]
    accel_bias = [[sums_accel[i][k] / counts[i] for k in range(3)] for i in range(3)]
    # Z-axis expected reading at level = +9.80665 m/s² (z-up sensor convention)
    for i in range(3):
        accel_bias[i][2] -= 9.80665

    # Display computed biases
    tbl = Table(title="Computed Biases", show_header=True)
    tbl.add_column("IMU")
    tbl.add_column("gx (rad/s)", justify="right")
    tbl.add_column("gy (rad/s)", justify="right")
    tbl.add_column("gz (rad/s)", justify="right")
    tbl.add_column("ax (m/s²)", justify="right")
    tbl.add_column("ay (m/s²)", justify="right")
    tbl.add_column("az (m/s²)", justify="right")
    for i in range(3):
        tbl.add_row(
            f"IMU{i}",
            *[f"{gyro_bias[i][k]:+.5f}"  for k in range(3)],
            *[f"{accel_bias[i][k]:+.5f}" for k in range(3)],
        )
    console.print(tbl)
    console.print(f"  Samples: IMU0={counts[0]}  IMU1={counts[1]}  IMU2={counts[2]}")

    # Warn if any gyro std-dev is high (motion detected)
    try:
        ans = input("\nWrite to flight controller flash? [y/N] ").strip().lower()
    except EOFError:
        ans = "n"
    if ans != "y":
        console.print("[yellow]Calibration not written.")
        return

    # Re-open serial (reader was stopped)
    writer = ser

    for i in range(3):
        gx, gy, gz = gyro_bias[i]
        ax, ay, az = accel_bias[i]
        cmd = f"CAL,set,{i},{gx:.6f},{gy:.6f},{gz:.6f},{ax:.6f},{ay:.6f},{az:.6f}"
        writer.write((cmd + "\n").encode())
        writer.flush()
        time.sleep(0.1)

    writer.write(b"CAL,commit\n")
    writer.flush()

    # Wait for CAL,OK
    deadline = time.monotonic() + 3.0
    buf = b""
    ok_received = False
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line_b, buf = buf.split(b"\n", 1)
                resp = line_b.decode("ascii", errors="replace").strip()
                if resp == "CAL,OK":
                    ok_received = True
                    break
        if ok_received:
            break
        time.sleep(0.05)

    if not ok_received:
        console.print("[red]No CAL,OK received — write may have failed.")
        return

    console.print("[green]Calibration written to flash.")

    # Verify round-trip via CAL,query
    ser.write(b"CAL,query\n")
    ser.flush()
    deadline = time.monotonic() + 3.0
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line_b, buf = buf.split(b"\n", 1)
                resp = line_b.decode("ascii", errors="replace").strip()
                if resp.startswith("CAL,DATA,"):
                    console.print(f"[dim]Stored: {resp}")
                    return
        time.sleep(0.05)
    console.print("[yellow]No CAL,DATA response from query.")


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

    sub.add_parser("ekf-status",  help="Per-lane EKF attitude/rate display (DEBUG build required)")

    sub.add_parser("can-status",  help="Read FDCAN1 protocol status and error counters")

    sub.add_parser("dshot-health", help="Live DShot TX/IC rate monitor + ESC arm detector (runs continuously)")
    sub.add_parser("dshot-diag",  help="One-shot DShot bidirectional telemetry snapshot (edge counts, timing)")

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

    cmd_p = sub.add_parser("cmd", help="Send a raw USB command and print the response")
    cmd_p.add_argument("raw", help="Command string to send (e.g. \"LOG,status\")")

    cal_p = sub.add_parser("calibrate", help="Collect IMU static bias calibration and write to flash")
    cal_p.add_argument("--duration", type=int, default=30,
                       help="Collection time in seconds (default: 30)")

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

        elif args.command == "ekf-status":
            cmd_ekf_status(ser, args)

        elif args.command == "can-status":
            cmd_can_status(ser, args)

        elif args.command == "dshot-health":
            cmd_dshot_health(ser, args)

        elif args.command == "dshot-diag":
            cmd_dshot_diag(ser, args)

        elif args.command == "motor-test":
            cmd_motor_test(ser, args)

        elif args.command == "calibrate":
            cmd_calibrate(ser, args)

        elif args.command == "cmd":
            ser.reset_input_buffer()
            send_cmd(ser, args.raw)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                resp = _read_line(ser, timeout=deadline - time.monotonic())
                if resp is None:
                    print("(no response)")
                    break
                if not resp.startswith("$"):
                    print(resp)
                    break

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
