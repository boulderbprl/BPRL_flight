#!/usr/bin/env python3
"""
BPRL Hardware Status Monitor — one-glance bench connectivity check.

Polls the always-built HW,status USB command and shows a live green/red
panel for every "is this actually plugged in and talking" flag the
firmware tracks: IMUs, CAN INS (IMX5), barometer, strain-rate sensor,
each encoder RPM node, RC link, SD card, and raw CAN bus traffic counters.

No BPRL_DEBUG build required — this is meant to work on a plain flight
build so you can bench-check wiring without reflashing.

Usage:
    python3 tools/hw_status.py                # default: live status display
    python3 tools/hw_status.py hw-status

Options:
    --port /dev/ttyACMx   Serial port (auto-detected if omitted)
    --baud N              Baud rate (default 115200, ignored by USB CDC)
"""

import argparse
import re
import time
from typing import Optional

from bprl_common import console, open_port, add_port_args

from rich.live import Live
from rich.panel import Panel
from rich.table import Table

NUM_ENC_NODES = 2

ROW_RE = re.compile(r"HW,([A-Z0-9]+),(.*)")


def _poll_hw_status(ser) -> Optional[dict]:
    """Send HW,status and return {key: value_str} for every HW,<key>,<rest> line up to HW,END."""
    ser.reset_input_buffer()
    ser.write(b"HW,status\r\n")
    rows = {}
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if not line:
            continue
        if line == "HW,END":
            return rows
        m = ROW_RE.match(line)
        if m:
            rows[m.group(1)] = m.group(2)
    return rows or None


def _dot(ok: bool) -> str:
    return "[green]●[/green]" if ok else "[red]○[/red]"


def cmd_hw_status(ser, _args):
    rows: dict = {}
    last_rx = 0.0

    def build_panel() -> Panel:
        age   = time.monotonic() - last_rx
        stale = last_rx == 0.0 or age > 3.0

        tbl = Table.grid(padding=(0, 3))
        tbl.add_column(min_width=16)
        tbl.add_column(min_width=10)
        tbl.add_column()

        def row(label, key, extra=""):
            val = rows.get(key)
            ok  = val is not None and val.split(",")[0] == "1"
            tbl.add_row(f"[bold]{label}[/bold]", _dot(ok and not stale), extra)

        row("IMU0 (pri)", "IMU0")
        row("IMU1 (ext)", "IMU1")
        row("IMU2",       "IMU2")
        row("CAN INS (IMX5)", "CANIMU")
        row("Barometer",  "BARO")
        row("Strain rate", "STRAIN")
        for i in range(NUM_ENC_NODES):
            val = rows.get(f"ENC{i}", "")
            parts = val.split(",")
            rpm = f"{float(parts[1]):.1f} rpm" if len(parts) > 1 else ""
            row(f"Encoder {i}", f"ENC{i}", rpm)
        row("RC link",    "RADIO")
        row("SD card",    "SD")

        can = rows.get("CANBUS", "")
        can_txt = f"[dim]{can}[/dim]" if can else "[dim]no data[/dim]"

        body = Table.grid()
        body.add_row(tbl)
        body.add_row("")
        body.add_row(f"[bold]CAN bus[/bold]  {can_txt}")

        title = "Hardware Status" + ("  [yellow](stale)[/yellow]" if stale else "")
        return Panel(body, title=title, border_style="cyan")

    console.print("[dim]Polling HW,status at ~2 Hz — Ctrl-C to exit[/dim]")
    try:
        with Live(build_panel(), refresh_per_second=10, console=console) as live:
            while True:
                result = _poll_hw_status(ser)
                if result is not None:
                    rows   = result
                    last_rx = time.monotonic()
                live.update(build_panel())
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BPRL hardware status monitor — live display via HW,status USB command")
    add_port_args(parser)
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("hw-status",
                   help="Live hardware connectivity monitor")

    args = parser.parse_args()
    if args.command is None:
        args.command = "hw-status"

    ser = open_port(args.port, args.baud)
    try:
        cmd_hw_status(ser, args)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
